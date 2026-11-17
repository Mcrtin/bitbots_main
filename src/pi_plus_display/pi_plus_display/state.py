# Alles, was die Seiten (pages.py) anzeigen, in einem Objekt, dazu was sich daraus ableiten laesst.
#   - display.py (die Node) uebergibt, was sie von anderen Nodes bekommt (set_servo_status, set_joint_torques,
#     game_state_received),
#   - hardware_interfaces.py liefert die Werte des Rechners und den Akku,
#   - update() leitet daraus ab, was sich aus Empfangszeiten ergibt (GC verbunden, Motoren, Beinlast).
# Die Seiten lesen nur die oeffentlichen Felder; Standardwerte bedeuten "noch nichts empfangen" (None = unbekannt).
# Alles laeuft im Executor der Node, daher gibt es weder Threads noch Locks.

import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Literal, NamedTuple

from . import hardware_interfaces as hw

GC_TIMEOUT_S = 3.0  # so lange gilt der Game Controller nach der letzten Nachricht als verbunden
JOINT_TIMEOUT_S = 0.5
HW_POLL_S = 1.0
TORQUE_SAMPLE_S = 0.1  # ein Wert pro Zehntelsekunde fuer den Last-Verlauf
TORQUE_HISTORY = 60  # 6 s

Group = Literal["left_leg", "right_leg", "arms", "head"]


@dataclass
class Motor:
    id: int  # Position in der Liste, beginnend bei 1
    name: str
    group: Group
    ok: bool = True
    error: str = ""  # kurzer Fehlertext, wenn nicht ok
    torque_nm: float | None = None  # Betrag aus joint_states, None = unbekannt


# x [m], y [m], Blickrichtung [Grad]
class Pose(NamedTuple):
    x: float
    y: float
    theta_deg: int


# Beinmotoren in der Reihenfolge von der Huefte zum Fuss. Namen wie in livelybot_serial/config/robot.yaml.
LEG_JOINTS = ["hip_pitch", "hip_roll", "thigh", "calf", "ankle_pitch", "ankle_roll"]


# (Seite, Position) eines Beingelenks, sonst None
def _leg_joint(name: str) -> tuple[int, int] | None:
    for side, prefix in enumerate(("l_", "r_")):
        for i, joint in enumerate(LEG_JOINTS):
            if name.startswith(prefix + joint):
                return side, i
    return None


def _group(name: str) -> Group:
    leg = _leg_joint(name)
    if leg:
        return "right_leg" if leg[0] else "left_leg"
    return "head" if name.startswith("head") else "arms"


# Reihenfolge der Motorliste: erst linkes, dann rechtes Bein von der Huefte zum Fuss,
# danach alle anderen (Kopf, Arme) alphabetisch
def _motor_order(name: str) -> tuple[int, int, str]:
    side, i = _leg_joint(name) or (2, 0)
    return side, i, name


# 'l_calf_joint' -> 'l-calf' (Anzeigename)
def _short_joint(name: str) -> str:
    return name.replace("_joint", "").replace("_", "-")[:12]


def _round(value: float | None) -> int | None:
    return None if value is None else round(value)


@dataclass
class RobotState:
    clock: Callable[[], float] = field(default=time.monotonic, repr=False)  # Zeitquelle in Sekunden
    gc_connected: bool = False

    # Verhalten und Lokalisierung
    role: str = "-"
    action: str = "warte auf Daten"
    loc_percent: int | None = None
    pose: Pose | None = None
    pose_trail: list[tuple[float, float]] = field(default_factory=list)  # letzte Positionen fuer die Feldkarte

    battery: hw.Battery = field(default_factory=hw.Battery)

    # Motoren in Anzeigereihenfolge; Beinlast [Nm]: hoechster Betrag der Beinmotoren, ein Wert pro
    # Zehntelsekunde, aeltester zuerst
    motors: list[Motor] = field(default_factory=list)
    leg_torque_history: list[float] = field(default_factory=list)

    # Rechner
    cpu_percent: int | None = None
    cpu_temp_c: int | None = None
    ram_percent: int | None = None
    wifi_rssi: int | None = None
    net_ifaces: list[hw.NetIface] = field(default_factory=list)

    def __post_init__(self) -> None:
        self._hardware = hw.Hardware(self.clock)
        self._servo_status: dict[str, tuple[bool, str]] = {}  # Gelenkname -> (ok, Fehlertext)
        self._torques: dict[str, float] = {}  # Gelenkname -> Betrag des Drehmoments [Nm]
        self._game_state_at: float | None = None  # Empfangszeiten
        self._joint_states_at: float | None = None
        self._leg_peak: float | None = None  # hoechste Bein-Last seit dem letzten Verlaufswert
        self._last_torque_sample: float | None = None
        self._next_hw_poll = 0.0

    # ------------------------------------------------------------- Seiten-Sicht

    @property
    def wlan_ip(self) -> str:
        return next((n.ip for n in self.net_ifaces if n.ip and n.name.startswith("wl")), "-")

    @property
    def bad_motors(self) -> list[Motor]:
        return [m for m in self.motors if not m.ok]

    # Motor mit dem groessten bekannten Drehmoment
    @property
    def peak_motor(self) -> Motor | None:
        return max((m for m in self.motors if m.torque_nm is not None), key=lambda m: m.torque_nm or 0.0, default=None)

    def motors_of(self, *groups: Group) -> list[Motor]:
        return [m for m in self.motors if m.group in groups]

    # ------------------------------------------------------------- Eingaben der Node

    # servos: Gelenkname -> (ok, Fehlertext) aus der Diagnose
    def set_servo_status(self, servos: dict[str, tuple[bool, str]]) -> None:
        self._servo_status = servos

    # Eine Nachricht des Game Controllers ist angekommen. Der Empfaenger veroeffentlicht gamestate nur fuer
    # gueltige Pakete, die Nachrichten selbst beweisen also, dass der Game Controller gerade zu hoeren ist.
    def game_state_received(self) -> None:
        self._game_state_at = self.clock()

    # efforts: Gelenkname -> Drehmoment [Nm] aus joint_states
    def set_joint_torques(self, efforts: dict[str, float]) -> None:
        self._torques = {name: abs(e) for name, e in efforts.items()}
        self._joint_states_at = self.clock()
        legs = [t for name, t in self._torques.items() if _leg_joint(name)]
        if legs:
            self._leg_peak = max(legs + [self._leg_peak or 0.0])

    # ------------------------------------------------------------- Ableitungen

    # Einmal pro Frame
    def update(self) -> None:
        now = self.clock()
        self.gc_connected = self._fresh(self._game_state_at, GC_TIMEOUT_S)
        self.motors = self._build_motors(self._fresh(self._joint_states_at, JOINT_TIMEOUT_S))
        self._sample_torque(now)
        if now >= self._next_hw_poll:
            self._next_hw_poll = now + HW_POLL_S
            self._poll_hardware()

    # Kam die Meldung in den letzten timeout Sekunden?
    def _fresh(self, received_at: float | None, timeout: float) -> bool:
        return received_at is not None and self.clock() - received_at < timeout

    # Motorliste aus Diagnose (Zustand) und joint_states (Drehmoment)
    def _build_motors(self, joints_fresh: bool) -> list[Motor]:
        names = sorted(dict.fromkeys([*self._servo_status, *self._torques]), key=_motor_order)
        motors: list[Motor] = []
        for i, name in enumerate(names):
            if name in self._servo_status:
                ok, error = self._servo_status[name]
            else:
                ok, error = joints_fresh, "" if joints_fresh else "KEINE DATEN"
            torque = self._torques.get(name) if joints_fresh else None
            motors.append(Motor(i + 1, _short_joint(name), _group(name), ok, error, torque))
        return motors

    def _sample_torque(self, now: float) -> None:
        if self._last_torque_sample is not None and now - self._last_torque_sample < TORQUE_SAMPLE_S:
            return
        self._last_torque_sample = now
        if self._leg_peak is None:
            return
        self.leg_torque_history.append(self._leg_peak)
        del self.leg_torque_history[:-TORQUE_HISTORY]
        self._leg_peak = None

    def _poll_hardware(self) -> None:
        self.cpu_percent = _round(self._hardware.cpu_percent())
        self.ram_percent = _round(hw.ram_percent())
        self.cpu_temp_c = _round(hw.cpu_temperature())
        self.wifi_rssi = hw.wifi_rssi()
        self.net_ifaces = self._hardware.net_ifaces()
        self.battery = hw.battery()
