# Fuellt den RobotState aus den Nachrichten des Bit-Bots-Stacks.
# Die Subscriptions liegen in display.py; deren Callbacks reichen jede Nachricht an die passende on_*-Methode weiter.
# Es gibt keinen eigenen Thread und keinen eigenen Executor.

import copy
import math
import socket
import threading
import time
from collections import deque

from transforms3d.euler import quat2euler

from .local_hardware import LocalHardware
from .state import Motor, RobotState, TopicStat
from .state import Node as NodeStatus

ROLE_NAMES = {0: "?", 1: "PAUSE", 2: "ANDERE", 3: "STURM", 4: "UNTERST.", 5: "ABWEHR", 6: "TOR"}
ACTION_NAMES = {
    0: "-",
    1: "Positionieren",
    2: "Ball anlaufen",
    3: "Tor schiessen",
    4: "Warten",
    5: "Schiessen",
    6: "Ball suchen",
    7: "Lokalisieren",
    8: "Passiv",
}
# RobotControlState: Zustaende, die wichtiger sind als die Verhaltensaktion
ROBOT_STATE_TEXT = {
    1: "Faellt",
    2: "Gefallen",
    3: "Steht auf",
    5: "Startet",
    6: "Faehrt runter",
    7: "Strafe",
    11: "Motoren aus",
    12: "HCM aus",
    13: "Hardwarefehler",
    14: "Hochgehoben",
}

GC_TIMEOUT_S = 3.0
IMU_TIMEOUT_S = 1.0
CAMERA_TIMEOUT_S = 1.0

# Reihenfolge im Motorraster: erst linkes, dann rechtes Bein von der Huefte zum Fuss,
# danach alle anderen Motoren (Arme, Kopf) alphabetisch. Namen wie in livelybot_serial/config/robot.yaml.
LEG_JOINTS = ["hip_pitch", "hip_roll", "thigh", "calf", "ankle_pitch", "ankle_roll"]
# Meldungen aus robot::motorDiagnostic (livelybot_serial) -> kurzer Text fuer die Anzeige
MOTOR_MESSAGES = {"No CAN data": "KEINE ANTW."}
JOINT_TIMEOUT_S = 0.5
# Datenfluss-Seite: Kurzname je Topic-Schluessel, in dieser Reihenfolge wenn gleich schnell
TOPIC_LABELS = {
    "joint_states": "GELENKE",
    "camera_info": "KAMERA",
    "pose": "LOKALIS.",
    "strategy": "STRATEGIE",
    "diagnostics": "DIAGNOSE",
    "gamestate": "GC",
    "battery": "AKKU",
    "imu": "IMU",
    "ball": "BALL",
    "robot_state": "HCM",
    "workload": "SYSTEM",
    "team_data": "TEAM",
}
HZ_WINDOW = 20  # Empfangszeiten je Topic fuer die Rate
EFFORT_SAMPLE_S = 0.1  # ein Wert pro Zehntelsekunde fuer den Last-Verlauf
EFFORT_HISTORY = 60  # 6 s
POSE_TRAIL = 30


class RosStatusSource:
    # Datenquelle fuer DisplayApp: start/stop/update/snapshot wie im Mockup (sources.py)

    name = "ros2"
    simulated = False

    # battery_*_v: Spannungen fuer 0 % und 100 %, an den Akku des Pi Plus anpassen
    # clock: Zeitquelle in Sekunden, Standard time.monotonic; die Log-Wiedergabe setzt die Simulationszeit ein
    def __init__(
        self,
        team_id=None,
        player_number=None,
        battery_empty_v=21.0,
        battery_full_v=25.2,
        clock=None,
    ):
        self.clock = clock or time.monotonic
        self.team_id = team_id
        self.battery_empty_v = battery_empty_v
        self.battery_full_v = battery_full_v
        self._lock = threading.Lock()
        self._stamps = {}  # Topic-Schluessel -> Empfangszeit (monotonic)
        self._diag_groups = []  # NodeStatus der Diagnose-Gruppen, ohne Kamera
        self._recv = {}  # Topic-Schluessel -> deque der letzten Empfangszeiten
        self._diag_motors = {}  # Gelenkname -> (ok, Fehlertext) aus .../Servos/<joint>
        self._joint_names = []  # Reihenfolge aus joint_states
        self._efforts = {}  # Gelenkname -> Drehmoment [Nm]
        self._leg_peak = None  # hoechste Bein-Last seit dem letzten Verlaufswert
        self._last_effort_sample = None
        self._gc_diag = ""  # Meldung der Diagnose "/Robot/Other/Game Controller", z. B. "Connected"
        self._strategy_action = "-"
        self._robot_state_text = None
        self._hardware = LocalHardware()
        self._local_hw = False  # True, sobald hardware_interfaces Werte liefert; dann zaehlt system_workload nicht

        s = self._state = RobotState()
        s.robot_name = socket.gethostname().split(".")[0].upper()[:8]
        if player_number is not None:
            s.player_number = player_number
        # Testwerte aus RobotState leeren, bis echte Nachrichten kommen; Unbekanntes ist None,
        # nur die Lage steht bis zur ersten Nachricht auf 0
        s.action = "warte auf Daten"
        s.role = "-"
        s.gc_connected = False
        s.loc_confidence = None
        s.battery_voltage = s.battery_percent = None
        s.battery_current_a = s.battery_temp_c = s.bms_error = None
        s.net_ifaces = []
        s.pose = None
        s.roll_deg = s.pitch_deg = 0.0
        s.cpu_percent = s.cpu_temp_c = s.ram_percent = None
        s.wlan_ip = "-"
        s.wifi_rssi = None
        s.motors, s.nodes = [], []

    # ------------------------------------------------------------- Schnittstelle fuer DisplayApp

    def start(self):
        pass

    def stop(self):
        pass

    def snapshot(self):
        with self._lock:
            return copy.deepcopy(self._state)

    # Aus dem Menue, wirkt bisher nur auf die Anzeige
    def set_role(self, role):
        with self._lock:
            self._state.role = role

    # Spielernummer, sobald das parameter_blackboard antwortet (StatusDisplay)
    def set_player_number(self, number):
        with self._lock:
            self._state.player_number = number

    def sim_key(self, ch):
        pass

    # Aus Empfangszeiten abgeleitete Werte (verbunden, gesehen, ...)
    def update(self, dt):
        now = self.clock()
        with self._lock:
            s = self._state
            # Ohne gamestate (display.py abonniert es nicht) zaehlt die Diagnose des GC-Clients
            gc_diag_ok = self._gc_diag == "Connected" and now - self._stamps.get("diagnostics", -1e9) < GC_TIMEOUT_S
            s.gc_connected = now - self._stamps.get("gamestate", -1e9) < GC_TIMEOUT_S or gc_diag_ok
            s.imu_ok = now - self._stamps.get("imu", -1e9) < IMU_TIMEOUT_S
            camera_ok = now - self._stamps.get("camera_info", -1e9) < CAMERA_TIMEOUT_S
            s.nodes = self._diag_groups[:5] + [NodeStatus("zed", camera_ok)]
            s.motors = self._build_motors(now)
            self._sample_effort(now)
            s.topics = self._topic_stats(now)

    def _touch(self, key):
        now = self.clock()
        self._stamps[key] = now
        self._recv.setdefault(key, deque(maxlen=HZ_WINDOW)).append(now)

    # Motorliste aus Diagnose (Zustand) und joint_states (Drehmoment), sortiert wie im Raster
    def _build_motors(self, now):
        joints_fresh = now - self._stamps.get("joint_states", -1e9) < JOINT_TIMEOUT_S
        names = list(dict.fromkeys(list(self._diag_motors) + self._joint_names))
        motors = []
        for i, name in enumerate(sorted(names, key=_joint_order)):
            if name in self._diag_motors:
                ok, error = self._diag_motors[name]
            else:
                ok, error = joints_fresh, "" if joints_fresh else "KEINE DATEN"
            effort = self._efforts.get(name) if joints_fresh else None
            motors.append(Motor(i + 1, _short_joint(name), ok, temp_c=None, error=error, effort_nm=effort))
        return motors

    def _sample_effort(self, now):
        if self._last_effort_sample is not None and now - self._last_effort_sample < EFFORT_SAMPLE_S:
            return
        self._last_effort_sample = now
        if self._leg_peak is None:
            return
        hist = self._state.leg_effort_history
        hist.append(self._leg_peak)
        del hist[:-EFFORT_HISTORY]
        self._leg_peak = None

    def _topic_stats(self, now):
        stats = []
        for key, label in TOPIC_LABELS.items():
            times = self._recv.get(key)
            if not times:
                stats.append(TopicStat(label))
                continue
            span = times[-1] - times[0]
            hz = (len(times) - 1) / span if span > 0 else None
            stats.append(TopicStat(label, hz, now - times[-1]))
        # Empfangene zuerst, schnellste oben; dann die fehlenden in fester Reihenfolge
        return sorted(stats, key=lambda t: (t.age_s is None, -(t.hz or 0.0)))

    # ------------------------------------------------------------- Callbacks

    # Nur fuer die GC-Verbindung und die Datenfluss-Seite; Spielstand und Strafen zeigt keine Seite
    def on_gamestate(self, msg):
        with self._lock:
            self._touch("gamestate")

    def on_strategy(self, msg):
        with self._lock:
            self._touch("strategy")
            self._state.role = ROLE_NAMES.get(msg.role, "?")
            self._strategy_action = ACTION_NAMES.get(msg.action, "-")
            self._apply_action()

    def on_robot_state(self, msg):
        with self._lock:
            self._touch("robot_state")
            self._robot_state_text = ROBOT_STATE_TEXT.get(msg.state)
            self._apply_action()

    def _apply_action(self):
        # Zustand der Hardware (gefallen, Motoren aus) geht vor der Verhaltensaktion
        self._state.action = self._robot_state_text or self._strategy_action

    # Nur fuer die Datenfluss-Seite (Rate von "BALL"); die Entfernung zeigt keine Seite mehr
    def on_ball(self, msg):
        with self._lock:
            self._touch("ball")

    def on_pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        _, _, yaw = quat2euler((q.w, q.x, q.y, q.z))
        cov = msg.pose.covariance
        # Grobe Konfidenz aus Positions- und Winkelunsicherheit (1 = sicher). Heuristik, ggf. anpassen;
        # in der Simulation sind die x/y-Varianzen 0, nur die Yaw-Varianz ist gesetzt
        sigma = math.sqrt(max(0.0, cov[0]) + max(0.0, cov[7])) + math.sqrt(max(0.0, cov[35]))
        with self._lock:
            self._touch("pose")
            s = self._state
            s.pose = (p.x, p.y, round(math.degrees(yaw)))
            s.loc_confidence = math.exp(-sigma)
            s.pose_trail.append((p.x, p.y))
            del s.pose_trail[:-POSE_TRAIL]

    def on_battery(self, msg):
        v = float(msg.data)
        span = self.battery_full_v - self.battery_empty_v
        with self._lock:
            self._touch("battery")
            self._state.battery_voltage = v
            self._state.battery_percent = max(0.0, min(100.0, (v - self.battery_empty_v) / span * 100))

    def on_battery_current(self, msg):
        with self._lock:
            self._touch("battery_current")
            self._state.battery_current_a = float(msg.data)

    def on_battery_temperature(self, msg):
        with self._lock:
            self._touch("battery_temperature")
            self._state.battery_temp_c = float(msg.data)

    # Fehlercode des BMS, 0 = kein Fehler
    def on_bms_error(self, msg):
        with self._lock:
            self._touch("bms_error")
            self._state.bms_error = int(msg.data)

    def on_imu(self, msg):
        q = msg.orientation
        roll, pitch, _ = quat2euler((q.w, q.x, q.y, q.z))
        with self._lock:
            self._touch("imu")
            self._state.roll_deg, self._state.pitch_deg = math.degrees(roll), math.degrees(pitch)

    def on_camera_info(self, msg):
        with self._lock:
            self._touch("camera_info")

    # Drehmoment je Gelenk; Zustand (ok/Fehler) kommt aus der Diagnose, sonst gilt "Daten kommen" als ok
    def on_joint_states(self, msg):
        efforts = dict(zip(msg.name, msg.effort, strict=False))
        legs = [abs(e) for n, e in efforts.items() if _joint_order(n)[0] < 2]
        with self._lock:
            self._touch("joint_states")
            if list(msg.name) != self._joint_names:
                self._joint_names = list(msg.name)
            self._efforts = efforts
            if legs:
                self._leg_peak = max(legs) if self._leg_peak is None else max(self._leg_peak, max(legs))

    def on_workload(self, msg):
        wlan = None
        for iface in msg.network_interfaces:
            ip = next((a for a in iface.ip_addresses if ":" not in a), None)
            if ip and iface.name.startswith("wl"):
                wlan = ip
                break
        with self._lock:
            self._touch("workload")
            if self._local_hw:
                return  # dieselben Werte kommen schon lokal aus hardware_interfaces
            s = self._state
            s.cpu_percent = int(msg.cpu_usage_overall)
            if msg.memory_total:
                s.ram_percent = int(100 * msg.memory_used / msg.memory_total)
            s.wlan_ip = wlan or "-"

    # Gruppen des Diagnostic Aggregators und je Motor ein Eintrag unter .../Servos/<joint>.
    # Der Aggregator haengt alles unter einen Wurzelpfad ("/Robot", siehe bitbots_diagnostic/config).
    # Angezeigt werden die Gruppen eine Ebene darunter, z. B. "/Robot/Servos".
    def on_diagnostics(self, msg):
        if not msg.status:
            return
        depth = min(st.name.count("/") for st in msg.status)
        if sum(1 for st in msg.status if st.name.count("/") == depth) == 1:
            depth += 1  # einzelne Wurzel ueberspringen
        groups = [
            NodeStatus(st.name.rsplit("/", 1)[-1].strip().lower()[:7], _level(st) <= 1)
            for st in msg.status
            if st.name.count("/") == depth
        ]
        motors = {_servo_name(st): _motor_status(st) for st in msg.status if "/Servos/" in st.name}
        gc = next((st.message for st in msg.status if st.name.endswith("/Game Controller")), "")
        with self._lock:
            self._touch("diagnostics")
            self._diag_groups = groups
            self._diag_motors = motors
            self._gc_diag = gc

    # Nur fuer die Datenfluss-Seite (Rate von "TEAM")
    def on_team_data(self, msg):
        with self._lock:
            self._touch("team_data")

    # Werte, die es nicht als Topic gibt: CPU-Temperatur und WLAN-Signal
    def poll_local(self):
        temp = _read_cpu_temp()
        rssi = _read_wifi_rssi()
        with self._lock:
            if temp is not None:
                self._state.cpu_temp_c = temp
            self._state.wifi_rssi = rssi

    # Rechner-Werte aus hardware_interfaces; laeuft im 1-s-Timer von StatusDisplay
    def poll_hardware(self):
        hw = self._hardware.poll()  # liest Dateien, daher ausserhalb des Locks
        with self._lock:
            s = self._state
            if hw.cpu_percent is not None or hw.ram_percent is not None:
                self._local_hw = True
            if self._local_hw:
                s.cpu_percent = None if hw.cpu_percent is None else round(hw.cpu_percent)
                s.ram_percent = None if hw.ram_percent is None else round(hw.ram_percent)
                wlan = next((n.ip for n in hw.net if n.ip and n.name.startswith("wl")), None)
                s.wlan_ip = wlan or "-"
            s.net_ifaces = hw.net


# DiagnosticStatus.level ist in rclpy ein bytes-Objekt der Laenge 1
def _level(status):
    lv = status.level
    return lv[0] if isinstance(lv, bytes | bytearray) else int(lv)


def _servo_name(status):
    return status.name.split("/Servos/", 1)[1].strip()


def _joint_order(name):
    for side, prefix in enumerate(("l_", "r_")):
        for i, joint in enumerate(LEG_JOINTS):
            if name.startswith(prefix + joint):
                return (side, i, name)
    return (2, 0, name)


# (ok, Fehlertext); OK und WARN (z. B. Drehmoment-Ueberlast) zaehlen als laufend, ERROR und STALE nicht
def _motor_status(status):
    ok = _level(status) <= 1
    error = "" if ok else MOTOR_MESSAGES.get(status.message, status.message.upper()[:11])
    return ok, error


# 'LKnee' / 'left_knee_joint' -> kurzer Anzeigename
def _short_joint(name):
    return name.replace("_joint", "").replace("_", "-")[:12]


def _read_cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return int(f.read().strip()) // 1000
    except (OSError, ValueError):
        return None


# WLAN-Signal in dBm aus /proc/net/wireless, sonst None
def _read_wifi_rssi():
    try:
        with open("/proc/net/wireless") as f:
            for line in f.readlines()[2:]:
                return int(float(line.split()[3].rstrip(".")))
    except (OSError, ValueError, IndexError):
        pass
    return None
