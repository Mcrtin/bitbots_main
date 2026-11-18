# Die Node: abonniert, was die Anzeige braucht, und schreibt es in den RobotState (state.py).
# In einem Timer rendert sie das 128x64-Bild (pages.py) und schickt es per UART an die Platine (board.py);
# deren Joystick steuert die Seiten.
# Parameter (config/display.yaml): demo, serial_port, baudrate, frame_rate.
#   demo:=true   Fenster am Rechner statt der Platine, serial_port wird ignoriert
#   demo:=false  die Platine an serial_port muss sich oeffnen lassen
# Parameter gibt man mit --ros-args -p an: ros2 run pi_plus_display display --ros-args -p demo:=true
# (ohne --ros-args -p wertet ROS demo:=true als Topic-Umleitung und die Node sieht den Parameter nie)
# Kann die Node so nicht sinnvoll laufen (Port fehlt, Fehler auf dem UART, Demo-Fenster geht nicht auf), beendet sie
# sich mit einer Fehlermeldung (FatalError). Dass andere Nodes noch fehlen, ist dagegen normal: dann zeigt die Anzeige
# "keine Daten" und wartet.

import math
import os
import sys
import time
from typing import Any, NamedTuple

import rclpy
from PIL import Image
from bitbots_msgs.msg import Strategy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from game_controller_hsl_interfaces.msg import GameState
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import JointState

from bitbots_msgs.srv import SetTeachingMode
from .board import Key, SerialPort, StatusBoardUart
from .pages import STATUS_PAGES, render_alarm, render_status
from .state import TEACH_MODES, Pose, RobotState, TeachMode

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
# Meldungen aus robot::motorDiagnostic (livelybot_serial) -> kurzer Text fuer die Anzeige
MOTOR_MESSAGES = {"No CAN data": "KEINE ANTW."}
POSE_TRAIL = 30
LOW_BATTERY_PERCENT = 10  # darunter Alarm
ALARM_LINE_CHARS = 21  # Zeichen pro Zeile im Alarm
WARN, ERROR = 1, 2  # DiagnosticStatus.WARN / .ERROR; STALE = 3


# Die Node kann so nicht sinnvoll laufen: Meldung ausgeben und beenden (siehe main)
class FatalError(Exception):
    pass


# Offener serieller Port; FatalError, wenn er fehlt, nicht zugreifbar ist oder sich nicht oeffnen laesst
def open_serial(port: str, baudrate: int) -> SerialPort:
    hint = "(ohne Platine: ros2 run pi_plus_display display --ros-args -p demo:=true)"
    if not port:
        raise FatalError(f"serial_port ist nicht gesetzt {hint}")
    if not os.path.exists(port):
        raise FatalError(f"serial_port {port} gibt es nicht {hint}")
    if not os.access(port, os.R_OK | os.W_OK):
        raise FatalError(f"keine Lese- und Schreibrechte auf {port} (Benutzer in der Gruppe dialout?)")
    try:
        import serial  # pyserial
    except ModuleNotFoundError as e:
        raise FatalError(f"pyserial fehlt ({e}); in bitbots_main: pixi add --feature ros 'pyserial>=3.5,<4'") from e
    try:
        ser: SerialPort = serial.Serial(port, baudrate, timeout=0, write_timeout=0.1)
        return ser
    except (OSError, ValueError) as e:  # serial.SerialException ist ein OSError
        raise FatalError(f"{port} laesst sich nicht mit {baudrate} Baud oeffnen: {e}") from e


class Alarm(NamedTuple):
    title: str
    line1: str
    line2: str
    end: float  # time.monotonic()


class DisplayNode(Node):
    def __init__(self) -> None:
        super().__init__("display")
        demo = self.declare_parameter("demo", False).value
        serial_port = self.declare_parameter("serial_port", "/dev/ttyS4").value
        baudrate = self.declare_parameter("baudrate", 115200).value
        frame_rate = self.declare_parameter("frame_rate", 10.0).value
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, False)])
        if frame_rate <= 0:
            raise FatalError(f"frame_rate muss groesser als 0 sein, ist {frame_rate}")

        self.state = RobotState()

        self.page = 1  # Seite 1 (page_behavior); links davon (Index 0) liegt page_teach
        self._teach_client = self.create_client(SetTeachingMode, "teaching_mode")

        self.alarm: Alarm | None = None
        self.fatal_error: str | None = None  # von Hilfs-Threads gemeldet (Demo-Fenster), main() gibt es aus
        self._failed: set[str] | None = None  # fehlerhafte Diagnosen; None = noch keine Meldung erhalten
        self._battery_low = False

        ser: SerialPort
        if demo:
            from .display_demo import LoopbackSerial, check_window, start_window

            try:
                check_window()
            except RuntimeError as e:
                raise FatalError(str(e)) from e
            loopback = LoopbackSerial()  # das Fenster sieht nur, was ueber das Protokoll kommt
            start_window(self, loopback)
            ser = loopback
        else:
            ser = open_serial(serial_port, baudrate)
        self.uart = StatusBoardUart(ser)

        # Steady Clock: die Anzeige soll auch laufen, wenn use_sim_time gesetzt ist und keine /clock kommt
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / frame_rate, self._on_frame, clock=steady)

        self.create_subscription(DiagnosticArray, "diagnostics_agg", self.on_diagnostics, 1)
        self.create_subscription(GameState, "gamestate", self.on_game_state, 1)
        self.create_subscription(JointState, "joint_states", self.on_joint_states, 1)
        self.create_subscription(Strategy, "strategy", self.on_strategy, 1)
        self.create_subscription(PoseWithCovarianceStamped, "pose_with_covariance", self.on_pose, 1)

    def destroy_node(self) -> None:
        self.uart.close()
        super().destroy_node()

    # ------------------------------------------------------------- Platine, Rendern, Joystick

    def _on_frame(self) -> None:
        try:
            keys = self.uart.read_keys()
        except OSError as e:
            raise FatalError(f"Lesen vom UART fehlgeschlagen: {e}") from e
        for key in keys:
            self.joystick(key)
        self.state.update()
        self._alarm_on_low_battery()
        try:
            self.uart.send_image(self.render())
        except OSError as e:
            raise FatalError(f"Senden per UART fehlgeschlagen: {e}") from e

    # key: "up", "down", "left", "right" oder "center"; jede Taste quittiert den Alarm
    def joystick(self, key: Key) -> None:
        self.alarm = None
        on_teach = STATUS_PAGES[self.page] is page_teach
        match key:
            case "left" | "right":
                if on_teach and self.state.teach_mode != TeachMode.OFF:
                    self._request_teach_mode(TeachMode.OFF)
                self.page = (self.page + (1 if key == "right" else -1)) % len(STATUS_PAGES)
            case "center" if on_teach:
                next_mode = TeachMode.HOLD if self.state.teach_mode == TeachMode.TEACH else TeachMode.TEACH
                self._request_teach_mode(next_mode)

    def _request_teach_mode(self, mode: TeachMode) -> None:
        if not self._teach_client.service_is_ready():
            self.raise_alarm("TEACH", "HCM Service", "nicht bereit", seconds=3)
            return
        req = SetTeachingMode.Request()
        req.state = int(mode)
        future = self._teach_client.call_async(req)
        future.add_done_callback(lambda f: self._on_teach_response(mode, f))

    def _on_teach_response(self, requested: TeachMode, future: Any) -> None:
        try:
            resp = future.result()
        except Exception as e:
            self.raise_alarm("TEACH", "Service Fehler", str(e), seconds=4)
            return
        if resp is not None and resp.success:
            self.state.teach_mode = requested
        else:
            # Z.B. wenn POSE direkt aus NONE gewaehlt wurde oder der Roboter nicht CONTROLLABLE ist
            self.raise_alarm("TEACH", "Wechsel abgelehnt", f"-> {requested.name}", seconds=3)

    # Blinkt, bis es quittiert wird oder die Zeit abgelaufen ist
    def raise_alarm(self, title: str, line1: str, line2: str, seconds: float = 30) -> None:
        self.alarm = Alarm(title, line1[:ALARM_LINE_CHARS], line2[:ALARM_LINE_CHARS], time.monotonic() + seconds)

    # Alarm, sobald der Akku unter die Schwelle faellt (auch beim Start); erst nach Erholung wieder
    def _alarm_on_low_battery(self) -> None:
        percent = self.state.battery.percent
        low = percent is not None and percent < LOW_BATTERY_PERCENT
        if low and not self._battery_low:
            self.raise_alarm("AKKU", f"{percent} %", "bitte laden", seconds=300)
        self._battery_low = low

    def render(self) -> Image.Image:
        now = time.monotonic()
        if self.alarm and now > self.alarm.end:
            self.alarm = None
        if self.alarm and int(now * 2) % 2 == 0:  # blinkt mit 1 Hz
            return render_alarm(self.alarm.title, (self.alarm.line1, self.alarm.line2))
        return render_status(self.state, self.page)

    # ------------------------------------------------------------- Nachrichten

    def on_strategy(self, msg: Strategy) -> None:
        self.state.role = ROLE_NAMES.get(msg.role, "?")
        self.state.action = ACTION_NAMES.get(msg.action, "-")

    # Der Game-Controller-Empfaenger veroeffentlicht gamestate nur fuer gueltige Pakete; der Inhalt ist hier egal
    def on_game_state(self, msg: GameState) -> None:
        self.state.game_state_received()

    def on_joint_states(self, msg: JointState) -> None:
        self.state.set_joint_torques(dict(zip(msg.name, msg.effort, strict=False)))

    def on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        cov = msg.pose.covariance
        # Grobe Konfidenz aus Positions- und Winkelunsicherheit (1 = sicher). Heuristik, ggf. anpassen;
        # in der Simulation sind die x/y-Varianzen 0, nur die Yaw-Varianz ist gesetzt
        sigma = math.sqrt(max(0.0, cov[0]) + max(0.0, cov[7])) + math.sqrt(max(0.0, cov[35]))
        s = self.state
        s.pose = Pose(p.x, p.y, round(math.degrees(yaw)))
        s.loc_percent = round(100 * math.exp(-sigma))
        s.pose_trail.append((p.x, p.y))
        del s.pose_trail[:-POSE_TRAIL]

    # Diagnose des Aggregators (bitbots_diagnostic), Topic diagnostics_agg: ein DiagnosticArray mit je einem
    # DiagnosticStatus pro Eintrag. Jeder hat name (Pfad wie "/Robot/Servos/ l_calf_joint"), level
    # (OK=0, WARN=1, ERROR=2, STALE=3) und message (Kurztext). Gruppen fassen ihre Unterpunkte zusammen.
    def on_diagnostics(self, msg: DiagnosticArray) -> None:
        if not msg.status:
            return
        servos = {st.name.split("/Servos/", 1)[1].strip(): _motor_status(st) for st in msg.status if "/Servos/" in st.name}
        self.state.set_servo_status(servos)
        self._alarm_on_new_failures(msg.status)

    # Alarm, sobald ein Eintrag ERROR oder STALE wird. Nur einzelne Eintraege ohne Unterpunkte, sonst
    # meldeten auch alle Gruppen darueber dasselbe. Was schon bei der ersten Meldung fehlerhaft ist, loest keinen Alarm aus.
    def _alarm_on_new_failures(self, statuses: list[DiagnosticStatus]) -> None:
        groups = {st.name.rsplit("/", 1)[0] for st in statuses}
        failed = {st.name: st for st in statuses if _level(st) > WARN and st.name not in groups}
        if self._failed is not None:
            for name in sorted(failed.keys() - self._failed):
                st = failed[name]
                title = "FEHLER" if _level(st) == ERROR else "STALE"
                self.raise_alarm(title, name.rsplit("/", 1)[-1].strip(), st.message)
        self._failed = set(failed)


# Gierwinkel in Radiant aus einem Quaternion
def quat_to_yaw(q: Any) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


# DiagnosticStatus.level ist in rclpy ein bytes-Objekt der Laenge 1
def _level(status: DiagnosticStatus) -> int:
    lv = status.level
    return lv[0] if isinstance(lv, bytes | bytearray) else int(lv)


# (ok, Fehlertext); WARN (z. B. Drehmoment-Ueberlast) zaehlt als laufend
def _motor_status(status: DiagnosticStatus) -> tuple[bool, str]:
    ok = _level(status) <= WARN
    return ok, "" if ok else MOTOR_MESSAGES.get(status.message, status.message.upper()[:11])


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: DisplayNode | None = None
    error: str | None = None
    try:
        node = DisplayNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Strg+C oder Demo-Fenster geschlossen
    except FatalError as e:
        error = str(e)
    finally:
        if node is not None:
            error = error or node.fatal_error
            node.destroy_node()
        rclpy.try_shutdown()
    if error:
        print(f"[display] FATAL: {error}", file=sys.stderr)
        sys.exit(1)
