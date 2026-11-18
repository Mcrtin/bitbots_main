# The node: subscribes to what the display needs and writes it into the RobotState (state.py).
# In a timer it renders the 128x64 image (pages.py) and sends it over UART to the board (board.py);
# the board's joystick controls the pages.
# Parameters (config/display.yaml): demo, serial_port, baudrate, frame_rate.
#   demo:=true   window on the computer instead of the board, serial_port is ignored
#   demo:=false  the board on serial_port must be openable
# Parameters are passed with --ros-args -p: ros2 run pi_plus_display display --ros-args -p demo:=true
# (without --ros-args -p, ROS treats demo:=true as a topic remapping and the node never sees the parameter)
# If the node cannot run sensibly (port missing, UART error, demo window does not open), it exits
# with an error message (FatalError). Other nodes still missing is normal, though: the display then shows
# "no data" and waits.

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

ROLE_NAMES = {0: "?", 1: "PAUSE", 2: "OTHER", 3: "STRIKER", 4: "SUPPORT", 5: "DEFENDER", 6: "GOALIE"}
ACTION_NAMES = {
    0: "-",
    1: "Positioning",
    2: "Going to ball",
    3: "Trying to score",
    4: "Waiting",
    5: "Kicking",
    6: "Searching ball",
    7: "Localizing",
    8: "Passive",
}
# Messages from robot::motorDiagnostic (livelybot_serial) -> short text for the display
MOTOR_MESSAGES = {"No CAN data": "NO RESPONSE"}
POSE_TRAIL = 30
LOW_BATTERY_PERCENT = 10  # alarm below this
ALARM_LINE_CHARS = 21  # characters per line in the alarm
WARN, ERROR = 1, 2  # DiagnosticStatus.WARN / .ERROR; STALE = 3


# The node cannot run sensibly like this: print a message and exit (see main)
class FatalError(Exception):
    pass


# Open serial port; FatalError if it is missing, not accessible or cannot be opened
def open_serial(port: str, baudrate: int) -> SerialPort:
    hint = "(without the board: ros2 run pi_plus_display display --ros-args -p demo:=true)"
    if not port:
        raise FatalError(f"serial_port is not set {hint}")
    if not os.path.exists(port):
        raise FatalError(f"serial_port {port} does not exist {hint}")
    if not os.access(port, os.R_OK | os.W_OK):
        raise FatalError(f"no read and write permission on {port} (is the user in the dialout group?)")
    try:
        import serial  # pyserial
    except ModuleNotFoundError as e:
        raise FatalError(f"pyserial is missing ({e}); in bitbots_main: pixi add --feature ros 'pyserial>=3.5,<4'") from e
    try:
        ser: SerialPort = serial.Serial(port, baudrate, timeout=0, write_timeout=0.1)
        return ser
    except (OSError, ValueError) as e:  # serial.SerialException is an OSError
        raise FatalError(f"{port} cannot be opened at {baudrate} baud: {e}") from e


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
            raise FatalError(f"frame_rate must be greater than 0, is {frame_rate}")

        self.state = RobotState()

        self.page = 1  # page 1 (page_behavior); page_teach is to its left (index 0)
        self._teach_client = self.create_client(SetTeachingMode, "teaching_mode")

        self.alarm: Alarm | None = None
        self.fatal_error: str | None = None  # reported by helper threads (demo window), main() prints it
        self._failed: set[str] | None = None  # failed diagnostics; None = no message received yet
        self._battery_low = False

        ser: SerialPort
        if demo:
            from .display_demo import LoopbackSerial, check_window, start_window

            try:
                check_window()
            except RuntimeError as e:
                raise FatalError(str(e)) from e
            loopback = LoopbackSerial()  # the window only sees what comes through the protocol
            start_window(self, loopback)
            ser = loopback
        else:
            ser = open_serial(serial_port, baudrate)
        self.uart = StatusBoardUart(ser)

        # Steady clock: the display should keep running even if use_sim_time is set and no /clock arrives
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

    # ------------------------------------------------------------- Board, rendering, joystick

    def _on_frame(self) -> None:
        try:
            keys = self.uart.read_keys()
        except OSError as e:
            raise FatalError(f"Reading from UART failed: {e}") from e
        for key in keys:
            self.joystick(key)
        self.state.update()
        self._alarm_on_low_battery()
        try:
            self.uart.send_image(self.render())
        except OSError as e:
            raise FatalError(f"Sending over UART failed: {e}") from e

    # key: "up", "down", "left", "right" or "center"; any key acknowledges the alarm
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
            self.raise_alarm("TEACH", "HCM service", "not ready", seconds=3)
            return
        req = SetTeachingMode.Request()
        req.state = int(mode)
        future = self._teach_client.call_async(req)
        future.add_done_callback(lambda f: self._on_teach_response(mode, f))

    def _on_teach_response(self, requested: TeachMode, future: Any) -> None:
        try:
            resp = future.result()
        except Exception as e:
            self.raise_alarm("TEACH", "Service error", str(e), seconds=4)
            return
        if resp is not None and resp.success:
            self.state.teach_mode = requested
        else:
            # E.g. if POSE was chosen directly from NONE or the robot is not CONTROLLABLE
            self.raise_alarm("TEACH", "Switch rejected", f"-> {requested.name}", seconds=3)

    # Blinks until acknowledged or until the time has run out
    def raise_alarm(self, title: str, line1: str, line2: str, seconds: float = 30) -> None:
        self.alarm = Alarm(title, line1[:ALARM_LINE_CHARS], line2[:ALARM_LINE_CHARS], time.monotonic() + seconds)

    # Alarm as soon as the battery drops below the threshold (also at startup); again only after recovery
    def _alarm_on_low_battery(self) -> None:
        percent = self.state.battery.percent
        low = percent is not None and percent < LOW_BATTERY_PERCENT
        if low and not self._battery_low:
            self.raise_alarm("BATTERY", f"{percent} %", "please charge", seconds=300)
        self._battery_low = low

    def render(self) -> Image.Image:
        now = time.monotonic()
        if self.alarm and now > self.alarm.end:
            self.alarm = None
        if self.alarm and int(now * 2) % 2 == 0:  # blinks at 1 Hz
            return render_alarm(self.alarm.title, (self.alarm.line1, self.alarm.line2))
        return render_status(self.state, self.page)

    # ------------------------------------------------------------- Messages

    def on_strategy(self, msg: Strategy) -> None:
        self.state.role = ROLE_NAMES.get(msg.role, "?")
        self.state.action = ACTION_NAMES.get(msg.action, "-")

    # The game controller receiver publishes gamestate only for valid packets; the content does not matter here
    def on_game_state(self, msg: GameState) -> None:
        self.state.game_state_received()

    def on_joint_states(self, msg: JointState) -> None:
        self.state.set_joint_torques(dict(zip(msg.name, msg.effort, strict=False)))

    def on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        cov = msg.pose.covariance
        # Rough confidence from position and angle uncertainty (1 = certain). Heuristic, adjust if needed;
        # in simulation the x/y variances are 0, only the yaw variance is set
        sigma = math.sqrt(max(0.0, cov[0]) + max(0.0, cov[7])) + math.sqrt(max(0.0, cov[35]))
        s = self.state
        s.pose = Pose(p.x, p.y, round(math.degrees(yaw)))
        s.loc_percent = round(100 * math.exp(-sigma))
        s.pose_trail.append((p.x, p.y))
        del s.pose_trail[:-POSE_TRAIL]

    # Diagnostics from the aggregator (bitbots_diagnostic), topic diagnostics_agg: a DiagnosticArray with one
    # DiagnosticStatus per entry. Each has name (path like "/Robot/Servos/ l_calf_joint"), level
    # (OK=0, WARN=1, ERROR=2, STALE=3) and message (short text). Groups summarise their children.
    def on_diagnostics(self, msg: DiagnosticArray) -> None:
        if not msg.status:
            return
        servos = {st.name.split("/Servos/", 1)[1].strip(): _motor_status(st) for st in msg.status if "/Servos/" in st.name}
        self.state.set_servo_status(servos)
        self._alarm_on_new_failures(msg.status)

    # Alarm as soon as an entry becomes ERROR or STALE. Only leaf entries without children, otherwise
    # all groups above would report the same thing. Whatever is already failed in the first message raises no alarm.
    def _alarm_on_new_failures(self, statuses: list[DiagnosticStatus]) -> None:
        groups = {st.name.rsplit("/", 1)[0] for st in statuses}
        failed = {st.name: st for st in statuses if _level(st) > WARN and st.name not in groups}
        if self._failed is not None:
            for name in sorted(failed.keys() - self._failed):
                st = failed[name]
                title = "ERROR" if _level(st) == ERROR else "STALE"
                self.raise_alarm(title, name.rsplit("/", 1)[-1].strip(), st.message)
        self._failed = set(failed)


# Yaw angle in radians from a quaternion
def quat_to_yaw(q: Any) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


# In rclpy, DiagnosticStatus.level is a bytes object of length 1
def _level(status: DiagnosticStatus) -> int:
    lv = status.level
    return lv[0] if isinstance(lv, bytes | bytearray) else int(lv)


# (ok, error text); WARN (e.g. torque overload) counts as running
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
        pass  # Ctrl+C or demo window closed
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
