# Tests for the node's callbacks (display.py); without a running ROS, see conftest.py

import math
import os
import sys
import types
from types import SimpleNamespace as NS

import pytest

from pi_plus_display import display
from pi_plus_display.pages import STATUS_PAGES
from pi_plus_display.state import RobotState


@pytest.fixture
def node():
    n = object.__new__(display.DisplayNode)  # without the rclpy constructor
    n.state = RobotState()
    n.page, n.alarm, n._failed, n._battery_low = 0, None, None, False
    return n


def test_quat_to_yaw():
    half = math.sqrt(0.5)
    assert abs(display.quat_to_yaw(NS(w=half, x=0.0, y=0.0, z=half)) - math.pi / 2) < 1e-9
    assert display.quat_to_yaw(NS(w=1.0, x=0.0, y=0.0, z=0.0)) == 0.0


def test_strategy(node):
    node.on_strategy(NS(role=3, action=2))
    assert node.state.role == "STRIKER" and node.state.action == "Going to ball"
    node.on_strategy(NS(role=99, action=99))
    assert node.state.role == "?" and node.state.action == "-"


def test_pose_confidence_and_trail(node):
    def msg(x, var):
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[35] = var
        return NS(pose=NS(pose=NS(position=NS(x=x, y=1.0, z=0.0), orientation=NS(w=1.0, x=0.0, y=0.0, z=0.0)), covariance=cov))

    node.on_pose(msg(2.0, 0.0))
    assert node.state.pose == (2.0, 1.0, 0) and node.state.loc_percent == 100
    node.on_pose(msg(2.5, 0.25))
    assert node.state.loc_percent < 100
    for i in range(50):
        node.on_pose(msg(float(i), 0.0))
    assert len(node.state.pose_trail) == display.POSE_TRAIL


def test_joint_states_feed_torques_and_history(node):
    node.on_joint_states(NS(name=["l_calf_joint", "r_calf_joint"], effort=[1.0, -8.0]))
    node.state.update()
    assert [(m.name, m.torque_nm) for m in node.state.motors] == [("l-calf", 1.0), ("r-calf", 8.0)]
    assert node.state.leg_torque_history == [8.0]


def _diag(name, level, message="OK"):
    return NS(name=name, level=bytes([level]), message=message)


# Names as from the aggregator (bitbots_diagnostic) and from livelybot_serial
def test_diagnostics_servos(node):
    node.on_diagnostics(
        NS(
            status=[
                _diag("/Robot", 2, "Error"),
                _diag("/Robot/Camera", 0),
                _diag("/Robot/Servos", 2, "Error"),
                _diag("/Robot/Servos/ r_hip_pitch_joint", 0),
                _diag("/Robot/Servos/ l_calf_joint", 3, "No CAN data"),
                _diag("/Robot/Servos/ l_hip_pitch_joint", 2, "Fault 0x04"),
                _diag("/Robot/Servos/ l_thigh_joint", 1, "Torque overload (12 N·m)"),
            ]
        )
    )
    node.state.update()
    motors = {m.name: (m.ok, m.error) for m in node.state.motors}
    assert motors["l-calf"] == (False, "NO RESPONSE")
    assert motors["l-hip-pitch"] == (False, "FAULT 0X04")
    assert motors["l-thigh"] == (True, "")  # WARN counts as running
    assert motors["r-hip-pitch"] == (True, "")
    node.on_diagnostics(NS(status=[]))  # an empty message changes nothing
    node.state.update()
    assert len(node.state.motors) == 4


def test_diagnostics_level_may_be_int(node):
    node.on_diagnostics(NS(status=[NS(name="/A/Servos/ m", level=2, message="x")]))
    node.state.update()
    assert [(m.name, m.ok, m.error) for m in node.state.motors] == [("m", False, "X")]


def _statuses(**levels):
    return NS(status=[_diag("/Robot", max(levels.values(), default=0))] + [
        _diag(f"/Robot/{name}", lvl, "broken") for name, lvl in levels.items()
    ])


def test_no_alarm_for_failures_present_at_first_message(node):
    node.on_diagnostics(_statuses(Camera=2))
    node.on_diagnostics(_statuses(Camera=2))
    assert node.alarm is None


def test_alarm_on_new_error_or_stale_leaf(node):
    node.on_diagnostics(_statuses(Camera=0, Battery=0))
    node.on_diagnostics(_statuses(Camera=2, Battery=0))
    assert node.alarm[:3] == ("ERROR", "Camera", "broken")
    node.joystick("center")
    node.on_diagnostics(_statuses(Camera=2, Battery=3))  # Camera persists, Battery is new
    assert node.alarm[:3] == ("STALE", "Battery", "broken")
    node.joystick("center")
    node.on_diagnostics(_statuses(Camera=2, Battery=3))
    assert node.alarm is None  # nothing new


def test_warn_and_group_do_not_alarm(node):
    node.on_diagnostics(_statuses(Camera=0))
    node.on_diagnostics(_statuses(Camera=1))  # WARN
    assert node.alarm is None
    # only the leaf reports, not the group "/Robot/Servos" above it
    leaf = lambda lvl: NS(status=[_diag("/Robot/Servos", lvl), _diag("/Robot/Servos/ l_calf_joint", lvl, "No CAN data")])  # noqa: E731
    node.on_diagnostics(leaf(0))
    node.on_diagnostics(leaf(2))
    assert node.alarm[:3] == ("ERROR", "l_calf_joint", "No CAN data")


def test_alarm_again_after_recovery(node):
    node.on_diagnostics(_statuses(Camera=0))
    node.on_diagnostics(_statuses(Camera=2))
    node.joystick("center")
    node.on_diagnostics(_statuses(Camera=0))
    node.on_diagnostics(_statuses(Camera=2))
    assert node.alarm is not None


# ------------------------------------------------------------- Joystick, alarm, rendering


def test_joystick_pages_wrap_and_clear_alarm(node):
    node.raise_alarm("X", "a", "b")
    node.joystick("left")
    assert node.page == len(STATUS_PAGES) - 1 and node.alarm is None
    node.joystick("right")
    assert node.page == 0
    node.joystick("center")
    assert node.page == 0


def test_render_always_returns_an_image(node, monkeypatch):
    monkeypatch.setattr(display.time, "monotonic", lambda: 0.0)
    node.raise_alarm("ERROR", "a" * 40, "b", seconds=1)
    assert len(node.alarm[1]) == 21  # fits on one line
    for now in (0.0, 0.5, 10.0):  # alarm on, alarm off, expired
        monkeypatch.setattr(display.time, "monotonic", lambda now=now: now)
        assert node.render().size == (128, 64)
    assert node.alarm is None


def test_alarm_blinks_between_alarm_and_status(node, monkeypatch):
    monkeypatch.setattr(display.time, "monotonic", lambda: 0.0)
    node.raise_alarm("NODE", "vision", "down")
    on = node.render()
    monkeypatch.setattr(display.time, "monotonic", lambda: 0.5)
    assert on.tobytes() != node.render().tobytes()


def test_on_frame_handles_keys_and_sends_image(node):
    class Board:
        def __init__(self):
            self.sent = []

        def read_keys(self):
            return ["right"]

        def send_image(self, img):
            self.sent.append(img)

    node.uart = Board()
    node._on_frame()
    assert node.page == 1 and len(node.uart.sent) == 1


def test_uart_errors_are_fatal(node):
    class Broken:
        def __init__(self, fail_on):
            self.fail_on = fail_on

        def read_keys(self):
            if self.fail_on == "read":
                raise OSError("gone")
            return []

        def send_image(self, img):
            if self.fail_on == "write":
                raise OSError("gone")

    for fail_on, text in (("read", "Reading"), ("write", "Sending")):
        node.uart = Broken(fail_on)
        with pytest.raises(display.FatalError, match=text):
            node._on_frame()


def test_game_state_messages_mean_connected(node):
    node.state.update()
    assert not node.state.gc_connected  # without a game controller
    node.on_game_state(NS())
    node.state.update()
    assert node.state.gc_connected


# ------------------------------------------------------------- Low battery alarm


def test_low_battery_alarm_once_and_again_after_recovery(node):
    node.state.battery.percent = None
    node._alarm_on_low_battery()
    assert node.alarm is None  # unknown is no alarm
    node.state.battery.percent = 11
    node._alarm_on_low_battery()
    assert node.alarm is None
    node.state.battery.percent = 9
    node._alarm_on_low_battery()
    assert node.alarm[:3] == ("BATTERY", "9 %", "please charge")
    node.joystick("center")
    node.state.battery.percent = 8
    node._alarm_on_low_battery()
    assert node.alarm is None  # stays low: only once
    node.state.battery.percent = 30
    node._alarm_on_low_battery()
    node.state.battery.percent = 5
    node._alarm_on_low_battery()
    assert node.alarm is not None


def test_low_battery_at_startup_alarms(node):
    node.state.battery.percent = 4
    node._alarm_on_low_battery()
    assert node.alarm[:2] == ("BATTERY", "4 %")


# ------------------------------------------------------------- Node and protocol together


def test_node_and_board_talk_through_the_protocol(node):
    from pi_plus_display.board import StatusBoardUart
    from pi_plus_display.display_demo import LoopbackSerial

    ser = LoopbackSerial()
    node.uart = StatusBoardUart(ser)
    node._on_frame()
    assert ser.image().tobytes() == node.render().convert("1").tobytes()  # the board shows what the node renders
    ser.press("right")  # board key -> key frame -> page change -> new image
    node._on_frame()
    assert node.page == 1
    assert ser.image().tobytes() == node.render().convert("1").tobytes()


# ------------------------------------------------------------- Error cases: better to exit than to half work


def test_open_serial_errors_are_fatal(tmp_path, monkeypatch):
    with pytest.raises(display.FatalError, match="not set"):
        display.open_serial("", 115200)
    with pytest.raises(display.FatalError, match="does not exist.*demo:=true"):
        display.open_serial(str(tmp_path / "ttyX"), 115200)
    port = tmp_path / "ttyS"
    port.write_text("")
    port.chmod(0o000)
    if os.geteuid() != 0:  # as root there are no permission problems
        with pytest.raises(display.FatalError, match="dialout"):
            display.open_serial(str(port), 115200)
    port.chmod(0o600)

    monkeypatch.setitem(sys.modules, "serial", None)  # not installed
    with pytest.raises(display.FatalError, match="pyserial"):
        display.open_serial(str(port), 115200)

    fake = types.ModuleType("serial")
    def boom(*args, **kwargs):
        raise OSError("port busy")
    fake.Serial = boom
    monkeypatch.setitem(sys.modules, "serial", fake)
    with pytest.raises(display.FatalError, match="cannot be opened.*port busy"):
        display.open_serial(str(port), 115200)

    opened = []
    fake.Serial = lambda *args, **kwargs: opened.append((args, kwargs)) or "port"
    assert display.open_serial(str(port), 9600) == "port"
    assert opened == [((str(port), 9600), {"timeout": 0, "write_timeout": 0.1})]


def _run_main(monkeypatch, capsys, *, node_factory, spin):
    fake_rclpy = types.SimpleNamespace(init=lambda args=None: None, spin=spin, try_shutdown=lambda: None)
    monkeypatch.setattr(display, "rclpy", fake_rclpy)
    monkeypatch.setattr(display, "DisplayNode", node_factory)
    with pytest.raises(SystemExit) as exit_info:
        display.main()
    return exit_info.value.code, capsys.readouterr().err


class _FakeNode:
    fatal_error = None
    destroyed = False

    def destroy_node(self):
        self.destroyed = True


def test_main_prints_the_error_and_exits_with_1(monkeypatch, capsys):
    def factory():
        raise display.FatalError("port gone")

    code, err = _run_main(monkeypatch, capsys, node_factory=factory, spin=lambda node: None)
    assert code == 1 and "FATAL: port gone" in err

    node = _FakeNode()

    def spin(n):
        raise display.FatalError("UART broken")

    code, err = _run_main(monkeypatch, capsys, node_factory=lambda: node, spin=spin)
    assert code == 1 and "FATAL: UART broken" in err and node.destroyed


def test_main_reports_errors_from_the_demo_window_thread(monkeypatch, capsys):
    node = _FakeNode()

    def spin(n):
        n.fatal_error = "Demo window: no screen"
        raise KeyboardInterrupt  # this is how the window interrupts the main thread

    code, err = _run_main(monkeypatch, capsys, node_factory=lambda: node, spin=spin)
    assert code == 1 and "no screen" in err


def test_main_exits_quietly_on_ctrl_c(monkeypatch, capsys):
    node = _FakeNode()

    def spin(n):
        raise KeyboardInterrupt

    monkeypatch.setattr(display, "rclpy", types.SimpleNamespace(init=lambda args=None: None, spin=spin, try_shutdown=lambda: None))
    monkeypatch.setattr(display, "DisplayNode", lambda: node)
    display.main()  # no SystemExit
    assert capsys.readouterr().err == "" and node.destroyed
