# Tests fuer die Statusanzeige, laufen ohne ROS und ohne Platine

import math
from types import SimpleNamespace

from pi_plus_display.app import DisplayApp
from pi_plus_display.framebuffer import FRAME_HEAD, FRAME_TAIL, TYPE_FB_PAGE, framebuffer_frames, to_pages
from pi_plus_display.pages import STATUS_PAGES, render_status
from pi_plus_display.ros_source import RosStatusSource
from pi_plus_display.state import RobotState
from pi_plus_display.uart import TYPE_JOYSTICK, FrameParser, joystick_events


def _source(monkeypatch):
    monkeypatch.setattr(RosStatusSource, "_subscribe", lambda self: None)
    return RosStatusSource(node=None, team_id=6, player_number=2)


def test_all_status_pages_render():
    for i in range(len(STATUS_PAGES)):
        img = render_status(RobotState(), i)
        assert img.size == (128, 64)
        assert img.mode == "1"
        assert img.getbbox() is not None


def test_framebuffer_page_frames():
    img = render_status(RobotState(), 0)
    frames, data = framebuffer_frames(img)
    assert len(data) == 1024
    assert len(frames) == 8
    for page, f in enumerate(frames):
        assert f[:2] == FRAME_HEAD and f[-3:] == FRAME_TAIL
        assert f[2] == 1 + 1 + 128 + 2  # LEN = Typ + Page-Index + Daten + CRC16, passt in ein Byte
        assert f[3] == TYPE_FB_PAGE and f[4] == page
    unchanged, _ = framebuffer_frames(img, changed_only_from=data)
    assert unchanged == []


def test_framebuffer_bit_layout():
    img = render_status(RobotState(), 0).point(lambda _: 0)
    img.putpixel((5, 9), 1)  # Page 1, Bit 1
    data = to_pages(img)
    assert data[1 * 128 + 5] == 0b10
    assert sum(data) == 0b10


def _joystick_frame(key, pressed=1):
    return FRAME_HEAD + bytes([3, TYPE_JOYSTICK, key, pressed]) + FRAME_TAIL


def test_frame_parser_split_and_garbage():
    parser = FrameParser()
    stream = b"\x00\xa5" + _joystick_frame(3) + _joystick_frame(4, pressed=0) + _joystick_frame(2)
    frames = []
    for i in range(0, len(stream), 3):  # in kleinen Stuecken wie vom UART
        frames += parser.feed(stream[i : i + 3])
    assert joystick_events(frames) == ["right", "left"]


def test_joystick_drives_app(monkeypatch):
    src = _source(monkeypatch)
    app = DisplayApp(src)
    app.mode = "status"
    for key in joystick_events(FrameParser().feed(_joystick_frame(3))):
        app.joystick(key)
    assert app.page == 1


def test_callbacks_fill_state(monkeypatch):
    src = _source(monkeypatch)
    v = SimpleNamespace
    src.on_strategy(v(role=3, action=2))
    src.on_battery(v(data=src.battery_full_v))
    half = math.sqrt(0.5)
    src.on_imu(v(orientation=v(w=half, x=half, y=0.0, z=0.0)))  # 90 Grad Roll
    src.on_ball(v(pose=v(pose=v(position=v(x=3.0, y=4.0, z=0.0)))))
    src.update(0.1)
    s = src.snapshot()
    assert s.role == "STURM" and s.action == "Ball anlaufen"
    assert s.battery_percent == 100
    assert abs(s.roll_deg - 90) < 1e-6 and abs(s.pitch_deg) < 1e-6
    assert s.ball_seen and s.ball_dist_m == 5.0
    assert s.player_number == 2
    src.on_robot_state(v(state=2))
    assert src.snapshot().action == "Gefallen"


def _diag(name, level, message="OK"):
    return SimpleNamespace(name=name, level=bytes([level]), message=message)


# Namen wie vom Aggregator (bitbots_diagnostic) und von livelybot_serial (DS <joint>)
def test_diagnostics_groups_and_motors(monkeypatch):
    src = _source(monkeypatch)
    statuses = [
        _diag("/Robot", 2, "Error"),
        _diag("/Robot/Camera", 0),
        _diag("/Robot/Servos", 2, "Error"),
        _diag("/Robot/System", 0),
        _diag("/Robot/Servos/ r_hip_pitch_joint", 0),
        _diag("/Robot/Servos/ head_yaw_joint", 0),
        _diag("/Robot/Servos/ l_calf_joint", 3, "No CAN data"),
        _diag("/Robot/Servos/ l_hip_pitch_joint", 2, "Fault 0x04"),
        _diag("/Robot/Servos/ l_thigh_joint", 1, "Torque overload (12 N·m)"),
    ]
    src.on_diagnostics(SimpleNamespace(status=statuses))
    src.update(0.1)
    s = src.snapshot()
    assert [n.name for n in s.nodes] == ["camera", "servos", "system", "zed"]
    assert [n.ok for n in s.nodes] == [True, False, True, False]  # zed: keine camera_info empfangen
    assert [m.name for m in s.motors] == ["l-hip-pitch", "l-thigh", "l-calf", "r-hip-pitch", "head-yaw"]
    assert [m.id for m in s.motors] == [1, 2, 3, 4, 5]
    assert [m.ok for m in s.motors] == [False, True, False, True, True]
    assert s.motors[0].error == "FAULT 0X04"
    assert s.motors[2].error == "KEINE ANTW."
    src.on_camera_info(SimpleNamespace())
    src.update(0.1)
    assert src.snapshot().nodes[-1].ok
    img = render_status(s, 3)  # Motorseite mit Fehlertext
    assert img.getbbox() is not None
