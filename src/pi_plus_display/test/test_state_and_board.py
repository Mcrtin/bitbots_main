# Tests fuer Seiten und RobotState; laufen ohne ROS und ohne Platine

from pi_plus_display.hardware_interfaces import Battery, NetIface
from pi_plus_display.pages import STATUS_PAGES, render_status
from pi_plus_display.state import Group, Motor, Pose, RobotState


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def _group_of(i: int) -> Group:  # wie die Motorliste: 6 links, 6 rechts, danach Arme
    return "left_leg" if i < 6 else "right_leg" if i < 12 else "arms"


def _full_state():
    s = RobotState(gc_connected=True, role="STURM", action="Ball anlaufen")
    s.loc_percent, s.pose, s.pose_trail = 78, Pose(1.3, -0.8, 45), [(0.0, 0.0), (1.0, -0.5)]
    s.battery = Battery(23.4, 72, 3.2, 31.0)
    s.cpu_percent, s.cpu_temp_c, s.ram_percent, s.wifi_rssi = 63, 71, 48, -58
    s.motors = [Motor(i + 1, f"m{i}", _group_of(i), torque_nm=i * 1.5) for i in range(12)]
    s.motors[3] = Motor(4, "l-calf", "left_leg", ok=False, error="KEINE ANTW.")
    s.leg_torque_history = [1.0, 5.0, 12.0, 3.0]
    s.net_ifaces = [NetIface("wlan0", True, "172.20.1.13", 12500.0, 3100.0), NetIface("can0", False)]
    return s


def test_all_status_pages_render_empty_and_full():
    for state in (RobotState(), _full_state()):
        for i in range(len(STATUS_PAGES)):
            img = render_status(state, i)
            assert img.size == (128, 64) and img.mode == "1"
            assert img.getbbox() is not None


def test_wlan_ip_comes_from_net_ifaces():
    assert RobotState().wlan_ip == "-"
    assert _full_state().wlan_ip == "172.20.1.13"


# ------------------------------------------------------------- RobotState (Ableitungen)


def test_gc_connected_only_while_game_states_arrive():
    clock = Clock()
    src = RobotState(clock=clock)
    src.update()
    assert not src.gc_connected  # nie etwas empfangen
    src.game_state_received()
    src.update()
    assert src.gc_connected
    clock.t += 2.9
    src.update()
    assert src.gc_connected
    clock.t += 0.2  # GC_TIMEOUT_S ohne neue Nachricht
    src.update()
    assert not src.gc_connected
    src.game_state_received()
    src.update()
    assert src.gc_connected  # kommt wieder


def test_motors_sorted_like_grid_with_torques():
    clock = Clock()
    src = RobotState(clock=clock)
    src.set_servo_status({"l_calf_joint": (False, "KEINE ANTW."), "l_hip_pitch_joint": (True, "")})
    src.set_joint_torques({"head_yaw_joint": 0.5, "r_hip_pitch_joint": -4.0, "l_hip_pitch_joint": 2.0})
    src.update()
    m = src.motors
    assert [x.name for x in m] == ["l-hip-pitch", "l-calf", "r-hip-pitch", "head-yaw"]
    assert [x.group for x in m] == ["left_leg", "left_leg", "right_leg", "head"]
    assert [x.id for x in m] == [1, 2, 3, 4]
    assert [x.ok for x in m] == [True, False, True, True]  # r_hip_pitch/head_yaw: nicht in der Diagnose, Daten frisch
    assert m[1].error == "KEINE ANTW." and m[0].torque_nm == 2.0 and m[1].torque_nm is None
    assert m[2].torque_nm == 4.0  # nur der Betrag zaehlt
    clock.t += 1  # joint_states veraltet
    src.update()
    stale = src.motors
    assert [x.ok for x in stale] == [True, False, False, False]
    assert stale[2].error == "KEINE DATEN" and all(x.torque_nm is None for x in stale)


def test_groups_and_helpers_for_the_pages():
    src = RobotState(clock=Clock())
    src.set_servo_status({"l_calf_joint": (False, "x"), "r_calf_joint": (True, "")})
    src.set_joint_torques({"head_pitch_joint": 1.0, "l_elbow_joint": 2.0, "l_calf_joint": 9.0, "r_calf_joint": -12.0})
    src.update()
    assert [m.name for m in src.motors_of("left_leg")] == ["l-calf"]
    assert [m.name for m in src.motors_of("head", "arms")] == ["head-pitch", "l-elbow"]
    assert [m.name for m in src.bad_motors] == ["l-calf"]
    peak = src.peak_motor
    assert peak is not None and peak.name == "r-calf" and peak.torque_nm == 12.0
    assert RobotState().peak_motor is None and RobotState().bad_motors == []


def test_leg_torque_history_keeps_peak_per_sample():
    clock = Clock()
    src = RobotState(clock=clock)
    src.set_joint_torques({"l_calf_joint": 3.0, "head_yaw_joint": 99.0})  # Kopf zaehlt nicht
    src.set_joint_torques({"l_calf_joint": -7.0})
    src.set_joint_torques({"l_calf_joint": 2.0})
    src.update()
    assert src.leg_torque_history == [7.0]
    src.update()  # gleiche Zehntelsekunde, kein neuer Wert
    clock.t += 0.2
    src.update()  # nichts Neues gemessen
    assert src.leg_torque_history == [7.0]
    for _ in range(100):
        clock.t += 0.2
        src.set_joint_torques({"r_knee": 1.0, "r_thigh_joint": 1.0})
        src.update()
    assert len(src.leg_torque_history) == 60


def test_hardware_polled_once_per_second():
    clock = Clock()
    src = RobotState(clock=clock)
    calls = []
    src._poll_hardware = lambda: calls.append(clock.t)
    src.update()
    src.update()
    clock.t += 1.1
    src.update()
    assert len(calls) == 2


def test_poll_hardware_fills_state(monkeypatch):
    from pi_plus_display import hardware_interfaces as hw

    src = RobotState()
    monkeypatch.setattr(hw, "ram_percent", lambda: 47.6)
    monkeypatch.setattr(hw, "cpu_temperature", lambda: 70.4)
    monkeypatch.setattr(hw, "wifi_rssi", lambda: -60)
    monkeypatch.setattr(hw, "battery", lambda: Battery(23.4, 72, 3.2, 31.0))
    monkeypatch.setattr(src._hardware, "cpu_percent", lambda: 12.5)
    monkeypatch.setattr(src._hardware, "net_ifaces", lambda: [NetIface("wlan0", True, "10.0.0.2")])
    src.update()
    s = src
    assert (s.cpu_percent, s.ram_percent, s.cpu_temp_c, s.wifi_rssi) == (12, 48, 70, -60)
    assert s.wlan_ip == "10.0.0.2"
    assert s.battery == Battery(23.4, 72, 3.2, 31.0)


def test_motors_page_shows_arms_row_and_failure_count():
    s = _full_state()
    s.motors = [Motor(i + 1, f"m{i}", _group_of(i)) for i in range(22)]
    ok = render_status(s, 2)
    s.motors[15] = Motor(16, "r-elbow", "arms", ok=False, error="FAULT")  # Arme/Kopf-Reihe
    one = render_status(s, 2)
    s.motors[3] = Motor(4, "l-calf", "left_leg", ok=False, error="KEINE ANTW.")
    two = render_status(s, 2)
    assert len({ok.tobytes(), one.tobytes(), two.tobytes()}) == 3


def test_motor_boxes_are_shaded_by_torque():
    s = _full_state()
    images = []
    for torque in (None, 0.0, 5.0, 15.0, 40.0):
        s.motors = [Motor(1, "l-hip-pitch", "left_leg", torque_nm=torque)]
        images.append(render_status(s, 2).crop((9, 12, 27, 22)).tobytes())  # nur das erste Kaestchen
    assert images[0] == images[1]  # unbekannt sieht aus wie 0 Nm: leer
    assert len({images[1], images[2], images[3]}) == 3  # mehr Drehmoment, mehr Fuellung
    assert images[4] == images[3]  # ab der Skala voll
