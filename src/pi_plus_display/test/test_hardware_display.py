# Tests fuer die Hardware-Seiten: LocalHardware auf einem nachgebauten /sys und /proc, Akku-Callbacks, Seiten

from types import SimpleNamespace
from unittest.mock import patch

from pi_plus_display import hardware_interfaces
from pi_plus_display.app import DisplayApp, _display_values
from pi_plus_display.local_hardware import LocalHardware
from pi_plus_display.pages import STATUS_PAGES, render_status
from pi_plus_display.ros_source import RosStatusSource

HW_PAGES = ["akku", "rechner", "temperatur", "netz_io"]


def _write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)


# /sys und /proc mit CPU-Zaehlern, Speicher, zwei Temperaturfuehlern und zwei Interfaces
def _fake_root(root, user=100, idle=800, rx=1000):
    sys_, proc = root / "sys", root / "proc"
    _write(proc / "stat", f"cpu  {user} 0 50 {idle} 50 0 0 0\n")
    _write(proc / "meminfo", "MemTotal: 4194304 kB\nMemAvailable: 1048576 kB\n")
    _write(proc / "uptime", "4980.5 100.0\n")
    _write(proc / "loadavg", "1.21 0.97 0.80 1/115 591\n")
    _write(sys_ / "class/thermal/thermal_zone0/type", "cpu-thermal")
    _write(sys_ / "class/thermal/thermal_zone0/temp", "71000")
    _write(sys_ / "class/hwmon/hwmon0/name", "cpu_thermal")
    _write(sys_ / "class/hwmon/hwmon0/temp1_input", "70500")
    _write(sys_ / "class/thermal/thermal_zone1/type", "gpu-thermal")
    _write(sys_ / "class/thermal/thermal_zone1/temp", "64000")
    _write(sys_ / "devices/system/cpu/cpu0/cpufreq/scaling_cur_freq", "1800000")
    for name, state in (("wlan0", "up"), ("eth0", "down"), ("lo", "unknown")):
        base = sys_ / "class/net" / name
        _write(base / "operstate", state)
        _write(base / "address", "00:00:00:00:00:00")
        _write(base / "statistics/rx_bytes", str(rx))
        _write(base / "statistics/tx_bytes", "500")
    return sys_, proc


def _patched(tmp_path, **kw):
    sys_, proc = _fake_root(tmp_path, **kw)
    return (
        patch.object(hardware_interfaces, "SYS", sys_),
        patch.object(hardware_interfaces, "PROC", proc),
        patch.object(hardware_interfaces, "ipv4_addresses", lambda: {"wlan0": "172.20.1.13"}),
    )


def test_local_hardware_rates_from_two_polls(tmp_path):
    t = [10.0]
    hw = LocalHardware(clock=lambda: t[0])
    p_sys, p_proc, p_ip = _patched(tmp_path)
    with p_sys, p_proc, p_ip:
        first = hw.poll()
        _fake_root(tmp_path, user=130, idle=870, rx=3000)  # 30 von 100 Ticks belegt, 2000 Bytes mehr
        t[0] = 12.0
        second = hw.poll()
    assert first.cpu_percent is None  # erst ein Messpunkt
    assert second.cpu_percent == 30.0
    assert second.cpu_mhz == 1800.0
    assert second.load_avg == (1.21, 0.97, 0.80)
    assert second.uptime_s == 4980.5
    assert second.ram_percent == 75.0 and second.ram_total_gb == 4.0
    # cpu-thermal und cpu_thermal sind derselbe Fuehler: einmal, mit dem hoeheren Wert
    assert [(s.name, s.celsius) for s in second.temps] == [("CPU", 71.0), ("GPU", 64.0)]
    assert [n.name for n in second.net] == ["wlan0", "eth0"]  # lo fehlt, aktive zuerst
    wlan, eth = second.net
    assert wlan.up and wlan.ip == "172.20.1.13" and wlan.rx_bps == 1000.0 and wlan.tx_bps == 0.0
    assert not eth.up and eth.ip is None


def test_poll_hardware_fills_state_and_overrides_workload(tmp_path):
    src = RosStatusSource(team_id=6, player_number=2)
    p_sys, p_proc, p_ip = _patched(tmp_path)
    with p_sys, p_proc, p_ip:
        src.poll_hardware()
        _fake_root(tmp_path, user=150, idle=850)
        src.poll_hardware()
    s = src.snapshot()
    assert s.cpu_percent == 50 and s.cpu_history == [50.0]
    assert s.wlan_ip == "172.20.1.13" and s.eth_ip == "-"
    workload = SimpleNamespace(cpu_usage_overall=99.0, memory_used=1, memory_total=1, network_interfaces=[])
    src.on_workload(workload)
    assert src.snapshot().cpu_percent == 50  # lokale Werte gelten weiter


def test_battery_topics_and_bms_alarm():
    src = RosStatusSource(team_id=6, player_number=2)
    app = DisplayApp(src)
    v = SimpleNamespace
    src.on_battery(v(data=23.7))
    src.on_battery_current(v(data=2.5))
    src.on_battery_temperature(v(data=33.0))
    src.on_bms_error(v(data=0))
    app.tick(0.1)
    src.poll_hardware()
    s = src.snapshot()
    assert (s.battery_current_a, s.battery_temp_c, s.bms_error) == (2.5, 33.0, 0)
    assert s.battery_history == [23.7]
    assert app.alarm is None
    src.on_bms_error(v(data=3))
    app.tick(0.1)
    assert app.alarm == ("AKKU", "BMS-Fehler 3", "Akku pruefen")


def test_hardware_pages_render_without_data():
    names = [n for n, _ in STATUS_PAGES]
    s = _display_values(RosStatusSource().snapshot())
    for name in HW_PAGES:
        img = render_status(s, names.index(name))
        assert img.size == (128, 64) and img.getbbox() is not None
