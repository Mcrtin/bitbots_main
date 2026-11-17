# Tests fuer hardware_interfaces auf einem nachgebauten /sys und /proc

from unittest.mock import patch

import pytest

from pi_plus_display import hardware_interfaces as hw


def _write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)


# CPU-Zaehler, Speicher, zwei Thermal-Zonen, WLAN und drei Interfaces
def _fake_root(root, user=100, idle=800, rx=1000):
    sys_, proc = root / "sys", root / "proc"
    _write(proc / "stat", f"cpu  {user} 0 50 {idle} 50 0 0 0\n")
    _write(proc / "meminfo", "MemTotal: 4194304 kB\nMemAvailable: 1048576 kB\n")
    _write(proc / "net/wireless", "Inter-| sta\n face | tus\nwlan0: 0000   58.  -58.  -256  0 0 0 0 0 0\n")
    _write(sys_ / "class/thermal/thermal_zone0/temp", "71000")
    _write(sys_ / "class/thermal/thermal_zone0/type", "cpu-thermal")
    _write(sys_ / "class/thermal/thermal_zone1/temp", "90000")  # heisser, aber kein CPU-Sensor
    _write(sys_ / "class/thermal/thermal_zone1/type", "acpitz")
    for name, state in (("wlan0", "up"), ("eth0", "down"), ("can0", "unknown"), ("lo", "unknown")):
        base = sys_ / "class/net" / name
        _write(base / "operstate", state)
        _write(base / "statistics/rx_bytes", str(rx))
        _write(base / "statistics/tx_bytes", "500")
    return sys_, proc


@pytest.fixture
def fake(tmp_path):
    sys_, proc = _fake_root(tmp_path)
    with (
        patch.object(hw, "SYS", sys_),
        patch.object(hw, "PROC", proc),
        patch.object(hw, "ipv4_addresses", lambda: {"wlan0": "172.20.1.13"}),
    ):
        yield tmp_path


def test_ram_percent(fake):
    assert hw.ram_percent() == 75.0


def test_cpu_temperature_uses_cpu_sensors_only(fake):
    assert hw.cpu_temperature() == 71.0  # nicht die 90 Grad des acpitz-Fuehlers


def _hwmon(root, number, name, **temps):
    chip = root / "sys/class/hwmon" / f"hwmon{number}"
    _write(chip / "name", name)
    for i, (label, milli) in enumerate(temps.items(), start=1):
        _write(chip / f"temp{i}_input", str(milli))
        _write(chip / f"temp{i}_label", label.replace("_", " "))


def test_cpu_temperature_on_an_amd_pc_reads_hwmon_not_the_acpi_zone(tmp_path):
    # wie auf einem PC, bei dem die Thermal-Zonen nur 17 Grad zeigen, k10temp aber 49
    _write(tmp_path / "sys/class/thermal/thermal_zone0/temp", "17000")
    _write(tmp_path / "sys/class/thermal/thermal_zone0/type", "acpitz")
    _hwmon(tmp_path, 0, "nvme", Composite=61850)  # SSD, heisser als die CPU, zaehlt nicht
    _hwmon(tmp_path, 1, "k10temp", Tctl=49125)
    _hwmon(tmp_path, 2, "amdgpu", edge=55000)
    with patch.object(hw, "SYS", tmp_path / "sys"):
        assert hw.cpu_temperature() == 49.125


def test_cpu_temperature_on_an_intel_pc_takes_the_hottest_core(tmp_path):
    _hwmon(tmp_path, 0, "coretemp", Package_id_0=52000, Core_0=48000, Core_1=55000)
    with patch.object(hw, "SYS", tmp_path / "sys"):
        assert hw.cpu_temperature() == 55.0


def test_cpu_temperature_unknown_sensors_or_garbage_give_none(tmp_path):
    _write(tmp_path / "sys/class/thermal/thermal_zone0/temp", "17000")
    _write(tmp_path / "sys/class/thermal/thermal_zone0/type", "acpitz")
    _hwmon(tmp_path, 0, "nvme", Composite=40000)
    with patch.object(hw, "SYS", tmp_path / "sys"):
        assert hw.cpu_temperature() is None  # nichts sieht nach CPU aus
    _hwmon(tmp_path, 1, "coretemp", Core_0=0, Core_1=-274000, Core_2=999000)
    with patch.object(hw, "SYS", tmp_path / "sys"):
        assert hw.cpu_temperature() is None  # nur Unsinnswerte


def test_wifi_rssi(fake):
    assert hw.wifi_rssi() == -58


def test_missing_proc_gives_none(tmp_path):
    with patch.object(hw, "SYS", tmp_path / "sys"), patch.object(hw, "PROC", tmp_path / "proc"):
        assert hw.ram_percent() is None
        assert hw.cpu_temperature() is None
        assert hw.wifi_rssi() is None
        assert hw.Hardware().cpu_percent() is None
        assert hw.Hardware().net_ifaces() == []


def test_meminfo_without_available(tmp_path):
    _write(tmp_path / "proc/meminfo", "MemTotal: 1000 kB\n")
    with patch.object(hw, "PROC", tmp_path / "proc"):
        assert hw.ram_percent() is None


def test_cpu_percent_needs_two_polls(fake):
    h = hw.Hardware()
    assert h.cpu_percent() is None
    _fake_root(fake, user=130, idle=870)  # 30 von 100 Ticks belegt
    assert h.cpu_percent() == 30.0


def test_net_ifaces_rates_and_order(fake):
    t = [10.0]
    h = hw.Hardware(clock=lambda: t[0])
    first = h.net_ifaces()
    assert all(n.rx_bps is None for n in first)
    _fake_root(fake, rx=3000)  # 2000 Bytes mehr
    t[0] = 12.0
    can, wlan, eth = h.net_ifaces()  # aktive zuerst (alphabetisch), lo fehlt
    assert (can.name, wlan.name, eth.name) == ("can0", "wlan0", "eth0")
    assert wlan.up and wlan.ip == "172.20.1.13" and wlan.rx_bps == 1000.0 and wlan.tx_bps == 0.0
    assert can.up and can.ip is None
    assert not eth.up


def test_ipv4_addresses_parses_ioctl():
    response = bytearray(24)
    response[20:24] = bytes([192, 168, 1, 10])
    with (
        patch.object(hw.socket, "if_nameindex", return_value=[(2, "eth0")]),
        patch.object(hw.fcntl, "ioctl", return_value=bytes(response)),
    ):
        assert hw.ipv4_addresses() == {"eth0": "192.168.1.10"}


def test_battery_from_power_supply(tmp_path):
    ps = tmp_path / "sys/class/power_supply"
    _write(ps / "AC/type", "Mains")
    for name, value in (("type", "Battery"), ("voltage_now", "23400000"), ("current_now", "-3200000"), ("temp", "315"), ("capacity", "72")):
        _write(ps / "BAT0" / name, value)
    with patch.object(hw, "SYS", tmp_path / "sys"):
        assert hw.battery() == hw.Battery(23.4, 72, 3.2, 31.5)
        assert hw.battery().power_w == 23.4 * 3.2


def test_battery_missing(tmp_path):
    with patch.object(hw, "SYS", tmp_path / "sys"):
        assert hw.battery() == hw.Battery()
        assert hw.battery().power_w is None
