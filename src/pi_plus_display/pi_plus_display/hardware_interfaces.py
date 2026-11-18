# Values of the computer the display runs on, from /proc and /sys.
# Nothing blocks: CPU load and network rates come from the difference to the previous call.
# Without /proc (e.g. on a Mac) the values stay None or empty.

import fcntl
import socket
import struct
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

SYS = Path("/sys")
PROC = Path("/proc")


# Network interface with data rate
@dataclass
class NetIface:
    name: str
    up: bool = True
    ip: str | None = None  # None = no IPv4 address (e.g. can0)
    rx_bps: float | None = None  # bytes per second; None = only one sample so far
    tx_bps: float | None = None


# Battery from sysfs; None = the battery does not report the value
@dataclass
class Battery:
    voltage_v: float | None = None
    percent: int | None = None
    current_a: float | None = None  # magnitude, direction does not matter
    temp_c: float | None = None

    @property
    def power_w(self) -> float | None:
        return None if self.voltage_v is None or self.current_a is None else self.voltage_v * self.current_a


def _read_int(path: Path) -> int | None:
    try:
        return int(path.read_text().strip())
    except (OSError, ValueError):
        return None


def ram_percent() -> float | None:
    try:
        values: dict[str, int] = {}
        for line in (PROC / "meminfo").read_text().splitlines():
            key, value = line.split(":", 1)
            values[key] = int(value.split()[0])
        return 100 * (values["MemTotal"] - values["MemAvailable"]) / values["MemTotal"]
    except (OSError, ValueError, IndexError, KeyError, ZeroDivisionError):
        return None


def _read_text(path: Path) -> str:
    try:
        return path.read_text().strip()
    except OSError:
        return ""


# All temperature sensors as (name, degrees Celsius): thermal zones under their type, hwmon chips with chip name and
# label of the measuring point. Many PCs have the CPU only under hwmon (AMD: k10temp), while the thermal zones
# only contain ACPI sensors that barely measure the CPU.
def _temperature_sensors() -> list[tuple[str, float]]:
    sensors: list[tuple[str, float]] = []
    for zone in sorted((SYS / "class/thermal").glob("thermal_zone*")):
        if (milli := _read_int(zone / "temp")) is not None:
            sensors.append((_read_text(zone / "type"), milli / 1000))
    for chip in sorted((SYS / "class/hwmon").glob("hwmon*")):
        for temp_input in sorted(chip.glob("temp*_input")):
            if (milli := _read_int(temp_input)) is not None:
                label = _read_text(temp_input.with_name(temp_input.name.replace("_input", "_label")))
                sensors.append((f"{_read_text(chip / 'name')} {label}", milli / 1000))
    return sensors


# Parts of sensor names (lowercase) that belong to the CPU: Intel (coretemp, x86_pkg_temp), AMD (k10temp,
# zenpower, Tctl/Tdie), ARM boards (cpu-thermal, soc-thermal, ...)
CPU_SENSORS = ("coretemp", "x86_pkg_temp", "k10temp", "zenpower", "tctl", "tdie", "package", "cpu", "soc")


# Hottest CPU temperature in degrees Celsius; None if no sensor looks like a CPU (better to show nothing than
# the sensor of a graphics card or SSD). Nonsensical values (<= 0 or >= 150 degrees) do not count.
def cpu_temperature() -> float | None:
    temps = [t for name, t in _temperature_sensors() if any(k in name.lower() for k in CPU_SENSORS) and 0 < t < 150]
    return max(temps) if temps else None


# Wi-Fi signal in dBm from /proc/net/wireless
def wifi_rssi() -> int | None:
    try:
        for line in (PROC / "net/wireless").read_text().splitlines()[2:]:
            return int(float(line.split()[3].rstrip(".")))
    except (OSError, ValueError, IndexError):
        pass
    return None


# First battery under /sys/class/power_supply; without a battery all values are None
def battery() -> Battery:
    for supply in sorted((SYS / "class/power_supply").glob("*")):
        try:
            if (supply / "type").read_text().strip() != "Battery":
                continue
        except OSError:
            continue
        volt, amp, temp = (_read_int(supply / n) for n in ("voltage_now", "current_now", "temp"))
        return Battery(
            voltage_v=None if volt is None else volt / 1e6,  # sysfs: microvolts
            percent=_read_int(supply / "capacity"),
            current_a=None if amp is None else abs(amp) / 1e6,  # microamperes
            temp_c=None if temp is None else temp / 10,  # tenths of a degree
        )
    return Battery()


# IPv4 address per interface, if any
def ipv4_addresses() -> dict[str, str]:
    result: dict[str, str] = {}
    try:
        names = [name for _, name in socket.if_nameindex()]
    except OSError:
        return result
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for name in names:
            try:
                response = fcntl.ioctl(sock.fileno(), 0x8915, struct.pack("256s", name.encode()[:15]))
                result[name] = socket.inet_ntoa(response[20:24])
            except OSError:
                continue  # no IPv4 address or interface disappeared
    return result


class Hardware:
    # clock: time source in seconds for the network rates
    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._cpu: list[int] | None = None  # last counters from /proc/stat
        self._net: dict[str, tuple[float, int, int]] = {}  # interface -> (time, rx_bytes, tx_bytes)

    # CPU load since the previous call in percent; None on the first call
    def cpu_percent(self) -> float | None:
        try:
            fields = (PROC / "stat").read_text().splitlines()[0].split()
            now = [int(v) for v in fields[1:9]]  # user nice system idle iowait irq softirq steal
        except (OSError, ValueError, IndexError):
            return None
        prev, self._cpu = self._cpu, now
        if prev is None or len(prev) != len(now):
            return None
        delta = [b - a for a, b in zip(prev, now, strict=True)]
        total = sum(delta)
        idle = delta[3] + delta[4]
        return 100 * (total - idle) / total if total > 0 else None

    # Interfaces without lo and veth, active ones first
    def net_ifaces(self) -> list[NetIface]:
        try:
            names = sorted(p.name for p in (SYS / "class/net").iterdir())
        except OSError:
            return []
        ips = ipv4_addresses()
        now = self._clock()
        result = []
        for name in names:
            if name == "lo" or name.startswith("veth"):
                continue
            base = SYS / "class/net" / name
            rx, tx = _read_int(base / "statistics/rx_bytes"), _read_int(base / "statistics/tx_bytes")
            rx_bps = tx_bps = None
            if rx is not None and tx is not None:
                prev = self._net.get(name)
                if prev and now > prev[0]:
                    dt = now - prev[0]
                    rx_bps, tx_bps = max(0.0, (rx - prev[1]) / dt), max(0.0, (tx - prev[2]) / dt)
                self._net[name] = (now, rx, tx)
            try:
                # "unknown" is reported by interfaces without link detection that are running anyway
                up = (base / "operstate").read_text().strip() in ("up", "unknown")
            except OSError:
                up = False
            result.append(NetIface(name, up, ips.get(name), rx_bps, tx_bps))
        return sorted(result, key=lambda n: (not n.up, n.name))
