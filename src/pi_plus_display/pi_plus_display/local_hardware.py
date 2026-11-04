# Hardware des Rechners, auf dem die Anzeige laeuft, ueber hardware_interfaces (/sys, /proc).
# poll() blockiert nicht: CPU-Last und Netzraten kommen aus der Differenz zum vorigen Aufruf, nicht aus sleep.
# Ohne /proc (z. B. auf dem Mac) bleiben die Werte None.

import shutil
import time
from dataclasses import dataclass, field

from . import hardware_interfaces as hw
from .state import NetIface, TempSensor

GB = 1024**3


@dataclass
class HardwareSample:
    cpu_percent: float | None = None
    cpu_mhz: float | None = None
    load_avg: tuple | None = None
    uptime_s: float | None = None
    ram_percent: float | None = None
    ram_used_gb: float | None = None
    ram_total_gb: float | None = None
    disk_percent: float | None = None
    temps: list = field(default_factory=list)  # TempSensor, heissester zuerst
    net: list = field(default_factory=list)  # NetIface ohne lo, aktive zuerst


class LocalHardware:
    # clock: Zeitquelle fuer die Netzraten; disk_path: Dateisystem fuer die Belegung
    def __init__(self, clock=time.monotonic, disk_path="/"):
        self.clock = clock
        self.disk_path = disk_path
        self._cpu = None  # letzte Zaehler aus /proc/stat
        self._net = {}  # Interface -> (Zeit, rx_bytes, tx_bytes)

    def poll(self):
        s = HardwareSample()
        s.cpu_percent = self._cpu_percent()
        freqs = hw.cpu_frequencies()
        s.cpu_mhz = max(freqs.values()) if freqs else None
        diag = hw.proc_diagnostics()
        s.uptime_s = diag["uptime_seconds"]
        s.load_avg = _load_avg(diag["loadavg"])
        mem = hw.memory()
        if mem and mem["used_bytes"] is not None:
            s.ram_percent = mem["used_percent"]
            s.ram_used_gb = mem["used_bytes"] / GB
            s.ram_total_gb = mem["total_bytes"] / GB
        try:
            disk = shutil.disk_usage(self.disk_path)
            s.disk_percent = 100 * disk.used / disk.total if disk.total else None
        except OSError:
            pass
        s.temps = _temp_sensors(hw.temperatures())
        s.net = self._net_ifaces()
        return s

    # Wie hardware_interfaces.cpu_usage, nur mit dem vorigen Aufruf als erstem Messpunkt
    def _cpu_percent(self):
        now = hw.cpu_counters()
        prev, self._cpu = self._cpu, now
        if prev is None or now is None:
            return None
        delta = [b - a for a, b in zip(prev, now, strict=True)]
        total = sum(delta)
        idle = delta[3] + delta[4]  # idle + iowait
        return 100 * (total - idle) / total if total > 0 else None

    def _net_ifaces(self):
        try:
            ifaces = hw.network_interfaces()
        except OSError:
            return []
        ips = hw.ipv4_addresses()
        now = self.clock()
        result = []
        for name, info in ifaces.items():
            if name == "lo" or name.startswith("veth"):
                continue
            rx, tx = info["rx_bytes"], info["tx_bytes"]
            rx_bps = tx_bps = None
            prev = self._net.get(name)
            if prev and rx is not None and tx is not None and now > prev[0]:
                dt = now - prev[0]
                rx_bps = max(0.0, (rx - prev[1]) / dt) if prev[1] is not None else None
                tx_bps = max(0.0, (tx - prev[2]) / dt) if prev[2] is not None else None
            self._net[name] = (now, rx, tx)
            # "unknown" melden Interfaces ohne Link-Erkennung, die trotzdem laufen
            up = info["state"] in ("up", "unknown")
            result.append(NetIface(name, up, ips.get(name), rx_bps, tx_bps))
        return sorted(result, key=lambda n: (not n.up, n.name))


# "0.06 0.01 0.00 1/115 591" -> (0.06, 0.01, 0.0)
def _load_avg(text):
    try:
        return tuple(float(v) for v in text.split()[:3])
    except (AttributeError, ValueError):
        return None


# Kurzer Name je Fuehler; Thermal-Zonen und hwmon melden oft denselben Fuehler, daher je Name nur der hoechste Wert
def _temp_sensors(temps):
    best = {}
    for info in temps.values():
        raw = info.get("label") or info.get("type") or info.get("chip") or "?"
        name = raw.replace("-thermal", "").replace("_thermal", "").replace("_", "-").upper()[:10]
        best[name] = max(best.get(name, info["celsius"]), info["celsius"])
    return sorted((TempSensor(n, c) for n, c in best.items()), key=lambda t: -t.celsius)
