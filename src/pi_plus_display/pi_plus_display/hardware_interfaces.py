from pathlib import Path
import time
import fcntl
import socket
import struct

SYS = Path("/sys")
PROC = Path("/proc")


def read_text(path):
    try:
        return path.read_text().strip()
    except OSError:
        return None


def read_int(path):
    try:
        return int(path.read_text().strip())
    except (OSError, ValueError):
        return None


def cpu_counters():
    """Read aggregate CPU counters from /proc/stat."""
    try:
        fields = (PROC / "stat").read_text().splitlines()[0].split()
        values = list(map(int, fields[1:]))
        # user, nice, system, idle, iowait, irq, softirq, steal
        values += [0] * (8 - len(values))
        return values[:8]
    except (OSError, ValueError, IndexError):
        return None


def cpu_usage(interval=0.5):
    first = cpu_counters()
    if first is None:
        return None

    time.sleep(interval)
    second = cpu_counters()
    if second is None:
        return None

    delta = [b - a for a, b in zip(first, second)]
    total = sum(delta)
    idle = delta[3] + delta[4]  # idle + iowait
    return round(100 * (total - idle) / total, 1) if total else 0.0


def memory():
    values = {}
    try:
        for line in (PROC / "meminfo").read_text().splitlines():
            key, value = line.split(":", 1)
            parts = value.split()
            if parts and parts[0].isdigit():
                values[key] = int(parts[0]) * 1024  # bytes
    except OSError:
        return None

    total = values.get("MemTotal")
    available = values.get("MemAvailable")
    if total is None:
        return None

    used = total - available if available is not None else None
    return {
        "total_bytes": total,
        "available_bytes": available,
        "used_bytes": used,
        "used_percent": round(100 * used / total, 1) if used is not None else None,
    }


def temperatures():
    result = {}

    # Thermal zones
    for zone in sorted((SYS / "class/thermal").glob("thermal_zone*")):
        temp = read_int(zone / "temp")
        if temp is not None:
            result[zone.name] = {
                "type": read_text(zone / "type"),
                "celsius": temp / 1000,
            }

    # Hardware-monitor sensors
    for hwmon in sorted((SYS / "class/hwmon").glob("hwmon*")):
        chip = read_text(hwmon / "name")
        for path in sorted(hwmon.glob("temp*_input")):
            temp = read_int(path)
            if temp is not None:
                label_path = path.with_name(path.name.replace("_input", "_label"))
                result[f"{hwmon.name}/{path.stem}"] = {
                    "chip": chip,
                    "label": read_text(label_path),
                    "celsius": temp / 1000,
                }

    return result


def block_devices():
    result = {}
    for dev in sorted((SYS / "class/block").iterdir()):
        if (dev / "partition").exists():
            continue

        sectors = read_int(dev / "size")
        sector_size = read_int(dev / "queue/logical_block_size")
        result[dev.name] = {
            "size_bytes": sectors * 512 if sectors is not None else None,
            "logical_sector_bytes": sector_size,
            "model": read_text(dev / "device/model"),
        }
    return result


def network_interfaces():
    result = {}
    for iface in sorted((SYS / "class/net").iterdir()):
        result[iface.name] = {
            "state": read_text(iface / "operstate"),
            "mac": read_text(iface / "address"),
            "rx_bytes": read_int(iface / "statistics/rx_bytes"),
            "tx_bytes": read_int(iface / "statistics/tx_bytes"),
        }
    return result


def pci_gpus():
    result = {}
    for dev in sorted((SYS / "bus/pci/devices").iterdir()):
        device_class = read_text(dev / "class")
        if device_class and device_class.startswith("0x03"):
            result[dev.name] = {
                "vendor_id": read_text(dev / "vendor"),
                "device_id": read_text(dev / "device"),
                "class": device_class,
            }
    return result




def ipv4_addresses():
    """Return IPv4 addresses for interfaces, where available."""
    result = {}

    try:
        interfaces = socket.if_nameindex()
    except OSError:
        return result

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for _, name in interfaces:
            try:
                request = struct.pack("256s", name.encode()[:15])
                response = fcntl.ioctl(sock.fileno(), 0x8915, request)
                result[name] = socket.inet_ntoa(response[20:24])
            except OSError:
                # Interface may have no IPv4 address or may disappear during the scan.
                continue

    return result


def cpu_frequencies():
    """Return available current CPU frequencies in MHz."""
    result = {}

    for cpu in sorted(SYS.glob("devices/system/cpu/cpu[0-9]*")):
        cpu_name = cpu.name

        # Prefer the current scaling frequency; fall back to cpuinfo_cur_freq.
        paths = (
            cpu / "cpufreq/scaling_cur_freq",
            cpu / "cpufreq/cpuinfo_cur_freq",
        )

        for path in paths:
            value_khz = read_int(path)
            if value_khz is not None:
                result[cpu_name] = value_khz / 1000
                break

    return result


def read_sysfs_attributes(directory):
    """Read simple file attributes directly inside a sysfs directory."""
    attributes = {}
    try:
        entries = directory.iterdir()
    except OSError:
        return attributes

    for path in entries:
        if path.is_file():
            value = read_text(path)
            if value is not None:
                attributes[path.name] = value

    return attributes


def power_supplies():
    result = {}
    base = SYS / "class/power_supply"

    try:
        supplies = sorted(base.iterdir())
    except OSError:
        return result

    for supply in supplies:
        result[supply.name] = read_sysfs_attributes(supply)

    return result


def iio_devices():
    result = {}
    base = SYS / "bus/iio/devices"

    try:
        devices = sorted(base.glob("iio:device*"))
    except OSError:
        return result

    for device in devices:
        attributes = read_sysfs_attributes(device)

        # Include readable attributes from scan-element subdirectory too.
        scan_elements = device / "scan_elements"
        try:
            attributes["scan_elements"] = read_sysfs_attributes(scan_elements)
        except OSError:
            pass

        result[device.name] = attributes

    return result


def hwmon_devices():
    result = {}
    base = SYS / "class/hwmon"

    try:
        devices = sorted(base.glob("hwmon*"))
    except OSError:
        return result

    for device in devices:
        result[device.name] = {
            "name": read_text(device / "name"),
            "attributes": read_sysfs_attributes(device),
        }

    return result


def proc_diagnostics():
    result = {
        "uptime_seconds": None,
        "loadavg": None,
        "pressure": {},
    }

    uptime_text = read_text(PROC / "uptime")
    if uptime_text:
        try:
            result["uptime_seconds"] = float(uptime_text.split()[0])
        except (ValueError, IndexError):
            pass

    result["loadavg"] = read_text(PROC / "loadavg")

    pressure_dir = PROC / "pressure"
    try:
        for path in sorted(pressure_dir.iterdir()):
            if path.is_file():
                value = read_text(path)
                if value is not None:
                    result["pressure"][path.name] = value
    except OSError:
        pass

    return result

if __name__ == "__main__":
    print("CPU usage:", cpu_usage(), "%")
    print("CPU frequencies (MHz):", cpu_frequencies())
    print("RAM:", memory())
    print("Temperatures:", temperatures())
    print("Block devices:", block_devices())
    print("Network:", network_interfaces())
    print("IPv4 addresses:", ipv4_addresses())
    print("PCI display devices:", pci_gpus())
    print("Power supplies:", power_supplies())
    print("IIO devices:", iio_devices())
    print("Hardware monitors:", hwmon_devices())
    print("Proc diagnostics:", proc_diagnostics())

