import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from pi_plus_display import hardware_interfaces


class DiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.root = Path(self.temp_dir.name)
        self.sys = self.root / "sys"
        self.proc = self.root / "proc"
        self.sys.mkdir()
        self.proc.mkdir()

        # Redirect the module's filesystem paths to the temporary test tree.
        self.sys_patcher = patch.object(hardware_interfaces, "SYS", self.sys)
        self.proc_patcher = patch.object(hardware_interfaces, "PROC", self.proc)
        self.sys_patcher.start()
        self.proc_patcher.start()
        self.addCleanup(self.sys_patcher.stop)
        self.addCleanup(self.proc_patcher.stop)
        self.addCleanup(self.temp_dir.cleanup)

    def write(self, path, content):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)

    def test_memory(self):
        self.write(
            self.proc / "meminfo",
            "MemTotal: 1000 kB\nMemAvailable: 400 kB\n",
        )

        result = hardware_interfaces.memory()

        self.assertEqual(result["total_bytes"], 1_024_000)
        self.assertEqual(result["available_bytes"], 409_600)
        self.assertEqual(result["used_bytes"], 614_400)
        self.assertEqual(result["used_percent"], 60.0)

    def test_cpu_usage(self):
        counters = iter(
            [
                [100, 0, 50, 800, 50, 0, 0, 0],
                [120, 0, 60, 900, 60, 0, 0, 0],
            ]
        )

        with (
            patch.object(
                hardware_interfaces, "cpu_counters", side_effect=lambda: next(counters)
            ),
            patch.object(hardware_interfaces.time, "sleep"),
        ):
            result = hardware_interfaces.cpu_usage(interval=0)

        # 30 busy ticks out of 140 total ticks.
        self.assertEqual(result, 21.4)

    def test_temperatures(self):
        self.write(self.sys / "class/thermal/thermal_zone0/type", "cpu")
        self.write(self.sys / "class/thermal/thermal_zone0/temp", "42000")

        result = hardware_interfaces.temperatures()

        self.assertEqual(result["thermal_zone0"]["type"], "cpu")
        self.assertEqual(result["thermal_zone0"]["celsius"], 42.0)

    def test_block_device(self):
        self.write(self.sys / "class/block/sda/size", "1000")
        self.write(self.sys / "class/block/sda/queue/logical_block_size", "512")
        self.write(self.sys / "class/block/sda/device/model", "TestDisk")

        result = hardware_interfaces.block_devices()

        self.assertEqual(result["sda"]["size_bytes"], 512_000)
        self.assertEqual(result["sda"]["model"], "TestDisk")

    def test_network_interface(self):
        base = self.sys / "class/net/eth0"
        self.write(base / "operstate", "up")
        self.write(base / "address", "00:11:22:33:44:55")
        self.write(base / "statistics/rx_bytes", "1234")
        self.write(base / "statistics/tx_bytes", "5678")

        result = hardware_interfaces.network_interfaces()

        self.assertEqual(result["eth0"]["state"], "up")
        self.assertEqual(result["eth0"]["rx_bytes"], 1234)
        self.assertEqual(result["eth0"]["tx_bytes"], 5678)

    def test_pci_gpu(self):
        base = self.sys / "bus/pci/devices/0000:01:00.0"
        self.write(base / "class", "0x030000")
        self.write(base / "vendor", "0x1234")
        self.write(base / "device", "0xabcd")

        result = hardware_interfaces.pci_gpus()

        self.assertEqual(result["0000:01:00.0"]["vendor_id"], "0x1234")
        self.assertEqual(result["0000:01:00.0"]["device_id"], "0xabcd")

    def test_cpu_frequencies(self):
        self.write(
            self.sys / "devices/system/cpu/cpu0/cpufreq/scaling_cur_freq",
            "1500000",
        )

        result = hardware_interfaces.cpu_frequencies()

        self.assertEqual(result["cpu0"], 1500.0)

    def test_ipv4_addresses(self):
        # ioctl returns the IPv4 address in bytes 20–23.
        response = bytearray(24)
        response[20:24] = bytes([192, 168, 1, 10])

        mock_socket = MagicMock()
        mock_socket.__enter__.return_value = mock_socket
        mock_socket.fileno.return_value = 7

        with (
            patch.object(
                hardware_interfaces.socket, "if_nameindex", return_value=[(2, "eth0")]
            ),
            patch.object(
                hardware_interfaces.socket, "socket", return_value=mock_socket
            ),
            patch.object(
                hardware_interfaces.fcntl, "ioctl", return_value=bytes(response)
            ),
        ):
            result = hardware_interfaces.ipv4_addresses()

        self.assertEqual(result["eth0"], "192.168.1.10")

    def test_power_supplies(self):
        base = self.sys / "class/power_supply/BAT0"
        self.write(base / "type", "Battery")
        self.write(base / "capacity", "85")
        self.write(base / "status", "Discharging")

        result = hardware_interfaces.power_supplies()

        self.assertEqual(result["BAT0"]["type"], "Battery")
        self.assertEqual(result["BAT0"]["capacity"], "85")
        self.assertEqual(result["BAT0"]["status"], "Discharging")

    def test_iio_devices(self):
        base = self.sys / "bus/iio/devices/iio:device0"
        self.write(base / "name", "test_imu")
        self.write(base / "in_accel_x_raw", "123")
        self.write(base / "scan_elements/in_accel_x_en", "1")

        result = hardware_interfaces.iio_devices()

        self.assertEqual(result["iio:device0"]["name"], "test_imu")
        self.assertEqual(result["iio:device0"]["in_accel_x_raw"], "123")
        self.assertEqual(result["iio:device0"]["scan_elements"]["in_accel_x_en"], "1")

    def test_hwmon_devices(self):
        base = self.sys / "class/hwmon/hwmon0"
        self.write(base / "name", "test_sensor")
        self.write(base / "temp1_input", "42000")
        self.write(base / "temp1_label", "CPU")

        result = hardware_interfaces.hwmon_devices()

        self.assertEqual(result["hwmon0"]["name"], "test_sensor")
        self.assertEqual(result["hwmon0"]["attributes"]["temp1_input"], "42000")
        self.assertEqual(result["hwmon0"]["attributes"]["temp1_label"], "CPU")

    def test_proc_diagnostics(self):
        self.write(self.proc / "uptime", "123.45 67.89\n")
        self.write(self.proc / "loadavg", "0.10 0.20 0.30 1/100 1234\n")
        self.write(
            self.proc / "pressure/cpu",
            "some avg10=0.00 avg60=0.00 avg300=0.00 total=0\n",
        )

        result = hardware_interfaces.proc_diagnostics()

        self.assertEqual(result["uptime_seconds"], 123.45)
        self.assertEqual(result["loadavg"], "0.10 0.20 0.30 1/100 1234")
        self.assertIn("cpu", result["pressure"])
        self.assertIn("avg10=0.00", result["pressure"]["cpu"])


if __name__ == "__main__":
    unittest.main()
