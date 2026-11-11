import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
import soccer_vision_3d_msgs.msg as sv3dm
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import JointState
from bitbots_msgs.msg import Strategy, TeamData
from sensor_msgs.msg import CameraInfo
from bitbots_msgs.msg import RobotControlState, Workload
from game_controller_hsl_interfaces.msg import GameState
from sensor_msgs.msg import Imu
from std_msgs.msg import Int8

from pi_plus_display.status_display import StatusDisplay
import pi_plus_display.hardware_interfaces as hwif


class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.demo = self.declare_parameter("demo", True).value
        self.serial_port = self.declare_parameter("serial_port", "/dev/ttyS4").value
        self.baudrate = self.declare_parameter("baudrate", 115200).value
        self.frame_rate = self.declare_parameter("frame_rate", 10.0).value
        self.set_parameters(
            [
                rclpy.parameter.Parameter(
                    "use_sim_time",
                    rclpy.Parameter.Type.BOOL,
                    False,
                ),
            ]
        )

        self.status_display = StatusDisplay(self)
        display = self.status_display.source
        if self.demo:
            from pi_plus_display.display_demo import start_window

            start_window(self, self.status_display)

        self.pose = None
        self.camera_info = None
        self.strategy = None
        self.joints = None
        self.battery_voltage = None
        self.servo_overload = False
        self.servo_overheat = False
        self.servo_diag_error = None
        self.imu_diag_error = None

        self.cpu_usage = None
        self.cpu_frequencies = None
        self.memory = None
        self.temperatures = None
        self.block_devices = None
        self.network_interfaces = None
        self.ipv4_addresses = None
        self.pci_gpus = None
        self.power_supplies = None
        self.iio_devices = None
        self.hwmon_devices = None
        self.proc_diagnostics = None

        self.create_subscription(Float32, "battery_voltage", self.voltage_cb, 1)
        self.create_subscription(DiagnosticArray, "diagnostics_agg", self.diag_cb, 1)
        self.create_subscription(JointState, "joint_states", self.joint_state_cb, 1)
        self.create_subscription(Strategy, "strategy", self.strategy_cb, 1)
        self.create_subscription(
            PoseWithCovarianceStamped, "pose_with_covariance", self.pose_cb, 1
        )
        self.create_subscription(
            CameraInfo, "/zed/zed_node/rgb/camera_info", self.camera_info_callback, 1
        )

        # Create timer
        self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("Node initialized successfully.")

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        self.pose = msg.pose
        self.status_display.source.on_pose(msg)

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg
        self.status_display.source.on_camera_info(msg)

    def strategy_cb(self, msg: Strategy):
        self.strategy = msg
        self.status_display.source.on_strategy(msg)

    def joint_state_cb(self, msg: JointState):
        self.joints = msg
        self.status_display.source.on_joint_states(msg)

    def voltage_cb(self, msg: Float32):
        """Receives data sent by the battery management system."""
        self.battery_voltage = msg.data
        self.status_display.source.on_battery(msg)

    def diag_cb(self, msg: DiagnosticArray):
        """Updates the diagnostic state."""
        self.status_display.source.on_diagnostics(msg)
        status: DiagnosticStatus
        for status in msg.status:
            if "/Servos/" in status.name:
                if status.level == DiagnosticStatus.ERROR:
                    if "Overload" in status.message:
                        self.servo_overload = True
                    elif "Overheat" in status.message:
                        self.servo_overheat = True

            elif "/Servos" in status.name:
                self.servo_diag_error = status.level in (
                    DiagnosticStatus.ERROR,
                    DiagnosticStatus.STALE,
                )
            elif "/IMU" in status.name:
                self.imu_diag_error = status.level in (
                    DiagnosticStatus.ERROR,
                    DiagnosticStatus.STALE,
                )

    def timer_callback(self):
        self.cpu_usage = hwif.cpu_usage()
        self.cpu_frequencies = hwif.cpu_frequencies()
        self.memory = hwif.memory()
        self.temperatures = hwif.temperatures()
        self.block_devices = hwif.block_devices()
        self.network_interfaces = hwif.network_interfaces()
        self.ipv4_addresses = hwif.ipv4_addresses()
        self.pci_gpus = hwif.pci_gpus()
        self.power_supplies = hwif.power_supplies()
        self.iio_devices = hwif.iio_devices()
        self.hwmon_devices = hwif.hwmon_devices()
        self.proc_diagnostics = hwif.proc_diagnostics()


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
