import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
import soccer_vision_3d_msgs.msg as sv3dm
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import JointState
from bitbots_msgs.msg import Strategy, TeamData
from sensor_msgs.msg import CameraInfo


class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.get_logger().info("Activate sim time")
        self.set_parameters(
            [
                rclpy.parameter.Parameter(
                    "use_sim_time",
                    rclpy.Parameter.Type.BOOL,
                    True,
                )
            ]
        )

        self.pose = None
        self.camera_info = None
        self.strategy = None
        self.joints = None
        self.battery_voltage = None
        self.servo_overload = False
        self.servo_overheat = False
        self.servo_diag_error = None
        self.imu_diag_error = None


        self.create_subscription(Float32, "battery_voltage", self.voltage_cb, 1)
        self.create_subscription(DiagnosticArray, "diagnostics_agg", self.diag_cb, 1)
        self.create_subscription(JointState, "joint_states", self.joint_state_cb, 1)
        self.create_subscription(Strategy, "strategy", self.strategy_cb, 1)
        self.create_subscription(PoseWithCovarianceStamped, "pose_with_covariance", self.pose_cb, 1)
        self.create_subscription(CameraInfo, "/zed/zed_node/rgb/camera_info", self.camera_info_callback, 1)

        # Create timer
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("Node initialized successfully.")

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        self.pose = msg.pose

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def strategy_cb(self, msg: Strategy):
        self.strategy = msg

    def joint_state_cb(self, msg: JointState):
        self.joints = msg;

    def voltage_cb(self, msg: Float32):
        """Receives data sent by the battery management system."""
        self.battery_voltage = msg.data

    def diag_cb(self, msg: DiagnosticArray):
        """Updates the diagnostic state."""
        status: DiagnosticStatus
        for status in msg.status:
            if "/Servos/" in status.name:
                if status.level == DiagnosticStatus.ERROR:
                    if "Overload" in status.message:
                        self.servo_overload = True
                    elif "Overheat" in status.message:
                        self.servo_overheat = True

            elif "/Servos" in status.name:
                self.servo_diag_error = status.level in (DiagnosticStatus.ERROR, DiagnosticStatus.STALE)
            elif "/IMU" in status.name:
                self.imu_diag_error = status.level in (DiagnosticStatus.ERROR, DiagnosticStatus.STALE)

    def timer_callback(self):
        # This method will be called every 0.5 seconds
        self.get_logger().info("Timer callback executed.")


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
