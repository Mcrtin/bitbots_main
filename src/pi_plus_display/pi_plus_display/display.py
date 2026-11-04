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

from pi_plus_display.status_display import StatusDisplay


class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.get_logger().info("Activate sim time")
        self.declare_parameter("demo", True)
        self.set_parameters(
            [
                rclpy.parameter.Parameter(
                    "use_sim_time",
                    rclpy.Parameter.Type.BOOL,
                    True,
                ),
               rclpy.parameter.Parameter(
                    "demo",
                    rclpy.Parameter.Type.BOOL,
                    True,
                )
            ]
        )

        demo = self.get_parameter("demo").get_parameter_value().bool_value

        # Statusanzeige; die Callbacks unten reichen ihr jede Nachricht weiter
        self.status_display = StatusDisplay(self)
        display = self.status_display.source
        if demo:
            # Demo-Fenster mit Pfeiltasten als Joystick, faellt mit display_demo.py weg
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
        # Nur fuer die Statusanzeige, direkt an deren Callbacks
        self.create_subscription(GameState, "gamestate", display.on_gamestate, 1)
        self.create_subscription(RobotControlState, "robot_state", display.on_robot_state, 1)
        self.create_subscription(
            PoseWithCovarianceStamped, "ball_position_relative_filtered", display.on_ball, 1
        )
        self.create_subscription(Imu, "imu/data", display.on_imu, 1)
        self.create_subscription(Workload, "system_workload", display.on_workload, 1)
        self.create_subscription(TeamData, "team_data", display.on_team_data, 1)

        # Create timer
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("Node initialized successfully.")

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        self.pose = msg.pose
        self.status_display.source.on_pose(msg)

    def camera_info_callback(self, msg: CameraInfo):
        self.get_logger().info(f'{msg=}')
        self.camera_info = msg
        self.status_display.source.on_camera_info(msg)

    def strategy_cb(self, msg: Strategy):
        self.get_logger().info(f'{msg=}')
        self.strategy = msg
        self.status_display.source.on_strategy(msg)

    def joint_state_cb(self, msg: JointState):
        self.get_logger().info(f'{msg=}')
        self.joints = msg
        self.status_display.source.on_joint_states(msg)

    def voltage_cb(self, msg: Float32):
        self.get_logger().info(f'{msg=}')
        """Receives data sent by the battery management system."""
        self.battery_voltage = msg.data
        self.status_display.source.on_battery(msg)

    def diag_cb(self, msg: DiagnosticArray):
        """Updates the diagnostic state."""
        self.get_logger().info(f'{msg=}')
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
