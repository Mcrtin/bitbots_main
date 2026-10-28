import rclpy
from rclpy.node import Node


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

        # Create timer
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("Node initialized successfully.")

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
