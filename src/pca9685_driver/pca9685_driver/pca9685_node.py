import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from .driver import PCA9685Driver


class PCA9685Node(Node):

    def __init__(self):
        super().__init__('pca9685_node')

        # PARAMETERS
        self.declare_parameter('frequency', 50)
        self.declare_parameter('steering_channel', 0)
        self.declare_parameter('throttle_channel', 1)

        self.frequency = self.get_parameter('frequency').value
        self.steering_channel = self.get_parameter('steering_channel').value
        self.throttle_channel = self.get_parameter('throttle_channel').value

        # DRIVER
        self.driver = PCA9685Driver(self.frequency)

        # SUBS
        self.create_subscription(
            Int16,
            '/cmd_steering_pwm',
            self.steer_cb,
            10
        )

        self.create_subscription(
            Int16,
            '/cmd_throttle_pwm',
            self.throttle_cb,
            10
        )

        self.get_logger().info("PCA9685 node ready")

    def steer_cb(self, msg):
        self.driver.set_pwm_us(self.steering_channel, msg.data)

    def throttle_cb(self, msg):
        self.driver.set_pwm_us(self.throttle_channel, msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = PCA9685Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()