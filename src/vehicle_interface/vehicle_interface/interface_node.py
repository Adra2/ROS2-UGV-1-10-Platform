import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16


class VehicleInterface(Node):

    def __init__(self):
        super().__init__('vehicle_interface')

        # ─────────────────────────────
        # PARAMETERS
        # ─────────────────────────────

        self.declare_parameter('steering_center_us', 1555)
        self.declare_parameter('steering_range_us', 400)

        self.declare_parameter('throttle_center_us', 1500)
        self.declare_parameter('throttle_range_us', 300)

        self.declare_parameter('deadman_timeout', 0.2)

        self.steer_center = self.get_parameter('steering_center_us').value
        self.steer_range = self.get_parameter('steering_range_us').value

        self.throttle_center = self.get_parameter('throttle_center_us').value
        self.throttle_range = self.get_parameter('throttle_range_us').value

        self.deadman = self.get_parameter('deadman_timeout').value

        # ─────────────────────────────
        # STATE
        # ─────────────────────────────

        self.last_steer_time = self.get_clock().now().nanoseconds
        self.last_throttle_time = self.get_clock().now().nanoseconds

        self.steer_val = 0.0
        self.throttle_val = 0.0

        # ─────────────────────────────
        # SUBSCRIBERS
        # ─────────────────────────────

        self.create_subscription(
            Float32,
            '/cmd_steering',
            self.steer_cb,
            10
        )

        self.create_subscription(
            Float32,
            '/cmd_throttle',
            self.throttle_cb,
            10
        )

        # ─────────────────────────────
        # PUBLISHERS
        # ─────────────────────────────

        self.steer_pub = self.create_publisher(
            Int16,
            '/cmd_steering_pwm',
            10
        )

        self.throttle_pub = self.create_publisher(
            Int16,
            '/cmd_throttle_pwm',
            10
        )

        # ─────────────────────────────
        # LOOP
        # ─────────────────────────────

        self.create_timer(0.02, self.update)  # 50 Hz

        self.get_logger().info("Vehicle Interface READY")

    # ─────────────────────────────
    # UTILS
    # ─────────────────────────────

    def to_pwm(self, x, center, range_):
        x = max(-1.0, min(1.0, x))
        return int(center + x * range_)

    # ─────────────────────────────
    # CALLBACKS
    # ─────────────────────────────

    def steer_cb(self, msg):
        self.steer_val = msg.data
        self.last_steer_time = self.get_clock().now().nanoseconds

    def throttle_cb(self, msg):
        self.throttle_val = msg.data
        self.last_throttle_time = self.get_clock().now().nanoseconds

    # ─────────────────────────────
    # MAIN LOOP
    # ─────────────────────────────

    def update(self):
        now = self.get_clock().now().nanoseconds

        steer_age = (now - self.last_steer_time) * 1e-9
        throttle_age = (now - self.last_throttle_time) * 1e-9

        steer_pwm = Int16()
        throttle_pwm = Int16()

        # ───── STEERING ─────
        if steer_age > self.deadman:
            steer_pwm.data = self.steer_center
        else:
            steer_pwm.data = self.to_pwm(
                self.steer_val,
                self.steer_center,
                self.steer_range
            )

        # ───── THROTTLE ─────
        if throttle_age > self.deadman:
            throttle_pwm.data = self.throttle_center
        else:
            throttle_pwm.data = self.to_pwm(
                self.throttle_val,
                self.throttle_center,
                self.throttle_range
            )

        # ───── PUBLISH ─────
        self.steer_pub.publish(steer_pwm)
        self.throttle_pub.publish(throttle_pwm)


def main():
    rclpy.init()
    node = VehicleInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()