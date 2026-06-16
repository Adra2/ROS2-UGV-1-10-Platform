import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8

MODE_RC = 0
MODE_TELEOP = 1
MODE_AUTO = 2
MODE_STOP = 3


class ControlMuxNode(Node):

    def __init__(self):
        super().__init__('control_mux_node')

        self.declare_parameter('watchdog_timeout', 0.3)
        self.timeout = self.get_parameter('watchdog_timeout').value

        # State
        self.mode = MODE_STOP

        self.rc_steering = 0.0
        self.rc_throttle = 0.0

        self.teleop_steering = 0.0
        self.teleop_throttle = 0.0

        self.auto_steering = 0.0
        self.auto_throttle = 0.0

        self.last_rc_time = self.get_clock().now().nanoseconds

        # SUBS
        self.create_subscription(Int8, '/control_mode', self.mode_cb, 10)

        self.create_subscription(Float32, '/rc/steering', self.rc_steer_cb, 10)
        self.create_subscription(Float32, '/rc/throttle', self.rc_throttle_cb, 10)

        self.create_subscription(Float32, '/teleop/steering', self.tel_steer_cb, 10)
        self.create_subscription(Float32, '/teleop/throttle', self.tel_throttle_cb, 10)

        self.create_subscription(Float32, '/auto/steering', self.auto_steer_cb, 10)
        self.create_subscription(Float32, '/auto/throttle', self.auto_throttle_cb, 10)

        # PUB
        self.pub_steer = self.create_publisher(Float32, '/cmd_steering', 10)
        self.pub_throttle = self.create_publisher(Float32, '/cmd_throttle', 10)

        self.create_timer(0.02, self.update)  # 50 Hz

        self.get_logger().info("Control Mux READY")

    # CALLBACKS

    def mode_cb(self, msg):
        self.mode = msg.data

    def rc_steer_cb(self, msg):
        self.rc_steering = msg.data
        self.last_rc_time = self.get_clock().now().nanoseconds

    def rc_throttle_cb(self, msg):
        self.rc_throttle = msg.data
        self.last_rc_time = self.get_clock().now().nanoseconds

    def tel_steer_cb(self, msg):
        self.teleop_steering = msg.data

    def tel_throttle_cb(self, msg):
        self.teleop_throttle = msg.data

    def auto_steer_cb(self, msg):
        self.auto_steering = msg.data

    def auto_throttle_cb(self, msg):
        self.auto_throttle = msg.data

    # MAIN LOOP

    def update(self):
        now = self.get_clock().now().nanoseconds
        rc_age = (now - self.last_rc_time) * 1e-9

        steer = 0.0
        throttle = 0.0

        # STOP always wins
        if self.mode == MODE_STOP:
            steer, throttle = 0.0, 0.0

        elif self.mode == MODE_RC:
            steer = self.rc_steering
            throttle = self.rc_throttle

        elif self.mode == MODE_TELEOP:
            steer = self.teleop_steering
            throttle = self.teleop_throttle

        elif self.mode == MODE_AUTO:
            steer = self.auto_steering
            throttle = self.auto_throttle

        # WATCHDOG (only for RC safety fallback)
        if self.mode == MODE_RC and rc_age > self.timeout:
            steer, throttle = 0.0, 0.0
            self.get_logger().warn("RC lost → STOP")

        self.pub_steer.publish(Float32(data=steer))
        self.pub_throttle.publish(Float32(data=throttle))


def main():
    rclpy.init()
    node = ControlMuxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()