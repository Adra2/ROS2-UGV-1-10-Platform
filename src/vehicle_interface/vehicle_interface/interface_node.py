import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16
import time

class VehicleInterface(Node):

    def __init__(self):
        super().__init__('vehicle_interface')

        # params
        self.declare_parameter('pwm_center', 1500)
        self.declare_parameter('pwm_range', 500)
        self.declare_parameter('deadman_timeout', 0.2)

        self.center = self.get_parameter('pwm_center').value
        self.range = self.get_parameter('pwm_range').value
        self.deadman = self.get_parameter('deadman_timeout').value

        self.last_time = time.time()

        # subs
        self.create_subscription(Float32, '/cmd_steering', self.steer_cb, 10)
        self.create_subscription(Float32, '/cmd_throttle', self.throttle_cb, 10)

        # pubs
        self.steer_pub = self.create_publisher(Int16, '/cmd_steering_pwm', 10)
        self.throttle_pub = self.create_publisher(Int16, '/cmd_throttle_pwm', 10)

        self.timer = self.create_timer(0.05, self.watchdog)

        self.steer_val = 0.0
        self.throttle_val = 0.0

    def to_pwm(self, x):
        x = max(-1.0, min(1.0, x))
        return int(self.center + x * self.range)

    def steer_cb(self, msg):
        self.steer_val = msg.data
        self.last_time = time.time()

        pwm = Int16()
        pwm.data = self.to_pwm(self.steer_val)
        self.steer_pub.publish(pwm)

    def throttle_cb(self, msg):
        self.throttle_val = msg.data
        self.last_time = time.time()

        pwm = Int16()
        pwm.data = self.to_pwm(self.throttle_val)
        self.throttle_pub.publish(pwm)

    def watchdog(self):
        if time.time() - self.last_time > self.deadman:
            pwm = Int16()
            pwm.data = self.center

            self.steer_pub.publish(pwm)
            self.throttle_pub.publish(pwm)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()