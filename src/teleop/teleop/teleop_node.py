import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys
import tty
import termios


class TeleopPC(Node):

    def __init__(self):
        super().__init__('teleop_pc')

        self.pub_steer = self.create_publisher(Float32, '/teleop/steering', 10)
        self.pub_throttle = self.create_publisher(Float32, '/teleop/throttle', 10)

        self.create_timer(0.05, self.loop)

        self.get_logger().info("TELEOP PC READY (WASD)")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def loop(self):
        key = self.get_key()

        steer = 0.0
        throttle = 0.0

        if key == 'w':
            throttle = 0.4
        elif key == 's':
            throttle = -0.3
        elif key == 'a':
            steer = -0.5
        elif key == 'd':
            steer = 0.5
        elif key == ' ':
            steer = 0.0
            throttle = 0.0

        self.pub_steer.publish(Float32(data=steer))
        self.pub_throttle.publish(Float32(data=throttle))


def main():
    rclpy.init()
    node = TeleopPC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()