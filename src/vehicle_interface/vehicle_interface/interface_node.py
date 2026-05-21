"""
interface_node.py  —  Vehicle Interface Node
Package:   vehicle_interface

Sits between high-level commands (normalised Float32) and the PCA9685 node
(raw µs Int16). Handles:
  • Normalised → µs conversion  (-1.0…+1.0  →  center±range µs)
  • Deadman watchdog            (centre/neutral if no cmd in deadman_timeout s)
  • ESC arming sequence         (/esc/arm service — call once at startup)
  • Emergency stop              (/estop service)

Topics subscribed:
  /cmd_steering   (std_msgs/Float32)   [-1.0, +1.0]
  /cmd_throttle   (std_msgs/Float32)   [-1.0, +1.0]

Topics published → pca9685_node:
  /cmd_steering_pwm  (std_msgs/Int16)  µs
  /cmd_throttle_pwm  (std_msgs/Int16)  µs

Services:
  /esc/arm    (std_srvs/Trigger)  ESC arming sequence (hold neutral arm_time_s)
  /estop      (std_srvs/Trigger)  Emergency stop (disables throttle until re-armed)

Parameters (vehicle_interface.yaml):
  pwm_center        int    1500   µs centre / neutral
  pwm_range         int    500    µs from centre to full deflection
  deadman_timeout   float  0.2    s — publishes neutral if no cmd received
  arm_time_s        float  2.0    s — time to hold neutral during ESC arming
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16
from std_srvs.srv import Trigger


class VehicleInterface(Node):

    def __init__(self):
        super().__init__('vehicle_interface')

        # ── parameters ────────────────────────────────────────────────────────
        self.declare_parameter('pwm_center',      1500)
        self.declare_parameter('pwm_range',       500)
        self.declare_parameter('deadman_timeout', 0.2)
        self.declare_parameter('arm_time_s',      2.0)

        self.center   = self.get_parameter('pwm_center').value
        self.range    = self.get_parameter('pwm_range').value
        self.deadman  = self.get_parameter('deadman_timeout').value
        self.arm_time = self.get_parameter('arm_time_s').value

        # ── state ─────────────────────────────────────────────────────────────
        self.last_time    = time.time()
        self.steer_val    = 0.0
        self.throttle_val = 0.0
        self._esc_armed   = False    # throttle blocked until /esc/arm called
        self._estopped    = False

        # ── subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Float32, '/cmd_steering',  self.steer_cb,    10)
        self.create_subscription(Float32, '/cmd_throttle',  self.throttle_cb, 10)

        # ── publishers → pca9685_node ─────────────────────────────────────────
        self.steer_pub    = self.create_publisher(Int16, '/cmd_steering_pwm', 10)
        self.throttle_pub = self.create_publisher(Int16, '/cmd_throttle_pwm', 10)

        # ── services ──────────────────────────────────────────────────────────
        self.create_service(Trigger, '/esc/arm', self._svc_arm)
        self.create_service(Trigger, '/estop',   self._svc_estop)

        # ── deadman watchdog at 20 Hz ─────────────────────────────────────────
        self.timer = self.create_timer(0.05, self.watchdog)

        self.get_logger().info(
            f'VehicleInterface ready  '
            f'center={self.center}µs  range=±{self.range}µs  '
            f'deadman={self.deadman}s\n'
            f'  ⚠  Call /esc/arm before sending throttle commands.'
        )

    # ── helpers ───────────────────────────────────────────────────────────────

    def _to_pwm(self, x: float) -> int:
        x = max(-1.0, min(1.0, x))
        return int(self.center + x * self.range)

    def _pub_neutral(self):
        """Publish centre/neutral to both channels (safe state)."""
        msg = Int16(data=self.center)
        self.steer_pub.publish(msg)
        self.throttle_pub.publish(msg)

    # ── subscribers ───────────────────────────────────────────────────────────

    def steer_cb(self, msg: Float32):
        self.steer_val = msg.data
        self.last_time = time.time()
        self.steer_pub.publish(Int16(data=self._to_pwm(self.steer_val)))

    def throttle_cb(self, msg: Float32):
        self.throttle_val = msg.data
        self.last_time    = time.time()

        if not self._esc_armed:
            self.get_logger().warn(
                'Throttle ignored — ESC not armed. Call /esc/arm first.',
                throttle_duration_sec=2.0
            )
            return
        if self._estopped:
            self.get_logger().warn(
                'Throttle ignored — E-STOP active. Call /esc/arm to reset.',
                throttle_duration_sec=2.0
            )
            return

        self.throttle_pub.publish(Int16(data=self._to_pwm(self.throttle_val)))

    # ── watchdog ──────────────────────────────────────────────────────────────

    def watchdog(self):
        if time.time() - self.last_time > self.deadman:
            self._pub_neutral()

    # ── services ──────────────────────────────────────────────────────────────

    def _svc_arm(self, _, response):
        self.get_logger().info(
            f'Arming ESC — holding neutral ({self.center}µs) for {self.arm_time}s …'
        )
        self._estopped = False

        # Hold neutral for arm_time_s (blocks the service call — acceptable here)
        msg = Int16(data=self.center)
        start = time.time()
        while time.time() - start < self.arm_time:
            self.throttle_pub.publish(msg)
            time.sleep(0.1)

        self._esc_armed = True
        response.success = True
        response.message = 'ESC armed ✓ — throttle commands now accepted'
        self.get_logger().info('ESC armed ✓')
        return response

    def _svc_estop(self, _, response):
        self._estopped  = True
        self._esc_armed = False
        self._pub_neutral()
        response.success = True
        response.message = 'E-STOP — throttle locked. Call /esc/arm to reset.'
        self.get_logger().warn('⚠ E-STOP activated')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VehicleInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()