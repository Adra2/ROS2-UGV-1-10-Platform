"""
esc_node.py  —  Electronic Speed Controller Node
Package:   vehicle_interface
Hardware:  GoolRC 3650 3100KV + 60A Sensorless ESC  on PCA9685 channel 1

⚠️  ARMING SEQUENCE — must be done before sending throttle:
    Call service  /esc/arm  → holds neutral (1500µs) for arm_time_s
    ESC will beep once when armed.

Topics subscribed:
  /cmd/throttle      (std_msgs/Float32)  [-1.0 full reverse … +1.0 full forward]
  /cmd/esc_raw_us    (std_msgs/Int32)    direct µs (calibration)

Topics published:
  /state/throttle    (std_msgs/Float32)  current normalised throttle
  /state/esc_us      (std_msgs/Int32)    current pulse µs

Services:
  /esc/arm           (std_srvs/Trigger)  arming sequence (CALL FIRST)
  /esc/neutral       (std_srvs/Trigger)  snap to neutral
  /esc/disable       (std_srvs/Trigger)  cut command (holds neutral, disarms)

Parameters (esc.yaml):
  channel             int    1
  freq_hz             float  50.0
  pulse_neutral_us    float  1500.0
  pulse_fwd_max_us    float  2000.0
  pulse_rev_max_us    float  1000.0   ← if ESC doesn't support reverse, set = neutral
  arm_time_s          float  2.0
  i2c_bus             int    1
  i2c_address         int    0x40
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from std_srvs.srv import Trigger

from pca9685_driver.driver import PCA9685


class ESCNode(Node):

    def __init__(self):
        super().__init__('esc_node')

        # ── parameters ───────────────────────────────────────────────────────
        self.declare_parameter('channel',           1)
        self.declare_parameter('freq_hz',           50.0)
        self.declare_parameter('pulse_neutral_us',  1500.0)
        self.declare_parameter('pulse_fwd_max_us',  2000.0)
        self.declare_parameter('pulse_rev_max_us',  1000.0)
        self.declare_parameter('arm_time_s',        2.0)
        self.declare_parameter('i2c_bus',           1)
        self.declare_parameter('i2c_address',       0x40)
        self._sync_params()

        # ── PCA9685 ──────────────────────────────────────────────────────────
        self._pca = PCA9685(
            i2c_bus=self.get_parameter('i2c_bus').value,
            address=self.get_parameter('i2c_address').value,
        )
        self._pca.set_pwm_freq(self._freq_hz)
        self._armed   = False
        self._enabled = False

        # Hold neutral immediately — safe state before arming
        self._pca.set_pulse_us(self._channel, self._pulse_neutral_us, self._freq_hz)
        self.get_logger().info(
            'ESCNode ready — holding neutral.  '
            'Call /esc/arm service before sending throttle commands.'
        )

        # ── pub / sub ─────────────────────────────────────────────────────────
        self.create_subscription(Float32, '/cmd/throttle',   self._cb_throttle, 10)
        self.create_subscription(Int32,   '/cmd/esc_raw_us', self._cb_raw,      10)

        self._pub_throttle = self.create_publisher(Float32, '/state/throttle', 10)
        self._pub_us       = self.create_publisher(Int32,   '/state/esc_us',   10)

        # ── services ─────────────────────────────────────────────────────────
        self.create_service(Trigger, '/esc/arm',     self._svc_arm)
        self.create_service(Trigger, '/esc/neutral', self._svc_neutral)
        self.create_service(Trigger, '/esc/disable', self._svc_disable)

    # ── helpers ───────────────────────────────────────────────────────────────

    def _sync_params(self):
        self._channel           = self.get_parameter('channel').value
        self._freq_hz           = float(self.get_parameter('freq_hz').value)
        self._pulse_neutral_us  = float(self.get_parameter('pulse_neutral_us').value)
        self._pulse_fwd_max_us  = float(self.get_parameter('pulse_fwd_max_us').value)
        self._pulse_rev_max_us  = float(self.get_parameter('pulse_rev_max_us').value)
        self._arm_time_s        = float(self.get_parameter('arm_time_s').value)

    def _throttle_to_us(self, t: float) -> float:
        t = max(-1.0, min(1.0, t))
        if t >= 0.0:
            return self._pulse_neutral_us + t * (
                self._pulse_fwd_max_us - self._pulse_neutral_us)
        else:
            return self._pulse_neutral_us + t * (
                self._pulse_neutral_us - self._pulse_rev_max_us)

    def _us_to_throttle(self, us: float) -> float:
        if us >= self._pulse_neutral_us:
            return (us - self._pulse_neutral_us) / (
                self._pulse_fwd_max_us - self._pulse_neutral_us + 1e-9)
        else:
            return -(self._pulse_neutral_us - us) / (
                self._pulse_neutral_us - self._pulse_rev_max_us + 1e-9)

    def _set_us(self, us: float) -> None:
        if not self._enabled:
            self.get_logger().warn('ESC not armed — call /esc/arm first', throttle_duration_sec=2.0)
            return
        us = max(self._pulse_rev_max_us, min(self._pulse_fwd_max_us, us))
        self._pca.set_pulse_us(self._channel, us, self._freq_hz)
        self._pub_us.publish(Int32(data=int(us)))
        self._pub_throttle.publish(Float32(data=float(max(-1.0, min(1.0, self._us_to_throttle(us))))))
        self.get_logger().debug(f'ESC → {us:.0f} µs')

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _cb_throttle(self, msg: Float32):
        self._set_us(self._throttle_to_us(msg.data))

    def _cb_raw(self, msg: Int32):
        self._set_us(float(msg.data))

    # ── services ──────────────────────────────────────────────────────────────

    def _svc_arm(self, _, response):
        self.get_logger().info(
            f'Arming ESC — holding neutral ({self._pulse_neutral_us:.0f} µs) '
            f'for {self._arm_time_s} s …'
        )
        self._pca.set_pulse_us(self._channel, self._pulse_neutral_us, self._freq_hz)
        time.sleep(self._arm_time_s)
        self._armed   = True
        self._enabled = True
        response.success = True
        response.message = 'ESC armed and ready ✓'
        self.get_logger().info('ESC armed ✓')
        return response

    def _svc_neutral(self, _, response):
        self._pca.set_pulse_us(self._channel, self._pulse_neutral_us, self._freq_hz)
        response.success = True
        response.message = f'ESC neutral ({self._pulse_neutral_us:.0f} µs)'
        return response

    def _svc_disable(self, _, response):
        self._enabled = False
        self._armed   = False
        self._pca.set_pulse_us(self._channel, self._pulse_neutral_us, self._freq_hz)
        response.success = True
        response.message = 'ESC disabled — neutral held'
        return response

    # ── cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._pca.set_pulse_us(self._channel, self._pulse_neutral_us, self._freq_hz)
        self._pca.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
