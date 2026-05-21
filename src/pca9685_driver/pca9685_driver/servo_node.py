"""
servo_node.py  —  Steering Servo Node
Package:   vehicle_interface
Hardware:  GOUPRC 20kg Low-Profile Servo on PCA9685 channel 0

Depends on pca9685_driver package (already in your workspace).

Topics subscribed:
  /cmd/steering_angle  (std_msgs/Float32)  degrees [-max_angle, +max_angle]
                                            + = right,  - = left
  /cmd/servo_raw_us    (std_msgs/Int32)    direct µs (for calibration only)

Topics published:
  /state/servo_angle   (std_msgs/Float32)  current commanded angle (deg)
  /state/servo_us      (std_msgs/Int32)    current commanded pulse (µs)

Services:
  /servo/centre        (std_srvs/Trigger)  snap to 1500 µs
  /servo/disable       (std_srvs/Trigger)  cut PWM (servo goes limp)

Parameters (servo.yaml):
  channel        int    0
  freq_hz        float  50.0
  pulse_min_us   float  500.0    full left
  pulse_max_us   float  2500.0   full right
  pulse_ctr_us   float  1500.0   centre trim ← adjust here if wheels aren't straight
  max_angle_deg  float  30.0
  i2c_bus        int    1
  i2c_address    int    0x40
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from std_srvs.srv import Trigger

# pca9685_driver is a sibling package in your workspace
from pca9685_driver.driver import PCA9685


class ServoNode(Node):

    def __init__(self):
        super().__init__('servo_node')

        # ── parameters ───────────────────────────────────────────────────────
        self.declare_parameter('channel',       0)
        self.declare_parameter('freq_hz',       50.0)
        self.declare_parameter('pulse_min_us',  500.0)
        self.declare_parameter('pulse_max_us',  2500.0)
        self.declare_parameter('pulse_ctr_us',  1500.0)
        self.declare_parameter('max_angle_deg', 30.0)
        self.declare_parameter('i2c_bus',       1)
        self.declare_parameter('i2c_address',   0x40)
        self._sync_params()

        # ── PCA9685 ──────────────────────────────────────────────────────────
        self._pca = PCA9685(
            i2c_bus=self.get_parameter('i2c_bus').value,
            address=self.get_parameter('i2c_address').value,
        )
        self._pca.set_pwm_freq(self._freq_hz)
        self._enabled = True
        self._set_us(self._pulse_ctr_us)      # start centred
        self.get_logger().info(
            f'ServoNode ready  ch={self._channel}  '
            f'freq={self._freq_hz}Hz  centre={self._pulse_ctr_us}µs'
        )

        # ── pub / sub ─────────────────────────────────────────────────────────
        self.create_subscription(Float32, '/cmd/steering_angle', self._cb_angle,  10)
        self.create_subscription(Int32,   '/cmd/servo_raw_us',   self._cb_raw,    10)

        self._pub_angle = self.create_publisher(Float32, '/state/servo_angle', 10)
        self._pub_us    = self.create_publisher(Int32,   '/state/servo_us',    10)

        # ── services ─────────────────────────────────────────────────────────
        self.create_service(Trigger, '/servo/centre',  self._svc_centre)
        self.create_service(Trigger, '/servo/disable', self._svc_disable)

    # ── helpers ───────────────────────────────────────────────────────────────

    def _sync_params(self):
        self._channel       = self.get_parameter('channel').value
        self._freq_hz       = float(self.get_parameter('freq_hz').value)
        self._pulse_min_us  = float(self.get_parameter('pulse_min_us').value)
        self._pulse_max_us  = float(self.get_parameter('pulse_max_us').value)
        self._pulse_ctr_us  = float(self.get_parameter('pulse_ctr_us').value)
        self._max_angle_deg = float(self.get_parameter('max_angle_deg').value)

    def _angle_to_us(self, deg: float) -> float:
        deg = max(-self._max_angle_deg, min(self._max_angle_deg, deg))
        if deg >= 0:
            t = deg / self._max_angle_deg
            return self._pulse_ctr_us + t * (self._pulse_max_us - self._pulse_ctr_us)
        else:
            t = -deg / self._max_angle_deg
            return self._pulse_ctr_us - t * (self._pulse_ctr_us - self._pulse_min_us)

    def _us_to_angle(self, us: float) -> float:
        if us >= self._pulse_ctr_us:
            t = (us - self._pulse_ctr_us) / (self._pulse_max_us - self._pulse_ctr_us + 1e-9)
            return t * self._max_angle_deg
        else:
            t = (self._pulse_ctr_us - us) / (self._pulse_ctr_us - self._pulse_min_us + 1e-9)
            return -t * self._max_angle_deg

    def _set_us(self, us: float) -> None:
        if not self._enabled:
            return
        us = max(self._pulse_min_us, min(self._pulse_max_us, us))
        self._pca.set_pulse_us(self._channel, us, self._freq_hz)
        self._pub_us.publish(Int32(data=int(us)))
        self._pub_angle.publish(Float32(data=self._us_to_angle(us)))
        self.get_logger().debug(f'servo → {us:.0f} µs')

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _cb_angle(self, msg: Float32):
        self._set_us(self._angle_to_us(msg.data))

    def _cb_raw(self, msg: Int32):
        self._set_us(float(msg.data))

    # ── services ──────────────────────────────────────────────────────────────

    def _svc_centre(self, _, response):
        self._enabled = True
        self._set_us(self._pulse_ctr_us)
        response.success = True
        response.message = f'Centred at {self._pulse_ctr_us} µs'
        return response

    def _svc_disable(self, _, response):
        self._enabled = False
        self._pca.set_all_off()
        response.success = True
        response.message = 'Servo PWM disabled'
        return response

    # ── cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        if self._enabled:
            self._set_us(self._pulse_ctr_us)
        self._pca.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
