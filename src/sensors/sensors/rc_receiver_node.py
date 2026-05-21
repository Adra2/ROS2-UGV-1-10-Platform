"""
rc_receiver_node.py  —  DUMBORC X6FG RC Receiver Node
Package:   sensors
Hardware:  DUMBORC X6FG 6-channel 2.4GHz (built-in gyro)

Reads the PPM SUM signal from the receiver using pigpio.
All 6 channels arrive on a single GPIO pin (much cleaner than 6 wires).

Wiring (Raspberry Pi 4):
  Receiver PPM/SIG pin  →  Pi GPIO 18 (BCM) / pin 12  ← configurable
  Receiver VCC (5V)     →  Pi pin 4 or external 5V BEC
  Receiver GND          →  Pi GND (pin 6)

⚠️  DUMBORC X6FG — PPM output:
  Enable PPM on the receiver: hold BIND button 3 s until LED changes.
  PPM frame: ~20 ms, 6 channels, each pulse 1000-2000 µs, gap >3000 µs.

One-time setup:
  sudo pigpiod                          # start daemon (or add to rc.local)
  pip3 install pigpio

Channel mapping (DUMBORC X6 transmitter, adjust if needed):
  CH1 → Right stick H  → steering
  CH2 → Right stick V  → throttle (some TX: left stick)
  CH3 → Left stick V   → throttle (mode dependent)
  CH4 → Left stick H   → (unused / future)
  CH5 → Aux switch     → mode (manual / autonomous)
  CH6 → Aux 2          → (unused)

Topics published:
  /rc/steering     (std_msgs/Float32)  normalised [-1.0, +1.0]
  /rc/throttle     (std_msgs/Float32)  normalised [-1.0, +1.0]
  /rc/mode         (std_msgs/Bool)     True = autonomous, False = manual
  /rc/raw_channels (std_msgs/Int32MultiArray)  raw µs all 6 channels (debug)
  /rc/connected    (std_msgs/Bool)     failsafe watchdog

Parameters (sensors.yaml):
  ppm_gpio        int    18     BCM GPIO pin
  num_channels    int    6
  pulse_min_us    int    1000
  pulse_max_us    int    2000
  pulse_ctr_us    int    1500
  failsafe_ms     int    500    ms without valid frame → publish 0s + connected=False
  steering_ch     int    0      0-indexed channel index
  throttle_ch     int    1
  mode_ch         int    4
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32MultiArray

try:
    import pigpio
    PIGPIO_AVAILABLE = True
except ImportError:
    PIGPIO_AVAILABLE = False


class PPMDecoder:
    """Decodes PPM SUM signal using pigpio edge callbacks."""

    GAP_US = 3000   # µs gap that marks end of PPM frame

    def __init__(self, pi, gpio: int, num_channels: int):
        self._pi       = pi
        self._gpio     = gpio
        self._n_ch     = num_channels
        self._channels = [1500] * num_channels
        self._idx      = 0
        self._last_tick = None
        self._last_frame_time = 0.0

        pi.set_mode(gpio, pigpio.INPUT)
        self._cb = pi.callback(gpio, pigpio.RISING_EDGE, self._edge)

    def _edge(self, gpio, level, tick):
        if self._last_tick is None:
            self._last_tick = tick
            return

        pulse_us = pigpio.tickDiff(self._last_tick, tick)
        self._last_tick = tick

        if pulse_us > self.GAP_US:                 # frame separator
            self._idx = 0
        elif self._idx < self._n_ch:
            self._channels[self._idx] = pulse_us
            self._idx += 1
            if self._idx == self._n_ch:
                self._last_frame_time = time.monotonic()

    @property
    def channels(self):
        return list(self._channels)

    @property
    def last_frame_age_ms(self):
        return (time.monotonic() - self._last_frame_time) * 1000.0

    def cancel(self):
        self._cb.cancel()


class RCReceiverNode(Node):

    def __init__(self):
        super().__init__('rc_receiver_node')

        self.declare_parameter('ppm_gpio',     18)
        self.declare_parameter('num_channels', 6)
        self.declare_parameter('pulse_min_us', 1000)
        self.declare_parameter('pulse_max_us', 2000)
        self.declare_parameter('pulse_ctr_us', 1500)
        self.declare_parameter('failsafe_ms',  500)
        self.declare_parameter('steering_ch',  0)
        self.declare_parameter('throttle_ch',  1)
        self.declare_parameter('mode_ch',      4)

        self._gpio      = self.get_parameter('ppm_gpio').value
        self._n_ch      = self.get_parameter('num_channels').value
        self._pmin      = float(self.get_parameter('pulse_min_us').value)
        self._pmax      = float(self.get_parameter('pulse_max_us').value)
        self._pctr      = float(self.get_parameter('pulse_ctr_us').value)
        self._fail_ms   = self.get_parameter('failsafe_ms').value
        self._steer_ch  = self.get_parameter('steering_ch').value
        self._thr_ch    = self.get_parameter('throttle_ch').value
        self._mode_ch   = self.get_parameter('mode_ch').value

        # ── publishers ────────────────────────────────────────────────────────
        self._pub_steer   = self.create_publisher(Float32,          '/rc/steering',     10)
        self._pub_thr     = self.create_publisher(Float32,          '/rc/throttle',     10)
        self._pub_mode    = self.create_publisher(Bool,             '/rc/mode',         10)
        self._pub_raw     = self.create_publisher(Int32MultiArray,  '/rc/raw_channels', 10)
        self._pub_conn    = self.create_publisher(Bool,             '/rc/connected',    10)

        # ── pigpio ────────────────────────────────────────────────────────────
        if not PIGPIO_AVAILABLE:
            self.get_logger().error('pigpio not installed — run: pip3 install pigpio')
            return

        self._pi = pigpio.pi()
        if not self._pi.connected:
            self.get_logger().error(
                'pigpio daemon not running — run: sudo pigpiod'
            )
            return

        self._decoder = PPMDecoder(self._pi, self._gpio, self._n_ch)
        self.get_logger().info(f'RCReceiverNode listening on GPIO {self._gpio}')

        self.create_timer(0.02, self._publish)   # 50 Hz publish rate

    # ── normalise µs to [-1, 1] ───────────────────────────────────────────────

    def _norm(self, us: int) -> float:
        us = max(self._pmin, min(self._pmax, float(us)))
        return (us - self._pctr) / ((self._pmax - self._pmin) / 2.0)

    # ── timer callback ────────────────────────────────────────────────────────

    def _publish(self):
        if not hasattr(self, '_decoder'):
            return

        connected = self._decoder.last_frame_age_ms < self._fail_ms
        self._pub_conn.publish(Bool(data=connected))

        ch = self._decoder.channels

        # raw
        raw_msg = Int32MultiArray()
        raw_msg.data = [int(c) for c in ch]
        self._pub_raw.publish(raw_msg)

        if not connected:
            # failsafe — zero everything
            self._pub_steer.publish(Float32(data=0.0))
            self._pub_thr.publish(Float32(data=0.0))
            self.get_logger().warn('RC signal lost — failsafe active', throttle_duration_sec=1.0)
            return

        self._pub_steer.publish(Float32(data=self._norm(ch[self._steer_ch])))
        self._pub_thr.publish(Float32(data=self._norm(ch[self._thr_ch])))

        # mode: aux switch CH5 — >1600µs = autonomous
        autonomous = ch[self._mode_ch] > 1600 if len(ch) > self._mode_ch else False
        self._pub_mode.publish(Bool(data=autonomous))

    # ── cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        if hasattr(self, '_decoder'):
            self._decoder.cancel()
        if hasattr(self, '_pi') and self._pi.connected:
            self._pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RCReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
