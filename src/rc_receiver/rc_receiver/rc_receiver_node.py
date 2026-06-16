"""ROS2 node: rc_receiver_node.

Reads SBUS frames from /dev/ttyAMA0 (inverted hardware signal already
corrected by the NPN transistor circuit) and publishes:

  /rc/steering   std_msgs/Float32   [-1.0 … +1.0]
  /rc/throttle   std_msgs/Float32   [-1.0 … +1.0]
  /rc/mode       std_msgs/Int8      0=manual 1=auto 2=stop

Futaba 8-channel default mapping (all overridable via ROS params):
  CH1 → steering  (right stick lateral)
  CH2 → throttle  (right stick vertical)
  CH5 → mode      (C switch, 3-position)
"""

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8

from .sbus_parser import (
    SBUS_FRAME_LEN,
    SBUS_START,
    parse_frame,
    raw_to_mode,
    raw_to_normalized,
)


class RcReceiverNode(Node):
    """Read SBUS serial stream and publish RC channel values as ROS2 topics."""

    def __init__(self):
        super().__init__('rc_receiver_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('steering_channel', 0)   # CH1 → index 0
        self.declare_parameter('throttle_channel', 1)   # CH2 → index 1
        self.declare_parameter('mode_channel', 4)       # CH5 → index 4
        self.declare_parameter('publish_rate_hz', 50.0)

        port = self.get_parameter('port').value
        self.ch_steer = self.get_parameter('steering_channel').value
        self.ch_throttle = self.get_parameter('throttle_channel').value
        self.ch_mode = self.get_parameter('mode_channel').value
        rate = self.get_parameter('publish_rate_hz').value

        # ── Serial port ───────────────────────────────────────────────────
        # SBUS: 100000 baud, 8E2, inverted (handled by hardware BJT circuit)
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=100000,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_EVEN,
                stopbits=serial.STOPBITS_TWO,
                timeout=0.1,
            )
            self.get_logger().info(f'SBUS serial open on {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open serial port {port}: {e}')
            raise

        # ── Publishers ────────────────────────────────────────────────────
        self.steer_pub = self.create_publisher(Float32, '/rc/steering', 10)
        self.throttle_pub = self.create_publisher(Float32, '/rc/throttle', 10)
        self.mode_pub = self.create_publisher(Int8, '/rc/mode', 10)

        # ── Internal state ────────────────────────────────────────────────
        self._buf = bytearray()
        self._last_mode = -1   # track mode changes for logging

        # ── Timer ─────────────────────────────────────────────────────────
        period = 1.0 / rate
        self.create_timer(period, self._read_and_publish)

        self.get_logger().info(
            f'rc_receiver_node ready — steering=CH{self.ch_steer + 1} '
            f'throttle=CH{self.ch_throttle + 1} '
            f'mode=CH{self.ch_mode + 1}'
        )

    # ── Main loop callback ────────────────────────────────────────────────

    def _read_and_publish(self):
        """Drain serial buffer, extract complete SBUS frames, publish."""
        # Read all bytes currently in the buffer
        waiting = self.ser.in_waiting
        if waiting:
            self._buf.extend(self.ser.read(waiting))

        # Extract and process every complete frame in the buffer
        while True:
            frame = self._extract_frame()
            if frame is None:
                break
            self._process_frame(frame)

    def _extract_frame(self) -> bytes | None:
        """Return the next valid 25-byte SBUS frame and remove it from buf.

        Scans for the 0x0F start byte, checks that byte 24 is 0x00,
        then returns the frame. Discards any leading garbage bytes.
        """
        # Find the next start byte
        start = self._buf.find(SBUS_START)
        if start == -1:
            self._buf.clear()
            return None

        # Discard bytes before the start byte
        if start > 0:
            self._buf = self._buf[start:]

        # Wait until we have a full frame
        if len(self._buf) < SBUS_FRAME_LEN:
            return None

        candidate = bytes(self._buf[:SBUS_FRAME_LEN])
        self._buf = self._buf[SBUS_FRAME_LEN:]
        return candidate

    def _process_frame(self, raw_frame: bytes):
        """Parse frame and publish ROS messages."""
        result = parse_frame(raw_frame)
        if result is None:
            self.get_logger().debug('Bad SBUS frame — skipping')
            return

        if result['failsafe']:
            self.get_logger().warn('SBUS FAILSAFE active — RC link lost!')
            return

        channels = result['channels']

        # Steering
        steer_msg = Float32()
        steer_msg.data = raw_to_normalized(channels[self.ch_steer])
        self.steer_pub.publish(steer_msg)

        # Throttle
        throttle_msg = Float32()
        throttle_msg.data = raw_to_normalized(channels[self.ch_throttle])
        self.throttle_pub.publish(throttle_msg)

        # Mode (C switch)
        mode = raw_to_mode(channels[self.ch_mode])
        mode_msg = Int8()
        mode_msg.data = mode
        self.mode_pub.publish(mode_msg)

        # Log mode changes only
        if mode != self._last_mode:
            labels = {0: 'RC MANUAL', 1: 'AUTONOMOUS', 2: 'STOP'}
            self.get_logger().info(f'Mode → {labels.get(mode, "UNKNOWN")} ({mode})')
            self._last_mode = mode

    # ── Cleanup ───────────────────────────────────────────────────────────

    def destroy_node(self):
        """Close serial port on shutdown."""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial port closed')
        super().destroy_node()


def main(args=None):
    """Entrypoint."""
    rclpy.init(args=args)
    node = RcReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()