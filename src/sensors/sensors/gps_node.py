"""
gps_node.py  —  NEO-6M GPS Node
Package:   sensors
Hardware:  Ublox NEO-6M via UART

Wiring (Raspberry Pi 4):
  Module TX  →  Pi GPIO15 / pin 10  (UART RX)
  Module RX  →  Pi GPIO14 / pin 8   (UART TX)
  VCC        →  3.3 V  (pin 1)
  GND        →  GND    (pin 6)

Enable UART on Pi (one-time setup):
  sudo raspi-config
    → Interface Options → Serial Port
    → Login shell over serial? NO
    → Serial port hardware enabled? YES
  Then reboot. Port will be /dev/ttyAMA0 or /dev/serial0

Install deps:
  pip3 install pyserial pynmea2

Topics published:
  /gps/fix          (sensor_msgs/NavSatFix)   lat/lon/alt + covariance status
  /gps/nmea_raw     (std_msgs/String)         raw NMEA sentence (debug)

Parameters (sensors.yaml):
  port          str    '/dev/serial0'
  baud          int    9600
  frame_id      str    'gps_link'
  publish_raw   bool   false
"""

import serial
import pynmea2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GPSNode(Node):

    def __init__(self):
        super().__init__('gps_node')

        self.declare_parameter('port',        '/dev/serial0')
        self.declare_parameter('baud',        9600)
        self.declare_parameter('frame_id',    'gps_link')
        self.declare_parameter('publish_raw', False)

        port       = self.get_parameter('port').value
        baud       = self.get_parameter('baud').value
        self._fid  = self.get_parameter('frame_id').value
        self._raw  = self.get_parameter('publish_raw').value

        # ── publishers ────────────────────────────────────────────────────────
        self._pub_fix = self.create_publisher(NavSatFix, '/gps/fix',      10)
        self._pub_raw = self.create_publisher(String,    '/gps/nmea_raw', 10)

        # ── serial port ───────────────────────────────────────────────────────
        try:
            self._ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f'GPS opened {port} @ {baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open GPS port {port}: {e}')
            raise

        # ── read timer (10 Hz should be fine for NEO-6M at 1 Hz GPS rate) ────
        self.create_timer(0.1, self._read_serial)

    # ── serial read ───────────────────────────────────────────────────────────

    def _read_serial(self):
        try:
            if not self._ser.in_waiting:
                return
            line = self._ser.readline().decode('ascii', errors='replace').strip()
        except serial.SerialException as e:
            self.get_logger().warn(f'GPS serial error: {e}')
            return

        if self._raw:
            self._pub_raw.publish(String(data=line))

        if not line.startswith('$GNGGA') and not line.startswith('$GPGGA'):
            return   # only parse GGA (position fix)

        try:
            msg = pynmea2.parse(line)
        except pynmea2.ParseError:
            return

        fix = NavSatFix()
        fix.header.stamp    = self.get_clock().now().to_msg()
        fix.header.frame_id = self._fid
        fix.latitude        = msg.latitude
        fix.longitude       = msg.longitude
        fix.altitude        = float(msg.altitude) if msg.altitude else float('nan')

        # Map NMEA fix quality to NavSatStatus
        quality = int(msg.gps_qual) if msg.gps_qual else 0
        if quality == 0:
            fix.status.status = NavSatStatus.STATUS_NO_FIX
        elif quality == 1:
            fix.status.status = NavSatStatus.STATUS_FIX
        elif quality == 2:
            fix.status.status = NavSatStatus.STATUS_SBAS_FIX
        else:
            fix.status.status = NavSatStatus.STATUS_GBAS_FIX

        fix.status.service    = NavSatStatus.SERVICE_GPS
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self._pub_fix.publish(fix)

    # ── cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        if hasattr(self, '_ser') and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
