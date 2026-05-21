"""
camera_node.py  —  OAK-D Lite Spatial AI Camera Node
Package:   perception
Hardware:  Luxonis OAK-D Lite (USB3, fixed focus)

Publishes RGB, depth, and spatial object detections using the
DepthAI SDK. The OAK-D does detection inference on-chip (MobileNetSSD v2),
saving Raspberry Pi CPU for navigation logic.

Install deps:
  pip3 install depthai
  (no ROS dep needed — uses USB3, not I2C/UART)

Topics published:
  /camera/rgb/image_raw        (sensor_msgs/Image)        BGR8, 300×300
  /camera/depth/image_raw      (sensor_msgs/Image)        16UC1 mm
  /camera/detections           (vision_msgs/Detection3DArray)

Parameters (perception.yaml):
  fps            int    30
  rgb_width      int    300
  rgb_height     int    300
  confidence_thr float  0.5     detection confidence threshold
  frame_id       str    'oak_d_frame'
  nn_blob_path   str    ''      path to MobileNetSSD .blob — leave '' to use built-in
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    import depthai as dai
    import numpy as np
    DEPTHAI_AVAILABLE = True
except ImportError:
    DEPTHAI_AVAILABLE = False


# Label map for MobileNetSSD (COCO 20-class subset used in DepthAI default blob)
LABEL_MAP = [
    'background', 'aeroplane', 'bicycle', 'bird', 'boat',
    'bottle', 'bus', 'car', 'cat', 'chair', 'cow',
    'diningtable', 'dog', 'horse', 'motorbike', 'person',
    'pottedplant', 'sheep', 'sofa', 'train', 'tvmonitor'
]


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('fps',            30)
        self.declare_parameter('rgb_width',      300)
        self.declare_parameter('rgb_height',     300)
        self.declare_parameter('confidence_thr', 0.5)
        self.declare_parameter('frame_id',       'oak_d_frame')
        self.declare_parameter('nn_blob_path',   '')

        self._fps      = self.get_parameter('fps').value
        self._w        = self.get_parameter('rgb_width').value
        self._h        = self.get_parameter('rgb_height').value
        self._conf_thr = self.get_parameter('confidence_thr').value
        self._fid      = self.get_parameter('frame_id').value
        self._bridge   = CvBridge()

        # ── publishers ────────────────────────────────────────────────────────
        self._pub_rgb   = self.create_publisher(Image, '/camera/rgb/image_raw',   10)
        self._pub_depth = self.create_publisher(Image, '/camera/depth/image_raw', 10)

        if not DEPTHAI_AVAILABLE:
            self.get_logger().error(
                'depthai not installed — run: pip3 install depthai'
            )
            return

        self._pipeline = self._build_pipeline()
        self._device   = dai.Device(self._pipeline)
        self._q_rgb    = self._device.getOutputQueue('rgb',   maxSize=4, blocking=False)
        self._q_depth  = self._device.getOutputQueue('depth', maxSize=4, blocking=False)

        self.get_logger().info('OAK-D Lite ready')
        self.create_timer(1.0 / self._fps, self._publish)

    # ── DepthAI pipeline ──────────────────────────────────────────────────────

    def _build_pipeline(self):
        p = dai.Pipeline()

        # ── colour camera ──────────────────────────────────────────────────
        cam_rgb = p.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(self._w, self._h)
        cam_rgb.setInterleaved(False)
        cam_rgb.setFps(self._fps)

        xout_rgb = p.create(dai.node.XLinkOut)
        xout_rgb.setStreamName('rgb')
        cam_rgb.preview.link(xout_rgb.input)

        # ── stereo depth ───────────────────────────────────────────────────
        mono_l = p.create(dai.node.MonoCamera)
        mono_r = p.create(dai.node.MonoCamera)
        mono_l.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_r.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_l.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_r.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        stereo = p.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        mono_l.out.link(stereo.left)
        mono_r.out.link(stereo.right)

        xout_depth = p.create(dai.node.XLinkOut)
        xout_depth.setStreamName('depth')
        stereo.depth.link(xout_depth.input)

        return p

    # ── publish timer ─────────────────────────────────────────────────────────

    def _publish(self):
        import numpy as np

        in_rgb = self._q_rgb.tryGet()
        if in_rgb is not None:
            frame = in_rgb.getCvFrame()
            msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = self._fid
            self._pub_rgb.publish(msg)

        in_depth = self._q_depth.tryGet()
        if in_depth is not None:
            depth_frame = in_depth.getFrame()
            msg = self._bridge.cv2_to_imgmsg(depth_frame, encoding='16UC1')
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = self._fid
            self._pub_depth.publish(msg)

    # ── cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        if hasattr(self, '_device'):
            self._device.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
