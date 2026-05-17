#!/usr/bin/env python3
"""
webcam_realsense_bridge.py
Publishes webcam frames on RealSense-compatible topics using D435i intrinsics.
Drop this into your ROS2 package and run alongside your ArUco detection node.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2


# ── D435i intrinsics (640×480 mode) ──────────────────────────────────────────
# Replace these values if you calibrated your webcam separately.
D435I_WIDTH  = 640
D435I_HEIGHT = 480
D435I_FX     = 615.0      # focal length x  (pixels)
D435I_FY     = 615.0      # focal length y  (pixels)
D435I_CX     = 320.0      # principal point x
D435I_CY     = 240.0      # principal point y

# Distortion coefficients [k1, k2, p1, p2, k3]
# D435i colour sensor has very low distortion — zeros are fine for simulation.
D435I_DIST   = [0.0, 0.0, 0.0, 0.0, 0.0]
# ─────────────────────────────────────────────────────────────────────────────


class WebcamRealsenseBridge(Node):

    def __init__(self):
        super().__init__('webcam_realsense_bridge')

        # Parameters
        self.declare_parameter('camera_index', 0)   # change if webcam ≠ /dev/video0
        self.declare_parameter('publish_rate', 30.0)

        cam_idx  = self.get_parameter('camera_index').value
        rate_hz  = self.get_parameter('publish_rate').value

        # OpenCV capture
        self.cap = cv2.VideoCapture(cam_idx)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  D435I_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, D435I_HEIGHT)

        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open webcam at index {cam_idx}')
            raise RuntimeError('Webcam not available')

        self.bridge = CvBridge()

        # Publishers — same topic names the ArUco node expects
        self.pub_image = self.create_publisher(
            Image, '/camera/color/image_raw', 10)
        self.pub_info  = self.create_publisher(
            CameraInfo, '/camera/color/camera_info', 10)

        self.timer = self.create_timer(1.0 / rate_hz, self.publish_frame)
        self.get_logger().info(
            f'Bridge running: webcam {cam_idx} → /camera/color/* '
            f'at {rate_hz:.0f} Hz')

    # ── Camera info (published with every frame) ──────────────────────────────
    def _make_camera_info(self, stamp) -> CameraInfo:
        info = CameraInfo()
        info.header = Header(stamp=stamp, frame_id='camera_color_optical_frame')
        info.width  = D435I_WIDTH
        info.height = D435I_HEIGHT
        info.distortion_model = 'plumb_bob'
        info.d  = D435I_DIST
        info.k  = [D435I_FX,  0.0,     D435I_CX,
                   0.0,       D435I_FY, D435I_CY,
                   0.0,       0.0,      1.0]
        info.r  = [1.0, 0.0, 0.0,
                   0.0, 1.0, 0.0,
                   0.0, 0.0, 1.0]
        info.p  = [D435I_FX,  0.0,     D435I_CX, 0.0,
                   0.0,       D435I_FY, D435I_CY, 0.0,
                   0.0,       0.0,      1.0,      0.0]
        return info

    # ── Main publish callback ─────────────────────────────────────────────────
    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Dropped frame — webcam read failed')
            return

        stamp = self.get_clock().now().to_msg()

        # Image message
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header = Header(stamp=stamp,
                                frame_id='camera_color_optical_frame')
        self.pub_image.publish(img_msg)

        # Camera info message (same stamp)
        self.pub_info.publish(self._make_camera_info(stamp))

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamRealsenseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()