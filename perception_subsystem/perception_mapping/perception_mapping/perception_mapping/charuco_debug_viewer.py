#!/usr/bin/env python3
"""
charuco_debug_viewer.py
=======================
Live camera feed with ChArUco detection overlay for diagnosing
whether the board is being detected and why calibration TF is missing.

What it shows on screen:
  - Raw camera feed at full resolution
  - Every detected ArUco marker outlined in green with its ID
  - Every detected ChArUco corner as a coloured dot
  - 3D pose axes (X=red Y=green Z=blue) drawn on the board when detected
  - HUD overlay:
      - Number of ArUco markers found
      - Number of ChArUco corners found
      - Board pose (x, y, z in metres) when detected
      - TF publish status (green = publishing, red = not publishing)
      - Camera topic being used
      - Detection status message

Also publishes:
  /charuco_debug/image  — annotated image (viewable in rqt_image_view or RViz)

Usage:
  ros2 run perception_mapping charuco_debug_viewer

Then view the feed in one of:
  Option A (opens its own window — simplest):
    The node opens a cv2.imshow window directly.
  Option B (rqt):
    ros2 run rqt_image_view rqt_image_view
    → select /charuco_debug/image from the dropdown

Parameters (override on command line with --ros-args -p name:=value):
  image_topic   (string, default: /camera/color/image_raw)
  info_topic    (string, default: /camera/color/camera_info)
  squares_x     (int,    default: 7)
  squares_y     (int,    default: 5)
  square_size   (float,  default: 0.040)   metres
  marker_size   (float,  default: 0.030)   metres
"""

import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math


# ── Overlay colours (BGR) ────────────────────────────────────────────────────
COL_GREEN   = (0,   220, 60)
COL_RED     = (0,   60,  220)
COL_YELLOW  = (0,   220, 220)
COL_WHITE   = (255, 255, 255)
COL_BLACK   = (0,   0,   0)
COL_BLUE    = (220, 80,  0)
COL_ORANGE  = (0,   160, 255)

FONT        = cv2.FONT_HERSHEY_SIMPLEX


def _text(img, text, pos, scale=0.55, colour=COL_WHITE, thickness=1):
    """Draw text with a thin black shadow for readability on any background."""
    cv2.putText(img, text, (pos[0]+1, pos[1]+1),
                FONT, scale, COL_BLACK, thickness + 1, cv2.LINE_AA)
    cv2.putText(img, text, pos,
                FONT, scale, colour, thickness, cv2.LINE_AA)


def _hud_row(img, label, value, y, label_col=COL_YELLOW, value_col=COL_WHITE):
    _text(img, label, (12, y), colour=label_col)
    _text(img, value, (200, y), colour=value_col)
    return y + 22


class CharucoDebugViewer(Node):

    def __init__(self):
        super().__init__('charuco_debug_viewer')

        # ── Parameters — override without editing code ──────────────────────
        self.declare_parameter('image_topic',  '/camera/color/image_raw')
        self.declare_parameter('info_topic',   '/camera/color/camera_info')
        self.declare_parameter('squares_x',    7)
        self.declare_parameter('squares_y',    5)
        self.declare_parameter('square_size',  0.040)
        self.declare_parameter('marker_size',  0.030)

        image_topic  = self.get_parameter('image_topic').value
        info_topic   = self.get_parameter('info_topic').value
        self.SQUARES_X   = self.get_parameter('squares_x').value
        self.SQUARES_Y   = self.get_parameter('squares_y').value
        self.SQUARE_SIZE = self.get_parameter('square_size').value
        self.MARKER_SIZE = self.get_parameter('marker_size').value

        # ── ChArUco board ───────────────────────────────────────────────────
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.board = aruco.CharucoBoard_create(
            self.SQUARES_X,
            self.SQUARES_Y,
            self.SQUARE_SIZE,
            self.MARKER_SIZE,
            self.dictionary
        )
        self.detector_params = aruco.DetectorParameters_create()

        # ── State ───────────────────────────────────────────────────────────
        self.bridge         = CvBridge()
        self.camera_matrix  = None
        self.dist_coeffs    = None
        self.image_topic    = image_topic
        self.tf_publishing  = False
        self.last_tvec      = None
        self.last_num_markers   = 0
        self.last_num_corners   = 0
        self.last_status_msg    = 'Waiting for camera...'
        self.last_status_colour = COL_YELLOW

        # ── TF broadcaster ──────────────────────────────────────────────────
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Publishers ──────────────────────────────────────────────────────
        self.debug_pub = self.create_publisher(Image, '/charuco_debug/image', 10)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(CameraInfo, info_topic,  self.info_cb,  10)
        self.create_subscription(Image,      image_topic, self.image_cb, 10)

        self.get_logger().info(f'ChArUco debug viewer started')
        self.get_logger().info(f'  Image topic : {image_topic}')
        self.get_logger().info(f'  Info  topic : {info_topic}')
        self.get_logger().info(
            f'  Board       : {self.SQUARES_X}x{self.SQUARES_Y} squares, '
            f'{self.SQUARE_SIZE*1000:.0f}mm sq / {self.MARKER_SIZE*1000:.0f}mm marker'
        )
        self.get_logger().info(
            'Opening display window — press Q in the window to quit'
        )

    # ── Camera intrinsics ────────────────────────────────────────────────────
    def info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs   = np.array(msg.d)
            self.get_logger().info('Camera intrinsics received')

    # ── Main detection loop ─────────────────────────────────────────────────
    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        display = frame.copy()
        h, w = display.shape[:2]

        if self.camera_matrix is None:
            _text(display, 'Waiting for camera_info...', (12, 40),
                  colour=COL_ORANGE, scale=0.7)
            self._show_and_publish(display, msg)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ── Step 1: detect raw ArUco markers ────────────────────────────────
        marker_corners, marker_ids, rejected = aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_params
        )

        num_markers = len(marker_ids) if marker_ids is not None else 0
        self.last_num_markers = num_markers

        # Draw all detected marker outlines + IDs
        if marker_ids is not None:
            aruco.drawDetectedMarkers(display, marker_corners, marker_ids)

        # Draw rejected candidates in grey (helps diagnose poor lighting/angle)
        if rejected:
            aruco.drawDetectedMarkers(display, rejected, borderColor=(120, 120, 120))

        # ── Step 2: refine to ChArUco corners ───────────────────────────────
        num_corners = 0
        charuco_corners = None
        charuco_ids     = None

        if marker_ids is not None and num_markers >= 4:
            num_corners, charuco_corners, charuco_ids = \
                aruco.interpolateCornersCharuco(
                    marker_corners, marker_ids, gray, self.board,
                    cameraMatrix=self.camera_matrix,
                    distCoeffs=self.dist_coeffs
                )

            if charuco_corners is not None and num_corners > 0:
                # Draw ChArUco corners as coloured dots with IDs
                aruco.drawDetectedCornersCharuco(
                    display, charuco_corners, charuco_ids,
                    cornerColor=(0, 255, 128)
                )

        self.last_num_corners = num_corners

        # ── Step 3: estimate board pose ──────────────────────────────────────
        self.tf_publishing  = False
        self.last_tvec      = None

        if charuco_ids is not None and num_corners >= 4:
            success, rvec, tvec = aruco.estimatePoseCharucoBoard(
                charuco_corners, charuco_ids, self.board,
                self.camera_matrix, self.dist_coeffs, None, None
            )

            if success:
                # Draw 3D coordinate axes on the board
                # X = red, Y = green, Z = blue (pointing out of board)
                cv2.drawFrameAxes(
                    display,
                    self.camera_matrix, self.dist_coeffs,
                    rvec, tvec,
                    self.SQUARE_SIZE * 2   # axis length = 2 squares
                )

                self.last_tvec     = tvec.flatten()
                self.tf_publishing = True

                # Publish TF
                self._publish_tf(msg.header.stamp, tvec, rvec)

                self.last_status_msg    = 'Board detected — TF publishing'
                self.last_status_colour = COL_GREEN
            else:
                self.last_status_msg    = 'Corners found but pose failed'
                self.last_status_colour = COL_ORANGE
        elif num_markers >= 1:
            needed = max(0, 4 - num_markers)
            self.last_status_msg    = f'{num_markers} markers found — need {needed} more'
            self.last_status_colour = COL_ORANGE
        else:
            self.last_status_msg    = 'No markers detected'
            self.last_status_colour = COL_RED

        # ── HUD overlay ──────────────────────────────────────────────────────
        self._draw_hud(display, h)

        self._show_and_publish(display, msg)

    def _draw_hud(self, display, h):
        # Semi-transparent dark panel behind HUD text
        overlay = display.copy()
        cv2.rectangle(overlay, (6, 6), (420, 170), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.55, display, 0.45, 0, display)

        y = 26
        y = _hud_row(display, 'Topic:',
                     self.image_topic, y)
        y = _hud_row(display, 'ArUco markers:',
                     f'{self.last_num_markers}  (need ≥4 for ChArUco)', y)
        y = _hud_row(display, 'ChArUco corners:',
                     f'{self.last_num_corners}  (need ≥4 for pose)', y)

        if self.last_tvec is not None:
            tx, ty, tz = self.last_tvec
            y = _hud_row(display, 'Board pose:',
                         f'x:{tx:+.3f}  y:{ty:+.3f}  z:{tz:+.3f} m', y)
        else:
            y = _hud_row(display, 'Board pose:', '—', y)

        tf_col = COL_GREEN if self.tf_publishing else COL_RED
        tf_str = 'YES  (camera→marker)' if self.tf_publishing else 'NO'
        y = _hud_row(display, 'TF publishing:', tf_str, y,
                     value_col=tf_col)

        # Big status bar at bottom of HUD
        cv2.rectangle(display, (6, y), (420, y + 26), (20, 20, 20), -1)
        _text(display, self.last_status_msg, (12, y + 18),
              colour=self.last_status_colour, scale=0.6)

        # Tip at bottom of frame
        _text(display, 'Press Q to quit | rqt_image_view /charuco_debug/image',
              (12, h - 12), scale=0.45, colour=(180, 180, 180))

    def _publish_tf(self, stamp, tvec, rvec):
        rot_mat, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = self._rot_to_quat(rot_mat)

        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = 'camera_color_optical_frame'
        t.child_frame_id  = 'tracking_marker_frame'
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def _show_and_publish(self, display, msg):
        # Publish annotated image for rqt_image_view / RViz
        self.debug_pub.publish(
            self.bridge.cv2_to_imgmsg(display, encoding='bgr8')
        )

        # Show local window (resize to fit screen if image is large)
        h, w = display.shape[:2]
        max_w = 1280
        if w > max_w:
            scale = max_w / w
            display = cv2.resize(display, (max_w, int(h * scale)))

        cv2.imshow('ChArUco Debug Viewer', display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:   # Q or Escape
            self.get_logger().info('Quit requested — shutting down')
            rclpy.shutdown()

    def _rot_to_quat(self, R):
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return x, y, z, w

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = CharucoDebugViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()


if __name__ == '__main__':
    main()