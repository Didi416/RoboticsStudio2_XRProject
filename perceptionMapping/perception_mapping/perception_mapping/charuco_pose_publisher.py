#!/usr/bin/env python3
"""
Detects a ChArUco board and publishes its pose as a TF frame
for easy_handeye2 calibration (eye-on-base OR eye-in-hand).

Board: 7x5 squares, 40mm square size, 30mm marker size, DICT_4X4_50

BUGS FIXED vs the version in src/:
  Bug 1: SQUARES_X / SQUARES_Y / SQUARE_SIZE / MARKER_SIZE were declared
         as plain local variables (SQUARES_X = 7) but then referenced as
         self.SQUARES_X in the logger call — AttributeError on startup,
         node crashes before subscribing to any topic.
         Fix: all board params are now self.* instance attributes.

  Bug 2: self.charuco_detector.detectBoard(gray) — self.charuco_detector
         was never created anywhere. CharucoDetector class only exists in
         OpenCV 4.7+; ROS2 Humble ships OpenCV 4.5.x.
         Fix: use the correct two-step OpenCV 4.5.x API:
           Step 1 — aruco.detectMarkers(...)
           Step 2 — aruco.interpolateCornersCharuco(...)
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


class CharucoPosePublisher(Node):

    def __init__(self):
        super().__init__('charuco_pose_publisher')

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs   = None

        # FIX 1: all board params as self.* so they are accessible everywhere
        self.dictionary  = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.SQUARES_X   = 7
        self.SQUARES_Y   = 5
        self.SQUARE_SIZE = 0.040   # metres — measure your actual printout
        self.MARKER_SIZE = 0.030   # metres — must be < SQUARE_SIZE

        self.board = aruco.CharucoBoard_create(
            self.SQUARES_X,
            self.SQUARES_Y,
            self.SQUARE_SIZE,
            self.MARKER_SIZE,
            self.dictionary
        )

        # FIX 2: DetectorParameters_create() is the correct OpenCV 4.5.x API
        # Do NOT create a CharucoDetector — that class does not exist here
        self.detector_params = aruco.DetectorParameters_create()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self.camera_info_cb, 10)
        self.create_subscription(
            Image, '/camera/color/image_raw',
            self.image_cb, 10)

        # FIX 1: self.SQUARES_X etc. now exist so this line no longer crashes
        self.get_logger().info(
            f'ChArUco pose publisher ready: '
            f'{self.SQUARES_X}x{self.SQUARES_Y} squares, '
            f'{self.SQUARE_SIZE * 1000:.0f}mm square / '
            f'{self.MARKER_SIZE * 1000:.0f}mm marker, DICT_4X4_50'
        )
        self.get_logger().info(
            'Waiting for /camera/color/camera_info and /camera/color/image_raw ...'
        )

    def camera_info_cb(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs   = np.array(msg.d)

    def image_cb(self, msg):
        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # FIX 2: two-step detection — the only correct API on OpenCV 4.5.x
        # Step 1: detect the raw ArUco markers inside the ChArUco board
        marker_corners, marker_ids, _ = aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.detector_params
        )

        if marker_ids is None or len(marker_ids) < 4:
            self.get_logger().debug(
                'Not enough ArUco markers detected (need ≥4)',
                throttle_duration_sec=2.0
            )
            return

        # Step 2: refine to sub-pixel ChArUco corners using the board geometry
        num_corners, charuco_corners, charuco_ids = \
            aruco.interpolateCornersCharuco(
                marker_corners,
                marker_ids,
                gray,
                self.board,
                cameraMatrix=self.camera_matrix,
                distCoeffs=self.dist_coeffs
            )

        if charuco_ids is None or num_corners < 4:
            self.get_logger().debug(
                'Not enough ChArUco corners found (need ≥4)',
                throttle_duration_sec=2.0
            )
            return

        # Estimate 6-DOF board pose in camera frame
        success, rvec, tvec = aruco.estimatePoseCharucoBoard(
            charuco_corners,
            charuco_ids,
            self.board,
            self.camera_matrix,
            self.dist_coeffs,
            None,   # rvec initial guess
            None    # tvec initial guess
        )

        if not success:
            self.get_logger().warn('Pose estimation failed', throttle_duration_sec=2.0)
            return

        rot_mat, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = self._rot_to_quat(rot_mat)

        # Publish TF: camera_color_optical_frame → tracking_marker_frame
        t = TransformStamped()
        t.header.stamp    = msg.header.stamp
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

        self.get_logger().info(
            f'Board detected ({num_corners} corners) — '
            f'x:{float(tvec[0]):.3f} '
            f'y:{float(tvec[1]):.3f} '
            f'z:{float(tvec[2]):.3f}',
            throttle_duration_sec=1.0
        )

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


def main():
    rclpy.init()
    node = CharucoPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()