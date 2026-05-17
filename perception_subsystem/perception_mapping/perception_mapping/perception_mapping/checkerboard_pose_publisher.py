#!/usr/bin/env python3
"""
Detects a checkerboard attached to the UR3e end-effector and publishes
its pose as a TF frame. Used during hand-eye calibration with easy_handeye2.

Board spec: 7x9 squares, 20mm per square
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import tf2_ros
from geometry_msgs.msg import TransformStamped


class CheckerboardPosePublisher(Node):

    # Your printed board measurements
    SQUARES_X = 7
    SQUARES_Y = 9
    SQUARE_SIZE = 0.020          # metres — 20mm
    CORNERS_X = SQUARES_X - 1   # 6 interior corners
    CORNERS_Y = SQUARES_Y - 1   # 8 interior corners

    def __init__(self):
        super().__init__('checkerboard_pose_publisher')

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # TF broadcaster — publishes checkerboard pose for easy_handeye2
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self.camera_info_cb, 10)
        self.create_subscription(
            Image, '/camera/color/image_raw',
            self.image_cb, 10)

        # Checkerboard corner pattern (3D object points)
        self.objp = np.zeros(
            (self.CORNERS_X * self.CORNERS_Y, 3), np.float32)
        self.objp[:, :2] = np.mgrid[
            0:self.CORNERS_X, 0:self.CORNERS_Y].T.reshape(-1, 2)
        self.objp *= self.SQUARE_SIZE

        self.get_logger().info(
            f'Checkerboard detector ready: '
            f'{self.CORNERS_X}x{self.CORNERS_Y} interior corners, '
            f'{self.SQUARE_SIZE*1000:.0f}mm squares'
        )

    def camera_info_cb(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def image_cb(self, msg):
        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(
            gray,
            (self.CORNERS_X, self.CORNERS_Y),
            cv2.CALIB_CB_ADAPTIVE_THRESH +
            cv2.CALIB_CB_NORMALIZE_IMAGE +
            cv2.CALIB_CB_FAST_CHECK
        )

        if not ret:
            self.get_logger().debug('Checkerboard not detected', throttle_duration_sec=2.0)
            return

        # Refine corner locations to sub-pixel accuracy
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), criteria)

        # Estimate pose
        success, rvec, tvec = cv2.solvePnP(
            self.objp, corners_refined,
            self.camera_matrix, self.dist_coeffs
        )

        if not success:
            return

        # Convert rotation vector to matrix then to quaternion
        rot_mat, _ = cv2.Rodrigues(rvec)
        quat = self._rot_to_quat(rot_mat)

        # Publish as TF — easy_handeye2 reads this frame
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'camera_color_optical_frame'
        t.child_frame_id = 'tracking_marker_frame'   # must match calibrate.launch.py
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(
            f'Board detected — '
            f'x:{tvec[0]:.3f} y:{tvec[1]:.3f} z:{tvec[2]:.3f}',
            throttle_duration_sec=1.0
        )

    def _rot_to_quat(self, R):
        import math
        trace = R[0,0] + R[1,1] + R[2,2]
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2,1] - R[1,2]) * s
            y = (R[0,2] - R[2,0]) * s
            z = (R[1,0] - R[0,1]) * s
        elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
            s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
            w = (R[2,1] - R[1,2]) / s
            x = 0.25 * s
            y = (R[0,1] + R[1,0]) / s
            z = (R[0,2] + R[2,0]) / s
        elif R[1,1] > R[2,2]:
            s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
            w = (R[0,2] - R[2,0]) / s
            x = (R[0,1] + R[1,0]) / s
            y = 0.25 * s
            z = (R[1,2] + R[2,1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
            w = (R[1,0] - R[0,1]) / s
            x = (R[0,2] + R[2,0]) / s
            y = (R[1,2] + R[2,1]) / s
            z = 0.25 * s
        return [x, y, z, w]


def main():
    rclpy.init()
    node = CheckerboardPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()