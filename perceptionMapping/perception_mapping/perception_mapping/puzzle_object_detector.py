#!/usr/bin/env python3
# escapeXR_perception/puzzle_object_detector.py

import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs


class PuzzleObjectDetector(Node):
    """
    Detects ArUco-tagged puzzle objects and publishes their poses
    in the robot base frame using the saved hand-eye calibration.
    
    Each puzzle object gets a unique ArUco ID:
      ID 1 = button panel
      ID 2 = dial
      ID 3 = frog egg rack
      etc.
    """

    PUZZLE_OBJECTS = {
        1: 'button_panel',
        2: 'containment_dial',
        3: 'egg_rack',
        4: 'lily_pad_maze',
    }

    def __init__(self):
        super().__init__('puzzle_object_detector')

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_size = 0.05  # metres — size of puzzle object markers

        # TF listener — uses the published calibration automatically
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers — one topic per puzzle object
        self.pose_publishers = {
            obj_name: self.create_publisher(
                PoseStamped, f'/puzzle/{obj_name}/pose', 10
            )
            for obj_name in self.PUZZLE_OBJECTS.values()
        }

        # Subscribers
        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 self.camera_info_cb, 10)
        self.create_subscription(Image, '/camera/color/image_raw',
                                 self.image_cb, 10)

        # ArUco detector setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.detector_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.detector_params)

        self.get_logger().info('Puzzle object detector running')

    def camera_info_cb(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def image_cb(self, msg):
        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            return

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id not in self.PUZZLE_OBJECTS:
                continue

            # Estimate pose of this marker in camera frame
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                corners[i:i+1], self.marker_size,
                self.camera_matrix, self.dist_coeffs
            )

            # Build PoseStamped in camera frame
            pose_in_camera = PoseStamped()
            pose_in_camera.header.stamp = msg.header.stamp
            pose_in_camera.header.frame_id = 'camera_color_optical_frame'   
            pose_in_camera.pose.position.x = float(tvec[0][0][0])
            pose_in_camera.pose.position.y = float(tvec[0][0][1])
            pose_in_camera.pose.position.z = float(tvec[0][0][2])

            # Convert rotation vector to quaternion
            rot_mat, _ = cv2.Rodrigues(rvec[0][0])
            pose_in_camera.pose.orientation = self._rot_to_quat(rot_mat)

            # Transform to robot base frame using the hand-eye calibration TF
            try:
                pose_in_base = self.tf_buffer.transform(
                    pose_in_camera, 'base_link',
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                obj_name = self.PUZZLE_OBJECTS[marker_id]
                self.pose_publishers[obj_name].publish(pose_in_base)

            except Exception as e:
                self.get_logger().warn(
                    f'Could not transform marker {marker_id}: {e}'
                )

    def _rot_to_quat(self, R):
        from geometry_msgs.msg import Quaternion
        import math
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
        q = Quaternion()
        q.x, q.y, q.z, q.w = x, y, z, w
        return q


def main():
    rclpy.init()
    node = PuzzleObjectDetector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()