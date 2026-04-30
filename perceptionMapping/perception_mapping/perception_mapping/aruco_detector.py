import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # OpenCV 4.5.x API — no ArucoDetector class, use module-level functions
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_length = 0.05  # 5cm — match your printed marker size

        # Subscribe to colour image and camera intrinsics
        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        # Publisher for annotated debug image
        self.debug_pub = self.create_publisher(
            Image, '/aruco/debug_image', 10
        )

        # Pose publishers created dynamically per detected marker ID
        self.pose_publishers = {}

        self.get_logger().info('ArUco detector node started (OpenCV 4.5.x)')

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera intrinsics received')

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return  # wait until intrinsics arrive

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # OpenCV 4.5.x detection API — module-level function, not a class method
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):

                # Estimate 3D pose for this marker
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [corners[i]],
                    self.marker_length,
                    self.camera_matrix,
                    self.dist_coeffs
                )

                t = tvec[0][0]
                self.get_logger().info(
                    f'Marker ID {marker_id}: '
                    f'x={t[0]:.3f}m  y={t[1]:.3f}m  z={t[2]:.3f}m'
                )

                # Create publisher for this marker ID if it doesn't exist yet
                topic = f'/aruco/marker_{marker_id}/pose'
                if marker_id not in self.pose_publishers:
                    self.pose_publishers[marker_id] = \
                        self.create_publisher(PoseStamped, topic, 10)
                    self.get_logger().info(f'Publishing poses on {topic}')

                # Build and publish PoseStamped message
                pose_msg = PoseStamped()
                pose_msg.header = msg.header
                pose_msg.pose.position.x = float(t[0])
                pose_msg.pose.position.y = float(t[1])
                pose_msg.pose.position.z = float(t[2])
                pose_msg.pose.orientation = \
                    self.rotation_matrix_to_quaternion(rvec[0][0])

                self.pose_publishers[marker_id].publish(pose_msg)

            # Draw detected markers and axes on the debug image
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                cv2.aruco.drawAxis(
                    frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec[i], tvec[i],
                    0.03
                )

        self.debug_pub.publish(
            self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        )

    def rotation_matrix_to_quaternion(self, rvec):
        from geometry_msgs.msg import Quaternion
        import math

        # Convert rotation vector to rotation matrix
        rot_matrix, _ = cv2.Rodrigues(rvec)
        R = rot_matrix

        trace = R[0, 0] + R[1, 1] + R[2, 2]
        q = Quaternion()

        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            q.w = 0.25 / s
            q.x = (R[2, 1] - R[1, 2]) * s
            q.y = (R[0, 2] - R[2, 0]) * s
            q.z = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                q.w = (R[2, 1] - R[1, 2]) / s
                q.x = 0.25 * s
                q.y = (R[0, 1] + R[1, 0]) / s
                q.z = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                q.w = (R[0, 2] - R[2, 0]) / s
                q.x = (R[0, 1] + R[1, 0]) / s
                q.y = 0.25 * s
                q.z = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                q.w = (R[1, 0] - R[0, 1]) / s
                q.x = (R[0, 2] + R[2, 0]) / s
                q.y = (R[1, 2] + R[2, 1]) / s
                q.z = 0.25 * s

        return q


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()