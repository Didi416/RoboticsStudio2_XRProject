#!/usr/bin/env python3
"""
aruco_detector_node.py
──────────────────────
ROS2 node that:
  1. Reads frames from a webcam (simulating the Intel RealSense D435i).
  2. Detects the 4 ArUco corner markers fixed to the puzzle wall.
  3. Estimates the pose of each marker relative to the camera.
  4. Fuses the 4 corner detections into a single puzzle-wall centre pose.
  5. Applies an exponential moving average (EMA) filter.
  6. Publishes:
       /perception/puzzle_wall_pose     (geometry_msgs/PoseStamped, camera frame)
       /perception/detected_markers     (visualization_msgs/MarkerArray, for RViz)
       /perception/detection_status     (std_msgs/String, JSON heartbeat)
       /perception/image_debug          (sensor_msgs/Image, annotated frame)
  7. Broadcasts a dynamic TF:  camera_optical_frame → puzzle_wall

Run:
    ros2 run perception_mapping aruco_detector_node
"""

import os
import json
import time
import rclpy
import cv2
import numpy as np
import yaml

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R


# ─────────────────────────────────────────────────────────────────────────────
# Helper utilities
# ─────────────────────────────────────────────────────────────────────────────

def rvec_tvec_to_matrix(rvec, tvec):
    """Convert OpenCV rvec/tvec to 4x4 homogeneous transform matrix."""
    rot_mat, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = rot_mat
    T[:3, 3]  = tvec.flatten()
    return T


def matrix_to_pose(T, frame_id, stamp, node):
    """Convert 4x4 transform to geometry_msgs/PoseStamped."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp    = stamp

    pose.pose.position.x = float(T[0, 3])
    pose.pose.position.y = float(T[1, 3])
    pose.pose.position.z = float(T[2, 3])

    rot = R.from_matrix(T[:3, :3])
    q   = rot.as_quat()   # [x, y, z, w]
    pose.pose.orientation.x = float(q[0])
    pose.pose.orientation.y = float(q[1])
    pose.pose.orientation.z = float(q[2])
    pose.pose.orientation.w = float(q[3])
    return pose


def matrix_to_transform_stamped(T, parent_frame, child_frame, stamp):
    ts = TransformStamped()
    ts.header.stamp    = stamp
    ts.header.frame_id = parent_frame
    ts.child_frame_id  = child_frame

    ts.transform.translation.x = float(T[0, 3])
    ts.transform.translation.y = float(T[1, 3])
    ts.transform.translation.z = float(T[2, 3])

    rot = R.from_matrix(T[:3, :3])
    q   = rot.as_quat()
    ts.transform.rotation.x = float(q[0])
    ts.transform.rotation.y = float(q[1])
    ts.transform.rotation.z = float(q[2])
    ts.transform.rotation.w = float(q[3])
    return ts


def ema_pose(prev_T, new_T, alpha):
    """Exponential moving average on a 4x4 pose matrix (position + slerp rotation)."""
    if prev_T is None:
        return new_T
    # Position EMA
    pos_new = new_T[:3, 3]
    pos_prev = prev_T[:3, 3]
    pos_filt = alpha * pos_new + (1.0 - alpha) * pos_prev

    # Rotation SLERP
    rot_prev = R.from_matrix(prev_T[:3, :3])
    rot_new  = R.from_matrix(new_T[:3, :3])
    rot_filt = R.slerp_single(rot_prev, rot_new, alpha) if hasattr(R, "slerp_single") else \
               rot_prev.slerp(rot_new, alpha)  # older scipy API

    T_filt = np.eye(4)
    T_filt[:3, :3] = rot_filt.as_matrix()
    T_filt[:3, 3]  = pos_filt
    return T_filt


# ─────────────────────────────────────────────────────────────────────────────
# Main node
# ─────────────────────────────────────────────────────────────────────────────

class ArucoDetectorNode(Node):

    def __init__(self):
        super().__init__("aruco_detector_node")
        self.get_logger().info("[ArucoDetector] Initialising...")

        # ── Load config ───────────────────────────────────────────────────
        config_path = os.path.join(os.path.dirname(__file__), "config", "aruco_config.yaml")
        self.declare_parameter("config_path", config_path)
        config_path = self.get_parameter("config_path").get_parameter_value().string_value

        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f)

        cam_cfg    = cfg["camera"]
        aruco_cfg  = cfg["aruco"]
        wall_cfg   = cfg["puzzle_wall"]
        filter_cfg = cfg["filter"]
        ros_cfg    = cfg["ros"]

        # ── Camera ────────────────────────────────────────────────────────
        self._cam = cv2.VideoCapture(cam_cfg["device_index"])
        self._cam.set(cv2.CAP_PROP_FRAME_WIDTH,  cam_cfg["width"])
        self._cam.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_cfg["height"])
        self._cam.set(cv2.CAP_PROP_FPS,          cam_cfg["fps"])

        K = np.array(cam_cfg["camera_matrix"], dtype=np.float64)
        D = np.array(cam_cfg["dist_coeffs"],   dtype=np.float64)
        self._camera_matrix  = K
        self._dist_coeffs    = D

        # ── ArUco ─────────────────────────────────────────────────────────
        dict_map = {
            "DICT_4X4_50":   cv2.aruco.DICT_4X4_50,
            "DICT_5X5_100":  cv2.aruco.DICT_5X5_100,
            "DICT_6X6_250":  cv2.aruco.DICT_6X6_250,
        }
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_map[aruco_cfg["dictionary"]])
        self._aruco_params = cv2.aruco.DetectorParameters()
        self._aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, self._aruco_params)
        self._marker_size   = aruco_cfg["marker_size_m"]

        self._corner_ids = {
            "top_left":     aruco_cfg["corner_ids"]["top_left"],
            "top_right":    aruco_cfg["corner_ids"]["top_right"],
            "bottom_right": aruco_cfg["corner_ids"]["bottom_right"],
            "bottom_left":  aruco_cfg["corner_ids"]["bottom_left"],
        }
        self._id_set = set(self._corner_ids.values())

        # ── Puzzle wall geometry (for pose fusion from 4 corners) ─────────
        W = wall_cfg["width_m"]
        H = wall_cfg["height_m"]
        m = wall_cfg["marker_margin_m"]
        # 3D positions of marker centres in the puzzle wall frame
        # Origin = centre of wall, +X right, +Y up, +Z toward camera
        self._wall_marker_positions = {
            self._corner_ids["top_left"]:     np.array([-W/2 + m,  H/2 - m, 0.0]),
            self._corner_ids["top_right"]:    np.array([ W/2 - m,  H/2 - m, 0.0]),
            self._corner_ids["bottom_right"]: np.array([ W/2 - m, -H/2 + m, 0.0]),
            self._corner_ids["bottom_left"]:  np.array([-W/2 + m, -H/2 + m, 0.0]),
        }

        # ── Frames ────────────────────────────────────────────────────────
        self._camera_frame      = ros_cfg["camera_frame"]
        self._base_frame        = ros_cfg["base_frame"]
        self._puzzle_wall_frame = ros_cfg["puzzle_wall_frame"]

        # ── Filter ────────────────────────────────────────────────────────
        self._ema_alpha        = filter_cfg["pose_ema_alpha"]
        self._lost_thresh      = filter_cfg["lost_detection_frames"]
        self._filtered_T       = None   # last filtered 4x4 transform
        self._frames_since_det = 0

        # ── ROS publishers ────────────────────────────────────────────────
        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub_pose   = self.create_publisher(PoseStamped, ros_cfg["topic_puzzle_wall_pose"], 10)
        self._pub_rviz   = self.create_publisher(MarkerArray, ros_cfg["topic_marker_array"],     10)
        self._pub_status = self.create_publisher(String,      ros_cfg["topic_detection_status"], 10)
        self._pub_debug  = self.create_publisher(Image,       ros_cfg["topic_camera_debug"],     best_effort_qos)

        # ── TF ───────────────────────────────────────────────────────────
        self._tf_broadcaster = TransformBroadcaster(self)

        # ── CvBridge ─────────────────────────────────────────────────────
        self._bridge = CvBridge()

        # ── Timer: 30 Hz ─────────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / 30.0, self._process_frame)

        self.get_logger().info("[ArucoDetector] Ready. Streaming from camera...")

    # ─────────────────────────────────────────────────────────────────────
    def _process_frame(self):
        ret, frame = self._cam.read()
        if not ret:
            self.get_logger().warn("[ArucoDetector] Failed to read camera frame.")
            return

        stamp = self.get_clock().now().to_msg()
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        debug_frame = frame.copy()

        # ── Detect markers ────────────────────────────────────────────────
        corners_list, ids, rejected = self._aruco_detector.detectMarkers(gray)

        detected_ids = set()
        marker_transforms = {}   # id → 4x4 T_camera_marker

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(debug_frame, corners_list, ids)
            flat_ids = ids.flatten()

            for i, mid in enumerate(flat_ids):
                if mid not in self._id_set:
                    continue
                detected_ids.add(int(mid))

                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [corners_list[i]], self._marker_size,
                    self._camera_matrix, self._dist_coeffs
                )
                T = rvec_tvec_to_matrix(rvecs[0], tvecs[0])
                marker_transforms[int(mid)] = T

                # Draw axis on debug image
                cv2.drawFrameAxes(
                    debug_frame,
                    self._camera_matrix, self._dist_coeffs,
                    rvecs[0], tvecs[0], self._marker_size * 0.5
                )
                cx = int(corners_list[i][0][:, 0].mean())
                cy = int(corners_list[i][0][:, 1].mean())
                cv2.putText(debug_frame, f"ID {mid}", (cx, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # ── Fuse detections into puzzle wall pose ─────────────────────────
        puzzle_wall_T = None
        if len(marker_transforms) >= 2:
            puzzle_wall_T = self._fuse_markers_to_wall_pose(marker_transforms)

        if puzzle_wall_T is not None:
            self._frames_since_det = 0
            self._filtered_T = ema_pose(self._filtered_T, puzzle_wall_T, self._ema_alpha)
        else:
            self._frames_since_det += 1

        # ── Publish ───────────────────────────────────────────────────────
        if self._filtered_T is not None:
            pose_msg = matrix_to_pose(self._filtered_T, self._camera_frame, stamp, self)
            self._pub_pose.publish(pose_msg)

            tf_msg = matrix_to_transform_stamped(
                self._filtered_T, self._camera_frame, self._puzzle_wall_frame, stamp
            )
            self._tf_broadcaster.sendTransform(tf_msg)

            self._publish_rviz_markers(marker_transforms, stamp)

        status = {
            "stamp":         str(stamp),
            "detected_ids":  sorted(list(detected_ids)),
            "required_ids":  sorted(list(self._id_set)),
            "fused":         puzzle_wall_T is not None,
            "filtered":      self._filtered_T is not None,
            "lost_frames":   self._frames_since_det,
            "status":        "LOST" if self._frames_since_det > self._lost_thresh else (
                             "PARTIAL" if len(detected_ids) < len(self._id_set) else "OK"
                             ),
        }
        self._pub_status.publish(String(data=json.dumps(status)))

        # ── Publish annotated debug image ─────────────────────────────────
        # Status overlay
        status_text = f"IDs: {sorted(detected_ids)} | {status['status']}"
        cv2.putText(debug_frame, status_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
        try:
            self._pub_debug.publish(self._bridge.cv2_to_imgmsg(debug_frame, "bgr8"))
        except Exception as e:
            pass   # non-critical

    # ─────────────────────────────────────────────────────────────────────
    def _fuse_markers_to_wall_pose(self, marker_transforms):
        """
        Given N detected marker transforms (T_camera_marker), compute
        T_camera_puzzlewall (puzzle wall centre pose in camera frame).

        Strategy: for each detected marker, compute what T_camera_wall would be
        using  T_camera_wall = T_camera_marker * T_marker_wall
        where T_marker_wall is derived from the known layout.
        Then average the position estimates and slerp the rotations.
        """
        wall_poses = []

        for mid, T_cam_marker in marker_transforms.items():
            if mid not in self._wall_marker_positions:
                continue
            # Position of this marker in the wall frame
            p_wall = self._wall_marker_positions[mid]

            # T_marker_wall is a translation-only offset (markers are flat on wall)
            T_marker_wall = np.eye(4)
            T_marker_wall[:3, 3] = -p_wall  # wall origin relative to marker

            T_cam_wall = T_cam_marker @ T_marker_wall
            wall_poses.append(T_cam_wall)

        if not wall_poses:
            return None

        # Average positions
        pos_mean = np.mean([T[:3, 3] for T in wall_poses], axis=0)

        # Average rotations via quaternion averaging
        quats = []
        for T in wall_poses:
            q = R.from_matrix(T[:3, :3]).as_quat()
            quats.append(q)
        quats = np.array(quats)
        # Ensure quaternion sign consistency (dot product with first)
        ref = quats[0]
        for i in range(1, len(quats)):
            if np.dot(ref, quats[i]) < 0:
                quats[i] = -quats[i]
        q_mean = quats.mean(axis=0)
        q_mean /= np.linalg.norm(q_mean)

        T_fused = np.eye(4)
        T_fused[:3, :3] = R.from_quat(q_mean).as_matrix()
        T_fused[:3, 3]  = pos_mean
        return T_fused

    # ─────────────────────────────────────────────────────────────────────
    def _publish_rviz_markers(self, marker_transforms, stamp):
        """Publish small sphere markers in RViz for each detected ArUco."""
        arr = MarkerArray()
        for mid, T in marker_transforms.items():
            m = Marker()
            m.header.frame_id = self._camera_frame
            m.header.stamp    = stamp
            m.ns              = "aruco_markers"
            m.id              = int(mid)
            m.type            = Marker.SPHERE
            m.action          = Marker.ADD
            m.pose.position.x = float(T[0, 3])
            m.pose.position.y = float(T[1, 3])
            m.pose.position.z = float(T[2, 3])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.02
            m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.5; m.color.a = 1.0
            arr.markers.append(m)
        self._pub_rviz.publish(arr)

    # ─────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._cam.release()
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()