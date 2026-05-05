#!/usr/bin/env python3
"""
multi_aruco_detector.py

Detects multiple ArUco markers simultaneously and publishes a PoseStamped
for each detected marker ID, plus a combined MarkerArray for visualisation.

6-DOF pose uses the RealSense aligned depth channel for accurate Z measurement.
X and Y are recomputed from measured Z using camera intrinsics, giving true
3D position rather than mathematically estimated depth.

Puzzle ID assignment (Frog Containment Lab):
  ID 0  — Frog Call Console   (button matrix)
  ID 1  — Lily Pad Maze       (slider maze)
  ID 2  — Frog Egg Sorting    (pick & place rack)
  ID 3  — Containment Dial    (rotating lock)
  ID 10 — Hand-eye calibration target

Subscriptions:
  /camera/color/image_raw                        (sensor_msgs/Image)
  /camera/color/camera_info                      (sensor_msgs/CameraInfo)
  /camera/aligned_depth_to_color/image_raw       (sensor_msgs/Image)  ← NEW

Publications:
  /aruco/marker_<ID>/pose      (geometry_msgs/PoseStamped)
  /aruco/markers               (visualization_msgs/MarkerArray)
  /aruco/debug_image           (sensor_msgs/Image)

When running with the physical RealSense, remap topics:
  ros2 run perception_mapping multi_aruco_detector --ros-args
    -r /camera/color/image_raw:=/camera/camera/color/image_raw
    -r /camera/color/camera_info:=/camera/camera/color/camera_info
    -r /camera/aligned_depth_to_color/image_raw:=/camera/camera/aligned_depth_to_color/image_raw
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from scipy.spatial.transform import Rotation


# ── Puzzle marker registry ────────────────────────────────────────────────────
PUZZLE_MARKERS = {
    0:  'frog_call_console',
    1:  'lily_pad_maze',
    2:  'frog_egg_sorting',
    3:  'containment_dial',
    10: 'calibration_target',
}

# Physical marker size in metres — must match your printed markers.
MARKER_SIZE_M = 0.05

# Exponential moving average smoothing factor (0 = no update, 1 = no smoothing)
EMA_ALPHA = 0.4

# Frames a marker must be absent before flagging it as lost
LOST_FRAME_THRESHOLD = 10

# Depth lookup: sample an NxN window around the marker centre and take median.
# Larger window = more stable but less precise for small markers.
DEPTH_SAMPLE_RADIUS = 3   # pixels — gives a 7x7 sample window

# Depth values outside this range are treated as invalid (in metres)
DEPTH_MIN_M = 0.1
DEPTH_MAX_M = 2.0

# RealSense depth scale — D435i encodes depth as uint16 millimetres.
# Value of 1000 means 1 unit = 1 mm = 0.001 m.
DEPTH_SCALE = 0.001
# ─────────────────────────────────────────────────────────────────────────────


class MarkerState:
    """Tracks smoothed pose and visibility for one marker ID."""

    def __init__(self, marker_id: int, alpha: float):
        self.marker_id   = marker_id
        self.alpha       = alpha
        self.seen        = False
        self.lost_frames = 0
        self.pos         = np.zeros(3)
        self.quat        = np.array([0.0, 0.0, 0.0, 1.0])

    def update(self, pos: np.ndarray, quat: np.ndarray):
        """Apply EMA smoothing to a new pose measurement."""
        if not self.seen:
            self.pos  = pos.copy()
            self.quat = quat.copy()
            self.seen = True
        else:
            self.pos = self.alpha * pos + (1 - self.alpha) * self.pos
            dot = np.dot(self.quat, quat)
            if dot < 0.0:
                quat = -quat
            self.quat = self._slerp(self.quat, quat, self.alpha)
            self.quat /= np.linalg.norm(self.quat)
        self.lost_frames = 0

    def mark_lost(self):
        self.lost_frames += 1

    def reset(self):
        """Reset state when a lost marker reappears."""
        self.seen        = False
        self.lost_frames = 0
        self.pos         = np.zeros(3)
        self.quat        = np.array([0.0, 0.0, 0.0, 1.0])

    @property
    def is_lost(self) -> bool:
        return self.lost_frames >= LOST_FRAME_THRESHOLD

    @staticmethod
    def _slerp(q0, q1, t):
        dot = np.clip(np.dot(q0, q1), -1.0, 1.0)
        theta = np.arccos(dot)
        if abs(theta) < 1e-6:
            return q0
        return (np.sin((1 - t) * theta) * q0 +
                np.sin(t * theta) * q1) / np.sin(theta)


class MultiArucoDetector(Node):

    def __init__(self):
        super().__init__('multi_aruco_detector')

        # Parameters
        self.declare_parameter('marker_size', MARKER_SIZE_M)
        self.declare_parameter('ema_alpha',   EMA_ALPHA)
        self.declare_parameter('debug_image', True)

        self.marker_size = self.get_parameter('marker_size').value
        alpha            = self.get_parameter('ema_alpha').value
        self.debug       = self.get_parameter('debug_image').value

        # ArUco setup — OpenCV 4.5.x API (ROS2 Humble default)
        self.aruco_dict   = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()

        # Camera intrinsics — filled on first CameraInfo message
        self.camera_matrix = None
        self.dist_coeffs   = None
        self.fx = self.fy = self.cx = self.cy = None

        # Latest aligned depth frame — updated by _depth_cb
        self.depth_image: np.ndarray | None = None
        self.depth_available = False

        # Per-marker state trackers
        self.marker_states: dict[int, MarkerState] = {}
        for mid in PUZZLE_MARKERS:
            self.marker_states[mid] = MarkerState(mid, alpha)

        self.bridge = CvBridge()

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self._camera_info_cb, 10)
        self.create_subscription(
            Image, '/camera/color/image_raw',
            self._image_cb, 10)
        # Aligned depth — same resolution and pixel grid as colour image
        self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw',
            self._depth_cb, 10)

        # ── Publishers ───────────────────────────────────────────────────────
        self.pose_pubs: dict[int, rclpy.publisher.Publisher] = {}
        for mid in PUZZLE_MARKERS:
            self.pose_pubs[mid] = self.create_publisher(
                PoseStamped, f'/aruco/marker_{mid}/pose', 10)

        self.marker_array_pub = self.create_publisher(
            MarkerArray, '/aruco/markers', 10)
        self.debug_pub = self.create_publisher(
            Image, '/aruco/debug_image', 10)

        self.get_logger().info(
            f'Multi-ArUco detector ready (with depth). '
            f'Tracking IDs: {list(PUZZLE_MARKERS.keys())}')

    # ── Camera info callback ──────────────────────────────────────────────────
    def _camera_info_cb(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs   = np.array(msg.d, dtype=np.float64)
            # Store focal lengths and principal point for depth reprojection
            self.fx = self.camera_matrix[0, 0]
            self.fy = self.camera_matrix[1, 1]
            self.cx = self.camera_matrix[0, 2]
            self.cy = self.camera_matrix[1, 2]
            self.get_logger().info(
                f'Camera intrinsics received. '
                f'fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx:.1f} cy={self.cy:.1f}')

    # ── Depth image callback ──────────────────────────────────────────────────
    def _depth_cb(self, msg: Image):
        """Cache the latest aligned depth frame.
        The aligned depth image has the same resolution as the colour image
        so pixel (u, v) in colour maps directly to pixel (u, v) in depth."""
        self.depth_image     = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_available = True

    # ── Depth-refined pose computation ───────────────────────────────────────
    def _get_depth_refined_pose(self, cx_px: float, cy_px: float, rvec: np.ndarray, tvec: np.ndarray) -> tuple[np.ndarray, np.ndarray, bool]:
        """
        Refine the Z component of the pose using the depth sensor reading,
        then recompute X and Y from that measured Z using camera intrinsics.

        Args:
            cx_px, cy_px : marker centre in pixel coordinates
            rvec, tvec   : pose from estimatePoseSingleMarkers (fallback)

        Returns:
            pos  : refined 3D position (x, y, z) in metres
            quat : orientation quaternion (unchanged from rvec)
            used_depth : True if depth lookup succeeded
        """
        rot_mat, _ = cv2.Rodrigues(rvec)
        quat = Rotation.from_matrix(rot_mat).as_quat()  # x, y, z, w

        # Fall back to estimatePoseSingleMarkers result if depth unavailable
        if not self.depth_available or self.depth_image is None:
            return tvec.flatten(), quat, False

        # Sample a small window around the marker centre to get a stable depth
        h, w = self.depth_image.shape[:2]
        r    = DEPTH_SAMPLE_RADIUS
        u    = int(round(cx_px))
        v    = int(round(cy_px))

        # Clamp window to image bounds
        u0, u1 = max(0, u - r), min(w, u + r + 1)
        v0, v1 = max(0, v - r), min(h, v + r + 1)

        patch = self.depth_image[v0:v1, u0:u1].astype(np.float32)

        # Ignore zero values (no depth reading) and convert to metres
        valid = patch[patch > 0] * DEPTH_SCALE

        if valid.size == 0:
            self.get_logger().warn(
                f'No valid depth at pixel ({u}, {v}) — using estimate.',
                throttle_duration_sec=2.0)
            return tvec.flatten(), quat, False

        # Median is more robust than mean against depth noise spikes
        z = float(np.median(valid))

        if not (DEPTH_MIN_M <= z <= DEPTH_MAX_M):
            self.get_logger().warn(
                f'Depth {z:.3f}m out of range [{DEPTH_MIN_M}, {DEPTH_MAX_M}]m '
                f'— using estimate.',
                throttle_duration_sec=2.0)
            return tvec.flatten(), quat, False

        # Reproject pixel centre to 3D using measured Z
        # Standard pinhole camera model:
        #   X = (u - cx) * Z / fx
        #   Y = (v - cy) * Z / fy
        x = (cx_px - self.cx) * z / self.fx
        y = (cy_px - self.cy) * z / self.fy

        return np.array([x, y, z]), quat, True

    # ── Main image callback ───────────────────────────────────────────────────
    def _image_cb(self, msg: Image):
        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        stamp = msg.header.stamp

        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)

        detected_ids = set()

        if ids is not None:
            ids_flat = ids.flatten().tolist()

            for i, marker_id in enumerate(ids_flat):
                detected_ids.add(marker_id)

                # 2D pose estimate (used for orientation + fallback position)
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    [corners[i]], self.marker_size,
                    self.camera_matrix, self.dist_coeffs)

                rvec = rvecs[0][0]
                tvec = tvecs[0][0]

                # Marker centre in pixel coordinates
                marker_cx = float(np.mean(corners[i][0][:, 0]))
                marker_cy = float(np.mean(corners[i][0][:, 1]))

                # Refine position using depth channel
                pos, quat, used_depth = self._get_depth_refined_pose(
                    marker_cx, marker_cy, rvec, tvec)

                # Create state tracker for any new/unknown marker ID
                if marker_id not in self.marker_states:
                    alpha = self.get_parameter('ema_alpha').value
                    self.marker_states[marker_id] = MarkerState(marker_id, alpha)
                    self.pose_pubs[marker_id] = self.create_publisher(
                        PoseStamped, f'/aruco/marker_{marker_id}/pose', 10)

                self.marker_states[marker_id].update(pos, quat)
                self._publish_pose(marker_id, stamp)

                # Debug overlay
                if self.debug:
                    cv2.drawFrameAxes(
                        frame, self.camera_matrix, self.dist_coeffs,
                        rvec, tvec, self.marker_size * 0.5)
                    label = PUZZLE_MARKERS.get(marker_id, f'id_{marker_id}')
                    cx = int(marker_cx)
                    cy = int(marker_cy)
                    # Green label = depth-refined, yellow = estimate fallback
                    colour = (0, 255, 0) if used_depth else (0, 255, 255)
                    cv2.putText(frame,
                                f'ID{marker_id}: {label}',
                                (cx - 30, cy - 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 2)
                    cv2.putText(frame,
                                f'z={pos[2]:.3f}m '
                                f'{"[D]" if used_depth else "[E]"}',
                                (cx - 30, cy - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.40, colour, 1)

        # Lost / reacquired tracking
        for mid, state in self.marker_states.items():
            if mid not in detected_ids and state.seen:
                state.mark_lost()
                if state.is_lost:
                    marker_logger = self.get_logger().get_child(f'marker_{mid}')
                    marker_logger.warn(
                        f'Marker {mid} ({PUZZLE_MARKERS.get(mid, "unknown")}) '
                        f'lost for {state.lost_frames} frames',
                        throttle_duration_sec=2.0)
            elif mid in detected_ids and state.is_lost:
                state.reset()
                self.get_logger().info(
                    f'Marker {mid} ({PUZZLE_MARKERS.get(mid, "unknown")}) '
                    f'reacquired.')

        self._publish_marker_array(stamp)

        if self.debug:
            aruco.drawDetectedMarkers(frame, corners, ids)
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)

    # ── Publish one PoseStamped ───────────────────────────────────────────────
    def _publish_pose(self, marker_id: int, stamp):
        state = self.marker_states[marker_id]
        if not state.seen:
            return

        pose_msg = PoseStamped()
        pose_msg.header = Header(
            stamp=stamp, frame_id='camera_color_optical_frame')
        pose_msg.pose.position.x    = float(state.pos[0])
        pose_msg.pose.position.y    = float(state.pos[1])
        pose_msg.pose.position.z    = float(state.pos[2])
        pose_msg.pose.orientation.x = float(state.quat[0])
        pose_msg.pose.orientation.y = float(state.quat[1])
        pose_msg.pose.orientation.z = float(state.quat[2])
        pose_msg.pose.orientation.w = float(state.quat[3])
        self.pose_pubs[marker_id].publish(pose_msg)

    # ── Publish MarkerArray for RViz ──────────────────────────────────────────
    def _publish_marker_array(self, stamp):
        array_msg = MarkerArray()
        for mid, state in self.marker_states.items():
            if not state.seen or state.is_lost:
                continue
            m         = Marker()
            m.header  = Header(stamp=stamp, frame_id='camera_color_optical_frame')
            m.ns      = 'aruco_puzzles'
            m.id      = mid
            m.type    = Marker.CUBE
            m.action  = Marker.ADD
            m.pose.position.x    = float(state.pos[0])
            m.pose.position.y    = float(state.pos[1])
            m.pose.position.z    = float(state.pos[2])
            m.pose.orientation.x = float(state.quat[0])
            m.pose.orientation.y = float(state.quat[1])
            m.pose.orientation.z = float(state.quat[2])
            m.pose.orientation.w = float(state.quat[3])
            m.scale.x = self.marker_size
            m.scale.y = self.marker_size
            m.scale.z = 0.002
            m.color   = (ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)
                         if mid == 10 else
                         ColorRGBA(r=0.0, g=0.9, b=0.3, a=0.8))
            array_msg.markers.append(m)
        self.marker_array_pub.publish(array_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()