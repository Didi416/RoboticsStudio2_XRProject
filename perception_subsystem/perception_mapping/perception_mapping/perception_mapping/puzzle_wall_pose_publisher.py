#!/usr/bin/env python3
"""
puzzle_wall_pose_publisher.py
──────────────────────────────
Subscribes to /perception/puzzle_wall_pose  (camera frame)
and uses the TF tree to express it in the robot base frame (base_link).

This is the pose Unity needs: "where is the puzzle wall relative to the
robot base?" — which is what the VR scene should mirror.

Published topics:
    /perception/puzzle_wall_pose_base   geometry_msgs/PoseStamped  (base_link frame)
    /perception/unity_sync              std_msgs/String             (JSON for ROS-TCP)

The JSON payload on /perception/unity_sync is what the Unity C# script
ROSPuzzleWallSync.cs subscribes to.

JSON schema:
{
  "position":    {"x": float, "y": float, "z": float},
  "rotation":    {"x": float, "y": float, "z": float, "w": float},
  "frame":       "base_link",
  "timestamp":   float,
  "status":      "OK" | "LOST" | "PARTIAL",
  "confidence":  float   // 0.0–1.0 based on how many markers were visible
}

ROS → Unity coordinate convention:
    ROS  +X forward, +Y left,  +Z up    (right-hand)
    Unity +X right,  +Y up,   +Z forward (left-hand)
    Conversion applied in this node before publishing.

Run:
    ros2 run perception_mapping puzzle_wall_pose_publisher
"""

import os
import json
import time
import rclpy
import yaml
import numpy as np

from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from scipy.spatial.transform import Rotation as R


def ros_to_unity_pose(pos_ros, quat_ros_xyzw):
    """
    Convert a pose from ROS coordinate frame to Unity coordinate frame.
    ROS:   +X forward, +Y left, +Z up
    Unity: +X right,   +Y up,   +Z forward
    """
    x_r, y_r, z_r = pos_ros
    # Unity position
    pos_unity = {
        "x": float(-y_r),   # ROS -Y → Unity +X  (right)
        "y": float(z_r),    # ROS +Z → Unity +Y  (up)
        "z": float(x_r),    # ROS +X → Unity +Z  (forward)
    }

    # Quaternion conversion (negate handedness)
    qx, qy, qz, qw = quat_ros_xyzw
    rot_unity = {
        "x": float(-qy),
        "y": float(qz),
        "z": float(-qx),
        "w": float(qw),
    }
    return pos_unity, rot_unity


class PuzzleWallPosePublisher(Node):

    def __init__(self):
        super().__init__("puzzle_wall_pose_publisher")

        config_path = os.path.join(os.path.dirname(__file__), "config", "aruco_config.yaml")
        self.declare_parameter("config_path", config_path)
        config_path = self.get_parameter("config_path").get_parameter_value().string_value

        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f)

        ros_cfg = cfg["ros"]

        self._base_frame   = ros_cfg["base_frame"]
        self._camera_frame = ros_cfg["camera_frame"]

        # ── TF listener ───────────────────────────────────────────────────
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Publishers ────────────────────────────────────────────────────
        self._pub_base   = self.create_publisher(PoseStamped, "/perception/puzzle_wall_pose_base", 10)
        self._pub_unity  = self.create_publisher(String,      "/perception/unity_sync",            10)

        # ── Subscribe to camera-frame pose ───────────────────────────────
        self._latest_pose   = None
        self._latest_status = "LOST"
        self._sub_pose = self.create_subscription(
            PoseStamped,
            ros_cfg["topic_puzzle_wall_pose"],
            self._pose_callback,
            10
        )
        self._sub_status = self.create_subscription(
            String,
            ros_cfg["topic_detection_status"],
            self._status_callback,
            10
        )

        # ── Timer 30 Hz ───────────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / 30.0, self._publish)

        self.get_logger().info("[PuzzleWallPosePublisher] Ready.")

    def _pose_callback(self, msg: PoseStamped):
        self._latest_pose = msg

    def _status_callback(self, msg: String):
        try:
            d = json.loads(msg.data)
            self._latest_status = d.get("status", "LOST")
            n_detected = len(d.get("detected_ids", []))
            n_required = len(d.get("required_ids", [4]))
            self._confidence = float(n_detected) / max(1, n_required)
        except Exception:
            pass

    def _publish(self):
        if self._latest_pose is None:
            return

        # ── Transform pose from camera frame → base_link ──────────────────
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._latest_pose.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"[PuzzleWallPosePublisher] TF lookup failed: {e} — "
                f"Is calibration_publisher running? Is MoveIt sim up?",
                throttle_duration_sec=5.0
            )
            return

        # Apply transform manually (tf2_geometry_msgs does this, but let's be explicit)
        t = transform.transform.translation
        r_tf = transform.transform.rotation

        T_base_cam = self._tf_to_matrix(t, r_tf)

        p_cam = np.array([
            self._latest_pose.pose.position.x,
            self._latest_pose.pose.position.y,
            self._latest_pose.pose.position.z,
            1.0
        ])
        p_base_h = T_base_cam @ p_cam
        p_base = p_base_h[:3]

        # Rotate the orientation
        r_cam  = R.from_quat([
            self._latest_pose.pose.orientation.x,
            self._latest_pose.pose.orientation.y,
            self._latest_pose.pose.orientation.z,
            self._latest_pose.pose.orientation.w,
        ])
        r_base_cam_rot = R.from_matrix(T_base_cam[:3, :3])
        r_base_wall    = r_base_cam_rot * r_cam
        q_base = r_base_wall.as_quat()   # [x, y, z, w]

        # ── Publish in base_link frame ────────────────────────────────────
        out = PoseStamped()
        out.header.frame_id = self._base_frame
        out.header.stamp    = self.get_clock().now().to_msg()
        out.pose.position.x = float(p_base[0])
        out.pose.position.y = float(p_base[1])
        out.pose.position.z = float(p_base[2])
        out.pose.orientation.x = float(q_base[0])
        out.pose.orientation.y = float(q_base[1])
        out.pose.orientation.z = float(q_base[2])
        out.pose.orientation.w = float(q_base[3])
        self._pub_base.publish(out)

        # ── Convert to Unity and publish JSON ─────────────────────────────
        pos_unity, rot_unity = ros_to_unity_pose(p_base, q_base)

        payload = {
            "position":   pos_unity,
            "rotation":   rot_unity,
            "frame":      "base_link",
            "timestamp":  time.time(),
            "status":     self._latest_status,
            "confidence": getattr(self, "_confidence", 0.0),
        }
        self._pub_unity.publish(String(data=json.dumps(payload)))

    @staticmethod
    def _tf_to_matrix(translation, rotation):
        """Build 4x4 matrix from a TF transform."""
        T = np.eye(4)
        r = R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w])
        T[:3, :3] = r.as_matrix()
        T[:3, 3]  = [translation.x, translation.y, translation.z]
        return T


def main(args=None):
    rclpy.init(args=args)
    node = PuzzleWallPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()