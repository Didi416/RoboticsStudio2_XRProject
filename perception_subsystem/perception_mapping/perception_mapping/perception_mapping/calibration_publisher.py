#!/usr/bin/env python3
"""
calibration_publisher.py
─────────────────────────
Reads the eye-in-hand calibration result from aruco_config.yaml and
broadcasts a STATIC TF:

    /tool0  →  /camera_optical_frame

This lets every other node simply look up transforms in the TF tree rather
than hard-coding the camera offset.

Run:
    ros2 run perception_mapping calibration_publisher
    -- or via launch file --
"""

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import yaml
import numpy as np


class CalibrationPublisher(Node):
    def __init__(self):
        super().__init__("calibration_publisher")

        # ── Load config ───────────────────────────────────────────────────
        config_path = os.path.join(
            os.path.dirname(__file__), "config", "aruco_config.yaml"
        )
        self.declare_parameter("config_path", config_path)
        config_path = self.get_parameter("config_path").get_parameter_value().string_value

        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f)

        calib   = cfg["eye_in_hand_calibration"]
        frames  = cfg["ros"]

        t_xyz   = calib["translation_xyz"]   # [x, y, z]
        r_xyzw  = calib["rotation_xyzw"]     # [x, y, z, w]

        self.tool_frame   = frames["tool_frame"]
        self.camera_frame = frames["camera_frame"]

        # ── Broadcast static TF ───────────────────────────────────────────
        self._static_broadcaster = StaticTransformBroadcaster(self)

        ts = TransformStamped()
        ts.header.stamp         = self.get_clock().now().to_msg()
        ts.header.frame_id      = self.tool_frame
        ts.child_frame_id       = self.camera_frame

        ts.transform.translation.x = float(t_xyz[0])
        ts.transform.translation.y = float(t_xyz[1])
        ts.transform.translation.z = float(t_xyz[2])

        ts.transform.rotation.x = float(r_xyzw[0])
        ts.transform.rotation.y = float(r_xyzw[1])
        ts.transform.rotation.z = float(r_xyzw[2])
        ts.transform.rotation.w = float(r_xyzw[3])

        self._static_broadcaster.sendTransform(ts)

        self.get_logger().info(
            f"[CalibrationPublisher] Broadcasting static TF: "
            f"{self.tool_frame} → {self.camera_frame}\n"
            f"  translation: {t_xyz}\n"
            f"  rotation (xyzw): {r_xyzw}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()