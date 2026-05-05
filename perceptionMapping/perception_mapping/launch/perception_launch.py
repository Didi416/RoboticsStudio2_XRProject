"""
perception_launch.py
─────────────────────
Launches the full perception & mapping pipeline for the EscapeXRtist project.

Run:
    ros2 launch perception_mapping perception_launch.py

Optional overrides:
    ros2 launch perception_mapping perception_launch.py config_path:=/path/to/aruco_config.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.abspath(__file__))
    default_config = os.path.join(pkg_dir, "..", "config", "aruco_config.yaml")

    config_arg = DeclareLaunchArgument(
        "config_path",
        default_value=default_config,
        description="Path to aruco_config.yaml"
    )
    config_path = LaunchConfiguration("config_path")

    # ── 1. Static calibration TF broadcaster ─────────────────────────────
    calibration_publisher = Node(
        package    = "perception_mapping",
        executable = "calibration_publisher",
        name       = "calibration_publisher",
        output     = "screen",
        parameters = [{"config_path": config_path}],
    )

    # ── 2. ArUco detector (webcam → ROS topics + dynamic TF) ─────────────
    aruco_detector = Node(
        package    = "perception_mapping",
        executable = "aruco_detector_node",
        name       = "aruco_detector_node",
        output     = "screen",
        parameters = [{"config_path": config_path}],
    )

    # ── 3. Puzzle wall pose transformer (camera frame → base_link + Unity JSON) ──
    pose_publisher = Node(
        package    = "perception_mapping",
        executable = "puzzle_wall_pose_publisher",
        name       = "puzzle_wall_pose_publisher",
        output     = "screen",
        parameters = [{"config_path": config_path}],
    )

    return LaunchDescription([
        config_arg,
        LogInfo(msg="[perception_launch] Starting EscapeXRtist perception pipeline..."),
        calibration_publisher,
        aruco_detector,
        pose_publisher,
        LogInfo(msg="[perception_launch] All nodes launched. Topics:\n"
                    "  /perception/puzzle_wall_pose      — camera frame\n"
                    "  /perception/puzzle_wall_pose_base — robot base frame\n"
                    "  /perception/unity_sync            — JSON for Unity\n"
                    "  /perception/detection_status      — heartbeat JSON\n"
                    "  /perception/image_debug           — annotated camera feed\n"
                    "  TF: base_link → tool0 → camera_optical_frame → puzzle_wall"
        ),
    ])