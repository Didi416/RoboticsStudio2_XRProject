"""
egg_sorter_verifier.launch.py

Launches just the egg verification node.
Run this alongside the existing multi_aruco_detector pipeline.

Usage examples:
  # Default — correct egg is green (ID 1)
  ros2 launch perception_mapping egg_sorter_verifier.launch.py

  # Override for a seeded run where the correct egg is blue (ID 2)
  ros2 launch perception_mapping egg_sorter_verifier.launch.py correct_egg_aruco_id:=2

  # Tighten tolerance to 20 mm for a well-calibrated system
  ros2 launch perception_mapping egg_sorter_verifier.launch.py placement_tolerance_m:=0.02
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    correct_egg_arg = DeclareLaunchArgument(
        "correct_egg_aruco_id",
        default_value="1",
        description=(
            "ArUco ID of the egg that must go into the answer slot. "
            "1=green, 2=blue, 3=yellow, 4=purple"
        ),
    )

    tolerance_arg = DeclareLaunchArgument(
        "placement_tolerance_m",
        default_value="0.03",
        description="Distance tolerance (metres) for counting an egg as placed in a slot",
    )

    base_frame_arg = DeclareLaunchArgument(
        "robot_base_frame",
        default_value="base_link",
        description="Root TF frame for pose lookups",
    )

    verifier_node = Node(
        package="perception_mapping",
        executable="egg_sorter_verifier",
        name="egg_sorter_verifier",
        output="screen",
        parameters=[
            {
                "correct_egg_aruco_id":  LaunchConfiguration("correct_egg_aruco_id"),
                "placement_tolerance_m": LaunchConfiguration("placement_tolerance_m"),
                "robot_base_frame":      LaunchConfiguration("robot_base_frame"),
            }
        ],
    )

    return LaunchDescription([
        correct_egg_arg,
        tolerance_arg,
        base_frame_arg,
        verifier_node,
    ])