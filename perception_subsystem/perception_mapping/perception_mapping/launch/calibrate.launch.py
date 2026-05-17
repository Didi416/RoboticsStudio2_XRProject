"""
calibrate.launch.py — Eye-in-hand calibration for EscapeXRtist
=====================================================================

Eye-in-hand setup:
  - D435i camera is mounted ON the UR3e end-effector
  - ChArUco board is FIXED in the world (on a table/wall, does not move)

Works for BOTH:
  • Simulation  (no real camera/robot needed, uses easy_handeye2_demo style)
  • Real hardware (D435i + UR3e)

What easy_handeye2 solves for in this mode:
  tool0  →  camera_color_optical_frame
  (the static transform from the robot flange to the camera optical centre)
 
TF frames required at runtime:
  base_link → tool0                      (published by ur_robot_driver)
  camera_color_optical_frame → tracking_marker_frame   (published by charuco_pose_publisher)
 
easy_handeye2 internally also publishes a dummy static TF:
  tool0 → camera_color_optical_frame  (placeholder until calibration is done)
This lets the TF tree be valid before calibration is saved. Do not confuse
this dummy with the real result — the real result replaces it after you save.

Usage
-----
Simulation (workflow learning / pipeline testing):
  ros2 launch perception_mapping calibrate.launch.py mode:=sim

Real hardware (actual calibration result you will use):
  ros2 launch perception_mapping calibrate.launch.py mode:=real

Arguments
---------
  mode         'sim' | 'real'  (default: real)
  freehand     'true' | 'false' — if true, YOU move the robot manually
               via teach pendant / freedrive instead of MoveIt auto-move
               (default: false — MoveIt moves the robot automatically)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    """
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='real',
        description="'sim' to run without hardware, 'real' for D435i + UR3e"
    )
    """

    freehand_arg = DeclareLaunchArgument(
        'freehand',
        default_value='true',
        description="'true' = move robot manually via teach pendant; 'false' = MoveIt auto-moves"
    )

    #mode      = LaunchConfiguration('mode')
    freehand  = LaunchConfiguration('freehand')

    easy_handeye2_dir = get_package_share_directory('easy_handeye2')

    # ── Calibration core (same for both modes) ───────────────────────────────
    calibration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(easy_handeye2_dir, 'launch', 'calibrate.launch.py')
        ),
        launch_arguments={
            # --- THE KEY CHANGE ---
            'calibration_type':      'eye_in_hand',
 
            # Separate name so it saves to a different .calib file
            # and doesn't overwrite your eye-on-base result
            'name':                  'ur3e_eih_calib',
 
            # Robot frames — unchanged from eye-on-base
            'robot_base_frame':      'base_link',
            'robot_effector_frame':  'tool0',
 
            # Tracking frames — unchanged from eye-on-base
            # Your charuco node always publishes:
            #   camera_color_optical_frame → tracking_marker_frame
            'tracking_base_frame':   'camera_color_optical_frame',
            'tracking_marker_frame': 'tracking_marker_frame',
 
            'freehand_robot_movement': freehand,
        }.items()
    )


    # ── Marker detector node (real hardware only) ────────────────────────────
    # Your charuco detector — board is fixed on the table, camera is on the robot
    charuco_node = Node(
        package='perception_mapping',
        executable='charuco_pose_publisher',
        name='charuco_pose_publisher',
        output='screen',
        remappings=[
          ('/camera/color/image_raw',   '/camera/camera/color/image_raw'),
          ('/camera/color/camera_info', '/camera/camera/color/camera_info'),
      ]
    )

    # ── Simulation tracking faker (sim mode only) ────────────────────────────
    # This mimics what easy_handeye2_demo does: publishes a fake marker TF
    # with configurable noise so you can practice the calibration GUI workflow.
    # NOTE: this is the Panda-based demo from easy_handeye2_demo — useful
    # ONLY for learning the tool, not for generating a usable calibration.
    """
    sim_instructions_node = Node(
        package='perception_mapping',
        executable='sim_mode_instructions',   # see sim_mode_instructions.py below
        name='sim_mode_instructions',
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))
    )
    """

    return LaunchDescription([
        # mode_arg,
        freehand_arg,
        charuco_node,
        calibration,
        # sim_instructions_node,   # uncomment after adding sim_mode_instructions.py
    ])