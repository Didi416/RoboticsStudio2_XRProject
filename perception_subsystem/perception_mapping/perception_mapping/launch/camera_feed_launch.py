# camera_feed_launch.py
# Launches the RealSense D435i camera driver and QoS relay node
# for Unity live camera feed integration.
#
# Usage:
#   ros2 launch <package> camera_feed_launch.py
# OR run directly:
#   python3 camera_feed_launch.py (not recommended — use ros2 launch)

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    # ── 1. RealSense D435i Camera Driver ─────────────────────────────
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            'color_width':  1280,
            'color_height': 720,
            'color_fps':    30.0,
            'enable_depth': True,
            'enable_infra1': False,
            'enable_infra2': False,
        }],
        output='screen',
    )

    # ── 2. QoS Relay Node ─────────────────────────────────────────────
    # Delayed by 5 seconds to give the RealSense driver time to start
    # and begin publishing before the relay subscribes.
    relay_node = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='RealSense driver should be up — starting QoS relay...'),
            ExecuteProcess(
                cmd=[
                    'python3',
                    '/home/amadee/41069_ws/src/RoboticsStudio2_XRProject/'
                    'perception_subsystem/camera_relay.py'
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        LogInfo(msg='=== Camera Feed Launch Starting ==='),
        LogInfo(msg='Step 1: Launching Intel RealSense D435i...'),
        realsense_node,
        LogInfo(msg='Step 2: QoS relay will start in 5 seconds...'),
        relay_node,
        LogInfo(msg='=== Once relay starts, press F1 in Unity to switch to RealSense ==='),
    ])