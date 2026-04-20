"""
xr_servo_launch.py
─────────────────────────────────────────────────────────────────────────────
Launches the full XR teleoperation stack for UR3e:

  1. ros_tcp_endpoint   - Unity ↔ ROS bridge (TCP port 10000)
  2. move_group         - MoveIt2 planning / scene
  3. servo_node         - MoveIt Servo (composable, low-latency)
  4. xr_servo_node      - Our node: XR pose → TwistStamped → Servo
  5. joint_state_relay  - Mirrors /joint_states → /unity_joint_states
                          (handled inside xr_servo_node, no extra node needed)

Usage:
  ros2 launch ur3e_xr_servo xr_servo_launch.py \
      robot_ip:=192.168.1.100 \
      use_fake_hardware:=false
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    OpaqueFunction, TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import os
import yaml


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def load_yaml(package_name, file_path):
    package_path = FindPackageShare(package_name).find(package_name)
    absolute_path = os.path.join(package_path, file_path)
    with open(absolute_path, "r") as f:
        return yaml.safe_load(f)


# ──────────────────────────────────────────────────────────────────────────────
def generate_launch_description():

    # ── Declare arguments ─────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("robot_ip",           default_value="192.168.1.100"),
        DeclareLaunchArgument("use_fake_hardware",   default_value="false"),
        DeclareLaunchArgument("launch_rviz",         default_value="true"),
        DeclareLaunchArgument("ros_tcp_port",        default_value="10000"),
        DeclareLaunchArgument("ros_tcp_host",        default_value="0.0.0.0"),
    ]

    robot_ip         = LaunchConfiguration("robot_ip")
    use_fake         = LaunchConfiguration("use_fake_hardware")
    ros_tcp_port     = LaunchConfiguration("ros_tcp_port")
    ros_tcp_host     = LaunchConfiguration("ros_tcp_host")

    # ── UR3e MoveIt launch (from ur_moveit_config) ───────────────────────────
    ur_moveit_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("ur_moveit_config"),
            "launch", "ur_moveit.launch.py"
        ]),
        launch_arguments={
            "ur_type":            "ur3e",
            "robot_ip":           robot_ip,
            "use_fake_hardware":  use_fake,
            "launch_rviz":        LaunchConfiguration("launch_rviz"),
        }.items()
    )

    # ── MoveIt Servo (composable node for low latency) ────────────────────────
    servo_params = load_yaml("ur3e_xr_servo", "config/moveit_servo_config.yaml")

    servo_container = ComposableNodeContainer(
        name="servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[servo_params],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    # ── ROS-TCP-Endpoint (Unity bridge) ───────────────────────────────────────
    ros_tcp_node = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        name="ros_tcp_endpoint",
        parameters=[{
            "ROS_IP":   ros_tcp_host,
            "ROS_PORT": ros_tcp_port,
        }],
        output="screen",
    )

    # ── Our XR → Servo bridge node ────────────────────────────────────────────
    xr_servo_node = Node(
        package="ur3e_xr_servo",
        executable="xr_servo_node",
        name="xr_servo_node",
        parameters=[{
            "planning_frame":      "base_link",
            "ee_frame":            "tool0",
            "linear_scale":        1.5,
            "angular_scale":       2.0,
            "publish_period_ms":   34,
            "servo_command_topic": "/servo_node/delta_twist_cmds",
            "joint_state_topic":   "/joint_states",
            "unity_js_topic":      "/unity_joint_states",
        }],
        output="screen",
    )

    # ── Servo activation service call (after a short delay) ───────────────────
    # MoveIt Servo needs to receive a "start servo" service call before it
    # begins processing twist commands.
    activate_servo = TimerAction(
        period=5.0,   # wait for move_group to be ready
        actions=[
            Node(
                package="ros2cli",
                executable="ros2",
                arguments=["service", "call",
                           "/servo_node/start_servo",
                           "std_srvs/srv/Trigger", "{}"],
                output="screen",
            )
        ]
    )

    return LaunchDescription(args + [
        ur_moveit_launch,
        servo_container,
        ros_tcp_node,
        xr_servo_node,
        activate_servo,   # uncomment if auto-start is desired
    ])