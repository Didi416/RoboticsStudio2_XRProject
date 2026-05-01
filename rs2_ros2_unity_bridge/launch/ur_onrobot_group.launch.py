# xr_teleop.launch.py
# ─────────────────────────────────────────────────────────────────────────────
# Single launch file that brings up the full XR teleoperation stack:
#
#   1. ur_onrobot_control    — ros2_control node + controller spawners
#   2. ur_onrobot_moveit     — move_group + servo_node + RViz
#   3. ros_tcp_endpoint      — Unity ↔ ROS TCP bridge
#   4. switch_controllers    — activates forward_velocity_controller
#   5. servo lifecycle       — reset → start → unpause servo
#
# Usage:
#   ros2 launch ur_onrobot_moveit_config xr_teleop.launch.py \
#       ur_type:=ur3e onrobot_type:=rg2 use_fake_hardware:=true
#
# Drop this file into:
#   ur_onrobot_moveit_config/launch/xr_teleop.launch.py
# ─────────────────────────────────────────────────────────────────────────────

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Declare arguments ─────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur3e',
            choices=['ur3', 'ur3e', 'ur5', 'ur5e'],
            description='UR robot type.',
        ),
        DeclareLaunchArgument(
            'onrobot_type',
            default_value='rg2',
            choices=['rg2', 'rg6'],
            description='OnRobot gripper type.',
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.56.101',
            description='Robot IP (ignored when use_fake_hardware:=true).',
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Use mock hardware for testing without a real robot.',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz.',
        ),
        DeclareLaunchArgument(
            'ros_tcp_host',
            default_value='127.0.0.1',
            description='IP the ROS-TCP-Endpoint listens on.',
        ),
        DeclareLaunchArgument(
            'ros_tcp_port',
            default_value='10000',
            description='Port the ROS-TCP-Endpoint listens on.',
        ),
        DeclareLaunchArgument(
            'servo_controller',
            default_value='forward_velocity_controller',
            description='Controller to activate for MoveIt Servo.',
        ),
        DeclareLaunchArgument(
            'planner_controller',
            default_value='scaled_joint_trajectory_controller',
            description='Trajectory controller to deactivate when servo starts.',
        ),
    ]

    ur_type         = LaunchConfiguration('ur_type')
    onrobot_type    = LaunchConfiguration('onrobot_type')
    robot_ip        = LaunchConfiguration('robot_ip')
    use_fake        = LaunchConfiguration('use_fake_hardware')
    launch_rviz     = LaunchConfiguration('launch_rviz')
    ros_tcp_host    = LaunchConfiguration('ros_tcp_host')
    ros_tcp_port    = LaunchConfiguration('ros_tcp_port')
    servo_ctrl      = LaunchConfiguration('servo_controller')
    planner_ctrl    = LaunchConfiguration('planner_controller')

    # ── 1. ur_onrobot_control ─────────────────────────────────────────────────
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_onrobot_control'),
                'launch', 'start_robot.launch.py',
            ])
        ]),
        launch_arguments={
            'ur_type':                  ur_type,
            'onrobot_type':             onrobot_type,
            'robot_ip':                 robot_ip,
            'use_fake_hardware':        use_fake,
            'launch_rviz':              'true',   # RViz launched by moveit below
            'launch_dashboard_client':  'false',
            # Start with forward_velocity_controller active from the start
            # so the controller switch step below is instantaneous
            'initial_joint_controller': 'forward_velocity_controller',
            'activate_joint_controller': 'true',
        }.items(),
    )

    # ── 2. ur_onrobot_moveit (move_group + servo_node + optional RViz) ────────
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_onrobot_moveit_config'),
                'launch', 'ur_onrobot_moveit.launch.py',
            ])
        ]),
        launch_arguments={
            'ur_type':       ur_type,
            'onrobot_type':  onrobot_type,
            'use_sim_time':  'false',
            'launch_rviz':   launch_rviz,
            'launch_servo':  'true',
        }.items(),
    )

    # ── 3. ROS-TCP-Endpoint (Unity bridge) ────────────────────────────────────
    ros_tcp_node = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='ros_tcp_endpoint',
        parameters=[{
            'ROS_IP':   ros_tcp_host,
            'ROS_PORT': ros_tcp_port,
        }],
        output='screen',
    )

    # ── 4-7. Controller switch + servo lifecycle ───────────────────────────────
    # Delayed by 10 s to give control + moveit time to fully initialise.
    # Each step is chained via OnProcessExit so they run sequentially.

    switch_controllers = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'switch_controllers',
            '--activate',   'forward_velocity_controller',
            '--deactivate', 'scaled_joint_trajectory_controller',
            '--strict',
        ],
        output='screen',
        name='switch_controllers',
    )

    reset_servo = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/servo_node/reset_servo_status',
            'std_srvs/srv/Empty', '{}',
        ],
        output='screen',
        name='reset_servo',
    )

    start_servo = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/servo_node/start_servo',
            'std_srvs/srv/Trigger', '{}',
        ],
        output='screen',
        name='start_servo',
    )

    unpause_servo = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/servo_node/unpause_servo',
            'std_srvs/srv/Trigger', '{}',
        ],
        output='screen',
        name='unpause_servo',
    )

    # Delay the first step — wait for controller_manager and servo_node to be up
    delayed_switch = TimerAction(
        period=12.0,
        actions=[switch_controllers],
    )

    # Chain: switch → reset → start → unpause
    chain = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=switch_controllers,
                on_exit=[start_servo],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=start_servo,
                on_exit=[reset_servo],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=reset_servo,
                on_exit=[unpause_servo],
            )
        ),
    ]

    return LaunchDescription(
        args + [
            control_launch,
            moveit_launch,
            ros_tcp_node,
            delayed_switch,
        ] + chain
    )