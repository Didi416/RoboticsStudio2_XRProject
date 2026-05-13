import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler,
    LogInfo,
    DeclareLaunchArgument
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    args = [
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.56.101', # URSIM
            description='Robot IP, defaults to URSIM (ignored when use_fake_hardware:=true).',
        ),
        DeclareLaunchArgument(
            'ros_tcp_host',
            default_value='127.0.0.1', # Default Unity host IP
            description='IP the ROS-TCP-Endpoint listens on. (Default Unity host IP)',
        ),
        DeclareLaunchArgument(
            'ros_tcp_port',
            default_value='10000',
            description='Port the ROS-TCP-Endpoint listens on.',
        ),
    ]

    robot_ip        = LaunchConfiguration('robot_ip')
    ros_tcp_host    = LaunchConfiguration('ros_tcp_host')
    ros_tcp_port    = LaunchConfiguration('ros_tcp_port')
    # ──────────────────────────────────────────────
    # Step 1 – UR hardware / driver (HAS TO BE STARTED IN SEPARATE TERMINAL TO BE ABLE TO START EXTERNAL CONTROL IN URSIM BEFORE SWITCHING CONTROLLERS IN THIS LAUNCH FILE)
    # ──────────────────────────────────────────────
    # ur_hardware_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("rs2_ros2_unity_bridge"),
    #             "launch",
    #             "ur_driver_modified.launch.py",
    #         )
    #     ),
    #     launch_arguments={
    #         "ur_type": "ur3e",
    #         "robot_ip": robot_ip,
    #         "launch_rviz": "true",
    #     }.items(),
    # )

    # ──────────────────────────────────────────────
    # Step 2 – MoveIt + RViz  (delayed to let driver settle)
    # ──────────────────────────────────────────────
    ur_moveit_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("rs2_ros2_unity_bridge"),
                        "launch",
                        "ur_moveit_modified.launch.py",
                    )
                ),
                launch_arguments={
                    "ur_type": "ur3e",
                    "launch_rviz": "false",
                }.items(),
            )
        ],
    )

    # ──────────────────────────────────────────────
    # Step 3 – Switch controllers
    #   activate  : forward_velocity_controller
    #   deactivate: scaled_joint_trajectory_controller
    # Delayed further so MoveIt / controllers are fully up
    # ──────────────────────────────────────────────
    switch_controllers = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "control", "switch_controllers",
                    "--activate",   "forward_velocity_controller",
                    "--deactivate", "scaled_joint_trajectory_controller",
                ],
                output="screen",
                name="switch_controllers_servo",
            )
        ],
    )

    # switch_controllers = TimerAction(
    #     period=12.0,
    #     actions=[
    #         ExecuteProcess(
    #             cmd=[
    #                 "ros2", "control", "switch_controllers",
    #                 "--activate",   "finger_width_trajectory_controller",
    #                 "--deactivate", "finger_width_controller",
    #             ],
    #             output="screen",
    #             name="switch_controllers_fingers",
    #         )
    #     ],
    # )

    # ──────────────────────────────────────────────
    # Step 4 – Start servo node service call
    # Runs after controller switch completes
    # ──────────────────────────────────────────────
    start_servo = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "service", "call",
                    "/servo_node/start_servo",
                    "std_srvs/srv/Trigger",
                    "{}",
                ],
                output="screen",
                name="start_servo",
            )
        ],
    )

    # ──────────────────────────────────────────────
    # Step 5 – ROS TCP Endpoint
    # ──────────────────────────────────────────────
    tcp_endpoint = TimerAction(
        period=18.0,
        actions=[
            Node(
                package="ros_tcp_endpoint",
                executable="default_server_endpoint",
                name="ros_tcp_endpoint",
                output="screen",
                parameters=[{"ROS_IP": ros_tcp_host, "ROS_PORT": ros_tcp_port}],
            )
        ],
    )

    return LaunchDescription(
        args + [
            # LogInfo(msg="[1/5] Launching UR hardware driver + RViz..."),
            # ur_hardware_launch,

            LogInfo(msg="[2/5] Launching MoveIt (in 5 s)..."),
            ur_moveit_launch,

            LogInfo(msg="[3/5] Switching controllers (in 12 s)..."),
            switch_controllers,

            LogInfo(msg="[4/5] Starting servo (in 15 s)..."),
            start_servo,

            LogInfo(msg="[5/5] Starting ROS TCP Endpoint (in 18 s)..."),
            tcp_endpoint,
        ]
    )
