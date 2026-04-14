# # Launches ur_robot_driver AND ur_moveit_config from a single file,
# # guaranteeing both receive the same ur_type and tf_prefix.
# #
# # The SRDF mismatch error ("Semantic description not for same robot as URDF")
# # occurs when ur_robot_driver and ur_moveit_config are launched separately
# # with different ur_type or tf_prefix values — the URDF robot name then
# # differs from the name the SRDF was generated for.
# #
# # Launching both from here with shared LaunchConfiguration variables
# # ensures they always agree.
# #
# # Usage:
# #   # URSim:
# #   ros2 launch your_package vr_ik.launch.py ur_type:=ur3e use_fake_hardware:=true
# #
# #   # Real robot:
# #   ros2 launch your_package vr_ik.launch.py ur_type:=ur3e robot_ip:=192.168.1.100

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

#     # ── Shared arguments — used by BOTH driver and moveit_config ──────────────
#     # Keeping these as shared LaunchConfiguration objects is what prevents
#     # the SRDF/URDF robot-name mismatch.
    ur_type        = LaunchConfiguration("ur_type")
    robot_ip       = LaunchConfiguration("robot_ip")
    use_fake_hw    = LaunchConfiguration("use_fake_hardware")
    tf_prefix      = LaunchConfiguration("tf_prefix")
    planning_group = LaunchConfiguration("planning_group")

#     # Use FindPackageShare to locate the script's directory (assuming it's in a 'scripts' folder)
#     # The script must be installed to the share directory via setup.py or CMakeLists.txt
#     # script_path = PathJoinSubstitution([FindPackageShare("ur_client_library"),
#                                         # "scripts", 'start_ursim.sh'
#     # ])
# ── 0. Start URSim ───────────────────────────────────────────────
    ursim = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ur_client_library',
            'start_ursim.sh',
            '-m', 'ur3e'
        ],
        output='screen'
    )

    # ── 1. UR driver (delayed) ───────────────────────────────────────
    driver = TimerAction(
        period=8.0,  # give URSim time to boot
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("ur_robot_driver"),
                        "launch", "ur_control.launch.py"
                    ])
                ]),
                launch_arguments={
                    "ur_type":           ur_type,
                    "robot_ip":          robot_ip,
                    "use_fake_hardware": use_fake_hw,
                    "tf_prefix":         tf_prefix,
                    "launch_rviz":       "true",
                }.items(),
            ),
        ]
    )

    # ── 2. MoveIt (optional, delayed further) ────────────────────────
    moveit = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("ur_moveit_config"),
                        "launch", "ur_moveit.launch.py"
                    ])
                ]),
                launch_arguments={
                    "ur_type":           ur_type,
                    "robot_ip":          robot_ip,
                    "use_fake_hardware": use_fake_hw,
                    "tf_prefix":         tf_prefix,
                    "launch_rviz":       "false",
                }.items(),
            ),
        ]
    )

    # ── 3. TCP endpoint (after driver is alive) ──────────────────────
    tcp = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='ros_tcp_endpoint',
                executable='default_server_endpoint',
                name='tcp_endpoint',
                parameters=[{'ROS_IP': '127.0.0.1'}],
                output='screen'
            ),
        ]
    )

    # ── 4. Test node (last) ──────────────────────────────────────────
    test_node = TimerAction(
        period=20.0,
        actions=[
            Node(
                package="rs2_ros2_unity_bridge",
                executable="test_joint_state",
                name="testing_script",
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                }],
            ),
        ]
    )
    
    return LaunchDescription([
    # arguments...
    DeclareLaunchArgument("ur_type", default_value="ur3e"),
    DeclareLaunchArgument("robot_ip", default_value="192.168.56.101"),
    DeclareLaunchArgument("use_fake_hardware", default_value="false"),
    DeclareLaunchArgument("launch_rviz", default_value="true"),
    DeclareLaunchArgument("tf_prefix", default_value=""),
    DeclareLaunchArgument("planning_group", default_value="ur_manipulator"),

    # ursim,
    driver,
    tcp,
    test_node,
])

# def generate_launch_description():

#     # ── Shared arguments — used by BOTH driver and moveit_config ──────────────
#     # Keeping these as shared LaunchConfiguration objects is what prevents
#     # the SRDF/URDF robot-name mismatch.
#     ur_type        = LaunchConfiguration("ur_type")
#     robot_ip       = LaunchConfiguration("robot_ip")
#     use_fake_hw    = LaunchConfiguration("use_fake_hardware")
#     tf_prefix      = LaunchConfiguration("tf_prefix")
#     planning_group = LaunchConfiguration("planning_group")

#     # Use FindPackageShare to locate the script's directory (assuming it's in a 'scripts' folder)
#     # The script must be installed to the share directory via setup.py or CMakeLists.txt
#     # script_path = PathJoinSubstitution([FindPackageShare("ur_client_library"),
#                                         # "scripts", 'start_ursim.sh'
#     # ])

#     return LaunchDescription([

#         # ── Arguments ─────────────────────────────────────────────────────────
#         DeclareLaunchArgument("ur_type",           default_value="ur3e"),
#         DeclareLaunchArgument("robot_ip",          default_value="192.168.56.101"),
#         DeclareLaunchArgument("use_fake_hardware", default_value="false"),
#         DeclareLaunchArgument("launch_rviz",       default_value="true"),

#         # tf_prefix must be identical in driver and moveit_config.
#         # If you set a prefix in the driver, set the same one here.
#         # Leave empty ("") unless you specifically need namespacing.
#         DeclareLaunchArgument("tf_prefix",         default_value=""),

#         # Must match the group name in your SRDF.
#         # All UR-provided MoveIt configs use "ur_manipulator".
#         DeclareLaunchArgument("planning_group",    default_value="ur_manipulator"),

#         # Start URSim for UR3e
#         # ExecuteProcess(
#         #     cmd=[
#         #         FindExecutable(name='start_ursim.sh'),
#         #         '-m', 'ur3e'
#         #     ],
#         #     output='screen'
#         # ),

#         ExecuteProcess(
#             cmd=[
#                 'ros2', 'run', 'ur_client_library',
#                 'start_ursim.sh',
#                 '-m', 'ur3e'
#             ],
#             output='screen'
#         ),

#         # ── 1. UR robot driver ────────────────────────────────────────────────
#         # Loads the URDF (robot_description) onto the parameter server.
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([
#                 PathJoinSubstitution([
#                     FindPackageShare("ur_robot_driver"),
#                     "launch", "ur_control.launch.py"
#                 ])
#             ]),
#             launch_arguments={
#                 "ur_type":           ur_type,
#                 "robot_ip":          robot_ip,
#                 "use_fake_hardware": use_fake_hw,
#                 "tf_prefix":         tf_prefix,
#                 "launch_rviz":       "true",
#             }.items(),
#         ),
#         # ── 2. MoveIt config ──────────────────────────────────────────────────
#         # Loads the SRDF (robot_description_semantic) and starts move_group.
#         # MUST receive the same ur_type and tf_prefix as the driver above —
#         # this is what prevents the "not for same robot" SRDF error.
#         # TimerAction(
#         #     period=3.0,
#         #     actions=[
#         #         IncludeLaunchDescription(
#         #             PythonLaunchDescriptionSource([
#         #                 PathJoinSubstitution([
#         #                     FindPackageShare("ur_moveit_config"),
#         #                     "launch", "ur_moveit.launch.py"
#         #                 ])
#         #             ]),
#         #             launch_arguments={
#         #                 "ur_type":           ur_type,
#         #                 "robot_ip":          robot_ip,
#         #                 "use_fake_hardware": use_fake_hw,
#         #                 "tf_prefix":         tf_prefix,
#         #                 "launch_rviz":       "false",
#         #             }.items(),
#         #         ),
#         #     ]
#         # ),

#         Node(
#             package='ros_tcp_endpoint',
#             executable='default_server_endpoint',
#             name='tcp_endpoint',
#             parameters=[{'ROS_IP': '127.0.0.1'}],
#             output='screen'
#         ),
#         # ── 3. IK solver node ─────────────────────────────────────────────────
#         # Delayed further to give move_group time to finish loading SRDF.
#         # The node also polls internally, so this is just an extra safety margin.
#         # TimerAction(
#         #     period=10.0,
#         #     actions=[
#         #         Node(
#         #             package="rs2_ros2_unity_bridge",
#         #             executable="ur3e_pose_bridge",
#         #             name="ur3e_pose_bridge",
#         #             output="screen",
#         #             parameters=[{
#         #                 "use_sim_time":   False,
#         #                 # "planning_group": planning_group,
#         #             }],
#         #         ),
#         #     ]
#         # ),

#         # Test Script

#         TimerAction(
#             period=10.0,
#             actions=[
#                 Node(
#                     package="rs2_ros2_unity_bridge",
#                     executable="test_joint_state",
#                     name="testing_script",
#                     output="screen",
#                     parameters=[{
#                         "use_sim_time":   False,
#                         # "planning_group": planning_group,
#                     }],
#                 ),
#             ]
#         ),
#     ])