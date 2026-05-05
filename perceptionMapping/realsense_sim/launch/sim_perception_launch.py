import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('realsense_sim')
    world_file = os.path.join(pkg_share, 'worlds', 'aruco_test.sdf')

    return LaunchDescription([

        # 1. Start Gazebo Ignition with the world (-r = run immediately)
        ExecuteProcess(
            cmd=['ign', 'gazebo', world_file, '-r'],
            output='screen'
        ),

        # 2. Bridge: colour image  Ignition → ROS2
        #    Ignition topic type:  ignition.msgs.Image
        #    ROS2 topic type:      sensor_msgs/msg/Image
        TimerAction(period=3.0, actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='bridge_color',
                arguments=[
                    '/realsense/image'
                    '@sensor_msgs/msg/Image'
                    '[ignition.msgs.Image'
                ],
                remappings=[
                    ('/realsense/image', '/camera/color/image_raw')
                ],
                output='screen'
            ),
        ]),

        # 3. Bridge: depth image  Ignition → ROS2
        TimerAction(period=3.0, actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='bridge_depth',
                arguments=[
                    '/realsense/depth_image'
                    '@sensor_msgs/msg/Image'
                    '[ignition.msgs.Image'
                ],
                remappings=[
                    ('/realsense/depth_image', '/camera/depth/image_rect_raw')
                ],
                output='screen'
            ),
        ]),

        # 4. Bridge: camera_info  Ignition → ROS2
        TimerAction(period=3.0, actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='bridge_info',
                arguments=[
                    '/realsense/camera_info'
                    '@sensor_msgs/msg/CameraInfo'
                    '[ignition.msgs.CameraInfo'
                ],
                remappings=[
                    ('/realsense/camera_info', '/camera/color/camera_info')
                ],
                output='screen'
            ),
        ]),

        # 5. Bridge: point cloud  Ignition → ROS2
        TimerAction(period=3.0, actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='bridge_points',
                arguments=[
                    '/realsense/points'
                    '@sensor_msgs/msg/PointCloud2'
                    '[ignition.msgs.PointCloudPacked'
                ],
                remappings=[
                    ('/realsense/points', '/camera/depth/color/points')
                ],
                output='screen'
            ),
        ]),

        # 6. Static TF: camera optical frame relative to world origin
        #    Matches the camera pose set in the world SDF (x=0, z=0.4, pitch=0.52)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_pub',
            arguments=[
                '0', '0', '0.4',   # x y z translation
                '0', '0.52', '0',  # roll pitch yaw — matches SDF pose
                'world',
                'camera_color_optical_frame'
            ]
        ),

        # 7. Your ArUco detector — subscribes to /camera/color/image_raw
        #    This node is identical to what runs on real hardware
        TimerAction(period=4.0, actions=[
            Node(
                package='perception_mapping',
                executable='aruco_detector',
                name='aruco_detector',
                output='screen'
            ),
        ]),

    ])