# realsense_bridge.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # RealSense D435i driver
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            parameters=[{
                'color_width':  640,
                'color_height': 480,
                'color_fps':    30.0,
                'enable_depth': True,   # keep for point cloud later
            }]
        ),
        # ROS-TCP-Endpoint (Unity bridge)
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='ros_tcp_endpoint',
            parameters=[{'ROS_IP': '0.0.0.0', 'ROS_PORT': 10000}]
        ),
    ])