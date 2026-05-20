"""
publish_calibration.launch.py — publishes the eye-in-hand calibration result
 
After running calibrate.launch.py and saving, this publishes the static TF:
  tool0  →  camera_color_optical_frame
 
Include this in your main system launch file so the camera frame is always
known relative to the robot end-effector.
 
The 'name' argument must exactly match the name used in calibrate.launch.py.
"""

# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory
# import os

# def generate_launch_description():
#     easy_handeye2_dir = get_package_share_directory('easy_handeye2')

#     publisher = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(easy_handeye2_dir, 'launch', 'publish.launch.py')
#         ),
#         launch_arguments={
#             'name': 'ur3e_eih_calib',   # must match calibrate.launch.py
#         }.items()
#     )

#     return LaunchDescription([publisher])

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tool0_to_camera_optical',
            output='screen',
            arguments=[
                '--x',  '0.011129624811515896',
                '--y',  '0.0019579919932155992',
                '--z',  '0.009992941644070896',
                '--qx', '0.035284824380418356',
                '--qy', '-0.043867356232744736',
                '--qz', '0.37892669952238706',
                '--qw', '0.9237127219079873',
                '--frame-id',       'tool0',
                '--child-frame-id', 'camera_color_optical_frame',
            ],
        )
    ])