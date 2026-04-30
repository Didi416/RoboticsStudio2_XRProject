"""
publish_calibration.launch.py — publishes the eye-in-hand calibration result
 
After running calibrate.launch.py and saving, this publishes the static TF:
  tool0  →  camera_color_optical_frame
 
Include this in your main system launch file so the camera frame is always
known relative to the robot end-effector.
 
The 'name' argument must exactly match the name used in calibrate.launch.py.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    easy_handeye2_dir = get_package_share_directory('easy_handeye2')

    publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(easy_handeye2_dir, 'launch', 'publish.launch.py')
        ),
        launch_arguments={
            'name': 'ur3e_eih_calib',   # must match calibrate.launch.py
        }.items()
    )

    return LaunchDescription([publisher])