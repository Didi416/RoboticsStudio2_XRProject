# import os
# from glob import glob
# from setuptools import find_packages, setup

# package_name = 'perception_mapping'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
    
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         (os.path.join('share', package_name, 'launch'),  
#             glob('launch/*.launch.py')),
#     ],
    
    
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='amadee',
#     maintainer_email='amadee.r.thotawatta@student.uts.edu.au',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     extras_require={
#         'test': [
#             'pytest',
#         ],
#     },
#     entry_points={
#         'console_scripts': [
#             'aruco_detector = perception_mapping.aruco_detector:main',
#             'multi_aruco_detector = perception_mapping.multi_aruco_detector:main',
#             'puzzle_object_detector = perception_mapping.puzzle_object_detector:main',
#             'webcam_realsense_bridge = perception_mapping.webcam_realsense_bridge:main',
#             #'checkerboard_pose_publisher = perception_mapping.checkerboard_pose_publisher:main',
#             # REMOVE checkerboard_pose_publisher, ADD charuco:
#             'charuco_pose_publisher = perception_mapping.charuco_pose_publisher:main',
#             # Debug viewer — live camera feed with ChArUco detection overlay
#             'charuco_debug_viewer = perception_mapping.charuco_debug_viewer:main',
#         ],
#     },
#)


from setuptools import setup
import os
from glob import glob

package_name = 'perception_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('perception_subsystem/launch/*.py')),

        # ('share/ament_index/resource_index/packages', ['resource/perception_mapping']),
        # ('share/perception_mapping', ['package.xml']),
        # (os.path.join('share', 'perception_mapping', 'config'),glob('perception_mapping/config/*.yaml')),
        # (os.path.join('share', 'perception_mapping', 'launch'),glob('perception_mapping/launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'aruco_detector_node        = perception_mapping.aruco_detector_node:main',
            'aruco_marker_generator      = perception_mapping.aruco_marker_generator:main',
            'multi_aruco_detector = perception_mapping.multi_aruco_detector:main',
            'calibration_publisher      = perception_mapping.calibration_publisher:main',
            'checkerboard_pose_publisher = perception_mapping.checkerboard_pose_publisher:main',
            'charuco_pose_publisher = perception_mapping.charuco_pose_publisher:main',
            'charuco_debug_viewer = perception_mapping.charuco_debug_viewer:main',
            'puzzle_wall_pose_publisher = perception_mapping.puzzle_wall_pose_publisher:main',
            'puzzle_object_detector = perception_mapping.puzzle_object_detector:main',
            'pose_filter_node = perception_mapping.pose_filter_node:main',
            'webcam_realsense_bridge = perception_mapping.webcam_realsense_bridge:main',
            
            'egg_sorter_verifier = perception_mapping.egg_sorter_verifier:main',
            'egg_sorter_verifier_simple = perception_mapping.egg_sorter_verifier_simple:main',
            'unity_pose_publisher = perception_mapping.unity_pose_publisher:main',
        ],
    },
)