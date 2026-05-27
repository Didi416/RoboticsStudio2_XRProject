import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rs2_xr_puzzle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='dyandra',
    maintainer_email='didiprins@optusnet.com.au',
    description='Unity VR → MoveIt 2 Servo bridge for UR3e teleoperation',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arduino_bridge = rs2_xr_puzzle.arduino_bridge:main',
        ],
    },
)
