# RoboticsStudio2_XRProject
RS2 XR Project with Unity and UR3e robot - VR Escape Room

_NOTE_
_This is a draft ReadMe of initial commands to run the system. Please refer to this GitHub's Wiki page for comprehensive instructions and information. Edits to thsi ReadMe will come soon._

Commands to launch fake hardware test with ur_onrobot packages:
- Clone repository UR_Onrobot_ROS2
- Change the yaml file references from forward position to forward velocity controllers (in all packages, ur_onrobot_control and ur_onrobot_moveit_config)
	- changed controllers.yaml, ur_onrobot_controllers.yaml and ur_onrobot_servo.yaml
- ros2 launch ur_onrobot_control start_robot.launch.py use_fake_hardware:=true
- ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py ur_type:=ur3e onrobot_type:=rg2
- python3 keyboard_bridge.py (make sure output commands topic is also set to forward velocity controller)


Single Launch file method:
Launch URSim:
```sh
ros2 run ur_client_library start_ursim.sh -m ur3e
```

Launch XR full stack launch file (includes UR Driver, MoveIt, controller switching, Servo service trigger, ROS TCP Endpoint)
```sh
ros2 launch rs2_xr_puzzle xr_full_stack.launch.py 
```
Configurable launch arguments:
- `robot_ip` (default: `192.168.56.101`): Default URSim, can be change to real robot IP
- `ros_tcp_host`, (default: `127.0.0.1`): Default Unity host IP that ROS-TCP-Endpoint listens on.',

To change `ros_tcp_host`, you must first change the "Unity --> Robotics --> ROS Settings" ROS Host IP Address to your ROS machine IP, which can be found using:
```sh
hostname -I
```
(usually first IP address listed).

To launch OnRobot Gripper (use_fake_hardware:=true)
```sh
ros2 launch rs2_xr_puzzle ur_onrobot_group.launch.py 
```
___________________________________________________

If you want to launch each CLI call individually:
Launch Teleop with URSim:
```sh
ros2 run ur_client_library start_ursim.sh -m ur3e
```
URSIM launch
```sh
ros2 launch rs2_xr_puzzle ur_driver_modified.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=true 
```
(change port to 50001 if ti reads Addres already in use in terminal)
```sh
ros2 launch rs2_xr_puzzle ur_moveit_modified.launch.py ur_type:=ur3e launch_rviz:=true
```
```sh
ros2 control switch_controllers --activate forward_velocity_controller --deactivate scaled_joint_trajectory_controller
```
```sh
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
```
```sh
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1
```
```sh
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=172.19.116.163 
```
(for VR Headset connection)


For onrobot simulation (not URSim, enable use_fake_hardware:=true)
Note: You must make sure the yaml files (from above) have changed their target controllers, especially if/when you clone the OnRobot package direct from source.

ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 use_fake_hardware:=true
ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 robot_ip:=192.168.0.194
ros2 launch rs2_xr_puzzle ur_onrobot_group.launch.py robot_ip:=192.168.0.197 ros_tcp_host:=172.19.117.235

ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py ur_type:=ur3e onrobot_type:=rg2


