# RoboticsStudio2_XRProject
RS2 XR Project with Unity and UR3e robot - VR Escape Room

Commands to launch fake hardware test with ur_onrobot packages:
- Clone repository UR_Onrobot_ROS2
- Change the yaml file references from forward position to forward velocity controllers (in all packages, ur_onrobot_control and ur_onrobot_moveit_config)
	- changed controllers.yaml, ur_onrobot_controllers.yaml and ur_onrobot_servo.yaml
- ros2 launch ur_onrobot_control start_robot.launch.py use_fake_hardware:=true
- ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py ur_type:=ur3e onrobot_type:=rg2
- python3 keyboard_bridge.py (make sure output commands topic is also set to forward velocity controller)


Launch Teleop with URSim:
```ros2 run ur_client_library start_ursim.sh -m ur3e
```
```ros2 launch rs2_ros2_unity_bridge ur_test.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=true (change port to 50001 if ti reads Addres already in use in terminal)
```
```ros2 launch rs2_ros2_unity_bridge ur_moveit_test.launch.py ur_type:=ur3e launch_rviz:=true
```
```ros2 control switch_controllers --activate forward_velocity_controller --deactivate scaled_joint_trajectory_controller
```
```ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
```
```ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1
```
```ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=172.19.116.163 (for VR Headset connection)
```
