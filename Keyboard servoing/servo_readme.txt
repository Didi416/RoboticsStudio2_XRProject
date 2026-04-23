terminal 1:
ros2 launch ur_robot_driver ur_control.launch.py   ur_type:=ur3e use_fake_hardware:=true   launch_rviz:=false robot_ip:=127.0.0.1

terminal 2:
ros2 launch ur_moveit_config ur_moveit.launch.py   ur_type:=ur3e launch_rviz:=true   use_fake_hardware:=true

terminal 3:
python3 keyboard_bridge.py

notes: wait for servo to activate before trying inputs, also may need to use this command for first use:
sudo apt install ros-humble-controller-manager-msgs