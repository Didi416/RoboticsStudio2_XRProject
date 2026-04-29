
source /opt/ros/humble/setup.bash

terminal 1:
ros2 launch ur_onrobot_control start_robot.launch.py   ur_type:=ur3e onrobot_type:=rg2   use_fake_hardware:=true launch_rviz:=false


terminal 2:
ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py   ur_type:=ur3e onrobot_type:=rg2

terminal 3:
python3 keyboard_bridge.py

notes: wait for servo to activate before trying inputs, also may need to use this command for first use:
sudo apt install ros-humble-controller-manager-msgs