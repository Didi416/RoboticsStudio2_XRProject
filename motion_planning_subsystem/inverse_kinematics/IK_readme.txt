
Pre-req for each terminal:

source ~/ros2_ws/install/setup.bash

source install/setup.bash


source /opt/ros/humble/setup.bash


terminal 1: ros2 run ros_tcp_endpoint default_server_endpoint   --ros-args -p ROS_IP:=127.0.0.1


terminal 2:
ros2 launch ur_onrobot_control start_robot.launch.py   ur_type:=ur3e onrobot_type:=rg2   use_fake_hardware:=true launch_rviz:=false


terminal 3:
ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py   ur_type:=ur3e onrobot_type:=rg2

terminal 4:
python3 puzzle_task.py