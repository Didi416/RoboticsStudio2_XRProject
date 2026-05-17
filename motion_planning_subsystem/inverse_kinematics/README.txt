
source /opt/ros/humble/setup.bash

terminal 1:
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true launch_servo:=true use_sim_time:=false

terminal 2:
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e use_fake_hardware:=true launch_rviz:=false robot_ip:=127.0.0.1 

terminal 3:
python3 collision_tf.py