cmake_minimum_required(VERSION 3.8)
project(rs2_ros2_unity_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ur_rtde REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(test_ik_node src/test_ik_node.cpp)
ament_target_dependencies(test_ik_node rclcpp sensor_msgs geometry_msgs trajectory_msgs moveit_msgs control_msgs std_srvs tf2_ros tf2_geometry_msgs)

add_executable(ur3e_bridge src/ur3e_bridge.cpp)
ament_target_dependencies(ur3e_bridge rclcpp geometry_msgs sensor_msgs)

# target_link_libraries(ur3e_rtde_bridge
#   ur_rtde::rtde_control
#   ur_rtde::rtde_receive
# )

add_executable(ur3e_pose_bridge src/ur3e_pose_bridge.cpp)
ament_target_dependencies(ur3e_pose_bridge rclcpp sensor_msgs geometry_msgs std_msgs control_msgs moveit_ros_planning_interface)

add_executable(test_joint_state src/test_joint_publisher.cpp)
ament_target_dependencies(test_joint_state rclcpp sensor_msgs geometry_msgs std_msgs control_msgs moveit_ros_planning_interface)

target_include_directories(test_joint_state PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(ur3e_pose_bridge Eigen3::Eigen)

install(TARGETS ur3e_pose_bridge test_joint_state ur3e_bridge test_ik_node
DESTINATION lib/${PROJECT_NAME})
 
install(DIRECTORY configs/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
