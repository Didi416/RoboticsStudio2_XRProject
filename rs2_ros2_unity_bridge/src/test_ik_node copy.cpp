/**
 * ik_test_node.cpp
 *
 * Cycles through hardcoded EE poses, solves IK via MoveIt /compute_ik,
 * publishes JointState to /ik_solution (→ Unity) and sends trajectory
 * to /scaled_joint_trajectory_controller/joint_trajectory (→ URSim).
 *
 * Launch prerequisites:
 *   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=true
 *   ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=false
 *   ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <unordered_map>
#include <chrono>

using namespace std::chrono_literals;

// ── Hardcoded test poses [x, y, z, qx, qy, qz, qw] in base_link frame ────────
struct Pose7 { double x, y, z, qx, qy, qz, qw; const char* label; };

static const std::vector<Pose7> TEST_POSES = {
  { 0.3,  0.0,  0.4,  0.0,  0.0,  0.0,  0.0, "front_centre"  },
  { 0.2,  0.2,  0.4,  0.0,  0.0,  0.0,  0.0, "front_left"    },
  { 0.2, -0.2,  0.4,  0.0,  0.0,  0.0,  0.0, "front_right"   },
  { 0.3,  0.0,  0.3,  0.0,  0.0,  0.0,  0.0, "front_low"     },
  { 0.3,  0.0,  0.5,  0.0,  0.0,  0.0,  0.0, "front_high"    },
};

static const std::vector<std::string> JOINT_NAMES = {
  "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
  "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"
};

// ─────────────────────────────────────────────────────────────────────────────

class IKTestNode : public rclcpp::Node
{
public:
  IKTestNode()
  : Node("ik_test_node"),
    pose_index_(0)
  {
    // ── Publishers ────────────────────────────────────────────────────────────
    ik_solution_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/ik_solution", 10);

    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/scaled_joint_trajectory_controller/joint_trajectory", 10);

    // ── Subscribers ────────────────────────────────────────────────────────────
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        current_joint_state_ = *msg;
        has_joint_state_ = true;
      });

    // ── IK service client ─────────────────────────────────────────────────────
    ik_client_ = create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");

    RCLCPP_INFO(get_logger(), "Waiting for /compute_ik service...");
    if (!ik_client_->wait_for_service(10s)) {
      RCLCPP_FATAL(get_logger(),
        "/compute_ik not available. Is ur_moveit.launch.py running?");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "IK service ready. Starting pose cycle...");

    // ── Timer: step through poses every 4 seconds ─────────────────────────────
    timer_ = create_wall_timer(4s, std::bind(&IKTestNode::onTimer, this));
  }

private:
  // ── Timer callback — fires every 4s ───────────────────────────────────────
  void onTimer()
  {
    const Pose7 & p = TEST_POSES[pose_index_];
    RCLCPP_INFO(get_logger(),
      "[%zu/%zu] Solving IK for pose '%s' → [%.3f, %.3f, %.3f]",
      pose_index_ + 1, TEST_POSES.size(), p.label, p.x, p.y, p.z);

    solveAndSend(p);

    // Advance to next pose, wrap around
    pose_index_ = (pose_index_ + 1) % TEST_POSES.size();
  }

  // ── Build and send IK request ──────────────────────────────────────────────
  void solveAndSend(const Pose7 & p)
  {
    auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    auto & ik    = request->ik_request;

    ik.group_name       = "ur_manipulator";
    ik.avoid_collisions = true;
    ik.timeout.sec      = 2;

    ik.pose_stamped.header.frame_id = "base_link";
    ik.pose_stamped.header.stamp    = now();
    ik.pose_stamped.pose.position.x    = p.x;
    ik.pose_stamped.pose.position.y    = p.y;
    ik.pose_stamped.pose.position.z    = p.z;
    ik.pose_stamped.pose.orientation.x = p.qx;
    ik.pose_stamped.pose.orientation.y = p.qy;
    ik.pose_stamped.pose.orientation.z = p.qz;
    ik.pose_stamped.pose.orientation.w = p.qw;

    ik.robot_state.joint_state = current_joint_state_;

    // Async — callback handles response
    ik_client_->async_send_request(request,
      [this, label = std::string(p.label)](
        rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future)
      {
        onIKResponse(future, label);
      });
  }

  // ── IK response callback ───────────────────────────────────────────────────
  void onIKResponse(
    rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future,
    const std::string & label)
  {
    auto result = future.get();

    if (result->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_WARN(get_logger(),
        "IK failed for '%s', error code: %d", label.c_str(),
        result->error_code.val);
      return;
    }

    // ── Extract joint positions in correct order ────────────────────────────
    const auto & js = result->solution.joint_state;
    std::unordered_map<std::string, double> pos_map;
    for (size_t i = 0; i < js.name.size(); ++i)
      pos_map[js.name[i]] = js.position[i];

    std::vector<double> positions;
    positions.reserve(JOINT_NAMES.size());
    for (const auto & name : JOINT_NAMES) {
      if (pos_map.count(name) == 0) {
        RCLCPP_WARN(get_logger(), "Joint '%s' missing from IK solution", name.c_str());
        return;
      }
      positions.push_back(pos_map.at(name));
    }

    RCLCPP_INFO(get_logger(),
      "IK solved '%s': [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] rad",
      label.c_str(),
      positions[0], positions[1], positions[2],
      positions[3], positions[4], positions[5]);

    // ── 1. Publish JointState → Unity /ik_solution ─────────────────────────
    sensor_msgs::msg::JointState solution;
    solution.header.stamp = now();
    solution.name         = JOINT_NAMES;
    solution.position     = positions;
    solution.velocity     = std::vector<double>(6, 0.0);
    solution.effort       = std::vector<double>(6, 0.0);
    ik_solution_pub_->publish(solution);

    // ── 2. Send trajectory → URSim ─────────────────────────────────────────
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = now();
    traj.joint_names = JOINT_NAMES;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.velocities = std::vector<double>(6, 0.0);
    point.time_from_start.sec = 3;   // 3 seconds to reach pose
    point.time_from_start.nanosec = 0;

    traj.points = {point};
    traj_pub_->publish(traj);

    // ── Update our seed for the NEXT IK call ──────────────────────────────────
    // Use the solution we just sent as the seed for the next pose,
    // so each move chains from the previous solution
    for (size_t i = 0; i < JOINT_NAMES.size(); ++i) {
      for (size_t j = 0; j < current_joint_state_.name.size(); ++j) {
        if (current_joint_state_.name[j] == JOINT_NAMES[i]) {
          current_joint_state_.position[j] = positions[i];
          break;
        }
      }
    }

    RCLCPP_INFO(get_logger(),
      "Published to /ik_solution (Unity) and /scaled_joint_trajectory_controller (URSim)");
  }

  // ── Members ────────────────────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ik_solution_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t pose_index_;
  sensor_msgs::msg::JointState current_joint_state_;
  bool has_joint_state_ = false;
};

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKTestNode>());
  rclcpp::shutdown();
  return 0;
}



// /**
//  * servo_teleop_node.cpp
//  *
//  * Bridges Unity → MoveIt Servo for UR3e teleoperation.
//  *
//  * Unity publishes:   /unity/target_pose  (geometry_msgs/PoseStamped)
//  *                    absolute EE pose in base_link frame
//  *
//  * This node converts absolute poses → incremental TwistStamped deltas
//  * and streams them to MoveIt Servo at a fixed rate.
//  *
//  * Requires:
//  *   ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=false
//  *   ros2 launch moveit_servo servo.launch.py
//  */

// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/msg/twist_stamped.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <std_srvs/srv/trigger.hpp>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>

// using namespace std::chrono_literals;

// class ServoTeleopNode : public rclcpp::Node
// {
// public:
//   ServoTeleopNode()
//   : Node("servo_teleop_node"),
//     tf_buffer_(get_clock()),
//     tf_listener_(tf_buffer_),
//     has_target_(false),
//     has_current_(false)
//   {
//     // ── Parameters ────────────────────────────────────────────────────────────
//     declare_parameter("servo_rate_hz",     50.0);   // how fast we publish to servo
//     declare_parameter("linear_scale",       1.0);   // m/s per metre of error
//     declare_parameter("angular_scale",      1.0);   // rad/s per rad of error
//     declare_parameter("max_linear_vel",     0.15);  // m/s  — safety clamp
//     declare_parameter("max_angular_vel",    0.5);   // rad/s — safety clamp
//     declare_parameter("deadband_linear",    0.001); // m  — ignore tiny errors
//     declare_parameter("deadband_angular",   0.005); // rad

//     const double rate_hz       = get_parameter("servo_rate_hz").as_double();
//     linear_scale_              = get_parameter("linear_scale").as_double();
//     angular_scale_             = get_parameter("angular_scale").as_double();
//     max_linear_vel_            = get_parameter("max_linear_vel").as_double();
//     max_angular_vel_           = get_parameter("max_angular_vel").as_double();
//     deadband_linear_           = get_parameter("deadband_linear").as_double();
//     deadband_angular_          = get_parameter("deadband_angular").as_double();

//     // ── Publishers ────────────────────────────────────────────────────────────
//     twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
//       "/servo_node/delta_twist_cmds", 10);

//     posePub_ = this->create_publisher<sensor_msgs::msg::JointState>("/ik_solution", 10);

//     // ── Subscribers ───────────────────────────────────────────────────────────
//     joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
//             "/joint_states", 10,
//             std::bind(&ServoTeleopNode::jointCallback, this, std::placeholders::_1));

//     // Unity sends absolute target poses here
//     pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
//       "/ik_target", 10,
//       [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
//         target_pose_ = *msg;
//         has_target_  = true;
//         RCLCPP_INFO(get_logger(),"Receiving target pose: x=%.3f, y=%.3f, z=%.3f", msg->position.x, msg->position.y, msg->position.z);
//       });

//     // ── Start MoveIt Servo ────────────────────────────────────────────────────
//     startServo();

//     // ── Control loop timer ────────────────────────────────────────────────────
//     timer_ = create_wall_timer(
//       std::chrono::duration<double>(1.0 / rate_hz),
//       std::bind(&ServoTeleopNode::controlLoop, this));

//     RCLCPP_INFO(get_logger(),
//       "ServoTeleopNode ready. Listening on /ik_target at %.0f Hz", rate_hz);
//   }

// private:

//   void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
//     {
//         // RCLCPP_INFO(this->get_logger(), "Joint: %s", msg->name[0].c_str());
//         // RCLCPP_INFO(this->get_logger(), "Joint: %s", msg->name[1].c_str());
//         // RCLCPP_INFO(this->get_logger(), "Joint: %s", msg->name[2].c_str());
//         // RCLCPP_INFO(this->get_logger(), "Joint: %s", msg->name[3].c_str());
//         // RCLCPP_INFO(this->get_logger(), "Joint: %s", msg->name[4].c_str());
//         // RCLCPP_INFO(this->get_logger(), "Joint: %s", msg->name[5].c_str());
//         posePub_->publish(*msg);
//     }
//   // ── MoveIt Servo startup ───────────────────────────────────────────────────
//   void startServo()
//   {
//     auto client = create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
//     if (!client->wait_for_service(5s)) {
//       RCLCPP_WARN(get_logger(),
//         "/servo_node/start_servo not available — is servo.launch.py running?");
//       return;
//     }
//     auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
//     client->async_send_request(req);
//   }

//   // ── Main control loop ──────────────────────────────────────────────────────
//   void controlLoop()
//   {
//     if (!has_target_) {
//       publishZeroTwist();  // keep servo alive but don't move
//       return;
//     }

//     // Look up current EE pose via TF
//     geometry_msgs::msg::TransformStamped tf;
//     try {
//       tf = tf_buffer_.lookupTransform(
//         "base_link", "tool0", tf2::TimePointZero);
//     } catch (const tf2::TransformException & ex) {
//       RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
//         "TF lookup failed: %s", ex.what());
//       publishZeroTwist();
//       return;
//     }

//     // ── Compute linear error (base_link frame) ────────────────────────────────
//     double ex = target_pose_.position.x - tf.transform.translation.x;
//     double ey = target_pose_.position.y - tf.transform.translation.y;
//     double ez = target_pose_.position.z - tf.transform.translation.z;

//     // ── Compute angular error via quaternion difference ────────────────────────
//     tf2::Quaternion q_current(
//       tf.transform.rotation.x,
//       tf.transform.rotation.y,
//       tf.transform.rotation.z,
//       tf.transform.rotation.w);

//     tf2::Quaternion q_target(
//       target_pose_.orientation.x,
//       target_pose_.orientation.y,
//       target_pose_.orientation.z,
//       target_pose_.orientation.w);

//     tf2::Quaternion q_error = q_target * q_current.inverse();
//     q_error.normalize();

//     // Convert quaternion error → axis-angle
//     double angle = 2.0 * std::acos(std::clamp(q_error.w(), -1.0, 1.0));
//     double sin_half = std::sqrt(1.0 - q_error.w() * q_error.w());

//     double ax = 0.0, ay = 0.0, az = 0.0;
//     if (sin_half > 1e-6) {
//       ax = q_error.x() / sin_half;
//       ay = q_error.y() / sin_half;
//       az = q_error.z() / sin_half;
//     }

//     // ── Apply deadbands ────────────────────────────────────────────────────────
//     double linear_err = std::sqrt(ex*ex + ey*ey + ez*ez);
//     if (linear_err < deadband_linear_) { ex = ey = ez = 0.0; }

//     if (std::abs(angle) < deadband_angular_) { angle = 0.0; }

//     // ── Scale and clamp velocities ─────────────────────────────────────────────
//     auto clamp = [](double v, double limit) {
//       return std::clamp(v, -limit, limit);
//     };

//     double vx = clamp(linear_scale_ * ex, max_linear_vel_);
//     double vy = clamp(linear_scale_ * ey, max_linear_vel_);
//     double vz = clamp(linear_scale_ * ez, max_linear_vel_);

//     double wx = clamp(angular_scale_ * angle * ax, max_angular_vel_);
//     double wy = clamp(angular_scale_ * angle * ay, max_angular_vel_);
//     double wz = clamp(angular_scale_ * angle * az, max_angular_vel_);

//     // ── Publish TwistStamped ───────────────────────────────────────────────────
//     geometry_msgs::msg::TwistStamped twist;
//     twist.header.stamp    = now();
//     twist.header.frame_id = "base_link";  // frame the twist is expressed in
//     twist.twist.linear.x  = vx;
//     twist.twist.linear.y  = vy;
//     twist.twist.linear.z  = vz;
//     twist.twist.angular.x = wx;
//     twist.twist.angular.y = wy;
//     twist.twist.angular.z = wz;

//     twist_pub_->publish(twist);
//   }

//   void publishZeroTwist()
//   {
//     geometry_msgs::msg::TwistStamped twist;
//     twist.header.stamp    = now();
//     twist.header.frame_id = "base_link";
//     twist_pub_->publish(twist);
//   }

//   // ── Members ────────────────────────────────────────────────────────────────
//   rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
//   rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_client_;
//   rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr posePub_;
//   rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

//   tf2_ros::Buffer tf_buffer_;
//   tf2_ros::TransformListener tf_listener_;

//   geometry_msgs::msg::Pose target_pose_;
//   bool has_target_;
//   bool has_current_;

//   double linear_scale_;
//   double angular_scale_;
//   double max_linear_vel_;
//   double max_angular_vel_;
//   double deadband_linear_;
//   double deadband_angular_;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<ServoTeleopNode>());
//   rclcpp::shutdown();
//   return 0;
// }