/**
 * xr_servo_node.cpp
 * 
 * MoveIt2 Servo teleoperation node for UR3e driven by Meta Quest 2 XR input.
 * 
 * Architecture:
 *   Unity (C# XR script)
 *     └─► /xr_target_pose  (geometry_msgs/PoseStamped)
 *           └─► This node converts pose deltas → TwistStamped
 *                 └─► MoveIt Servo → joint trajectories
 *                       ├─► /joint_trajectory_controller/joint_trajectory  (real robot)
 *                       └─► /unity_joint_states  (digital twin feedback)
 * 
 * Build deps (package.xml / CMakeLists): see companion files.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <eigen3/Eigen/Geometry>
#include <moveit_msgs/msg/planning_scene.hpp>

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

// ──────────────────────────────────────────────────────────────────────────────
// Constants
// ──────────────────────────────────────────────────────────────────────────────

static constexpr double kLinearScale  = 1.5;   // m/s per metre of pose delta
static constexpr double kAngularScale = 2.0;   // rad/s per radian of pose delta
static constexpr double kDeadband     = 0.001; // metres – ignore sub-mm jitter
static constexpr double kAngDeadband  = 0.005; // radians

// ──────────────────────────────────────────────────────────────────────────────
// Node
// ──────────────────────────────────────────────────────────────────────────────

class XRServoNode : public rclcpp::Node
{
public:
  XRServoNode()
  : Node("xr_servo_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ── Parameters ────────────────────────────────────────────────────────────
    declare_parameter("planning_frame",       "base_link");
    declare_parameter("ee_frame",             "tool0");
    declare_parameter("linear_scale",         kLinearScale);
    declare_parameter("angular_scale",        kAngularScale);
    declare_parameter("publish_period_ms",    50);   // 20 Hz default; servo runs at its own rate
    declare_parameter("servo_command_topic",  "/servo_node/delta_twist_cmds");
    declare_parameter("joint_state_topic",    "/joint_states");
    declare_parameter("unity_js_topic",       "/unity_joint_states");

    planning_frame_  = get_parameter("planning_frame").as_string();
    ee_frame_        = get_parameter("ee_frame").as_string();
    lin_scale_       = get_parameter("linear_scale").as_double();
    ang_scale_       = get_parameter("angular_scale").as_double();
    int period_ms    = get_parameter("publish_period_ms").as_int();

    // ── Publishers ────────────────────────────────────────────────────────────

    // Feed MoveIt Servo its twist command
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      get_parameter("servo_command_topic").as_string(), 10);

    // Mirror joint states to Unity digital twin
    unity_js_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      get_parameter("unity_js_topic").as_string(), 10);

    // ── Subscribers ───────────────────────────────────────────────────────────

    // XR target pose from Unity C# script
    xr_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/xr_target_pose", 10,
      std::bind(&XRServoNode::xrPoseCallback, this, std::placeholders::_1));

    // Real robot joint states – relay to Unity
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      get_parameter("joint_state_topic").as_string(), 10,
      std::bind(&XRServoNode::jointStateCallback, this, std::placeholders::_1));

    // ── Timer: publish zero-twist when no XR command is fresh ─────────────────
    auto period = std::chrono::milliseconds(period_ms);
    timer_ = create_wall_timer(period, std::bind(&XRServoNode::timerCallback, this));

    RCLCPP_INFO(get_logger(),
      "XR Servo node ready. planning_frame=%s  ee_frame=%s",
      planning_frame_.c_str(), ee_frame_.c_str());
  }

private:
  // ── Callbacks ───────────────────────────────────────────────────────────────

  void xrPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Transform incoming pose into planning frame if needed
    geometry_msgs::msg::PoseStamped pose_in_base;
    try {
      tf_buffer_.transform(*msg, pose_in_base, planning_frame_,
                           tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      // If TF not ready yet, use message as-is
      pose_in_base = *msg;
      pose_in_base.header.frame_id = planning_frame_;
    }

    latest_target_ = pose_in_base;
    target_fresh_  = true;
    last_msg_time_ = now();
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Forward joint states to Unity digital twin unchanged
    unity_js_pub_->publish(*msg);
  }

  void timerCallback()
  {
    geometry_msgs::msg::TwistStamped twist_cmd;
    twist_cmd.header.stamp    = now();
    twist_cmd.header.frame_id = planning_frame_;

    // Stale target (>200 ms) → send zero twist so servo stops
    bool stale = (now() - last_msg_time_) > rclcpp::Duration::from_seconds(0.2);

    if (target_fresh_ && !stale) {
      // ── Retrieve current EE pose via TF ──────────────────────────────────
      geometry_msgs::msg::TransformStamped ee_tf;
      try {
        ee_tf = tf_buffer_.lookupTransform(
          planning_frame_, ee_frame_, tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Cannot get EE transform: %s", ex.what());
        twist_pub_->publish(twist_cmd); // zero twist
        return;
      }

      // ── Compute Cartesian delta ───────────────────────────────────────────
      double dx = latest_target_.pose.position.x - ee_tf.transform.translation.x;
      double dy = latest_target_.pose.position.y - ee_tf.transform.translation.y;
      double dz = latest_target_.pose.position.z - ee_tf.transform.translation.z;

      // Orientation delta via quaternion
      Eigen::Quaterniond q_cur(
        ee_tf.transform.rotation.w,
        ee_tf.transform.rotation.x,
        ee_tf.transform.rotation.y,
        ee_tf.transform.rotation.z);
      Eigen::Quaterniond q_tgt(
        latest_target_.pose.orientation.w,
        latest_target_.pose.orientation.x,
        latest_target_.pose.orientation.y,
        latest_target_.pose.orientation.z);

      Eigen::Quaterniond q_delta = q_cur.inverse() * q_tgt;
      q_delta.normalize();
      Eigen::AngleAxisd aa(q_delta);
      double angle = aa.angle();
      Eigen::Vector3d axis = aa.axis();
      // Wrap to [-π, π]
      if (angle > M_PI) { angle -= 2.0 * M_PI; }

      // ── Apply deadband & scale ────────────────────────────────────────────
      auto applyDeadband = [](double v, double db) {
        return (std::abs(v) < db) ? 0.0 : v;
      };

      dx = applyDeadband(dx, kDeadband) * lin_scale_;
      dy = applyDeadband(dy, kDeadband) * lin_scale_;
      dz = applyDeadband(dz, kDeadband) * lin_scale_;

      double da = applyDeadband(angle, kAngDeadband) * ang_scale_;

      twist_cmd.twist.linear.x  = dx;
      twist_cmd.twist.linear.y  = dy;
      twist_cmd.twist.linear.z  = dz;
      twist_cmd.twist.angular.x = da * axis.x();
      twist_cmd.twist.angular.y = da * axis.y();
      twist_cmd.twist.angular.z = da * axis.z();

      target_fresh_ = false; // consume
    }
    // else: zero twist already set

    twist_pub_->publish(twist_cmd);
  }

  // ── Members ─────────────────────────────────────────────────────────────────
  tf2_ros::Buffer   tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr  twist_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr       unity_js_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr xr_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr    joint_state_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PoseStamped latest_target_;
  bool target_fresh_{false};
  rclcpp::Time last_msg_time_{0, 0, RCL_ROS_TIME};

  std::string planning_frame_;
  std::string ee_frame_;
  double lin_scale_{kLinearScale};
  double ang_scale_{kAngularScale};
};

// ──────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XRServoNode>());
  rclcpp::shutdown();
  return 0;
}