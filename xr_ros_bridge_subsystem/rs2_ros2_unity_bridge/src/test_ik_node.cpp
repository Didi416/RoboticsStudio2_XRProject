#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <unordered_map>
#include <chrono>

using namespace std::chrono_literals;

struct Pose7 { double x, y, z, qx, qy, qz, qw; const char* label; };

static const std::vector<Pose7> TEST_POSES = {
  { 0.3,  0.0,  0.4,  0.0,  0.707,  0.0,  0.707, "front_centre" },
  { 0.2,  0.2,  0.4,  0.0,  0.707,  0.0,  0.707, "front_left"   },
  { 0.2, -0.2,  0.4,  0.0,  0.707,  0.0,  0.707, "front_right"  },
  { 0.3,  0.0,  0.3,  0.0,  0.707,  0.0,  0.707, "front_low"    },
  { 0.3,  0.0,  0.5,  0.0,  0.707,  0.0,  0.707, "front_high"   },
};

static const std::vector<std::string> JOINT_NAMES = {
  "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
  "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"
};

class IKTestNode : public rclcpp::Node
{
public:
  IKTestNode()
  : Node("ik_test_node"),
    pose_index_(0),
    has_joint_state_(false)
  {
    ik_solution_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/ik_solution", 10);

    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/scaled_joint_trajectory_controller/joint_trajectory", 10);

    ik_client_ = create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");

    // ── Subscribe to current joint states — used as IK seed ──────────────────
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        current_joint_state_ = *msg;
        has_joint_state_     = true;
      });

    RCLCPP_INFO(get_logger(), "Waiting for /compute_ik service...");
    if (!ik_client_->wait_for_service(10s)) {
      RCLCPP_FATAL(get_logger(), "/compute_ik not available.");
      rclcpp::shutdown();
      return;
    }

    // Wait until we have a joint state before starting
    RCLCPP_INFO(get_logger(), "Waiting for /joint_states...");
    while (!has_joint_state_) {
      rclcpp::spin_some(shared_from_this());
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(get_logger(), "Got joint states. Starting pose cycle.");

    timer_ = create_wall_timer(4s, std::bind(&IKTestNode::onTimer, this));
  }

private:
  void onTimer()
  {
    const Pose7 & p = TEST_POSES[pose_index_];
    RCLCPP_INFO(get_logger(),
      "[%zu/%zu] Solving IK for '%s' seeded from current joint state",
      pose_index_ + 1, TEST_POSES.size(), p.label);

    solveAndSend(p);
    pose_index_ = (pose_index_ + 1) % TEST_POSES.size();
  }

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

    // ── KEY FIX: seed IK with current robot joint state ───────────────────────
    // This tells the IK solver to start searching from where the robot
    // currently is, so it finds the closest valid solution instead of
    // a random one from scratch
    ik.robot_state.joint_state = current_joint_state_;

    ik_client_->async_send_request(request,
      [this, label = std::string(p.label)](
        rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future)
      {
        onIKResponse(future, label);
      });
  }

  void onIKResponse(
    rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture future,
    const std::string & label)
  {
    auto result = future.get();

    if (result->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_WARN(get_logger(), "IK failed for '%s', error: %d",
        label.c_str(), result->error_code.val);
      return;
    }

    const auto & js = result->solution.joint_state;
    std::unordered_map<std::string, double> pos_map;
    for (size_t i = 0; i < js.name.size(); ++i)
      pos_map[js.name[i]] = js.position[i];

    std::vector<double> positions;
    positions.reserve(JOINT_NAMES.size());
    for (const auto & name : JOINT_NAMES) {
      if (pos_map.count(name) == 0) {
        RCLCPP_WARN(get_logger(), "Joint '%s' missing", name.c_str());
        return;
      }
      positions.push_back(pos_map.at(name));
    }

    // Log joint delta from current to new — useful for debugging safety
    logJointDeltas(positions);

    // Publish to Unity
    sensor_msgs::msg::JointState solution;
    solution.header.stamp = now();
    solution.name         = JOINT_NAMES;
    solution.position     = positions;
    solution.velocity     = std::vector<double>(6, 0.0);
    solution.effort       = std::vector<double>(6, 0.0);
    ik_solution_pub_->publish(solution);

    // Send to URSim
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = now();
    traj.joint_names  = JOINT_NAMES;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions               = positions;
    point.velocities              = std::vector<double>(6, 0.0);
    point.time_from_start.sec     = 3;
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

    RCLCPP_INFO(get_logger(), "Done '%s' → seed updated for next call", label.c_str());
  }

  void logJointDeltas(const std::vector<double> & new_positions)
  {
    std::unordered_map<std::string, double> current_map;
    for (size_t i = 0; i < current_joint_state_.name.size(); ++i)
      current_map[current_joint_state_.name[i]] = current_joint_state_.position[i];

    double total_delta = 0.0;
    for (size_t i = 0; i < JOINT_NAMES.size(); ++i) {
      double delta = std::abs(new_positions[i] - current_map[JOINT_NAMES[i]]);
      total_delta += delta;
      RCLCPP_INFO(get_logger(), "  %s delta: %.3f rad (%.1f deg)",
        JOINT_NAMES[i].c_str(), delta, delta * 180.0 / M_PI);
    }
    RCLCPP_INFO(get_logger(), "  Total joint travel: %.3f rad", total_delta);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr          ik_solution_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr           ik_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr        joint_state_sub_;
  rclcpp::TimerBase::SharedPtr                                          timer_;

  sensor_msgs::msg::JointState current_joint_state_;
  bool                          has_joint_state_;
  size_t                        pose_index_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKTestNode>());
  rclcpp::shutdown();
  return 0;
}