// Publishes a simple sine wave on all 6 UR3e joints to /ik_solution.
// Use this to verify the full pipeline:
//   ROS node → ROS-TCP-Connector → Unity → ArticulationBody drives
//
// Build and run:
//   colcon build
//   ros2 run your_package joint_state_test_publisher

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cmath>
#include <vector>
#include <string>

class JointStateTestPublisher : public rclcpp::Node
{
public:
    JointStateTestPublisher() : Node("joint_state_test_publisher"), time_(0.0)
    {
        pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/ik_solution", 10);

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&JointStateTestPublisher::jointCallback, this, std::placeholders::_1));

        // timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(20),
        //     std::bind(&JointStateTestPublisher::publish, this));

        RCLCPP_INFO(this->get_logger(),
            "Publishing UR3e joint states on /ik_solution.");
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState msgJS;
    double time_;

    // Joint names must match exactly what your URDF declares.
    // These are the standard UR3e joint names.
    const std::vector<std::string> JOINT_NAMES = {
        "base_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };

    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        pub_->publish(*msg);
    }

    void publish()
    {
        time_ += 0.02;  // 20 ms per tick

        
        msgJS.header.stamp = this->get_clock()->now();
        msgJS.name         = JOINT_NAMES;

        // Each joint oscillates at a slightly different frequency and amplitude
        // so you can visually distinguish which joint is which in Unity.
        // Amplitudes are small (0.3 rad ~ 17 deg) so the arm stays safe.
        // msg.position = {
        //     0.3  * std::sin(1.0 * time_),   // shoulder_pan   — slowest
        //     0.3  * std::sin(1.2 * time_),   // shoulder_lift
        //     0.3  * std::sin(1.4 * time_),   // elbow
        //     0.3  * std::sin(1.6 * time_),   // wrist_1
        //     0.3  * std::sin(1.8 * time_),   // wrist_2
        //     0.3  * std::sin(2.0 * time_),   // wrist_3        — fastest
        // };
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateTestPublisher>());
    rclcpp::shutdown();
    return 0;
}