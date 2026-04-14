#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using std::placeholders::_1;

class UR3eSimpleBridge : public rclcpp::Node
{
public:
    UR3eSimpleBridge() : Node("ur3e_simple_bridge")
    {
        // Receive EE target (Unity or internal)
        ee_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/ik_target", 10,
            std::bind(&UR3eSimpleBridge::eeCallback, this, _1));

        // Send to UR controller
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(
            "/cartesian_controller/command", 10);

        // Receive robot joint states
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&UR3eSimpleBridge::jointCallback, this, _1));

        // Send to Unity
        unity_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/ik_solution", 10);

        // Optional: timer for testing without Unity
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&UR3eSimpleBridge::testPublish, this));

        RCLCPP_INFO(this->get_logger(), "Simple UR3e Bridge running");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ee_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr unity_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Pose last_pose_;

    double inc = 0.0;

    // From Unity → UR
    void eeCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // last_pose_ = convertPose(*msg);
        // pose_pub_->publish(last_pose_);
    }

    // From UR → Unity
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        unity_pub_->publish(*msg);
    }

    // TEST: send hardcoded pose (no Unity needed)
    void testPublish()
    {
        geometry_msgs::msg::Pose pose;

        pose.position.x = 0.3+0.1*sin(inc);
        pose.position.y = 0.0+0.1*sin(inc);
        pose.position.z = 0.3+0.1*sin(inc);

        pose.orientation.w = 1.0;
        inc+=0.1;
        pose_pub_->publish(pose);

        RCLCPP_INFO(this->get_logger(),"Published test pose.\n");
    }

    // Convert Unity → ROS frame (adjust as needed)
    geometry_msgs::msg::Pose convertPose(const geometry_msgs::msg::Pose& in)
    {
        geometry_msgs::msg::Pose out;

        out.position.x = in.position.z;
        out.position.y = -in.position.x;
        out.position.z = in.position.y;

        out.orientation = in.orientation;

        return out;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UR3eSimpleBridge>());
    rclcpp::shutdown();
    return 0;
}