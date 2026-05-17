#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <stdio.h>

#include <moveit/move_group_interface/move_group_interface.h>

class IKNode : public rclcpp::Node
{
public:
    IKNode() : Node("ik_solver_node"),
               move_group(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator")
    {
        sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/ik_target", 10,
            std::bind(&IKNode::poseCallback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/ik_solution", 10);

        move_group.setMaxVelocityScalingFactor(0.5);
        move_group.setMaxAccelerationScalingFactor(0.5);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;

    moveit::planning_interface::MoveGroupInterface move_group;

    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        std::cout << "Pose callback running"<<std::endl;
        RCLCPP_INFO(this->get_logger(),
        "Target: x=%f y=%f z=%f",
        msg->position.x,
        msg->position.y,
        msg->position.z);

        move_group.setPositionTarget(msg->position.x, msg->position.y, msg->position.z);

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success)
        {
            RCLCPP_WARN(this->get_logger(), "IK failed");
            return;
        }

        auto joint_values = plan.trajectory_.joint_trajectory.points.back().positions;

        sensor_msgs::msg::JointState js;
        js.header.stamp = this->get_clock()->now();
        js.position = joint_values;

        pub_->publish(js);

        // Optional: send to robot
        move_group.execute(plan);
    }
};

int main(int argc, char** argv)
{
    std::cout << "Starting Bridge. "<<std::endl;
    rclcpp::init(argc, argv);

    auto node = std::make_shared<IKNode>();
    rclcpp::spin(node);
    std::cout << "Node Running."<<std::endl;

    rclcpp::shutdown();
    return 0;
}