#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <cmath>

using std::placeholders::_1;

class UR3eRTDEBridge : public rclcpp::Node
{
public:
    UR3eRTDEBridge()
        : Node("ur3e_rtde_bridge"),
          rtde_control_("192.168.56.101"),
          rtde_receive_("192.168.56.101")
    {
        ee_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/ik_target", 10,
            std::bind(&UR3eRTDEBridge::eeCallback, this, _1));

        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/ik_solution", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&UR3eRTDEBridge::publishJointStates, this));

        RCLCPP_INFO(this->get_logger(), "RTDE Bridge Node Started");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ee_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    ur_rtde::RTDEControlInterface rtde_control_;
    ur_rtde::RTDEReceiveInterface rtde_receive_;

    std::vector<double> last_pose_{0, 0, 0, 0, 0, 0};

    // Unity → UR robot
    void eeCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // Convert pose
        std::vector<double> ur_pose = convertToURPose(*msg);

        last_pose_ = ur_pose;

        // Send servo command (real-time)
        rtde_control_.servoL(
            ur_pose,
            0.5,   // speed
            0.5,   // acceleration
            0.02,  // dt (must match loop rate ~50Hz)
            0.1,   // lookahead
            300    // gain
        );
    }

    // UR → Unity
    void publishJointStates()
    {
        std::vector<double> joints = rtde_receive_.getActualQ();

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->get_clock()->now();

        msg.name = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        };

        msg.position = joints;

        joint_pub_->publish(msg);
    }

    // Convert ROS Pose → UR pose (x,y,z,rx,ry,rz)
    std::vector<double> convertToURPose(const geometry_msgs::msg::Pose& pose)
    {
        double x = pose.position.x;
        double y = pose.position.y;
        double z = pose.position.z;

        // Quaternion → axis-angle
        double qx = pose.orientation.x;
        double qy = pose.orientation.y;
        double qz = pose.orientation.z;
        double qw = pose.orientation.w;

        double angle = 2 * acos(qw);
        double s = sqrt(1 - qw * qw);

        double rx = 0, ry = 0, rz = 0;

        if (s > 1e-6)
        {
            rx = qx / s * angle;
            ry = qy / s * angle;
            rz = qz / s * angle;
        }

        return {x, y, z, rx, ry, rz};
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UR3eRTDEBridge>());
    rclcpp::shutdown();
    return 0;
}