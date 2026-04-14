/**
 * vr_moveit_bridge.cpp
 *
 * ROS 2 node that:
 *  1. Listens for UDP joint-angle packets from Unity (same JSON format as before)
 *  2. Republishes them as sensor_msgs/JointState on /vr_joint_command
 *  3. MoveIt 2 Servo picks that topic up and drives the real robot via
 *     the ur_robot_driver FollowJointTrajectory action server
 *
 * Build with colcon inside a ROS 2 workspace (see CMakeLists.txt / package.xml).
 *
 * Run:
 *   ros2 run vr_moveit_bridge vr_moveit_bridge_node --ros-args \
 *       -p udp_port:=50001 \
 *       -p robot_ip:=192.168.56.101
 */

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// MoveIt 2 Servo command topic type
// Servo accepts a TwistStamped (velocity) OR a JointJog (joint delta).
// For direct joint-angle mirroring, JointJog is the right choice.
#include "control_msgs/msg/joint_jog.hpp"

static constexpr int NUM_JOINTS = 6;

// UR3e joint names — must match the URDF exactly
static const std::array<std::string, NUM_JOINTS> JOINT_NAMES = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
};

// Soft joint limits (radians)
static constexpr double JOINT_MIN[NUM_JOINTS] = { -6.28, -3.14, -3.14, -3.14, -6.28, -6.28 };
static constexpr double JOINT_MAX[NUM_JOINTS] = {  6.28,  0.00,  3.14,  3.14,  6.28,  6.28 };

// Max velocity the JointJog commands are allowed to imply (rad/s)
static constexpr double MAX_VEL = 0.8;


// ─────────────────────────────────────────────────────────────────
//  Minimal in-place JSON parser (same as the C++ RTDE bridge)
// ─────────────────────────────────────────────────────────────────
static bool parse_joints(const char* buf, size_t len,
                         std::array<double, NUM_JOINTS>& out)
{
    const char* p = static_cast<const char*>(std::memchr(buf, '[', len));
    if (!p) return false;
    ++p;
    size_t count = 0;
    char* end;
    while (count < NUM_JOINTS) {
        double v = std::strtod(p, &end);
        if (end == p) break;
        out[count++] = v;
        p = end;
        while (*p == ' ' || *p == ',') ++p;
        if (*p == ']') break;
    }
    return count == NUM_JOINTS;
}


// ─────────────────────────────────────────────────────────────────
//  The ROS 2 Node
// ─────────────────────────────────────────────────────────────────
class VRMoveItBridge : public rclcpp::Node
{
public:
    VRMoveItBridge() : Node("vr_moveit_bridge")
    {
        // ── Declare ROS parameters (settable from launch or CLI) ──
        // These call declare_parameter which registers each param with
        // the ROS 2 parameter server so you can inspect/change them at runtime.
        this->declare_parameter<int>("udp_port", 50001);
        this->declare_parameter<double>("publish_rate_hz", 125.0);
        this->declare_parameter<bool>("use_joint_jog", true);

        udp_port_     = this->get_parameter("udp_port").as_int();
        publish_hz_   = this->get_parameter("publish_rate_hz").as_double();
        use_joint_jog_= this->get_parameter("use_joint_jog").as_bool();

        // ── Publishers ────────────────────────────────────────────
        // JointState: lets RViz and MoveIt see what angles VR is commanding
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/vr_joint_command", 10);

        // JointJog: MoveIt Servo's native incremental-joint-command topic
        // Servo will blend this into its trajectory and enforce limits
        if (use_joint_jog_) {
            joint_jog_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
                "/servo_node/delta_joint_cmds", 10);
        }

        // ── Shared state init ─────────────────────────────────────
        last_joints_.fill(0.0);
        current_joints_.fill(0.0);
        last_recv_time_ = this->now();

        // ── Start UDP receiver thread ─────────────────────────────
        udp_running_ = true;
        udp_thread_ = std::thread(&VRMoveItBridge::udp_receiver_thread, this);

        // ── Timer: publish at fixed rate ──────────────────────────
        // create_wall_timer fires a callback at the given period.
        // This decouples receive rate (UDP) from publish rate (ROS topic).
        auto period = std::chrono::duration<double>(1.0 / publish_hz_);
        publish_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&VRMoveItBridge::publish_callback, this));

        RCLCPP_INFO(this->get_logger(),
            "VR MoveIt bridge started — UDP port %d, %.0f Hz", udp_port_, publish_hz_);
    }

    ~VRMoveItBridge()
    {
        udp_running_ = false;
        if (udp_thread_.joinable()) udp_thread_.join();
        if (udp_sock_ >= 0) close(udp_sock_);
    }

private:

    // ── UDP receiver (runs in its own thread) ──────────────────────
    void udp_receiver_thread()
    {
        // Create socket
        udp_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_sock_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "socket() failed");
            return;
        }

        // 50 ms recv timeout so the thread can check udp_running_
        struct timeval tv{ 0, 50000 };
        setsockopt(udp_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        // Bind to all interfaces on the configured port
        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port        = htons(static_cast<uint16_t>(udp_port_));

        if (bind(udp_sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "bind() failed on port %d", udp_port_);
            return;
        }

        char buf[2048];

        while (udp_running_) {
            ssize_t n = recv(udp_sock_, buf, sizeof(buf) - 1, 0);
            if (n <= 0) continue;   // timeout or error — loop and recheck udp_running_
            buf[n] = '\0';

            std::array<double, NUM_JOINTS> joints{};
            if (!parse_joints(buf, static_cast<size_t>(n), joints)) continue;

            // Clamp to soft limits
            bool valid = true;
            for (int i = 0; i < NUM_JOINTS; ++i) {
                if (!std::isfinite(joints[i])) { valid = false; break; }
                joints[i] = std::max(JOINT_MIN[i], std::min(JOINT_MAX[i], joints[i]));
            }
            if (!valid) continue;

            // Write under mutex — publish_callback reads from the other thread
            {
                std::lock_guard<std::mutex> lk(mtx_);
                current_joints_ = joints;
                last_recv_time_ = this->now();
            }
            recv_count_++;
        }
    }

    // ── Publish callback (ROS timer, runs on ROS executor thread) ──
    void publish_callback()
    {
        std::array<double, NUM_JOINTS> joints{};
        rclcpp::Time recv_time;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            joints    = current_joints_;
            recv_time = last_recv_time_;
        }

        // Stale check: if Unity hasn't sent anything in 0.5 s, hold position
        double staleness = (this->now() - recv_time).seconds();
        if (staleness > 0.5 && recv_count_ > 0) return;

        auto stamp = this->now();
        const double dt = 1.0 / publish_hz_;

        // ── Velocity-limit the step (same logic as before) ──
        for (int i = 0; i < NUM_JOINTS; ++i) {
            double delta = joints[i] - last_joints_[i];
            double max_d = MAX_VEL * dt;
            delta = std::max(-max_d, std::min(max_d, delta));
            joints[i] = last_joints_[i] + delta;
        }
        last_joints_ = joints;

        // ── Publish JointState (for RViz / MoveIt state monitor) ──
        sensor_msgs::msg::JointState js_msg;
        js_msg.header.stamp = stamp;
        // header.frame_id is left empty — joint states don't need a frame
        for (int i = 0; i < NUM_JOINTS; ++i) {
            js_msg.name.push_back(JOINT_NAMES[i]);
            js_msg.position.push_back(joints[i]);
            js_msg.velocity.push_back(0.0);  // not used by Servo
        }
        joint_state_pub_->publish(js_msg);

        // ── Publish JointJog (MoveIt Servo command) ──────────────
        // JointJog sends *deltas* (how much to move each joint this step),
        // not absolute positions. So we send (current - last) each tick.
        if (use_joint_jog_ && joint_jog_pub_) {
            control_msgs::msg::JointJog jog_msg;
            jog_msg.header.stamp    = stamp;
            jog_msg.header.frame_id = "base_link";  // robot base frame
            for (int i = 0; i < NUM_JOINTS; ++i) {
                jog_msg.joint_names.push_back(JOINT_NAMES[i]);
                // velocity field: Servo treats this as desired joint velocity (rad/s)
                double vel = (joints[i] - last_joints_[i]) / dt;
                jog_msg.velocities.push_back(vel);
            }
            // duration: how long Servo should apply this command before timing out
            jog_msg.duration = static_cast<float>(dt * 2.0);
            joint_jog_pub_->publish(jog_msg);
        }
    }

    // ── Member variables ───────────────────────────────────────────
    int         udp_port_      = 50001;
    double      publish_hz_    = 125.0;
    bool        use_joint_jog_ = true;
    int         udp_sock_      = -1;

    std::thread           udp_thread_;
    std::atomic<bool>     udp_running_{ false };
    std::atomic<uint64_t> recv_count_{ 0 };

    std::mutex                        mtx_;
    std::array<double, NUM_JOINTS>    current_joints_{};
    std::array<double, NUM_JOINTS>    last_joints_{};
    rclcpp::Time                      last_recv_time_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr  joint_state_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr   joint_jog_pub_;
    rclcpp::TimerBase::SharedPtr                                publish_timer_;
};


// ─────────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    // rclcpp::init parses ROS 2 arguments (--ros-args, -p param:=value, etc.)
    // and initialises the ROS 2 middleware (DDS by default)
    rclcpp::init(argc, argv);

    // spin() hands the node to the ROS executor, which drives timer callbacks,
    // topic callbacks, service handlers etc. Blocks until Ctrl-C / rclcpp::shutdown()
    rclcpp::spin(std::make_shared<VRMoveItBridge>());

    rclcpp::shutdown();
    return 0;
}