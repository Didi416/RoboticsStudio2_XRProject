"""
puzzle_pose_server.py
─────────────────────────────────────────────────────────────────────────────
ROS 2 node that subscribes to /puzzle_goal (String) and uses MoveIt2's
MoveGroupInterface to move the UR3e end-effector to pre-defined Cartesian
positions for each puzzle zone.

Puzzle zones (defined in robot base_link frame, metres):
  Puzzle 1 – Button Matrix   : x=0.30, y=-0.10, z=0.20  (front-left of board)
  Puzzle 2 – Slider Maze     : x=0.30, y= 0.00, z=0.20  (board centre)
  Puzzle 3 – Egg Sorting     : x=0.30, y= 0.10, z=0.20  (front-right of board)
  home                       : a safe up-and-back pose

Usage:
  ros2 run <your_package> puzzle_pose_server
  ros2 topic pub /puzzle_goal std_msgs/msg/String "data: 'puzzle1'"
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Point, Quaternion

import threading
import math

try:
    from moveit.planning import MoveItPy
    from moveit.core.robot_state import RobotState
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False
    print("[WARN] moveit_py not found – running in DRY-RUN mode (poses printed only)")


# ── Puzzle pose definitions (base_link frame) ─────────────────────────────────
#
# Orientation: end-effector pointing straight down (tool0 Z = -world_Z)
# Use a quaternion that represents a downward-pointing wrist:
#   w=0, x=1, y=0, z=0  →  180° rotation about X  (tool pointing down)
#
# Adjust x/y/z to match YOUR physical board layout!
# All positions assume the puzzle board is ~30 cm in front of the robot base.

DOWN_QUAT = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)   # end-effector pointing down

PUZZLE_POSES: dict[str, Pose] = {
    "puzzle1": Pose(
        position=Point(x=0.30, y=-0.12, z=0.20),   # Button Matrix – left side
        orientation=DOWN_QUAT,
    ),
    "puzzle2": Pose(
        position=Point(x=0.30, y=0.00, z=0.20),    # Slider Maze – centre
        orientation=DOWN_QUAT,
    ),
    "puzzle3": Pose(
        position=Point(x=0.30, y=0.12, z=0.20),    # Egg Sorting – right side
        orientation=DOWN_QUAT,
    ),
    "home": Pose(
        position=Point(x=0.18, y=0.00, z=0.35),    # safe retracted position
        orientation=DOWN_QUAT,
    ),
}

PLANNING_GROUP = "ur_manipulator"
VELOCITY_SCALE = 0.15       # 15 % of max – slow and safe for demos
ACCEL_SCALE    = 0.10


class PuzzlePoseServer(Node):

    def __init__(self):
        super().__init__("puzzle_pose_server")

        self._cbg  = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._busy = False

        # ── Publisher: status feedback for the GUI ────────────────────────────
        self._status_pub = self.create_publisher(
            String, "/puzzle_status", 10)

        self._busy_pub = self.create_publisher(
            Bool, "/puzzle_busy", 10)

        # ── Subscriber: receives goal names from GUI ──────────────────────────
        self._goal_sub = self.create_subscription(
            String,
            "/puzzle_goal",
            self._goal_cb,
            10,
            callback_group=self._cbg,
        )

        # ── MoveIt init ───────────────────────────────────────────────────────
        if MOVEIT_AVAILABLE:
            self.get_logger().info("Initialising MoveItPy…")
            self._moveit = MoveItPy(node_name="puzzle_pose_moveit_node")
            self._arm    = self._moveit.get_planning_component(PLANNING_GROUP)
            self.get_logger().info("MoveItPy ready.")
        else:
            self._moveit = None
            self._arm    = None

        self.get_logger().info(
            "PuzzlePoseServer ready.  "
            "Known puzzles: " + ", ".join(PUZZLE_POSES.keys())
        )
        self._publish_status("ready", busy=False)

    # ── Goal callback ─────────────────────────────────────────────────────────

    def _goal_cb(self, msg: String):
        goal_name = msg.data.strip().lower()

        if goal_name not in PUZZLE_POSES:
            self.get_logger().warn(f"Unknown puzzle goal: '{goal_name}'")
            self._publish_status(f"error: unknown goal '{goal_name}'", busy=False)
            return

        with self._lock:
            if self._busy:
                self.get_logger().warn("Robot busy – ignoring new goal.")
                self._publish_status("busy – rejected new goal", busy=True)
                return
            self._busy = True

        self.get_logger().info(f"Moving to: {goal_name}")
        self._publish_status(f"moving to {goal_name}", busy=True)

        threading.Thread(
            target=self._execute_move,
            args=(goal_name,),
            daemon=True,
        ).start()

    # ── Motion execution ──────────────────────────────────────────────────────

    def _execute_move(self, goal_name: str):
        target_pose = PUZZLE_POSES[goal_name]
        success = False

        try:
            if self._arm is not None:
                # Set pose target via MoveItPy
                robot_state = self._moveit.get_robot_state()
                self._arm.set_start_state(robot_state=robot_state)
                self._arm.set_goal_state(
                    pose_stamped_msg=self._make_pose_stamped(target_pose),
                    pose_link="tool0",
                )
                plan_result = self._arm.plan()

                if plan_result:
                    robot_traj = plan_result.trajectory
                    self._moveit.execute(
                        robot_traj,
                        controllers=["scaled_joint_trajectory_controller"],
                    )
                    success = True
                else:
                    self.get_logger().error(f"Planning failed for '{goal_name}'")
            else:
                # DRY-RUN: just print the target pose
                p = target_pose.position
                self.get_logger().info(
                    f"[DRY-RUN] Would move to {goal_name}: "
                    f"x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}"
                )
                import time; time.sleep(2.0)   # simulate motion delay
                success = True

        except Exception as exc:
            self.get_logger().error(f"Motion error: {exc}")

        with self._lock:
            self._busy = False

        status = f"reached {goal_name}" if success else f"failed {goal_name}"
        self._publish_status(status, busy=False)
        self.get_logger().info(status)

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _make_pose_stamped(pose: Pose):
        """Wrap a Pose in a PoseStamped for MoveItPy."""
        from geometry_msgs.msg import PoseStamped
        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        ps.pose = pose
        return ps

    def _publish_status(self, text: str, busy: bool):
        self._status_pub.publish(String(data=text))
        self._busy_pub.publish(Bool(data=busy))


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PuzzlePoseServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()