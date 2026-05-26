# import copy
# import time
# import threading
# import sys
# import tty
# import termios
# import rclpy
# from rclpy.action import ActionClient
# from rclpy.node import Node
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rclpy.executors import MultiThreadedExecutor
# from geometry_msgs.msg import Pose, PoseStamped
# from moveit_msgs.action import MoveGroup, ExecuteTrajectory
# from moveit_msgs.msg import (
#     CollisionObject, MoveItErrorCodes, PlanningScene,
#     MotionPlanRequest, Constraints, PositionConstraint,
#     OrientationConstraint, BoundingVolume, WorkspaceParameters,
# )
# from moveit_msgs.msg import BoundingVolume as MoveItBV
# from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
# from shape_msgs.msg import SolidPrimitive
# from tf2_ros import Buffer, TransformListener

# # ── Trolley (mobile cart) under robot ────────────────────────────────────────
# TROLLEY_WIDTH  =  0.765  # X (mm→m)
# TROLLEY_DEPTH  =  0.802  # Y (mm→m)
# TROLLEY_HEIGHT =  1.091  # Z — side face now on top
# # Robot sits +31mm in X and -179mm in Y from trolley geometric centre
# TROLLEY_X      = -0.031  # trolley centre in base_link X
# TROLLEY_Y      =  0.179  # trolley centre in base_link Y

# # ── Board geometry (left-side, pole-mounted setup) ───────────────────────────
# BOARD_Y         =  0.6   # distance to board face from robot base
# BOARD_X         =  0.0    # centred left/right
# BOARD_THICKNESS =  0.02
# BOARD_WIDTH     =  0.20   # board extent along X (20 cm)
# BOARD_HEIGHT    =  0.20   # board extent along Z (20 cm)
# # Board bottom 8 cm above robot base (z=0); board spans 0.08 → 0.28 m
# BOARD_Z         = 0.08 + BOARD_HEIGHT / 2  # = 0.18 m

# # Pole — vertical, spans from floor to board top
# POLE_RADIUS     =  0.025  # 5cm diameter
# POLE_HEIGHT     =  BOARD_Z + BOARD_HEIGHT / 2 - (-TROLLEY_HEIGHT)  # = 0.480
# POLE_X          =  0.0
# POLE_Y          =  BOARD_Y + BOARD_THICKNESS / 2 + POLE_RADIUS  # flush behind board

# # ── Hover pose (left-side approach) ──────────────────────────────────────────
# # HOVER_Y + gripper_length < BOARD_Y to avoid clipping
# # 0.25 + 0.16 = 0.41 < 0.55 ✓
# HOVER_X =  BOARD_X    # centred
# HOVER_Y =  BOARD_Y - 0.26   # standoff distance from wall in +Y direction
# HOVER_Z =  BOARD_Z  # safe retract height above robot base
# PUSH_Y  =  BOARD_Y - BOARD_THICKNESS - 0.17   # just in front of wall face

# # Orientation for left-side approach — end-effector pointing toward +Y (left wall)
# # x=-0.7071, w=0.7071 = -90° around X axis
# FORCED_ORI   = dict(x=-0.7071, y=0.0, z=0.0, w=0.7071)
# USE_LIVE_ORI = False

# # ── Board target points — feature centres from CAD (PuzzleBoardDesign2.SLDPRT)
# # Board X (mm from board centre) → robot X; Board Y (mm, up = +Z) → robot Z offset
# BOARD_POINTS = {
#     '1': dict(x= 0.0000, z=BOARD_Z + 0.0000, label='Joystick mount'),
#     '2': dict(x= 0.0548, z=BOARD_Z + 0.0483, label='Button matrix'),
#     '3': dict(x= 0.0155, z=BOARD_Z - 0.0300, label='Card holder'),
#     '4': dict(x= 0.0500, z=BOARD_Z - 0.0855, label='RFID slot'),
#     '5': dict(x=-0.0605, z=BOARD_Z + 0.0595, label='Left panel'),
# }

# # ── Cartesian settings ────────────────────────────────────────────────────────
# MAX_STEP       = 0.01
# JUMP_THRESHOLD = 0.0    # re-enable (5.0) once working end-to-end
# MIN_FRACTION   = 0.95

# # Change to 'ur_manipulator' if using standard ur_moveit_config (no gripper)
# # Change to 'ur_onrobot_manipulator' if using ur_onrobot_moveit_config (with gripper)
# GROUP_NAME     = 'ur_onrobot_manipulator'
# PLANNING_TIME  = 10.0


# class PuzzleTask(Node):
#     def __init__(self):
#         super().__init__('puzzle_task_node')

#         # ── ReentrantCallbackGroup lets service/action calls work from any
#         #    thread inside a MultiThreadedExecutor without deadlocking ──────────
#         self._cbg = ReentrantCallbackGroup()

#         self.tf_buffer   = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         self.apply_scene_client = self.create_client(
#             ApplyPlanningScene, 'apply_planning_scene',
#             callback_group=self._cbg)
#         self.cartesian_client   = self.create_client(
#             GetCartesianPath, 'compute_cartesian_path',
#             callback_group=self._cbg)
#         self.move_group_client  = ActionClient(
#             self, MoveGroup, 'move_action',
#             callback_group=self._cbg)
#         self.execute_client     = ActionClient(
#             self, ExecuteTrajectory, 'execute_trajectory',
#             callback_group=self._cbg)

#         self.get_logger().info('Waiting for MoveIt2 servers...')
#         for svc, name in [
#             (self.apply_scene_client, 'apply_planning_scene'),
#             (self.cartesian_client,   'compute_cartesian_path'),
#         ]:
#             if not svc.wait_for_service(timeout_sec=10.0):
#                 raise RuntimeError(f'{name} not available')
#         for ac, name in [
#             (self.move_group_client, 'move_action'),
#             (self.execute_client,    'execute_trajectory'),
#         ]:
#             if not ac.wait_for_server(timeout_sec=10.0):
#                 raise RuntimeError(f'{name} not available')

#         self.get_logger().info('All MoveIt2 servers online.')

#         # Poll /get_planning_scene — only available once move_group has
#         # fully loaded the robot model and kinematics plugin.
#         # This prevents the 99999 goal rejection on fast machines.
#         from moveit_msgs.srv import GetPlanningScene
#         ready_client = self.create_client(
#             GetPlanningScene, '/get_planning_scene')
#         self.get_logger().info('Waiting for move_group to fully initialise...')
#         deadline = time.time() + 30.0
#         while not ready_client.wait_for_service(timeout_sec=1.0):
#             if time.time() > deadline:
#                 self.get_logger().warn('move_group took >30s — proceeding anyway')
#                 break
#             self.get_logger().info('Still waiting for move_group...')
#         time.sleep(2.0)   # extra settle after service appears
#         self._busy = False

#     # ── Planning scene ────────────────────────────────────────────────────────

#     def setup_obstacles(self):
#         now = self.get_clock().now().to_msg()

#         def make_box(obj_id, dims, pos):
#             co                 = CollisionObject()
#             co.header.frame_id = 'base_link'
#             co.header.stamp    = now
#             co.id              = obj_id
#             co.operation       = CollisionObject.ADD
#             sh                 = SolidPrimitive()
#             sh.type            = SolidPrimitive.BOX
#             sh.dimensions      = dims
#             p                  = Pose()
#             p.position.x, p.position.y, p.position.z = pos
#             p.orientation.w    = 1.0
#             co.primitives.append(sh)
#             co.primitive_poses.append(p)
#             return co

#         def make_cylinder(obj_id, radius, height, pos):
#             co                 = CollisionObject()
#             co.header.frame_id = 'base_link'
#             co.header.stamp    = now
#             co.id              = obj_id
#             co.operation       = CollisionObject.ADD
#             sh                 = SolidPrimitive()
#             sh.type            = SolidPrimitive.CYLINDER
#             sh.dimensions      = [height, radius]   # [height, radius]
#             p                  = Pose()
#             p.position.x       = float(pos[0])
#             p.position.y       = float(pos[1])
#             p.position.z       = float(pos[2])
#             # Explicitly set identity quaternion — ensures cylinder is vertical
#             p.orientation.x    = 0.0
#             p.orientation.y    = 0.0
#             p.orientation.z    = 0.0
#             p.orientation.w    = 1.0
#             co.primitives.append(sh)
#             co.primitive_poses.append(p)
#             return co

#         # ── Puzzle board panel (thin, left side, face at BOARD_Y) ─────────────
#         board = make_box('puzzle_board',
#                          [BOARD_WIDTH, BOARD_THICKNESS, BOARD_HEIGHT],
#                          [BOARD_X, BOARD_Y, BOARD_Z])

#         # ── Support pole (cylinder behind the board) ──────────────────────────
#         pole = make_cylinder('board_pole',
#                              POLE_RADIUS,
#                              POLE_HEIGHT,
#                              [POLE_X, POLE_Y, -TROLLEY_HEIGHT + POLE_HEIGHT / 2])

#         # ── Trolley (mobile cart) under robot ─────────────────────────────────
#         # Only the lower half is used as a collision object; the upper half
#         # overlaps the robot's own base links and causes false collisions.
#         # Top of collision box sits at z ≈ -0.55 m, well below the arm's range.
#         trolley_coll_h = TROLLEY_HEIGHT - 0.03
#         trolley = make_box('robot_trolley',
#                            [TROLLEY_WIDTH, TROLLEY_DEPTH, trolley_coll_h],
#                            [TROLLEY_X, TROLLEY_Y, -TROLLEY_HEIGHT + trolley_coll_h / 2])

#         scene         = PlanningScene()
#         scene.is_diff = True
#         scene.world.collision_objects.extend([board, pole, trolley])

#         req       = ApplyPlanningScene.Request()
#         req.scene = scene
#         future    = self.apply_scene_client.call_async(req)
#         result    = self._wait(future)

#         if result is None:
#             self.get_logger().error('ApplyPlanningScene failed.')
#             return False

#         self.get_logger().info('Planning scene updated.')
#         time.sleep(1.0)
#         return True

#     # ── Helpers ───────────────────────────────────────────────────────────────

#     def _get_tool_pose(self, timeout_sec=5.0):
#         target, source = 'base_link', 'tool0'
#         deadline = time.time() + timeout_sec
#         while not self.tf_buffer.can_transform(target, source, rclpy.time.Time()):
#             if time.time() > deadline:
#                 self.get_logger().error('TF timeout waiting for tool0.')
#                 return None
#             time.sleep(0.1)
#         try:
#             trans = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
#         except Exception as e:
#             self.get_logger().error(f'TF lookup failed: {e}')
#             return None
#         p             = Pose()
#         p.position.x  = trans.transform.translation.x
#         p.position.y  = trans.transform.translation.y
#         p.position.z  = trans.transform.translation.z
#         p.orientation = trans.transform.rotation
#         return p

#     def _wait(self, future):
#         """
#         Block until a future completes, compatible with MultiThreadedExecutor.
#         Uses an Event rather than spin_until_future_complete so it works
#         safely from any thread without conflicting with the running executor.
#         """
#         done = threading.Event()
#         future.add_done_callback(lambda _: done.set())
#         done.wait()
#         return future.result()

#     # ── Joint-space move ──────────────────────────────────────────────────────

#     def _move_joint_space(self, x, y, z, label='pose'):
#         self.get_logger().info(
#             f'Joint-space move → {label} ({x:.3f}, {y:.3f}, {z:.3f})')

#         ps                    = PoseStamped()
#         ps.header.frame_id    = 'base_link'
#         ps.header.stamp       = self.get_clock().now().to_msg()
#         ps.pose.position.x    = x
#         ps.pose.position.y    = y
#         ps.pose.position.z    = z
#         ps.pose.orientation.x = FORCED_ORI['x']
#         ps.pose.orientation.y = FORCED_ORI['y']
#         ps.pose.orientation.z = FORCED_ORI['z']
#         ps.pose.orientation.w = FORCED_ORI['w']

#         pc                       = PositionConstraint()
#         pc.header.frame_id       = 'base_link'
#         pc.link_name             = 'tool0'
#         pc.target_point_offset.x = 0.0
#         pc.target_point_offset.y = 0.0
#         pc.target_point_offset.z = 0.0
#         bv                       = BoundingVolume()
#         region                   = SolidPrimitive()
#         region.type              = SolidPrimitive.SPHERE
#         region.dimensions        = [0.005]
#         bv.primitives.append(region)
#         bv.primitive_poses.append(ps.pose)
#         pc.constraint_region     = bv
#         pc.weight                = 1.0

#         oc                           = OrientationConstraint()
#         oc.header.frame_id           = 'base_link'
#         oc.link_name                 = 'tool0'
#         oc.orientation               = ps.pose.orientation
#         oc.absolute_x_axis_tolerance = 0.1
#         oc.absolute_y_axis_tolerance = 0.1
#         oc.absolute_z_axis_tolerance = 0.1
#         oc.weight                    = 1.0

#         goal_c = Constraints()
#         goal_c.position_constraints.append(pc)
#         goal_c.orientation_constraints.append(oc)

#         req                                 = MotionPlanRequest()
#         req.group_name                      = GROUP_NAME
#         req.num_planning_attempts           = 10
#         req.allowed_planning_time           = PLANNING_TIME
#         req.max_velocity_scaling_factor     = 0.3
#         req.max_acceleration_scaling_factor = 0.2

#         # Workspace parameters required — missing these causes 99999 rejection
#         ws = WorkspaceParameters()
#         ws.header.frame_id = 'base_link'
#         ws.header.stamp    = self.get_clock().now().to_msg()
#         ws.min_corner.x = ws.min_corner.y = ws.min_corner.z = -2.0
#         ws.max_corner.x = ws.max_corner.y = ws.max_corner.z =  2.0
#         req.workspace_parameters = ws

#         req.goal_constraints.append(goal_c)

#         goal         = MoveGroup.Goal()
#         goal.request = req

#         handle = self._wait(self.move_group_client.send_goal_async(goal))
#         if handle is None or not handle.accepted:
#             self.get_logger().error('MoveGroup goal rejected.')
#             return False

#         result = self._wait(handle.get_result_async())
#         code   = result.result.error_code.val
#         if code == MoveItErrorCodes.SUCCESS:
#             self.get_logger().info(f'Reached {label}.')
#             time.sleep(0.3)
#             return True
#         self.get_logger().error(f'MoveGroup failed for {label} — code {code}')
#         return False

#     # ── Cartesian move ────────────────────────────────────────────────────────

#     def _cartesian_move(self, target_x, target_y, target_z, label='target'):
#         live = self._get_tool_pose()
#         if live is None:
#             return False

#         ori = live.orientation if USE_LIVE_ORI else _forced_quat()

#         def wp(x, y, z):
#             p             = Pose()
#             p.position.x  = x
#             p.position.y  = y
#             p.position.z  = z
#             p.orientation = copy.deepcopy(ori)
#             return p

#         waypoints = [
#             wp(live.position.x, live.position.y, live.position.z),
#             wp(target_x, target_y, target_z),
#         ]

#         self.get_logger().info(
#             f'Cartesian → {label} ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})')

#         req                  = GetCartesianPath.Request()
#         req.header.frame_id  = 'base_link'
#         req.group_name       = GROUP_NAME
#         req.waypoints        = waypoints
#         req.max_step         = MAX_STEP
#         req.jump_threshold   = JUMP_THRESHOLD
#         req.avoid_collisions = True

#         response = self._wait(self.cartesian_client.call_async(req))
#         if response is None:
#             self.get_logger().error('Cartesian service returned no response.')
#             return False

#         fraction = response.fraction
#         self.get_logger().info(f'Fraction: {fraction * 100:.1f}%')

#         if fraction < MIN_FRACTION:
#             self.get_logger().error(
#                 f'Insufficient fraction ({fraction * 100:.1f}%). '
#                 'Check reachability of target point.')
#             return False

#         goal            = ExecuteTrajectory.Goal()
#         goal.trajectory = response.solution

#         handle = self._wait(self.execute_client.send_goal_async(goal))
#         if handle is None or not handle.accepted:
#             self.get_logger().error('ExecuteTrajectory rejected.')
#             return False

#         result = self._wait(handle.get_result_async())
#         code   = result.result.error_code.val
#         if code == MoveItErrorCodes.SUCCESS:
#             self.get_logger().info(f'Cartesian move to {label} complete.')
#             return True
#         self.get_logger().error(f'Execution failed — code {code}')
#         return False

#     # ── Board point visit ─────────────────────────────────────────────────────

#     def visit_board_point(self, key):
#         pt = BOARD_POINTS[key]
#         self.get_logger().info(
#             f'--- Point {key}: {pt["label"]} '
#             f'(x={pt["x"]:.3f}, z={pt["z"]:.3f}) ---')

#         # Retract to hover first
#         ok = self._cartesian_move(HOVER_X, HOVER_Y, HOVER_Z, label='hover')
#         if not ok:
#             self.get_logger().warn('Cartesian retract failed — trying joint-space.')
#             ok = self._move_joint_space(HOVER_X, HOVER_Y, HOVER_Z, label='hover')
#         if not ok:
#             self.get_logger().error('Could not retract — aborting.')
#             return

#         # Push to board point
#         self._cartesian_move(pt['x'], PUSH_Y, pt['z'], label=pt['label'])

#     def go_to_hover(self):
#         return self._move_joint_space(HOVER_X, HOVER_Y, HOVER_Z, label='hover')

#     # ── Keyboard listener ─────────────────────────────────────────────────────

#     def start_keyboard_listener(self):
#         threading.Thread(target=self._keyboard_loop, daemon=True).start()

#     def _keyboard_loop(self):
#         fd  = sys.stdin.fileno()
#         old = termios.tcgetattr(fd)
#         tty.setcbreak(fd)
#         print('\n--- Board point selector ---')
#         for k, pt in BOARD_POINTS.items():
#             print(f'  {k}  →  {pt["label"]}  (x={pt["x"]:.3f}, z={pt["z"]:.3f})')
#         print('  q  →  quit\n')
#         try:
#             while True:
#                 ch = sys.stdin.read(1)
#                 if ch == 'q':
#                     print('Quitting.')
#                     rclpy.shutdown()
#                     break
#                 if ch in BOARD_POINTS:
#                     if self._busy:
#                         print(f'[{ch}] ignored — arm is moving')
#                         continue
#                     self._busy = True
#                     threading.Thread(
#                         target=self._run_point,
#                         args=(ch,),
#                         daemon=True
#                     ).start()
#         finally:
#             termios.tcsetattr(fd, termios.TCSADRAIN, old)

#     def _run_point(self, key):
#         try:
#             self.visit_board_point(key)
#         finally:
#             self._busy = False


# def _forced_quat():
#     from geometry_msgs.msg import Quaternion
#     q   = Quaternion()
#     q.x = FORCED_ORI['x']
#     q.y = FORCED_ORI['y']
#     q.z = FORCED_ORI['z']
#     q.w = FORCED_ORI['w']
#     return q


# def main(args=None):
#     rclpy.init(args=args)
#     node = PuzzleTask()

#     # Start the MultiThreadedExecutor FIRST so every async call —
#     # including setup and the initial hover move — goes through the
#     # same executor. Running setup before spin() and then switching
#     # executors leaves internal state that causes the node to stop
#     # responding after the first blocking call completes.
#     executor = MultiThreadedExecutor()
#     executor.add_node(node)

#     # Spin the executor in a background thread so main() can continue.
#     spin_thread = threading.Thread(target=executor.spin, daemon=True)
#     spin_thread.start()

#     # Run the startup sequence in the main thread — _wait() is safe here
#     # because the executor is already spinning in the background.
#     try:
#         print('Setting up obstacles ...')
#         if not node.setup_obstacles():
#             print('ERROR: Could not set up planning scene.')
#             rclpy.shutdown()
#             return

#         print('Moving to hover position ...')
#         if not node.go_to_hover():
#             print('ERROR: Could not reach hover — check HOVER_X/Y/Z.')
#             rclpy.shutdown()
#             return

#         print('At hover. Press 1-4 to visit a board point, q to quit.')
#         node.start_keyboard_listener()

#         # Keep main alive until shutdown
#         spin_thread.join()

#     except KeyboardInterrupt:
#         pass
#     finally:
#         try:
#             node.destroy_node()
#         except Exception:
#             pass
#         try:
#             rclpy.shutdown()
#         except Exception:
#             pass


# if __name__ == '__main__':
#     main()

import copy
import time
import threading
import sys
import tty
import termios
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    CollisionObject, MoveItErrorCodes, PlanningScene,
    MotionPlanRequest, Constraints, PositionConstraint, JointConstraint,
    OrientationConstraint, BoundingVolume, WorkspaceParameters,
)
from moveit_msgs.msg import BoundingVolume as MoveItBV
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

# ── Board geometry (left-side, pole-mounted setup) ───────────────────────────
# Board panel sits at shoulder height on the left (+Y) side.
BOARD_Y         =  0.65   # moved 10cm further back from robot base
BOARD_X         =  0.0    # centred left/right
BOARD_Z         =  0.18   # board centre: 8cm above base + 10cm (half of 20cm board)
BOARD_THICKNESS =  0.02
BOARD_WIDTH     =  0.20   # actual board width 20cm
BOARD_HEIGHT    =  0.20   # actual board height 20cm

# Pole — vertical, sits directly behind the board centre
POLE_RADIUS     =  0.025  # 5cm diameter
POLE_HEIGHT     =  1.20   # 1.2m tall — floor to above board top
POLE_X          =  0.0
POLE_Y          =  BOARD_Y + BOARD_THICKNESS / 2 + POLE_RADIUS  # flush behind board

# ── Hover pose (left-side approach) ──────────────────────────────────────────
# Arm hovers to the left of the robot, in front of the wall.
# HOVER_Y + gripper_length < BOARD_Y to avoid clipping
# 0.25 + 0.16 = 0.41 < 0.55 ✓
HOVER_X =  0.0
HOVER_Y =  BOARD_Y - 0.30
HOVER_Z =  BOARD_Z
PUSH_Y  =  BOARD_Y - BOARD_THICKNESS - 0.08

# Pre-recorded joint configurations.
# Joint order: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
HOVER_JOINTS  = [1.3669, -0.6314, 1.7613, -4.4130, -1.3690, 18.8798]  # hover facing board
WALL_JOINTS   = [1.0348, -0.5828, 1.9254, -4.6456, -1.0406, 18.9330]  # board centre facing wall
FLOOR_JOINTS  = [1.3970, -1.3996, 1.8509, -2.4060, -1.5036, 18.6890]  # egg pickup facing floor

# Orientation for left-side approach — end-effector pointing toward +Y (left wall)
# x=-0.7071, w=0.7071 = -90° around X axis
# Approach direction confirmed as (0, +1, 0) — toward left wall, gripper flat
FORCED_ORI   = dict(x=-0.7071, y=0.0, z=0.0, w=0.7071)
USE_LIVE_ORI = True

# ── 4 board target points (left-side wall) ───────────────────────────────────
# Wall is on the left (+Y side). Points vary in X (left/right across wall)
# and Z (up/down). Y is fixed at PUSH_Y (approach depth to wall).
# Board points from CAD (PuzzleBoardDesign2.stp), relative to board centre.
# BOARD_Z = 0.18m (centre), board spans z=0.08 to z=0.28
BOARD_POINTS = {
    '1': dict(x= 0.0000, z=BOARD_Z + 0.0000, label='Joystick mount'),
    '2': dict(x= 0.0548, z=BOARD_Z + 0.0483, label='Button matrix'),
    '3': dict(x= 0.0155, z=BOARD_Z - 0.0300, label='Card holder'),
    '4': dict(x= 0.0500, z=BOARD_Z - 0.0855, label='RFID slot'),
    '5': dict(x=-0.0605, z=BOARD_Z + 0.0595, label='Left panel'),
}

# ── Easel visual constants ────────────────────────────────────────────────────
EASEL_X        =  POLE_X
EASEL_Y        =  BOARD_Y + BOARD_THICKNESS / 2 + 0.02
EASEL_Z        =  0.0
LEG_LENGTH     =  1.40
LEG_RADIUS     =  0.015
SPREAD_ANGLE   =  15.0
BACK_ANGLE     =  25.0
CROSSBAR_Z     =  0.70
CROSSBAR_R     =  0.012
WOOD_COLOUR    = (0.72, 0.53, 0.30, 1.0)   # r, g, b, a

# ── Cartesian settings ────────────────────────────────────────────────────────
MAX_STEP       = 0.01
JUMP_THRESHOLD = 0.0    # re-enable (5.0) once working end-to-end
MIN_FRACTION   = 0.95   # for general moves
MIN_FRACTION_PUSH = 0.10  # for short push moves — arm already near target

# Change to 'ur_manipulator' if using standard ur_moveit_config (no gripper)
# Change to 'ur_onrobot_manipulator' if using ur_onrobot_moveit_config (with gripper)
GROUP_NAME     = 'ur_onrobot_manipulator'
PLANNING_TIME  = 10.0


class PuzzleTask(Node):
    def __init__(self):
        super().__init__('puzzle_task_node')

        # ── ReentrantCallbackGroup lets service/action calls work from any
        #    thread inside a MultiThreadedExecutor without deadlocking ──────────
        self._cbg = ReentrantCallbackGroup()

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Easel visual marker publisher
        self._easel_pub = self.create_publisher(MarkerArray, '/easel_marker', 10)
        self._easel_markers = None   # built once in setup_obstacles

        self.apply_scene_client = self.create_client(
            ApplyPlanningScene, 'apply_planning_scene',
            callback_group=self._cbg)
        self.cartesian_client   = self.create_client(
            GetCartesianPath, 'compute_cartesian_path',
            callback_group=self._cbg)
        self.move_group_client  = ActionClient(
            self, MoveGroup, 'move_action',
            callback_group=self._cbg)
        self.execute_client     = ActionClient(
            self, ExecuteTrajectory, 'execute_trajectory',
            callback_group=self._cbg)

        self.get_logger().info('Waiting for MoveIt2 servers...')
        for svc, name in [
            (self.apply_scene_client, 'apply_planning_scene'),
            (self.cartesian_client,   'compute_cartesian_path'),
        ]:
            if not svc.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f'{name} not available')
        for ac, name in [
            (self.move_group_client, 'move_action'),
            (self.execute_client,    'execute_trajectory'),
        ]:
            if not ac.wait_for_server(timeout_sec=10.0):
                raise RuntimeError(f'{name} not available')

        self.get_logger().info('All MoveIt2 servers online.')

        # Poll /get_planning_scene — only available once move_group has
        # fully loaded the robot model and kinematics plugin.
        # This prevents the 99999 goal rejection on fast machines.
        from moveit_msgs.srv import GetPlanningScene
        ready_client = self.create_client(
            GetPlanningScene, '/get_planning_scene')
        self.get_logger().info('Waiting for move_group to fully initialise...')
        deadline = time.time() + 30.0
        while not ready_client.wait_for_service(timeout_sec=1.0):
            if time.time() > deadline:
                self.get_logger().warn('move_group took >30s — proceeding anyway')
                break
            self.get_logger().info('Still waiting for move_group...')
        time.sleep(2.0)   # extra settle after service appears
        self._busy = False

    # ── Planning scene ────────────────────────────────────────────────────────

    # ── Easel visual helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _cylinder_triangles(p1, p2, radius, segments=8):
        """Generate TRIANGLE_LIST points for a cylinder from p1 to p2."""
        import numpy as np
        p1 = np.array(p1, dtype=float)
        p2 = np.array(p2, dtype=float)
        axis = p2 - p1
        length = np.linalg.norm(axis)
        if length < 1e-6:
            return []
        axis /= length
        perp = np.cross(axis, [1, 0, 0]) if abs(axis[0]) < 0.9 else np.cross(axis, [0, 1, 0])
        perp /= np.linalg.norm(perp)
        perp2 = np.cross(axis, perp)

        angles      = [2 * math.pi * i / segments for i in range(segments)]
        top_ring    = [p2 + radius * (math.cos(a)*perp + math.sin(a)*perp2) for a in angles]
        bottom_ring = [p1 + radius * (math.cos(a)*perp + math.sin(a)*perp2) for a in angles]

        def pt(v):
            p = Point(); p.x, p.y, p.z = float(v[0]), float(v[1]), float(v[2]); return p

        pts = []
        for i in range(segments):
            j = (i+1) % segments
            pts += [pt(bottom_ring[i]), pt(bottom_ring[j]), pt(top_ring[i]),
                    pt(bottom_ring[j]), pt(top_ring[j]),    pt(top_ring[i])]
        for ring, centre in [(top_ring, p2), (bottom_ring, p1)]:
            for i in range(segments):
                j = (i+1) % segments
                pts += [pt(centre), pt(ring[i]), pt(ring[j])]
        return pts

    def _make_leg_marker(self, mid, p1, p2, radius):
        c = WOOD_COLOUR
        m = Marker()
        m.header.frame_id  = 'base_link'
        m.ns               = 'easel'
        m.id               = mid
        m.type             = Marker.TRIANGLE_LIST
        m.action           = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color            = ColorRGBA(r=c[0], g=c[1], b=c[2], a=c[3])
        m.points           = self._cylinder_triangles(p1, p2, radius)
        return m

    def _build_easel_markers(self):
        spread = math.radians(SPREAD_ANGLE)
        back   = math.radians(BACK_ANGLE)
        leg_h  = LEG_LENGTH * math.cos(back)
        apex   = [EASEL_X, EASEL_Y, EASEL_Z + leg_h]

        fl_foot = [EASEL_X - LEG_LENGTH*math.sin(spread),
                   EASEL_Y - LEG_LENGTH*math.sin(back)*0.5, EASEL_Z]
        fr_foot = [EASEL_X + LEG_LENGTH*math.sin(spread),
                   EASEL_Y - LEG_LENGTH*math.sin(back)*0.5, EASEL_Z]
        bk_foot = [EASEL_X, EASEL_Y + LEG_LENGTH*math.sin(back), EASEL_Z]

        def at_z(foot, top, z):
            t = (z - foot[2]) / (top[2] - foot[2])
            return [foot[0]+t*(top[0]-foot[0]), foot[1]+t*(top[1]-foot[1]), z]

        markers = []
        mid = 0
        for foot in [fl_foot, fr_foot, bk_foot]:
            markers.append(self._make_leg_marker(mid, foot, apex, LEG_RADIUS))
            mid += 1

        cb_l = at_z(fl_foot, apex, CROSSBAR_Z)
        cb_r = at_z(fr_foot, apex, CROSSBAR_Z)
        markers.append(self._make_leg_marker(mid, cb_l, cb_r, CROSSBAR_R)); mid += 1

        ldg_z = CROSSBAR_Z - 0.03
        ldg_l = at_z(fl_foot, apex, ldg_z)
        ldg_r = at_z(fr_foot, apex, ldg_z)
        ldg_lb = [ldg_l[0], ldg_l[1]+0.05, ldg_z]
        ldg_rb = [ldg_r[0], ldg_r[1]+0.05, ldg_z]
        markers.append(self._make_leg_marker(mid, ldg_l, ldg_r,  CROSSBAR_R)); mid += 1
        markers.append(self._make_leg_marker(mid, ldg_l, ldg_lb, CROSSBAR_R)); mid += 1
        markers.append(self._make_leg_marker(mid, ldg_r, ldg_rb, CROSSBAR_R)); mid += 1

        top_z = CROSSBAR_Z + 0.42
        top_l = at_z(fl_foot, apex, top_z)
        top_r = at_z(fr_foot, apex, top_z)
        markers.append(self._make_leg_marker(mid, top_l, top_r, CROSSBAR_R)); mid += 1

        return markers

    def publish_easel(self):
        if self._easel_markers is None:
            return
        now = self.get_clock().now().to_msg()
        ma  = MarkerArray()
        for m in self._easel_markers:
            m.header.stamp = now
            ma.markers.append(m)
        self._easel_pub.publish(ma)

    def setup_obstacles(self):
        now = self.get_clock().now().to_msg()

        def make_box(obj_id, dims, pos):
            co                 = CollisionObject()
            co.header.frame_id = 'base_link'
            co.header.stamp    = now
            co.id              = obj_id
            co.operation       = CollisionObject.ADD
            sh                 = SolidPrimitive()
            sh.type            = SolidPrimitive.BOX
            sh.dimensions      = dims
            p                  = Pose()
            p.position.x, p.position.y, p.position.z = pos
            p.orientation.w    = 1.0
            co.primitives.append(sh)
            co.primitive_poses.append(p)
            return co

        def make_cylinder(obj_id, radius, height, pos):
            co                 = CollisionObject()
            co.header.frame_id = 'base_link'
            co.header.stamp    = now
            co.id              = obj_id
            co.operation       = CollisionObject.ADD
            sh                 = SolidPrimitive()
            sh.type            = SolidPrimitive.CYLINDER
            sh.dimensions      = [height, radius]   # [height, radius]
            p                  = Pose()
            p.position.x       = float(pos[0])
            p.position.y       = float(pos[1])
            p.position.z       = float(pos[2])
            # Explicitly set identity quaternion — ensures cylinder is vertical
            p.orientation.x    = 0.0
            p.orientation.y    = 0.0
            p.orientation.z    = 0.0
            p.orientation.w    = 1.0
            co.primitives.append(sh)
            co.primitive_poses.append(p)
            return co

        # ── Puzzle board panel (thin, left side, face at BOARD_Y) ─────────────
        board = make_box('puzzle_board',
                         [BOARD_WIDTH, BOARD_THICKNESS, BOARD_HEIGHT],
                         [BOARD_X, BOARD_Y, BOARD_Z])

        # ── Support pole (cylinder behind the board) ──────────────────────────
        # Pole goes downward from board bottom to floor
        # Centre at half the distance from floor to board bottom
        pole_bottom_z  = 0.0           # floor
        pole_top_z     = BOARD_Z - BOARD_HEIGHT / 2  # bottom of board
        pole_h         = pole_top_z - pole_bottom_z
        pole_centre_z  = pole_bottom_z + pole_h / 2
        pole = make_cylinder('board_pole',
                             POLE_RADIUS,
                             pole_h,
                             [POLE_X, POLE_Y, pole_centre_z])

        scene         = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.extend([board, pole])

        req       = ApplyPlanningScene.Request()
        req.scene = scene
        future    = self.apply_scene_client.call_async(req)
        result    = self._wait(future)

        if result is None:
            self.get_logger().error('ApplyPlanningScene failed.')
            return False

        self.get_logger().info(
            'Planning scene updated: board on easel + trolley under robot.')

        # Build easel visual markers and start publishing them
        self._easel_markers = self._build_easel_markers()
        self.create_timer(2.0, self.publish_easel)
        self.get_logger().info(
            'Easel markers ready — add MarkerArray /easel_marker in RViz.')

        time.sleep(1.0)
        return True

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _get_tool_pose(self, timeout_sec=5.0):
        target, source = 'base_link', 'tool0'
        deadline = time.time() + timeout_sec
        while not self.tf_buffer.can_transform(target, source, rclpy.time.Time()):
            if time.time() > deadline:
                self.get_logger().error('TF timeout waiting for tool0.')
                return None
            time.sleep(0.1)
        try:
            trans = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f'TF lookup failed: {e}')
            return None
        p             = Pose()
        p.position.x  = trans.transform.translation.x
        p.position.y  = trans.transform.translation.y
        p.position.z  = trans.transform.translation.z
        p.orientation = trans.transform.rotation
        return p

    def _wait(self, future):
        """
        Block until a future completes, compatible with MultiThreadedExecutor.
        Uses an Event rather than spin_until_future_complete so it works
        safely from any thread without conflicting with the running executor.
        """
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        done.wait()
        return future.result()

    # ── Joint-space move ──────────────────────────────────────────────────────

    def _move_joint_space(self, x, y, z, label='pose'):
        self.get_logger().info(
            f'Joint-space move → {label} ({x:.3f}, {y:.3f}, {z:.3f})')

        ps                    = PoseStamped()
        ps.header.frame_id    = 'base_link'
        ps.header.stamp       = self.get_clock().now().to_msg()
        ps.pose.position.x    = x
        ps.pose.position.y    = y
        ps.pose.position.z    = z
        ps.pose.orientation.x = FORCED_ORI['x']
        ps.pose.orientation.y = FORCED_ORI['y']
        ps.pose.orientation.z = FORCED_ORI['z']
        ps.pose.orientation.w = FORCED_ORI['w']

        # Position constraint — 3cm sphere tolerance
        pc                       = PositionConstraint()
        pc.header.frame_id       = 'base_link'
        pc.link_name             = 'tool0'
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        bv                       = BoundingVolume()
        region                   = SolidPrimitive()
        region.type              = SolidPrimitive.SPHERE
        region.dimensions        = [0.03]
        bv.primitives.append(region)
        bv.primitive_poses.append(ps.pose)
        pc.constraint_region     = bv
        pc.weight                = 1.0

        goal_c = Constraints()
        goal_c.position_constraints.append(pc)
        # No orientation constraint — let planner find any valid arm config
        # that reaches the position. Gripper orientation handled by FORCED_ORI
        # in the Cartesian push phase.

        req                                 = MotionPlanRequest()
        req.group_name                      = GROUP_NAME
        req.num_planning_attempts           = 20
        req.allowed_planning_time           = PLANNING_TIME * 2
        req.max_velocity_scaling_factor     = 0.3
        req.max_acceleration_scaling_factor = 0.2

        # Workspace parameters required — missing these causes 99999 rejection
        ws = WorkspaceParameters()
        ws.header.frame_id = 'base_link'
        ws.header.stamp    = self.get_clock().now().to_msg()
        ws.min_corner.x = ws.min_corner.y = ws.min_corner.z = -2.0
        ws.max_corner.x = ws.max_corner.y = ws.max_corner.z =  2.0
        req.workspace_parameters = ws
        req.goal_constraints.append(goal_c)

        goal         = MoveGroup.Goal()
        goal.request = req

        handle = self._wait(self.move_group_client.send_goal_async(goal))
        if handle is None or not handle.accepted:
            self.get_logger().error('MoveGroup goal rejected.')
            return False

        result = self._wait(handle.get_result_async())
        code   = result.result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f'Reached {label}.')
            time.sleep(0.3)
            return True
        self.get_logger().error(f'MoveGroup failed for {label} — code {code}')
        return False

    # ── Cartesian move ────────────────────────────────────────────────────────

    def _cartesian_move(self, target_x, target_y, target_z, label='target', min_fraction=None):
        live = self._get_tool_pose()
        if live is None:
            return False

        ori = live.orientation if USE_LIVE_ORI else _forced_quat()

        def wp(x, y, z):
            p             = Pose()
            p.position.x  = x
            p.position.y  = y
            p.position.z  = z
            p.orientation = copy.deepcopy(ori)
            return p

        waypoints = [
            wp(live.position.x, live.position.y, live.position.z),
            wp(target_x, target_y, target_z),
        ]

        self.get_logger().info(
            f'Cartesian → {label} ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})')

        req                  = GetCartesianPath.Request()
        req.header.frame_id  = 'base_link'
        req.group_name       = GROUP_NAME
        req.waypoints        = waypoints
        req.max_step         = MAX_STEP
        req.jump_threshold   = JUMP_THRESHOLD
        req.avoid_collisions = True

        response = self._wait(self.cartesian_client.call_async(req))
        if response is None:
            self.get_logger().error('Cartesian service returned no response.')
            return False

        fraction = response.fraction
        self.get_logger().info(f'Fraction: {fraction * 100:.1f}%')

        threshold = min_fraction if min_fraction is not None else MIN_FRACTION
        if fraction < threshold:
            self.get_logger().error(
                f'Insufficient fraction ({fraction * 100:.1f}%). '
                'Check reachability of target point.')
            return False

        goal            = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution

        handle = self._wait(self.execute_client.send_goal_async(goal))
        if handle is None or not handle.accepted:
            self.get_logger().error('ExecuteTrajectory rejected.')
            return False

        result = self._wait(handle.get_result_async())
        code   = result.result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f'Cartesian move to {label} complete.')
            return True
        self.get_logger().error(f'Execution failed — code {code}')
        return False

    # ── Board point visit ─────────────────────────────────────────────────────

    def _at_hover(self):
        """Check if end-effector is already close to hover position."""
        pose = self._get_tool_pose()
        if pose is None:
            return False
        dx = abs(pose.position.x - HOVER_X)
        dy = abs(pose.position.y - HOVER_Y)
        dz = abs(pose.position.z - HOVER_Z)
        return dx < 0.05 and dy < 0.05 and dz < 0.05

    def visit_board_point(self, key):
        pt = BOARD_POINTS[key]
        self.get_logger().info(
            f'--- Point {key}: {pt["label"]} '
            f'(x={pt["x"]:.3f}, z={pt["z"]:.3f}) ---')

        # Only retract to hover if not already there
        if self._at_hover():
            self.get_logger().info('Already at hover — skipping retract.')
        else:
            ok = self._move_joint_space(HOVER_X, HOVER_Y, HOVER_Z, label='hover')
            if not ok:
                ok = self._cartesian_move(HOVER_X, HOVER_Y, HOVER_Z, label='hover')
            if not ok:
                self.get_logger().error('Could not retract to hover — aborting.')
                return

        # Phase 1: joint-space to get near the board point
        ok = self._move_joint_space(pt['x'], PUSH_Y, pt['z'], label=pt['label'])
        if not ok:
            self.get_logger().warn('Joint-space failed — trying Cartesian only.')
            self._cartesian_move(pt['x'], PUSH_Y, pt['z'], label=pt['label'])
            return

        # Phase 2: Cartesian push to board face
        # Use a low fraction threshold — arm is already near target so
        # even a short Cartesian path (10%+) gets us to the board
        self.get_logger().info('Cartesian push to board face...')
        self._cartesian_move(pt['x'], BOARD_Y - BOARD_THICKNESS - 0.01,
                             pt['z'], label=f'{pt["label"]} (push)',
                             min_fraction=0.10)

    def go_to_hover(self):
        """
        Move to hover position with gripper facing the board.
        If HOVER_JOINTS is set, uses a direct joint trajectory — guaranteed
        correct orientation. Otherwise falls back to IK-based joint-space move.
        Record HOVER_JOINTS by:
          1. Use keyboard_bridge to drive arm to hover with gripper facing board
          2. ros2 topic echo /joint_states --once
          3. Paste the 6 arm joint values into HOVER_JOINTS above
        """
        if HOVER_JOINTS is not None:
            return self._move_to_joints(HOVER_JOINTS, label='hover')
        # Fallback — IK may not face board correctly
        ok = self._move_joint_space(HOVER_X, HOVER_Y, HOVER_Z, label='hover')
        if not ok:
            return False
        self._cartesian_move(HOVER_X, HOVER_Y, HOVER_Z,
                             label='hover orientation', min_fraction=0.01)
        return True

    def _move_to_joints(self, joint_positions, label='joints'):
        """
        Move to exact joint configuration via MoveGroup with joint constraints.
        Collision-checked along the path but goal is unambiguous — no IK.
        Each joint is constrained to within 0.01 rad of the target value.
        """
        joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
        ]

        self.get_logger().info(f'Joint-config move → {label}')

        # Build joint constraints — one per joint, tight tolerance
        goal_c = Constraints()
        for name, pos in zip(joint_names, joint_positions):
            jc                 = JointConstraint()
            jc.joint_name      = name
            jc.position        = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight          = 1.0
            goal_c.joint_constraints.append(jc)

        req                                 = MotionPlanRequest()
        req.group_name                      = GROUP_NAME
        req.num_planning_attempts           = 10
        req.allowed_planning_time           = PLANNING_TIME
        req.max_velocity_scaling_factor     = 0.3
        req.max_acceleration_scaling_factor = 0.2

        ws = WorkspaceParameters()
        ws.header.frame_id = 'base_link'
        ws.header.stamp    = self.get_clock().now().to_msg()
        ws.min_corner.x = ws.min_corner.y = ws.min_corner.z = -2.0
        ws.max_corner.x = ws.max_corner.y = ws.max_corner.z =  2.0
        req.workspace_parameters = ws
        req.goal_constraints.append(goal_c)

        goal         = MoveGroup.Goal()
        goal.request = req

        handle = self._wait(self.move_group_client.send_goal_async(goal))
        if handle is None or not handle.accepted:
            self.get_logger().error(f'MoveGroup rejected joint-config goal for {label}.')
            return False

        result = self._wait(handle.get_result_async())
        code   = result.result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f'Reached {label}.')
            time.sleep(0.3)
            return True

        self.get_logger().error(f'Joint-config move failed for {label} — code {code}')
        return False

    # ── Keyboard listener ─────────────────────────────────────────────────────

    def start_keyboard_listener(self):
        threading.Thread(target=self._keyboard_loop, daemon=True).start()

    def _keyboard_loop(self):
        fd  = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        print('\n--- Board point selector ---')
        for k, pt in BOARD_POINTS.items():
            print(f'  {k}  →  {pt["label"]}  (x={pt["x"]:.3f}, z={pt["z"]:.3f})')
        print('  b  →  move to board centre (wall position)')
        print('  f  →  move to floor (egg pickup position)')
        print('  h  →  return to hover')
        print('  q  →  quit\n')
        try:
            while True:
                ch = sys.stdin.read(1)
                if ch == 'q':
                    print('Quitting.')
                    rclpy.shutdown()
                    break
                if self._busy:
                    print(f'[{ch}] ignored — arm is moving')
                    continue
                if ch == 'b':
                    self._busy = True
                    threading.Thread(
                        target=lambda: [self._move_to_joints(WALL_JOINTS, 'board centre'),
                                        setattr(self, '_busy', False)],
                        daemon=True).start()
                elif ch == 'f':
                    self._busy = True
                    threading.Thread(
                        target=lambda: [self._move_to_joints(FLOOR_JOINTS, 'floor pickup'),
                                        setattr(self, '_busy', False)],
                        daemon=True).start()
                elif ch == 'h':
                    self._busy = True
                    threading.Thread(
                        target=lambda: [self.go_to_hover(),
                                        setattr(self, '_busy', False)],
                        daemon=True).start()
                elif ch in BOARD_POINTS:
                    self._busy = True
                    threading.Thread(
                        target=self._run_point,
                        args=(ch,),
                        daemon=True
                    ).start()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _run_point(self, key):
        try:
            self.visit_board_point(key)
        finally:
            self._busy = False


def _forced_quat():
    from geometry_msgs.msg import Quaternion
    q   = Quaternion()
    q.x = FORCED_ORI['x']
    q.y = FORCED_ORI['y']
    q.z = FORCED_ORI['z']
    q.w = FORCED_ORI['w']
    return q


def main(args=None):
    rclpy.init(args=args)
    node = PuzzleTask()

    # Start the MultiThreadedExecutor FIRST so every async call —
    # including setup and the initial hover move — goes through the
    # same executor. Running setup before spin() and then switching
    # executors leaves internal state that causes the node to stop
    # responding after the first blocking call completes.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Spin the executor in a background thread so main() can continue.
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Run the startup sequence in the main thread — _wait() is safe here
    # because the executor is already spinning in the background.
    try:
        print('Setting up obstacles ...')
        if not node.setup_obstacles():
            print('ERROR: Could not set up planning scene.')
            rclpy.shutdown()
            return

        print('Moving to hover position ...')
        if not node.go_to_hover():
            print('ERROR: Could not reach hover — check HOVER_X/Y/Z.')
            rclpy.shutdown()
            return

        print('At hover. Press 1-4 to visit a board point, q to quit.')
        node.start_keyboard_listener()

        # Keep main alive until shutdown
        spin_thread.join()

    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()