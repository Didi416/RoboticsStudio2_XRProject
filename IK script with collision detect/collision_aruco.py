import copy
import time
import threading
import sys
import tty
import termios
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    CollisionObject, MoveItErrorCodes, PlanningScene,
    MotionPlanRequest, Constraints, PositionConstraint,
    OrientationConstraint, BoundingVolume,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # needed for do_transform_pose


# ── Board geometry ────────────────────────────────────────────────────────────
BOARD_X         = 0.42
BOARD_Y         = 0.0
BOARD_Z         = 0.25
BOARD_THICKNESS = 0.02
BOARD_WIDTH     = 0.40
BOARD_HEIGHT    = 0.40

# ── Hover pose ────────────────────────────────────────────────────────────────
HOVER_X = 0.25
HOVER_Y = 0.0
HOVER_Z = 0.25
PUSH_X  = BOARD_X - BOARD_THICKNESS - 0.01

FORCED_ORI   = dict(x=0.0, y=0.707, z=0.0, w=0.707)
USE_LIVE_ORI = True

# ── 4 board target points (y=left/right, z=up/down) ──────────────────────────
BOARD_POINTS = {
    '1': dict(y= 0.10, z=0.35, label='Top left'),
    '2': dict(y=-0.10, z=0.35, label='Top right'),
    '3': dict(y= 0.10, z=0.15, label='Bottom left'),
    '4': dict(y=-0.10, z=0.15, label='Bottom right'),
}

# ── ArUco integration ─────────────────────────────────────────────────────────
# Map ArUco marker IDs to the puzzle name (from multi_aruco_detector.py)
ARUCO_MARKER_IDS = {
    0: 'frog_call_console',
    1: 'lily_pad_maze',
    2: 'frog_egg_sorting',
    3: 'containment_dial',
}

# The robot's tool will stop this far in front of the board surface (metres).
# Increase this if you want the end-effector to stop short of the detected point.
ARUCO_APPROACH_STANDOFF = 0.05

# TF frame the ArUco detector publishes poses in (see multi_aruco_detector.py)
ARUCO_SOURCE_FRAME = 'camera_color_optical_frame'
# TF frame your MoveIt planner works in
ROBOT_BASE_FRAME   = 'base_link'

# ── Cartesian settings ────────────────────────────────────────────────────────
MAX_STEP       = 0.01
JUMP_THRESHOLD = 0.0
MIN_FRACTION   = 0.95

GROUP_NAME     = 'ur_manipulator'
PLANNING_TIME  = 5.0


class PuzzleTask(Node):
    def __init__(self):
        super().__init__('puzzle_task_node')

        self._cbg = ReentrantCallbackGroup()

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        self._busy = False

        # ── Latest ArUco poses in base_link frame ─────────────────────────────
        # Keyed by marker ID. Populated by _aruco_cb, read by keyboard handler.
        self._aruco_poses: dict[int, Pose] = {}
        self._aruco_lock  = threading.Lock()

        # Subscribe to every known marker's pose topic
        for mid in ARUCO_MARKER_IDS:
            self.create_subscription(
                PoseStamped,
                f'/aruco/marker_{mid}/pose',
                lambda msg, m=mid: self._aruco_cb(msg, m),
                10,
                callback_group=self._cbg,
            )
            self.get_logger().info(
                f'Subscribed to /aruco/marker_{mid}/pose')

    # ── ArUco callback ────────────────────────────────────────────────────────

    def _aruco_cb(self, msg: PoseStamped, marker_id: int):
        """
        Receives a PoseStamped in camera_color_optical_frame, transforms it
        into base_link, applies the approach standoff, and caches the result.
        """
        source_frame = msg.header.frame_id or ARUCO_SOURCE_FRAME

        # Wait up to 1 s for the TF transform to be available
        try:
            if not self.tf_buffer.can_transform(
                    ROBOT_BASE_FRAME, source_frame,
                    rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
                self.get_logger().warn(
                    f'TF not yet available: {source_frame} → {ROBOT_BASE_FRAME}',
                    throttle_duration_sec=5.0)
                return

            transform = self.tf_buffer.lookup_transform(
                ROBOT_BASE_FRAME, source_frame, rclpy.time.Time())

        except Exception as e:
            self.get_logger().warn(
                f'TF lookup failed for marker {marker_id}: {e}',
                throttle_duration_sec=5.0)
            return

        # Transform pose into base_link
        pose_in_base = tf2_geometry_msgs.do_transform_pose(msg.pose, transform)

        # Apply standoff: back the target off from the board surface.
        # We assume the board faces the robot along the X axis, so we
        # subtract the standoff from X to avoid touching the board.
        pose_in_base.position.x -= ARUCO_APPROACH_STANDOFF

        with self._aruco_lock:
            self._aruco_poses[marker_id] = pose_in_base

        self.get_logger().debug(
            f'Marker {marker_id} → base_link '
            f'({pose_in_base.position.x:.3f}, '
            f' {pose_in_base.position.y:.3f}, '
            f' {pose_in_base.position.z:.3f})')

    # ── Planning scene ────────────────────────────────────────────────────────

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

        board = make_box('puzzle_board',
                         [BOARD_THICKNESS, BOARD_WIDTH, BOARD_HEIGHT],
                         [BOARD_X, BOARD_Y, BOARD_Z])
        floor = make_box('workspace_floor',
                         [0.80, 0.80, 0.02],
                         [0.30, 0.0, -0.03])

        scene         = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.extend([board, floor])

        req       = ApplyPlanningScene.Request()
        req.scene = scene
        future    = self.apply_scene_client.call_async(req)
        result    = self._wait(future)

        if result is None:
            self.get_logger().error('ApplyPlanningScene failed.')
            return False

        self.get_logger().info('Planning scene updated.')
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

        pc                       = PositionConstraint()
        pc.header.frame_id       = 'base_link'
        pc.link_name             = 'tool0'
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        bv                       = BoundingVolume()
        region                   = SolidPrimitive()
        region.type              = SolidPrimitive.SPHERE
        region.dimensions        = [0.005]
        bv.primitives.append(region)
        bv.primitive_poses.append(ps.pose)
        pc.constraint_region     = bv
        pc.weight                = 1.0

        oc                           = OrientationConstraint()
        oc.header.frame_id           = 'base_link'
        oc.link_name                 = 'tool0'
        oc.orientation               = ps.pose.orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight                    = 1.0

        goal_c = Constraints()
        goal_c.position_constraints.append(pc)
        goal_c.orientation_constraints.append(oc)

        req                                 = MotionPlanRequest()
        req.group_name                      = GROUP_NAME
        req.num_planning_attempts           = 10
        req.allowed_planning_time           = PLANNING_TIME
        req.max_velocity_scaling_factor     = 0.3
        req.max_acceleration_scaling_factor = 0.2
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

    def _cartesian_move(self, target_x, target_y, target_z, label='target'):
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

        if fraction < MIN_FRACTION:
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

    # ── Board point visit (fixed 4-point mode) ────────────────────────────────

    def visit_board_point(self, key):
        pt = BOARD_POINTS[key]
        self.get_logger().info(
            f'--- Point {key}: {pt["label"]} '
            f'(y={pt["y"]:.3f}, z={pt["z"]:.3f}) ---')

        ok = self._cartesian_move(HOVER_X, HOVER_Y, HOVER_Z, label='hover')
        if not ok:
            self.get_logger().warn('Cartesian retract failed — trying joint-space.')
            ok = self._move_joint_space(HOVER_X, HOVER_Y, HOVER_Z, label='hover')
        if not ok:
            self.get_logger().error('Could not retract — aborting.')
            return

        self._cartesian_move(PUSH_X, pt['y'], pt['z'], label=pt['label'])

    # ── ArUco point visit ─────────────────────────────────────────────────────

    def visit_aruco_point(self, marker_id: int):
        """
        Move to the most recently detected pose of the given ArUco marker.
        The pose has already been transformed to base_link and the standoff
        applied by _aruco_cb.
        """
        with self._aruco_lock:
            pose = self._aruco_poses.get(marker_id)

        if pose is None:
            self.get_logger().warn(
                f'Marker {marker_id} not yet detected — '
                'make sure the camera can see it.')
            return

        name = ARUCO_MARKER_IDS.get(marker_id, f'marker_{marker_id}')
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z

        self.get_logger().info(
            f'--- ArUco target: {name} (ID {marker_id}) '
            f'@ ({x:.3f}, {y:.3f}, {z:.3f}) in base_link ---')

        # Retract to hover first
        ok = self._cartesian_move(HOVER_X, HOVER_Y, HOVER_Z, label='hover')
        if not ok:
            self.get_logger().warn('Cartesian retract failed — trying joint-space.')
            ok = self._move_joint_space(HOVER_X, HOVER_Y, HOVER_Z, label='hover')
        if not ok:
            self.get_logger().error('Could not retract — aborting.')
            return

        # Move to the ArUco-derived target
        self._cartesian_move(x, y, z, label=name)

    def go_to_hover(self):
        return self._move_joint_space(HOVER_X, HOVER_Y, HOVER_Z, label='hover')

    # ── Keyboard listener ─────────────────────────────────────────────────────

    def start_keyboard_listener(self):
        threading.Thread(target=self._keyboard_loop, daemon=True).start()

    def _keyboard_loop(self):
        fd  = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        print('\n--- Board point selector ---')
        for k, pt in BOARD_POINTS.items():
            print(f'  {k}  →  {pt["label"]}  (y={pt["y"]:.3f}, z={pt["z"]:.3f})')

        print('\n--- ArUco marker targets ---')
        for mid, name in ARUCO_MARKER_IDS.items():
            # Use keys a/b/c/d for IDs 0/1/2/3
            key = chr(ord('a') + mid)
            print(f'  {key}  →  {name}  (marker ID {mid})')

        print('\n  q  →  quit\n')

        # Map letter keys to ArUco marker IDs
        aruco_keys = {chr(ord('a') + mid): mid for mid in ARUCO_MARKER_IDS}

        try:
            while True:
                ch = sys.stdin.read(1)

                if ch == 'q':
                    print('Quitting.')
                    rclpy.shutdown()
                    break

                # Fixed board point (keys 1-4)
                if ch in BOARD_POINTS:
                    if self._busy:
                        print(f'[{ch}] ignored — arm is moving')
                        continue
                    self._busy = True
                    threading.Thread(
                        target=self._run_point,
                        args=(ch,),
                        daemon=True
                    ).start()

                # ArUco marker target (keys a-d)
                elif ch in aruco_keys:
                    if self._busy:
                        print(f'[{ch}] ignored — arm is moving')
                        continue
                    mid = aruco_keys[ch]
                    self._busy = True
                    threading.Thread(
                        target=self._run_aruco_point,
                        args=(mid,),
                        daemon=True
                    ).start()

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _run_point(self, key):
        try:
            self.visit_board_point(key)
        finally:
            self._busy = False

    def _run_aruco_point(self, marker_id: int):
        try:
            self.visit_aruco_point(marker_id)
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

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

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

        print('At hover. Press 1-4 for fixed board points, a-d for ArUco targets, q to quit.')
        node.start_keyboard_listener()

        spin_thread.join()

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()