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
from std_msgs.msg import ColorRGBA, String
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
 
# ── Board geometry ───────────────────────────────────────────────────────────
BOARD_Y         =  0.52
BOARD_X         =  0.0
BOARD_Z         =  0.3
BOARD_THICKNESS =  0.02
BOARD_WIDTH     =  0.20
BOARD_HEIGHT    =  0.20
 
POLE_RADIUS     =  0.025
POLE_HEIGHT     =  1.20
POLE_X          =  0.0
POLE_Y          =  BOARD_Y + BOARD_THICKNESS / 2 + POLE_RADIUS

TROLLEY_WIDTH  =  0.765
TROLLEY_DEPTH  =  0.802
TROLLEY_HEIGHT =  1.091
TROLLEY_X      = -0.031
TROLLEY_Y      =  0.179
 
HOVER_X =  0.0
HOVER_Y =  BOARD_Y - 0.40
HOVER_Z =  BOARD_Z + 0.5
PUSH_Y  =  BOARD_Y - BOARD_THICKNESS - 0.08
 
HOVER_JOINTS  = [0.7785,  0.0346, -2.2865, -0.8898, -0.7786,  0.0016]  # reuse board for now
WALL_JOINTS   = [0.7785,  0.0346, -2.2865, -0.8898, -0.7786,  0.0016]  # board facing
FLOOR_JOINTS  = [5.0425, -3.3372,  1.4877, -2.8162,  1.5323,  0.3074]  # floor facing
 
FORCED_ORI   = dict(x=-0.7071, y=0.0, z=0.0, w=0.7071)
USE_LIVE_ORI = True
 
BOARD_POINTS = {
    '1': dict(x= 0.0000, z=BOARD_Z + 0.0000, label='Joystick mount'),
    '2': dict(x= 0.0548, z=BOARD_Z + 0.0483, label='Button matrix'),
    '3': dict(x= 0.0155, z=BOARD_Z - 0.0300, label='Card holder'),
    '4': dict(x= 0.0500, z=BOARD_Z - 0.0855, label='RFID slot'),
    '5': dict(x=-0.0605, z=BOARD_Z + 0.0595, label='Left panel'),
}
 
EASEL_X        =  POLE_X
EASEL_Y        =  BOARD_Y + BOARD_THICKNESS / 2 + 0.02
EASEL_Z        =  0.0
LEG_LENGTH     =  1.40
LEG_RADIUS     =  0.015
SPREAD_ANGLE   =  15.0
BACK_ANGLE     =  25.0
CROSSBAR_Z     =  0.70
CROSSBAR_R     =  0.012
WOOD_COLOUR    = (0.72, 0.53, 0.30, 1.0)
 
MAX_STEP          = 0.01
JUMP_THRESHOLD    = 0.0
MIN_FRACTION      = 0.95
MIN_FRACTION_PUSH = 0.10
 
GROUP_NAME     = 'ur_onrobot_manipulator'
PLANNING_TIME  = 10.0
 
 
class PuzzleTask(Node):
    def __init__(self):
        super().__init__('puzzle_task_node')
 
        self._cbg = ReentrantCallbackGroup()
 
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
 
        self._easel_pub = self.create_publisher(MarkerArray, '/easel_marker', 10)
        self._easel_markers = None
 
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
        time.sleep(2.0)
 
        # ─────────────────────────────────────────
        # GUI COMMAND SUBSCRIBER
        # Receives commands from Unity VR GUI
        # ─────────────────────────────────────────
        self.create_subscription(
            String,
            '/robot_gui/command',
            self.on_gui_command,
            10,
            callback_group=self._cbg
        )
        self.get_logger().info("GUI command subscriber ready on /robot_gui/command")
 
        self._busy = False
 
    # ─────────────────────────────────────────
    # GUI COMMAND HANDLER
    # Called when Unity GUI button pressed
    # ─────────────────────────────────────────
 
    def on_gui_command(self, msg):
        """Receive robot commands from Unity VR GUI"""
        command = msg.data
        self.get_logger().info(f"GUI command received: {command}")
 
        if self._busy:
            self.get_logger().warn(f"Robot busy - ignoring: {command}")
            return
 
        self._busy = True
        threading.Thread(
            target=self._execute_command,
            args=(command,),
            daemon=True
        ).start()
 
    def _execute_command(self, command):
        """Execute robot command in separate thread"""
        try:
            if command == "hover":
                self.get_logger().info("Moving to hover position")
                self.go_to_hover()
 
            elif command == "face_board":
                self.get_logger().info("Moving to face puzzle board")
                self._move_to_joints(WALL_JOINTS, 'board centre')
 
            elif command == "face_eggs":
                self.get_logger().info("Moving to face egg puzzles")
                self._move_to_joints(FLOOR_JOINTS, 'floor pickup')
 
            elif command.startswith("point_"):
                key = command.split("_")[1]
                if key in BOARD_POINTS:
                    self.get_logger().info(f"Moving to board point {key}")
                    self.visit_board_point(key)
                else:
                    self.get_logger().warn(f"Unknown board point: {key}")
 
            else:
                self.get_logger().warn(f"Unknown command: {command}")
 
        finally:
            self._busy = False
 
    # ─────────────────────────────────────────
    # EASEL VISUAL HELPERS
    # ─────────────────────────────────────────
 
    @staticmethod
    def _cylinder_triangles(p1, p2, radius, segments=8):
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
 
    # ─────────────────────────────────────────
    # PLANNING SCENE
    # ─────────────────────────────────────────
 
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
            sh.dimensions      = [height, radius]
            p                  = Pose()
            p.position.x       = float(pos[0])
            p.position.y       = float(pos[1])
            p.position.z       = float(pos[2])
            p.orientation.x    = 0.0
            p.orientation.y    = 0.0
            p.orientation.z    = 0.0
            p.orientation.w    = 1.0
            co.primitives.append(sh)
            co.primitive_poses.append(p)
            return co
 
        board = make_box('puzzle_board',
                         [BOARD_WIDTH, BOARD_THICKNESS, BOARD_HEIGHT],
                         [BOARD_X, BOARD_Y, BOARD_Z])
 
        pole_bottom_z  = 0.0
        pole_top_z     = BOARD_Z - BOARD_HEIGHT / 2
        pole_h         = pole_top_z - pole_bottom_z
        pole_centre_z  = pole_bottom_z + pole_h / 2
        pole = make_cylinder('board_pole',
                             POLE_RADIUS,
                             pole_h,
                             [POLE_X, POLE_Y, pole_centre_z])
 
        scene         = PlanningScene()
        scene.is_diff = True

        trolley_coll_h = TROLLEY_HEIGHT - 0.03
        trolley = make_box('robot_trolley',
                   [TROLLEY_WIDTH, TROLLEY_DEPTH, trolley_coll_h],
                   [TROLLEY_X, TROLLEY_Y,
                    -TROLLEY_HEIGHT + trolley_coll_h / 2])

        scene.world.collision_objects.extend([board, pole, trolley])
 
        req       = ApplyPlanningScene.Request()
        req.scene = scene
        future    = self.apply_scene_client.call_async(req)
        result    = self._wait(future)
 
        if result is None:
            self.get_logger().error('ApplyPlanningScene failed.')
            return False
 
        self.get_logger().info('Planning scene updated.')
 
        self._easel_markers = self._build_easel_markers()
        self.create_timer(2.0, self.publish_easel)
 
        time.sleep(1.0)
        return True
 
    # ─────────────────────────────────────────
    # HELPERS
    # ─────────────────────────────────────────
 
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
 
    # ─────────────────────────────────────────
    # JOINT SPACE MOVE
    # ─────────────────────────────────────────
 
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
        region.dimensions        = [0.03]
        bv.primitives.append(region)
        bv.primitive_poses.append(ps.pose)
        pc.constraint_region     = bv
        pc.weight                = 1.0
 
        goal_c = Constraints()
        goal_c.position_constraints.append(pc)
 
        req                                 = MotionPlanRequest()
        req.group_name                      = GROUP_NAME
        req.num_planning_attempts           = 20
        req.allowed_planning_time           = PLANNING_TIME * 2
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
 
    # ─────────────────────────────────────────
    # CARTESIAN MOVE
    # ─────────────────────────────────────────
 
    def _cartesian_move(self, target_x, target_y, target_z,
                        label='target', min_fraction=None):
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
 
        fraction  = response.fraction
        threshold = min_fraction if min_fraction is not None else MIN_FRACTION
        self.get_logger().info(f'Fraction: {fraction * 100:.1f}%')
 
        if fraction < threshold:
            self.get_logger().error(
                f'Insufficient fraction ({fraction * 100:.1f}%).')
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
 
    # ─────────────────────────────────────────
    # BOARD POINT VISIT
    # ─────────────────────────────────────────
 
    def _at_hover(self):
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
            f'--- Point {key}: {pt["label"]} ---')
 
        if self._at_hover():
            self.get_logger().info('Already at hover — skipping retract.')
        else:
            ok = self._move_joint_space(HOVER_X, HOVER_Y, HOVER_Z, label='hover')
            if not ok:
                ok = self._cartesian_move(HOVER_X, HOVER_Y, HOVER_Z, label='hover')
            if not ok:
                self.get_logger().error('Could not retract to hover.')
                return
 
        ok = self._move_joint_space(pt['x'], PUSH_Y, pt['z'], label=pt['label'])
        if not ok:
            self.get_logger().warn('Joint-space failed — trying Cartesian.')
            self._cartesian_move(pt['x'], PUSH_Y, pt['z'], label=pt['label'])
            return
 
        self._cartesian_move(
            pt['x'], BOARD_Y - BOARD_THICKNESS - 0.01,
            pt['z'], label=f'{pt["label"]} (push)',
            min_fraction=0.10)
 
    def go_to_hover(self):
        if HOVER_JOINTS is not None:
            return self._move_to_joints(HOVER_JOINTS, label='hover')
        ok = self._move_joint_space(HOVER_X, HOVER_Y, HOVER_Z, label='hover')
        if not ok:
            return False
        self._cartesian_move(HOVER_X, HOVER_Y, HOVER_Z,
                             label='hover orientation', min_fraction=0.01)
        return True
 
    def _move_to_joints(self, joint_positions, label='joints'):
        joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
        ]
 
        self.get_logger().info(f'Joint-config move → {label}')
 
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
 
    # ─────────────────────────────────────────
    # KEYBOARD LISTENER
    # ─────────────────────────────────────────
 
    def start_keyboard_listener(self):
        threading.Thread(target=self._keyboard_loop, daemon=True).start()
 
    def _keyboard_loop(self):
        fd  = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        print('\n--- Board point selector ---')
        for k, pt in BOARD_POINTS.items():
            print(f'  {k}  →  {pt["label"]}')
        print('  b  →  face puzzle board')
        print('  f  →  face egg puzzles')
        print('  h  →  hover')
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
                        target=lambda: [
                            self._move_to_joints(WALL_JOINTS, 'board centre'),
                            setattr(self, '_busy', False)],
                        daemon=True).start()
                elif ch == 'f':
                    self._busy = True
                    threading.Thread(
                        target=lambda: [
                            self._move_to_joints(FLOOR_JOINTS, 'floor pickup'),
                            setattr(self, '_busy', False)],
                        daemon=True).start()
                elif ch == 'h':
                    self._busy = True
                    threading.Thread(
                        target=lambda: [
                            self.go_to_hover(),
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
 
    executor = MultiThreadedExecutor()
    executor.add_node(node)
 
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
 
    try:
        print('Setting up obstacles...')
        if not node.setup_obstacles():
            print('ERROR: Could not set up planning scene.')
            rclpy.shutdown()
            return
 
        print('Moving to hover position...')
        if not node.go_to_hover():
            print('ERROR: Could not reach hover.')
            rclpy.shutdown()
            return
 
        print('Ready! Press b=board, f=eggs, h=hover, 1-5=board points, q=quit')
        print('Unity VR GUI commands also accepted on /robot_gui/command')
        node.start_keyboard_listener()
 
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