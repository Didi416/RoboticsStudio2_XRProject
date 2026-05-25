# import sys
# import tty
# import termios
# import threading
# import time
# import rclpy
# from rclpy.node import Node
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.callback_groups import ReentrantCallbackGroup
# from geometry_msgs.msg import TwistStamped, Pose
# from std_msgs.msg import Float64MultiArray
# from std_srvs.srv import Trigger
# from sensor_msgs.msg import JointState
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from builtin_interfaces.msg import Duration
# from controller_manager_msgs.srv import SwitchController
# from moveit_msgs.srv import GetCartesianPath
# from moveit_msgs.action import MoveGroup, ExecuteTrajectory
# from moveit_msgs.msg import MoveItErrorCodes
# from rclpy.action import ActionClient
# from tf2_ros import Buffer, TransformListener

# # ── Tuning ────────────────────────────────────────────────────────────────────
# LINEAR_SPEED  = 0.05
# ANGULAR_SPEED = 0.3
# PUBLISH_RATE  = 30.0
# KEY_TIMEOUT   = 0.08
# COMMAND_FRAME = 'tool0'

# Minimum distance (metres) threshold for joystick spring-back. Defined
# here because the original constant in the commented header may be
# inactive in some contexts.
MIN_RETURN_DIST = 0.005
# Time allowed for joystick spring-back trajectory execution (seconds)
JOYSTICK_RETURN_TIME = 0.8

# # ── Controller names ──────────────────────────────────────────────────────────
# SERVO_CONTROLLER   = 'forward_position_controller'
# PLANNER_CONTROLLER = 'scaled_joint_trajectory_controller'
# SWITCH_CONTROLLER_SERVICE = '/controller_manager/switch_controller'
# GROUP_NAME = 'ur_onrobot_manipulator'

# # ── Servo services ────────────────────────────────────────────────────────────
# SERVO_TOPIC         = '/servo_node/delta_twist_cmds'
# SERVO_START_SERVICE = '/servo_node/start_servo'
# SERVO_STOP_SERVICE  = '/servo_node/stop_servo'
# SERVO_UNPAUSE       = '/servo_node/unpause_servo'
# SERVO_RESET         = '/servo_node/reset_servo_status'

# # ── Gripper ───────────────────────────────────────────────────────────────────
# GRIPPER_TOPIC      = '/finger_width_controller/commands'
# GRIPPER_OPEN       = 0.10
# GRIPPER_CLOSE      = 0.0
# GRIPPER_STEP       = 0.01
# GRIPPER_SPEED      = 0.04
# GRIPPER_ACTIVATE   = 'finger_width_controller'
# GRIPPER_DEACTIVATE = 'finger_width_trajectory_controller'

# # ── Wrist snap ────────────────────────────────────────────────────────────────
# WRIST_ROTATE_UP   = -1.5708
# WRIST_ROTATE_DOWN =  1.5708
# WRIST_MOVE_TIME   = 1.5

# WRIST_FLOOR_KEY = 'z'
# WRIST_WALL_KEY  = 'x'

# SNAP_JOINT_NAMES = [
#     'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
#     'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
# ]

# # ── Joystick mode ─────────────────────────────────────────────────────────────
# JOYSTICK_MODE_KEY    = 'j'
# JOYSTICK_RETURN_TIME = 0.8   # seconds for spring-back trajectory
# MIN_RETURN_DIST      = 0.005  # metres — don't return if already this close

# JOYSTICK_BINDINGS = {
#     'w': ( 0,  0,  1,  0,  0,  0),   # up    (+Z world)
#     's': ( 0,  0, -1,  0,  0,  0),   # down  (-Z world)
#     'a': ( 0, -1,  0,  0,  0,  0),   # left  (-Y world)
#     'd': ( 0,  1,  0,  0,  0,  0),   # right (+Y world)
# }

# # ── Key bindings (normal EE mode) ─────────────────────────────────────────────
# KEY_BINDINGS = {
#     'w': ( 1,  0,  0,  0,  0,  0),
#     's': (-1,  0,  0,  0,  0,  0),
#     'a': ( 0, -1,  0,  0,  0,  0),
#     'd': ( 0,  1,  0,  0,  0,  0),
#     'r': ( 0,  0,  1,  0,  0,  0),
#     'f': ( 0,  0, -1,  0,  0,  0),
#     'i': ( 0,  0,  0,  0,  0,  1),
#     'k': ( 0,  0,  0,  0,  0, -1),
# }

# BASE_ROT_KEYS = {
#     'u': ( 0,  0,  0,  0,  0,  1),
#     'o': ( 0,  0,  0,  0,  0, -1),
# }

# TOGGLE_KEY = ' '
# ESTOP_KEY  = 'p'
# QUIT_KEY   = 'q'


# class KeyboardServoNode(Node):
#     def __init__(self):
#         super().__init__('keyboard_servo_node')

#         self._cbg = ReentrantCallbackGroup()

#         # TF
#         self._tf_buffer   = Buffer()
#         self._tf_listener = TransformListener(self._tf_buffer, self)

#         # Publishers
#         self._pub         = self.create_publisher(TwistStamped, SERVO_TOPIC, 10)
#         self._gripper_pub = self.create_publisher(Float64MultiArray, GRIPPER_TOPIC, 10)
#         self._snap_pub    = self.create_publisher(
#             JointTrajectory, f'/{PLANNER_CONTROLLER}/joint_trajectory', 10)

#         # Servo services
#         self._start_client   = self.create_client(Trigger, SERVO_START_SERVICE, callback_group=self._cbg)
#         self._stop_client    = self.create_client(Trigger, SERVO_STOP_SERVICE,  callback_group=self._cbg)
#         self._unpause_client = self.create_client(Trigger, SERVO_UNPAUSE,       callback_group=self._cbg)
#         self._reset_client   = self.create_client(Trigger, SERVO_RESET,         callback_group=self._cbg)
#         self._switch_client  = self.create_client(SwitchController, SWITCH_CONTROLLER_SERVICE, callback_group=self._cbg)

#         # Cartesian path + execute for spring-back
#         self._cartesian_client = self.create_client(
#             GetCartesianPath, 'compute_cartesian_path',
#             callback_group=self._cbg)
#         self._execute_client   = ActionClient(
#             self, ExecuteTrajectory, 'execute_trajectory',
#             callback_group=self._cbg)

#         # Joint state
#         self._joint_state = None
#         self._js_lock     = threading.Lock()
#         self.create_subscription(JointState, '/joint_states',
#                                  self._js_callback, 10, callback_group=self._cbg)

#         self._lock      = threading.Lock()
#         self._vx = self._vy = self._vz = 0.0
#         self._rx = self._ry = self._rz = 0.0
#         self._cmd_frame = COMMAND_FRAME

#         self._running          = True
#         self._servo_active     = False
#         self._snapping         = False
#         self._estopped         = False
#         self._joystick_mode    = False
#         self._joystick_neutral = None
#         self._joystick_busy    = False   # True while returning to neutral
#         self._last_key_time    = 0.0
#         self._finger_width     = GRIPPER_OPEN
#         self._gripper_vel      = 0.0
#         self._gripper_key_time = 0.0

#         self._timer = self.create_timer(
#             1.0 / PUBLISH_RATE, self._publish, callback_group=self._cbg)

#     # ── Async wait ────────────────────────────────────────────────────────────

#     def _wait(self, future, timeout=5.0):
#         done = threading.Event()
#         future.add_done_callback(lambda _: done.set())
#         done.wait(timeout=timeout)
#         return future.result()

#     def _call(self, client, label, timeout=5.0):
#         if not client.wait_for_service(timeout_sec=timeout):
#             self.get_logger().warn(f'{label} not available.')
#             return False
#         result = self._wait(client.call_async(Trigger.Request()), timeout)
#         if result:
#             self.get_logger().info(f'{label}: {result.message}')
#         return result is not None

#     # ── Controller switching ──────────────────────────────────────────────────

#     def _switch_controllers(self, activate, deactivate):
#         if not self._switch_client.wait_for_service(timeout_sec=5.0):
#             self.get_logger().error('switch_controller not available.')
#             return False
#         req                        = SwitchController.Request()
#         req.activate_controllers   = [activate]
#         req.deactivate_controllers = [deactivate]
#         req.strictness             = SwitchController.Request.BEST_EFFORT
#         req.activate_asap          = True
#         req.timeout                = rclpy.duration.Duration(seconds=2.0).to_msg()
#         result = self._wait(self._switch_client.call_async(req), timeout=5.0)
#         if result and result.ok:
#             self.get_logger().info(f'+{activate}  -{deactivate}')
#             return True
#         return False

#     # ── Servo lifecycle ───────────────────────────────────────────────────────

#     def activate_servo(self):
#         self._switch_controllers(SERVO_CONTROLLER, PLANNER_CONTROLLER)
#         self._call(self._start_client,   'start_servo')
#         self._call(self._reset_client,   'reset_servo_status')
#         self._call(self._unpause_client, 'unpause_servo')
#         self._servo_active = True
#         print('\n[SERVO ON]  Hold keys to move. SPACE to switch back to planner.\n')

#     def deactivate_servo(self):
#         self._zero_velocity()
#         self._call(self._stop_client, 'stop_servo')
#         self._switch_controllers(PLANNER_CONTROLLER, SERVO_CONTROLLER)
#         self._servo_active = False
#         print('\n[PLANNER ON] RViz planning free. SPACE to re-enable servo.\n')

#     def toggle_servo(self):
#         if self._servo_active:
#             self.deactivate_servo()
#         else:
#             self.activate_servo()

#     # ── Velocity ──────────────────────────────────────────────────────────────

#     def _set_velocity(self, lx, ly, lz, rx, ry, rz, frame=None):
#         with self._lock:
#             self._vx        = lx * LINEAR_SPEED
#             self._vy        = ly * LINEAR_SPEED
#             self._vz        = lz * LINEAR_SPEED
#             self._rx        = rx * ANGULAR_SPEED
#             self._ry        = ry * ANGULAR_SPEED
#             self._rz        = rz * ANGULAR_SPEED
#             self._cmd_frame = frame if frame else COMMAND_FRAME

#     def _zero_velocity(self):
#         with self._lock:
#             self._vx = self._vy = self._vz = 0.0
#             self._rx = self._ry = self._rz = 0.0
#             self._cmd_frame = COMMAND_FRAME

#     def _publish(self):
#         if not self._servo_active:
#             return
#         with self._lock:
#             vx, vy, vz = self._vx, self._vy, self._vz
#             rx, ry, rz = self._rx, self._ry, self._rz
#             frame      = self._cmd_frame
#         msg                 = TwistStamped()
#         msg.header.stamp    = self.get_clock().now().to_msg()
#         msg.header.frame_id = frame
#         msg.twist.linear.x  = vx
#         msg.twist.linear.y  = vy
#         msg.twist.linear.z  = vz
#         msg.twist.angular.x = rx
#         msg.twist.angular.y = ry
#         msg.twist.angular.z = rz
#         self._pub.publish(msg)

#     # ── Gripper ───────────────────────────────────────────────────────────────

#     def _set_gripper(self, width):
#         width = max(GRIPPER_CLOSE, min(GRIPPER_OPEN, width))
#         self._finger_width = width
#         msg      = Float64MultiArray()
#         msg.data = [width]
#         self._gripper_pub.publish(msg)

#     def gripper_open(self):         self._set_gripper(GRIPPER_OPEN)
#     def gripper_close(self):        self._set_gripper(GRIPPER_CLOSE)
#     def gripper_increment(self, d): self._set_gripper(self._finger_width + d)

#     def _gripper_loop(self):
#         interval = 0.05
#         step     = GRIPPER_SPEED * interval
#         while self._running:
#             time.sleep(interval)
#             if self._gripper_vel == 0.0:
#                 continue
#             if time.time() - self._gripper_key_time > KEY_TIMEOUT:
#                 self._gripper_vel = 0.0
#                 continue
#             self._set_gripper(self._finger_width + self._gripper_vel * step)

#     # ── Emergency stop ────────────────────────────────────────────────────────

#     def emergency_stop(self):
#         self._estopped = True
#         self._zero_velocity()
#         self._call(self._stop_client, 'stop_servo')
#         print('\n' + '!'*54)
#         print('!  EMERGENCY STOP — all motion halted               !')
#         print(f'!  Press [{ESTOP_KEY.upper()}] again to clear                    !')
#         print('!'*54 + '\n')

#     def clear_estop(self):
#         self._estopped = False
#         if self._servo_active:
#             self._call(self._start_client,   'start_servo')
#             self._call(self._reset_client,   'reset_servo_status')
#             self._call(self._unpause_client, 'unpause_servo')
#         print('\n[E-STOP CLEARED]  Servo restored.\n')

#     # ── Joint state ───────────────────────────────────────────────────────────

#     def _js_callback(self, msg):
#         with self._js_lock:
#             self._joint_state = msg

#     # ── TF position ───────────────────────────────────────────────────────────

#     def _get_ee_position(self):
#         try:
#             import rclpy.time as rt
#             t  = self._tf_buffer.lookup_transform('base_link', 'tool0', rt.Time())
#             tr = t.transform.translation
#             return (tr.x, tr.y, tr.z)
#         except Exception:
#             return None

#     def _get_ee_orientation(self):
#         try:
#             import rclpy.time as rt
#             t = self._tf_buffer.lookup_transform('base_link', 'tool0', rt.Time())
#             return t.transform.rotation
#         except Exception:
#             return None

#     # ── Wrist snap ────────────────────────────────────────────────────────────

#     def snap_wrist(self, wrist_1_delta, label):
#         if self._snapping:
#             return
#         self._snapping = True
#         try:
#             with self._js_lock:
#                 js = self._joint_state
#             if js is None:
#                 return

#             positions = []
#             for name in SNAP_JOINT_NAMES:
#                 if name not in js.name:
#                     return
#                 val = js.position[js.name.index(name)]
#                 positions.append(val + wrist_1_delta if name == 'wrist_1_joint' else val)

#             print(f'\n[WRIST → {label.upper()}]')
#             self._zero_velocity()
#             self._call(self._stop_client, 'stop_servo')
#             self._switch_controllers(PLANNER_CONTROLLER, SERVO_CONTROLLER)

#             traj               = JointTrajectory()
#             traj.header.stamp  = self.get_clock().now().to_msg()
#             traj.joint_names   = SNAP_JOINT_NAMES
#             pt                 = JointTrajectoryPoint()
#             pt.positions       = positions
#             pt.velocities      = [0.0] * len(SNAP_JOINT_NAMES)
#             pt.time_from_start = Duration(sec=int(WRIST_MOVE_TIME),
#                                           nanosec=int((WRIST_MOVE_TIME % 1) * 1e9))
#             traj.points        = [pt]
#             self._snap_pub.publish(traj)
#             time.sleep(WRIST_MOVE_TIME + 0.3)

#             self._switch_controllers(SERVO_CONTROLLER, PLANNER_CONTROLLER)
#             self._call(self._start_client,   'start_servo')
#             self._call(self._reset_client,   'reset_servo_status')
#             self._call(self._unpause_client, 'unpause_servo')
#             print(f'[WRIST → {label.upper()}] done.\n')
#         finally:
#             self._snapping = False

#     # ── Joystick spring-back ──────────────────────────────────────────────────

#     def _joystick_spring_watchdog(self):
#         """
#         Watches for joystick key release, then triggers spring-back
#         to the stored neutral position via a Cartesian trajectory.
#         """
#         while self._running:
#             time.sleep(KEY_TIMEOUT / 2)

#             if not self._joystick_mode or not self._servo_active:
#                 continue
#             if self._joystick_busy or self._snapping or self._estopped:
#                 continue
#             if self._joystick_neutral is None:
#                 continue

#             # Key still held — don't spring yet
#             if time.time() - self._last_key_time < KEY_TIMEOUT:
#                 continue

#             # Check distance from neutral
#             current = self._get_ee_position()
#             if current is None:
#                 continue

#             nx, ny, nz = self._joystick_neutral
#             dist = ((nx-current[0])**2 + (ny-current[1])**2 + (nz-current[2])**2) ** 0.5

#             if dist < MIN_RETURN_DIST:
#                 self._joystick_neutral = None
#                 continue

#             # Trigger spring-back
#             self._joystick_busy = True
#             threading.Thread(
#                 target=self._spring_to_neutral,
#                 args=(nx, ny, nz, current),
#                 daemon=True).start()

#     def _spring_to_neutral(self, nx, ny, nz, start_pos):
#         """
#         Return the arm to the joystick neutral position using a
#         Cartesian trajectory — same controller switch pattern as wrist snap.
#         """
#         try:
#             self.get_logger().info(
#                 f'Joystick spring-back → ({nx:.3f}, {ny:.3f}, {nz:.3f})')

#             # Stop servo, switch to planner
#             self._zero_velocity()
#             self._call(self._stop_client, 'stop_servo')
#             self._switch_controllers(PLANNER_CONTROLLER, SERVO_CONTROLLER)

#             # Get current orientation to keep it unchanged during return
#             ori = self._get_ee_orientation()

#             def make_pose(x, y, z):
#                 p               = Pose()
#                 p.position.x    = x
#                 p.position.y    = y
#                 p.position.z    = z
#                 if ori:
#                     p.orientation = ori
#                 else:
#                     p.orientation.w = 1.0
#                 return p

#             # Plan Cartesian path from current position back to neutral
#             req                  = GetCartesianPath.Request()
#             req.header.frame_id  = 'base_link'
#             req.group_name       = GROUP_NAME
#             req.waypoints        = [
#                 make_pose(*start_pos),
#                 make_pose(nx, ny, nz),
#             ]
#             req.max_step         = 0.005   # 5mm steps — smooth return
#             req.jump_threshold   = 0.0
#             req.avoid_collisions = True

#             if not self._cartesian_client.wait_for_service(timeout_sec=3.0):
#                 self.get_logger().warn('Cartesian path service not available.')
#                 return

#             response = self._wait(self._cartesian_client.call_async(req), timeout=5.0)

#             if response is None or response.fraction < 0.5:
#                 self.get_logger().warn(
#                     f'Spring-back path only {response.fraction*100:.0f}% complete.')
#                 return

#             # Execute the trajectory
#             if not self._execute_client.wait_for_server(timeout_sec=3.0):
#                 self.get_logger().warn('Execute trajectory server not available.')
#                 return

#             goal            = ExecuteTrajectory.Goal()
#             goal.trajectory = response.solution
#             handle = self._wait(
#                 self._execute_client.send_goal_async(goal), timeout=5.0)

#             if handle and handle.accepted:
#                 self._wait(handle.get_result_async(),
#                            timeout=JOYSTICK_RETURN_TIME + 1.0)

#             self.get_logger().info('Spring-back complete.')

#         except Exception as e:
#             self.get_logger().error(f'Spring-back error: {e}')

#         finally:
#             # Always restart servo regardless of what happened
#             self._switch_controllers(SERVO_CONTROLLER, PLANNER_CONTROLLER)
#             self._call(self._start_client,   'start_servo')
#             self._call(self._reset_client,   'reset_servo_status')
#             self._call(self._unpause_client, 'unpause_servo')
#             self._joystick_neutral = None
#             self._joystick_busy    = False

#     # ── Key watchdog ──────────────────────────────────────────────────────────

#     def _start_watchdog(self):
#         while self._running:
#             time.sleep(KEY_TIMEOUT / 2)
#             if self._estopped:
#                 self._zero_velocity()
#                 continue
#             if not self._servo_active or self._snapping or self._joystick_busy:
#                 continue
#             if time.time() - self._last_key_time > KEY_TIMEOUT:
#                 self._zero_velocity()

#     # ── Keyboard ──────────────────────────────────────────────────────────────

#     def start_keyboard(self):
#         threading.Thread(target=self._keyboard_loop,           daemon=True).start()
#         threading.Thread(target=self._start_watchdog,          daemon=True).start()
#         threading.Thread(target=self._gripper_loop,            daemon=True).start()
#         threading.Thread(target=self._joystick_spring_watchdog,daemon=True).start()

#     def _keyboard_loop(self):
#         fd  = sys.stdin.fileno()
#         old = termios.tcgetattr(fd)
#         tty.setcbreak(fd)
#         self._print_controls()
#         try:
#             while self._running:
#                 ch = sys.stdin.read(1)

#                 # Always handled
#                 if ch == ESTOP_KEY:
#                     if self._estopped:
#                         threading.Thread(target=self.clear_estop, daemon=True).start()
#                     else:
#                         threading.Thread(target=self.emergency_stop, daemon=True).start()
#                     continue

#                 if ch == QUIT_KEY:
#                     self._running = False
#                     if self._servo_active:
#                         self.deactivate_servo()
#                     rclpy.shutdown()
#                     break

#                 if ch == TOGGLE_KEY:
#                     threading.Thread(target=self.toggle_servo, daemon=True).start()
#                     continue

#                 # Gripper — always active
#                 if ch == 'e':
#                     self._gripper_vel      =  1.0
#                     self._gripper_key_time = time.time()
#                     continue
#                 if ch == 'c':
#                     self._gripper_vel      = -1.0
#                     self._gripper_key_time = time.time()
#                     continue
#                 if ch == 'g':
#                     self._gripper_vel = 0.0
#                     self.gripper_open()
#                     continue
#                 if ch == 'b':
#                     self._gripper_vel = 0.0
#                     self.gripper_close()
#                     continue

#                 # E-stop blocks everything below
#                 if self._estopped:
#                     print(f'E-stop active — press [{ESTOP_KEY.upper()}] to clear.')
#                     continue

#                 # Servo required below
#                 if not self._servo_active:
#                     print('Servo off — press SPACE to enable.')
#                     continue

#                 if self._snapping or self._joystick_busy:
#                     continue

#                 # Joystick mode toggle
#                 if ch == JOYSTICK_MODE_KEY:
#                     self._joystick_mode    = not self._joystick_mode
#                     self._joystick_neutral = None
#                     self._zero_velocity()
#                     if self._joystick_mode:
#                         print('\n[JOYSTICK MODE ON]  w/s=up/down  a/d=left/right')
#                         print('                    Arm springs back to neutral on key release.')
#                         print(f'                    Press [{JOYSTICK_MODE_KEY.upper()}] to exit joystick mode.\n')
#                     else:
#                         print('\n[JOYSTICK MODE OFF]  Normal EE control restored.\n')
#                     continue

#                 # Wrist snap
#                 if ch == WRIST_FLOOR_KEY:
#                     threading.Thread(target=self.snap_wrist,
#                                      args=(WRIST_ROTATE_UP, 'floor'),
#                                      daemon=True).start()
#                     continue
#                 if ch == WRIST_WALL_KEY:
#                     threading.Thread(target=self.snap_wrist,
#                                      args=(WRIST_ROTATE_DOWN, 'wall'),
#                                      daemon=True).start()
#                     continue

#                 # Joystick mode — board-plane movement with spring-back
#                 if self._joystick_mode:
#                     if ch in JOYSTICK_BINDINGS:
#                         # Store neutral on first keypress
#                         if self._joystick_neutral is None:
#                             self._joystick_neutral = self._get_ee_position()
#                             if self._joystick_neutral:
#                                 self.get_logger().info(
#                                     f'Neutral stored: {self._joystick_neutral}')
#                         self._last_key_time = time.time()
#                         self._set_velocity(*JOYSTICK_BINDINGS[ch], frame='base_link')
#                     else:
#                         self._zero_velocity()
#                     continue

#                 # Base rotation
#                 if ch in BASE_ROT_KEYS:
#                     self._last_key_time = time.time()
#                     self._set_velocity(*BASE_ROT_KEYS[ch], frame='base_link')
#                     continue

#                 # Normal arm + wrist
#                 if ch in KEY_BINDINGS:
#                     self._last_key_time = time.time()
#                     self._set_velocity(*KEY_BINDINGS[ch])
#                 else:
#                     self._zero_velocity()

#         finally:
#             self._zero_velocity()
#             termios.tcsetattr(fd, termios.TCSADRAIN, old)

#     @staticmethod
#     def _print_controls():
#         print('\n── UR3e + RG2 keyboard control ───────────────────────')
#         print('  Arm (servo — HOLD to move):')
#         print('    w/s=fwd/back  a/d=left/right  r/f=up/down  (EE frame)')
#         print('    i/k=wrist     u/o=base rotation')
#         print()
#         print('  Joystick mode (J to toggle):')
#         print('    w/s=up/down   a/d=left/right  (world frame)')
#         print('    Springs back to neutral on key release')
#         print()
#         print('  Wrist snap:')
#         print('    z=floor (-90°)   x=wall (+90°)')
#         print()
#         print('  Gripper (always):')
#         print('    e/c=open/close (hold)   g/b=fully open/close')
#         print()
#         print('  SPACE=servo toggle  P=e-stop  q=quit')
#         print('──────────────────────────────────────────────────────\n')


# def main(args=None):
#     rclpy.init(args=args)
#     node = KeyboardServoNode()

#     executor = MultiThreadedExecutor()
#     executor.add_node(node)

#     spin_thread = threading.Thread(target=executor.spin, daemon=True)
#     spin_thread.start()

#     print('Switching gripper controller...')
#     node._switch_controllers(GRIPPER_ACTIVATE, GRIPPER_DEACTIVATE)
#     print('Gripper controller ready.')
#     print('Starting in planner mode. Press SPACE for servo.')
#     node.start_keyboard()

#     try:
#         spin_thread.join()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if node._servo_active:
#             node.deactivate_servo()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

import sys
import tty
import termios
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import TwistStamped, Pose
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener

# ── Tuning ────────────────────────────────────────────────────────────────────
LINEAR_SPEED  = 0.05
ANGULAR_SPEED = 0.3
PUBLISH_RATE  = 30.0
KEY_TIMEOUT   = 0.08
COMMAND_FRAME = 'tool0'

# ── Controller names ──────────────────────────────────────────────────────────
SERVO_CONTROLLER   = 'forward_velocity_controller'
PLANNER_CONTROLLER = 'scaled_joint_trajectory_controller'
SWITCH_CONTROLLER_SERVICE = '/controller_manager/switch_controller'
GROUP_NAME = 'ur_onrobot_manipulator'

# ── Servo services ────────────────────────────────────────────────────────────
SERVO_TOPIC         = '/servo_node/delta_twist_cmds'
SERVO_START_SERVICE = '/servo_node/start_servo'
SERVO_STOP_SERVICE  = '/servo_node/stop_servo'
SERVO_UNPAUSE       = '/servo_node/unpause_servo'
SERVO_RESET         = '/servo_node/reset_servo_status'

# ── Gripper ───────────────────────────────────────────────────────────────────
GRIPPER_TOPIC      = '/finger_width_controller/commands'
GRIPPER_OPEN       = 0.10
GRIPPER_CLOSE      = 0.0
GRIPPER_STEP       = 0.01
GRIPPER_SPEED      = 0.04
GRIPPER_ACTIVATE   = 'finger_width_controller'
GRIPPER_DEACTIVATE = 'finger_width_trajectory_controller'

# ── Wrist snap ────────────────────────────────────────────────────────────────
WRIST_ROTATE_UP   = -1.5708
WRIST_ROTATE_DOWN =  1.5708
WRIST_MOVE_TIME   = 1.5

WRIST_FLOOR_KEY = 'z'
WRIST_WALL_KEY  = 'x'

SNAP_JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

# ── Joint limits for wrist snap ──────────────────────────────────────────────
# Soft limits slightly inside the UR3e hardware limits to avoid singularities.
# wrist_1 is the joint we snap — clamp its target to this range.
WRIST_1_MIN = -3.9   # rad — ~-224 degrees
WRIST_1_MAX =  3.9   # rad — ~+224 degrees

# Singularity proximity — if wrist_2 is near 0 or ±pi the arm is near
# a wrist singularity. Warn and skip snap if too close.
WRIST_SINGULARITY_THRESHOLD = 0.15   # radians

# ── Joystick mode — fixed joint positions ────────────────────────────────────
# 5 pre-recorded joint configurations for the physical joystick puzzle.
# Drive the arm to each position and press the calibration key (see below)
# to record, or manually paste values from:
#   ros2 topic echo /joint_states --once
#
# Joint order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
#
# Set to None to use current position (will prompt to calibrate on first use)
JOYSTICK_MODE_KEY  = 'j'
JOYSTICK_CALIB_KEY = 'J'   # enter calibration mode to record positions
JOYSTICK_MOVE_TIME = 0.6   # seconds per joystick movement trajectory

JOYSTICK_POSITIONS = {
    'centre': None,   # neutral/rest position — press J+c to record
    'up':     None,   # w key — press J+w to record
    'down':   None,   # s key — press J+s to record
    'left':   None,   # a key — press J+a to record
    'right':  None,   # d key — press J+d to record
}

JOYSTICK_KEY_MAP = {
    'w': 'up',
    's': 'down',
    'a': 'left',
    'd': 'right',
}

# ── Key bindings (normal EE mode) ─────────────────────────────────────────────
KEY_BINDINGS = {
    'w': ( 0,  -1,  0,  0,  0,  0),
    's': ( 0,  1,  0,  0,  0,  0),
    'a': ( -1, 0,  0,  0,  0,  0),
    'd': ( 1,  0,  0,  0,  0,  0),
    'r': ( 0,  0,  -1,  0,  0,  0),
    'f': ( 0,  0, 1,  0,  0,  0),
    'i': ( 0,  0,  0,  0,  0,  1),
    'k': ( 0,  0,  0,  0,  0, -1),
}

BASE_ROT_KEYS = {
    'u': ( 0,  0,  0,  0,  0,  1),
    'o': ( 0,  0,  0,  0,  0, -1),
}

TOGGLE_KEY = ' '
ESTOP_KEY  = 'p'
QUIT_KEY   = 'q'


class KeyboardServoNode(Node):
    def __init__(self):
        super().__init__('keyboard_servo_node')

        self._cbg = ReentrantCallbackGroup()

        # TF
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Publishers
        self._pub         = self.create_publisher(TwistStamped, SERVO_TOPIC, 10)
        self._gripper_pub = self.create_publisher(Float64MultiArray, GRIPPER_TOPIC, 10)
        self._snap_pub    = self.create_publisher(
            JointTrajectory, f'/{PLANNER_CONTROLLER}/joint_trajectory', 10)

        # Servo services
        self._start_client   = self.create_client(Trigger, SERVO_START_SERVICE, callback_group=self._cbg)
        self._stop_client    = self.create_client(Trigger, SERVO_STOP_SERVICE,  callback_group=self._cbg)
        self._unpause_client = self.create_client(Trigger, SERVO_UNPAUSE,       callback_group=self._cbg)
        self._reset_client   = self.create_client(Trigger, SERVO_RESET,         callback_group=self._cbg)
        self._switch_client  = self.create_client(SwitchController, SWITCH_CONTROLLER_SERVICE, callback_group=self._cbg)

        # Cartesian path + execute for spring-back
        self._cartesian_client = self.create_client(
            GetCartesianPath, 'compute_cartesian_path',
            callback_group=self._cbg)
        self._execute_client   = ActionClient(
            self, ExecuteTrajectory, 'execute_trajectory',
            callback_group=self._cbg)

        # Joint state
        self._joint_state = None
        self._js_lock     = threading.Lock()
        self.create_subscription(JointState, '/joint_states',
                                 self._js_callback, 10, callback_group=self._cbg)

        self._lock      = threading.Lock()
        self._vx = self._vy = self._vz = 0.0
        self._rx = self._ry = self._rz = 0.0
        self._cmd_frame = COMMAND_FRAME

        self._running          = True
        self._servo_active     = False
        self._snapping         = False
        self._estopped         = False
        self._joystick_mode    = False
        self._joystick_busy    = False
        self._joystick_calib   = False   # True when in calibration mode
        self._joystick_pos     = dict(JOYSTICK_POSITIONS)  # live copy
        self._last_key_time    = 0.0
        self._finger_width     = GRIPPER_OPEN
        self._gripper_vel      = 0.0
        self._gripper_key_time = 0.0

        self._timer = self.create_timer(
            1.0 / PUBLISH_RATE, self._publish, callback_group=self._cbg)

    # ── Async wait ────────────────────────────────────────────────────────────

    def _wait(self, future, timeout=5.0):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        done.wait(timeout=timeout)
        return future.result()

    def _call(self, client, label, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f'{label} not available.')
            return False
        result = self._wait(client.call_async(Trigger.Request()), timeout)
        if result:
            self.get_logger().info(f'{label}: {result.message}')
        return result is not None

    # ── Controller switching ──────────────────────────────────────────────────

    def _switch_controllers(self, activate, deactivate):
        if not self._switch_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('switch_controller not available.')
            return False
        req                        = SwitchController.Request()
        req.activate_controllers   = [activate]
        req.deactivate_controllers = [deactivate]
        req.strictness             = SwitchController.Request.BEST_EFFORT
        req.activate_asap          = True
        req.timeout                = rclpy.duration.Duration(seconds=2.0).to_msg()
        result = self._wait(self._switch_client.call_async(req), timeout=5.0)
        if result and result.ok:
            self.get_logger().info(f'+{activate}  -{deactivate}')
            return True
        return False

    # ── Servo lifecycle ───────────────────────────────────────────────────────

    def activate_servo(self):
        self._switch_controllers(SERVO_CONTROLLER, PLANNER_CONTROLLER)
        self._call(self._start_client,   'start_servo')
        self._call(self._reset_client,   'reset_servo_status')
        self._call(self._unpause_client, 'unpause_servo')
        self._servo_active = True
        print('\n[SERVO ON]  Hold keys to move. SPACE to switch back to planner.\n')

    def deactivate_servo(self):
        self._zero_velocity()
        self._call(self._stop_client, 'stop_servo')
        self._switch_controllers(PLANNER_CONTROLLER, SERVO_CONTROLLER)
        self._servo_active = False
        print('\n[PLANNER ON] RViz planning free. SPACE to re-enable servo.\n')

    def toggle_servo(self):
        if self._servo_active:
            self.deactivate_servo()
        else:
            self.activate_servo()

    # ── Velocity ──────────────────────────────────────────────────────────────

    def _set_velocity(self, lx, ly, lz, rx, ry, rz, frame=None):
        with self._lock:
            self._vx        = lx * LINEAR_SPEED
            self._vy        = ly * LINEAR_SPEED
            self._vz        = lz * LINEAR_SPEED
            self._rx        = rx * ANGULAR_SPEED
            self._ry        = ry * ANGULAR_SPEED
            self._rz        = rz * ANGULAR_SPEED
            self._cmd_frame = frame if frame else COMMAND_FRAME

    def _zero_velocity(self):
        with self._lock:
            self._vx = self._vy = self._vz = 0.0
            self._rx = self._ry = self._rz = 0.0
            self._cmd_frame = COMMAND_FRAME

    def _publish(self):
        if not self._servo_active:
            return
        with self._lock:
            vx, vy, vz = self._vx, self._vy, self._vz
            rx, ry, rz = self._rx, self._ry, self._rz
            frame      = self._cmd_frame
        msg                 = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.twist.linear.x  = vx
        msg.twist.linear.y  = vy
        msg.twist.linear.z  = vz
        msg.twist.angular.x = rx
        msg.twist.angular.y = ry
        msg.twist.angular.z = rz
        self._pub.publish(msg)

    # ── Gripper ───────────────────────────────────────────────────────────────

    def _set_gripper(self, width):
        width = max(GRIPPER_CLOSE, min(GRIPPER_OPEN, width))
        self._finger_width = width
        msg      = Float64MultiArray()
        msg.data = [width]
        self._gripper_pub.publish(msg)

    def gripper_open(self):         self._set_gripper(GRIPPER_OPEN)
    def gripper_close(self):        self._set_gripper(GRIPPER_CLOSE)
    def gripper_increment(self, d): self._set_gripper(self._finger_width + d)

    def _gripper_loop(self):
        interval = 0.05
        step     = GRIPPER_SPEED * interval
        while self._running:
            time.sleep(interval)
            if self._gripper_vel == 0.0:
                continue
            if time.time() - self._gripper_key_time > KEY_TIMEOUT:
                self._gripper_vel = 0.0
                continue
            self._set_gripper(self._finger_width + self._gripper_vel * step)

    # ── Emergency stop ────────────────────────────────────────────────────────

    def emergency_stop(self):
        self._estopped = True
        self._zero_velocity()
        self._call(self._stop_client, 'stop_servo')
        print('\n' + '!'*54)
        print('!  EMERGENCY STOP — all motion halted               !')
        print(f'!  Press [{ESTOP_KEY.upper()}] again to clear                    !')
        print('!'*54 + '\n')

    def clear_estop(self):
        self._estopped = False
        if self._servo_active:
            self._call(self._start_client,   'start_servo')
            self._call(self._reset_client,   'reset_servo_status')
            self._call(self._unpause_client, 'unpause_servo')
        print('\n[E-STOP CLEARED]  Servo restored.\n')

    # ── Joint state ───────────────────────────────────────────────────────────

    def _js_callback(self, msg):
        with self._js_lock:
            self._joint_state = msg

    # ── TF position ───────────────────────────────────────────────────────────

    def _get_ee_position(self):
        try:
            import rclpy.time as rt
            t  = self._tf_buffer.lookup_transform('base_link', 'tool0', rt.Time())
            tr = t.transform.translation
            return (tr.x, tr.y, tr.z)
        except Exception:
            return None

    def _get_ee_orientation(self):
        try:
            import rclpy.time as rt
            t = self._tf_buffer.lookup_transform('base_link', 'tool0', rt.Time())
            return t.transform.rotation
        except Exception:
            return None

    # ── Wrist snap (collision-aware via MoveGroup) ────────────────────────────────

    def snap_wrist(self, wrist_1_delta, label):
        """
        Snap wrist_1_joint via MoveGroup so collision checking and
        joint limits are enforced. Falls back to raw trajectory if
        MoveGroup planning fails.
        Checks for singularity proximity before attempting the snap.
        """
        if self._snapping:
            return
        self._snapping = True
        try:
            with self._js_lock:
                js = self._joint_state
            if js is None:
                self.get_logger().warn('No joint state.')
                return

            # ── Singularity check ──────────────────────────────────────────
            if 'wrist_2_joint' in js.name:
                w2 = js.position[js.name.index('wrist_2_joint')]
                import math
                # wrist singularity at wrist_2 = 0 or ±pi
                for singular in [0.0, math.pi, -math.pi]:
                    if abs(w2 - singular) < WRIST_SINGULARITY_THRESHOLD:
                        print(f'\n[WRIST SNAP] Near singularity '
                              f'(wrist_2={w2:.3f}) — move arm first.\n')
                        return

            # ── Build target joint positions ───────────────────────────────
            positions = {}
            for name in SNAP_JOINT_NAMES:
                if name not in js.name:
                    self.get_logger().warn(f'{name} missing.')
                    return
                val = js.position[js.name.index(name)]
                if name == 'wrist_1_joint':
                    target = val + wrist_1_delta
                    # Clamp to soft limits
                    target = max(WRIST_1_MIN, min(WRIST_1_MAX, target))
                    self.get_logger().info(
                        f'wrist_1: {val:.3f} → {target:.3f} rad')
                    positions[name] = target
                else:
                    positions[name] = val

            print(f'\n[WRIST → {label.upper()}] planning with collision check...')
            self._zero_velocity()
            self._call(self._stop_client, 'stop_servo')
            self._switch_controllers(PLANNER_CONTROLLER, SERVO_CONTROLLER)

            # ── Try MoveGroup with joint constraints ───────────────────────
            goal_c = Constraints()
            for name, pos in positions.items():
                jc                 = JointConstraint()
                jc.joint_name      = name
                jc.position        = pos
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight          = 1.0
                goal_c.joint_constraints.append(jc)

            req                                 = MotionPlanRequest()
            req.group_name                      = GROUP_NAME
            req.num_planning_attempts           = 5
            req.allowed_planning_time           = 3.0
            req.max_velocity_scaling_factor     = 0.3
            req.max_acceleration_scaling_factor = 0.2
            req.goal_constraints.append(goal_c)

            from moveit_msgs.msg import WorkspaceParameters
            ws = WorkspaceParameters()
            ws.header.frame_id = 'base_link'
            ws.header.stamp    = self.get_clock().now().to_msg()
            ws.min_corner.x = ws.min_corner.y = ws.min_corner.z = -2.0
            ws.max_corner.x = ws.max_corner.y = ws.max_corner.z =  2.0
            req.workspace_parameters = ws

            from moveit_msgs.action import MoveGroup as MG
            mg_client = ActionClient(self, MG, 'move_action',
                                     callback_group=self._cbg)

            planned = False
            if mg_client.wait_for_server(timeout_sec=3.0):
                goal         = MG.Goal()
                goal.request = req
                handle = self._wait(mg_client.send_goal_async(goal))
                if handle and handle.accepted:
                    result = self._wait(handle.get_result_async(),
                                        timeout=8.0)
                    if (result and
                            result.result.error_code.val ==
                            MoveItErrorCodes.SUCCESS):
                        planned = True
                        self.get_logger().info(
                            f'Wrist snap planned with collision check.')
                    else:
                        code = result.result.error_code.val if result else 'none'
                        self.get_logger().warn(
                            f'MoveGroup failed (code {code}) — '
                            'falling back to raw trajectory.')

            # ── Fallback: raw trajectory (no collision check) ──────────────
            if not planned:
                self.get_logger().warn(
                    'Using raw trajectory fallback — no collision checking.')
                traj               = JointTrajectory()
                traj.header.stamp  = self.get_clock().now().to_msg()
                traj.joint_names   = SNAP_JOINT_NAMES
                pt                 = JointTrajectoryPoint()
                pt.positions       = [positions[n] for n in SNAP_JOINT_NAMES]
                pt.velocities      = [0.0] * len(SNAP_JOINT_NAMES)
                pt.time_from_start = Duration(
                    sec=int(WRIST_MOVE_TIME),
                    nanosec=int((WRIST_MOVE_TIME % 1) * 1e9))
                traj.points        = [pt]
                self._snap_pub.publish(traj)
                time.sleep(WRIST_MOVE_TIME + 0.3)

            self._switch_controllers(SERVO_CONTROLLER, PLANNER_CONTROLLER)
            self._call(self._start_client,   'start_servo')
            self._call(self._reset_client,   'reset_servo_status')
            self._call(self._unpause_client, 'unpause_servo')
            print(f'[WRIST → {label.upper()}] done.\n')

        finally:
            self._snapping = False

    # ── Joystick spring-back ──────────────────────────────────────────────────

    def _joystick_spring_watchdog(self):
        """
        Watches for joystick key release, then triggers spring-back
        to the stored neutral position via a Cartesian trajectory.
        """
        while self._running:
            time.sleep(KEY_TIMEOUT / 2)

            if not self._joystick_mode or not self._servo_active:
                continue
            if self._joystick_busy or self._snapping or self._estopped:
                continue
            if self._joystick_neutral is None:
                continue

            # Key still held — don't spring yet
            if time.time() - self._last_key_time < KEY_TIMEOUT:
                continue

            # Check distance from neutral
            current = self._get_ee_position()
            if current is None:
                continue

            nx, ny, nz = self._joystick_neutral
            dist = ((nx-current[0])**2 + (ny-current[1])**2 + (nz-current[2])**2) ** 0.5

            if dist < MIN_RETURN_DIST:
                self._joystick_neutral = None
                continue

            # Trigger spring-back
            self._joystick_busy = True
            threading.Thread(
                target=self._spring_to_neutral,
                args=(nx, ny, nz, current),
                daemon=True).start()

    def _spring_to_neutral(self, nx, ny, nz, start_pos):
        """
        Return the arm to the joystick neutral position using a
        Cartesian trajectory — same controller switch pattern as wrist snap.
        """
        try:
            self.get_logger().info(
                f'Joystick spring-back → ({nx:.3f}, {ny:.3f}, {nz:.3f})')

            # Stop servo, switch to planner
            self._zero_velocity()
            self._call(self._stop_client, 'stop_servo')
            self._switch_controllers(PLANNER_CONTROLLER, SERVO_CONTROLLER)

            # Get current orientation to keep it unchanged during return
            ori = self._get_ee_orientation()

            def make_pose(x, y, z):
                p               = Pose()
                p.position.x    = x
                p.position.y    = y
                p.position.z    = z
                if ori:
                    p.orientation = ori
                else:
                    p.orientation.w = 1.0
                return p

            # Plan Cartesian path from current position back to neutral
            req                  = GetCartesianPath.Request()
            req.header.frame_id  = 'base_link'
            req.group_name       = GROUP_NAME
            req.waypoints        = [
                make_pose(*start_pos),
                make_pose(nx, ny, nz),
            ]
            req.max_step         = 0.005   # 5mm steps — smooth return
            req.jump_threshold   = 0.0
            req.avoid_collisions = True

            if not self._cartesian_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().warn('Cartesian path service not available.')
                return

            response = self._wait(self._cartesian_client.call_async(req), timeout=5.0)

            if response is None or response.fraction < 0.5:
                self.get_logger().warn(
                    f'Spring-back path only {response.fraction*100:.0f}% complete.')
                return

            # Execute the trajectory
            if not self._execute_client.wait_for_server(timeout_sec=3.0):
                self.get_logger().warn('Execute trajectory server not available.')
                return

            goal            = ExecuteTrajectory.Goal()
            goal.trajectory = response.solution
            handle = self._wait(
                self._execute_client.send_goal_async(goal), timeout=5.0)

            if handle and handle.accepted:
                self._wait(handle.get_result_async(),
                           timeout=JOYSTICK_RETURN_TIME + 1.0)

            self.get_logger().info('Spring-back complete.')

        except Exception as e:
            self.get_logger().error(f'Spring-back error: {e}')

        finally:
            # Always restart servo regardless of what happened
            self._switch_controllers(SERVO_CONTROLLER, PLANNER_CONTROLLER)
            self._call(self._start_client,   'start_servo')
            self._call(self._reset_client,   'reset_servo_status')
            self._call(self._unpause_client, 'unpause_servo')
            self._joystick_neutral = None
            self._joystick_busy    = False

    # ── Key watchdog ──────────────────────────────────────────────────────────

    def _start_watchdog(self):
        while self._running:
            time.sleep(KEY_TIMEOUT / 2)
            if self._estopped:
                self._zero_velocity()
                continue
            if not self._servo_active or self._snapping or self._joystick_busy or self._joystick_calib:
                continue
            if time.time() - self._last_key_time > KEY_TIMEOUT:
                self._zero_velocity()

    # ── Keyboard ──────────────────────────────────────────────────────────────

    def start_keyboard(self):
        threading.Thread(target=self._keyboard_loop,           daemon=True).start()
        threading.Thread(target=self._start_watchdog,          daemon=True).start()
        threading.Thread(target=self._gripper_loop,            daemon=True).start()
        threading.Thread(target=self._joystick_spring_watchdog,daemon=True).start()

    def _keyboard_loop(self):
        fd  = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        self._print_controls()
        try:
            while self._running:
                ch = sys.stdin.read(1)

                # Always handled
                if ch == ESTOP_KEY:
                    if self._estopped:
                        threading.Thread(target=self.clear_estop, daemon=True).start()
                    else:
                        threading.Thread(target=self.emergency_stop, daemon=True).start()
                    continue

                if ch == QUIT_KEY:
                    self._running = False
                    if self._servo_active:
                        self.deactivate_servo()
                    rclpy.shutdown()
                    break

                if ch == TOGGLE_KEY:
                    threading.Thread(target=self.toggle_servo, daemon=True).start()
                    continue

                # Gripper — always active
                if ch == 'e':
                    self._gripper_vel      =  1.0
                    self._gripper_key_time = time.time()
                    continue
                if ch == 'c':
                    self._gripper_vel      = -1.0
                    self._gripper_key_time = time.time()
                    continue
                if ch == 'g':
                    self._gripper_vel = 0.0
                    self.gripper_open()
                    continue
                if ch == 'b':
                    self._gripper_vel = 0.0
                    self.gripper_close()
                    continue

                # E-stop blocks everything below
                if self._estopped:
                    print(f'E-stop active — press [{ESTOP_KEY.upper()}] to clear.')
                    continue

                # Servo required below
                if not self._servo_active:
                    print('Servo off — press SPACE to enable.')
                    continue

                if self._snapping or self._joystick_busy:
                    continue

                # Joystick calibration mode (shift+J)
                if ch == JOYSTICK_CALIB_KEY:
                    self._joystick_calib = not self._joystick_calib
                    if self._joystick_calib:
                        print('\n[JOYSTICK CALIB]  Drive arm to each position then press:')
                        print('  c = record centre   w = record up')
                        print('  s = record down     a = record left   d = record right')
                        print(f'  [{JOYSTICK_CALIB_KEY}] = exit calibration mode\n')
                    else:
                        print('\n[JOYSTICK CALIB] Exited calibration mode.\n')
                    continue

                # Joystick calibration — record positions
                if self._joystick_calib:
                    calib_map = {'c': 'centre', 'w': 'up', 's': 'down',
                                 'a': 'left', 'd': 'right'}
                    if ch in calib_map:
                        self._record_joystick_position(calib_map[ch])
                    elif ch == JOYSTICK_CALIB_KEY:
                        self._joystick_calib = False
                        print('[JOYSTICK CALIB] Exited.')
                    continue

                # Joystick mode toggle
                if ch == JOYSTICK_MODE_KEY:
                    self._joystick_mode = not self._joystick_mode
                    self._zero_velocity()
                    if self._joystick_mode:
                        print('\n[JOYSTICK MODE ON]  w/s/a/d = push joystick direction')
                        print('                    Arm moves to pre-recorded position then returns to centre.')
                        print(f'                    Press [{JOYSTICK_MODE_KEY.upper()}] to exit.\n')
                        if any(v is None for v in self._joystick_pos.values()):
                            missing = [k for k,v in self._joystick_pos.items() if v is None]
                            print(f'  WARNING: uncalibrated positions: {missing}')
                            print(f'  Press [{JOYSTICK_CALIB_KEY}] to enter calibration mode.\n')
                    else:
                        print('\n[JOYSTICK MODE OFF]  Normal EE control restored.\n')
                    continue

                # Wrist snap
                if ch == WRIST_FLOOR_KEY:
                    threading.Thread(target=self.snap_wrist,
                                     args=(WRIST_ROTATE_UP, 'floor'),
                                     daemon=True).start()
                    continue
                if ch == WRIST_WALL_KEY:
                    threading.Thread(target=self.snap_wrist,
                                     args=(WRIST_ROTATE_DOWN, 'wall'),
                                     daemon=True).start()
                    continue

                # Joystick mode — fixed position moves
                if self._joystick_mode:
                    if ch in JOYSTICK_KEY_MAP and not self._joystick_busy:
                        direction = JOYSTICK_KEY_MAP[ch]
                        threading.Thread(
                            target=self._joystick_move,
                            args=(direction,),
                            daemon=True).start()
                    continue

                # Base rotation
                if ch in BASE_ROT_KEYS:
                    self._last_key_time = time.time()
                    self._set_velocity(*BASE_ROT_KEYS[ch], frame='base_link')
                    continue

                # Normal arm + wrist
                if ch in KEY_BINDINGS:
                    self._last_key_time = time.time()
                    self._set_velocity(*KEY_BINDINGS[ch])
                else:
                    self._zero_velocity()

        finally:
            self._zero_velocity()
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    @staticmethod
    def _print_controls():
        print('\n── UR3e + RG2 keyboard control ───────────────────────')
        print('  Arm (servo — HOLD to move):')
        print('    w/s=fwd/back  a/d=left/right  r/f=up/down  (EE frame)')
        print('    i/k=wrist     u/o=base rotation')
        print()
        print('  Joystick mode:')
        print('    j      →  toggle joystick mode')
        print('    J      →  calibration mode (record positions)')
        print('    w/s/a/d → push joystick (springs back to centre)')
        print()
        print('  Wrist snap:')
        print('    z=floor (-90°)   x=wall (+90°)')
        print()
        print('  Gripper (always):')
        print('    e/c=open/close (hold)   g/b=fully open/close')
        print()
        print('  SPACE=servo toggle  P=e-stop  q=quit')
        print('──────────────────────────────────────────────────────\n')


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardServoNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print('Switching gripper controller...')
    node._switch_controllers(GRIPPER_ACTIVATE, GRIPPER_DEACTIVATE)
    print('Gripper controller ready.')
    print('Starting in planner mode. Press SPACE for servo.')
    node.start_keyboard()

    try:
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        if node._servo_active:
            node.deactivate_servo()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
