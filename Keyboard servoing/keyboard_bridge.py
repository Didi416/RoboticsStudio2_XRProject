# import sys
# import tty
# import termios
# import threading
# import time
# import rclpy
# from rclpy.node import Node
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.callback_groups import ReentrantCallbackGroup
# from geometry_msgs.msg import TwistStamped
# from std_srvs.srv import Trigger

# # ── Tuning ────────────────────────────────────────────────────────────────────
# LINEAR_SPEED  = 0.05
# ANGULAR_SPEED = 0.3
# PUBLISH_RATE  = 30.0

# # How long after the last keypress before velocity is zeroed.
# # This simulates key-release — lower = more responsive stop,
# # higher = more forgiving if you type slowly.
# # 0.08s (80ms) feels natural for keyboard hold-to-move.
# KEY_TIMEOUT = 0.08   # seconds

# COMMAND_FRAME = 'base_link'

# SERVO_TOPIC         = '/servo_node/delta_twist_cmds'
# SERVO_START_SERVICE = '/servo_node/start_servo'
# SERVO_STOP_SERVICE  = '/servo_node/stop_servo'
# SERVO_UNPAUSE       = '/servo_node/unpause_servo'
# SERVO_RESET         = '/servo_node/reset_servo_status'

# KEY_BINDINGS = {
#     'w': ( 1,  0,  0,  0,  0,  0),
#     's': (-1,  0,  0,  0,  0,  0),
#     'a': ( 0,  1,  0,  0,  0,  0),
#     'd': ( 0, -1,  0,  0,  0,  0),
#     'r': ( 0,  0,  1,  0,  0,  0),
#     'f': ( 0,  0, -1,  0,  0,  0),
#     'i': ( 0,  0,  0,  1,  0,  0),
#     'k': ( 0,  0,  0, -1,  0,  0),
#     'j': ( 0,  0,  0,  0,  1,  0),
#     'l': ( 0,  0,  0,  0, -1,  0),
#     'u': ( 0,  0,  0,  0,  0,  1),
#     'o': ( 0,  0,  0,  0,  0, -1),
# }

# TOGGLE_KEY = ' '
# QUIT_KEY   = 'q'


# class KeyboardServoNode(Node):
#     def __init__(self):
#         super().__init__('keyboard_servo_node')

#         self._cbg = ReentrantCallbackGroup()

#         self._pub = self.create_publisher(TwistStamped, SERVO_TOPIC, 10)

#         self._start_client   = self.create_client(
#             Trigger, SERVO_START_SERVICE, callback_group=self._cbg)
#         self._stop_client    = self.create_client(
#             Trigger, SERVO_STOP_SERVICE,  callback_group=self._cbg)
#         self._unpause_client = self.create_client(
#             Trigger, SERVO_UNPAUSE,       callback_group=self._cbg)
#         self._reset_client   = self.create_client(
#             Trigger, SERVO_RESET,         callback_group=self._cbg)

#         self._lock   = threading.Lock()
#         self._vx = self._vy = self._vz = 0.0
#         self._rx = self._ry = self._rz = 0.0

#         self._running      = True
#         self._servo_active = False

#         # Timestamp of the last valid keypress — used by the watchdog
#         self._last_key_time = 0.0

#         self._timer = self.create_timer(
#             1.0 / PUBLISH_RATE, self._publish,
#             callback_group=self._cbg)

#     # ── Safe async wait ───────────────────────────────────────────────────────

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

#     # ── Servo lifecycle ───────────────────────────────────────────────────────

#     def activate_servo(self):
#         self.get_logger().info('Activating servo...')
#         self._call(self._start_client,   'start_servo')
#         self._call(self._reset_client,   'reset_servo_status')
#         self._call(self._unpause_client, 'unpause_servo')
#         self._servo_active = True
#         print('\n[SERVO ON]  Hold a key to move. SPACE to toggle off.')

#     def deactivate_servo(self):
#         self._zero_velocity()
#         self._call(self._stop_client, 'stop_servo')
#         self._servo_active = False
#         print('\n[SERVO OFF] RViz planner is free. SPACE to re-enable.\n')

#     def toggle_servo(self):
#         if self._servo_active:
#             self.deactivate_servo()
#         else:
#             self.activate_servo()

#     # ── Velocity ──────────────────────────────────────────────────────────────

#     def _set_velocity(self, lx, ly, lz, rx, ry, rz):
#         with self._lock:
#             self._vx = lx * LINEAR_SPEED
#             self._vy = ly * LINEAR_SPEED
#             self._vz = lz * LINEAR_SPEED
#             self._rx = rx * ANGULAR_SPEED
#             self._ry = ry * ANGULAR_SPEED
#             self._rz = rz * ANGULAR_SPEED

#     def _zero_velocity(self):
#         self._set_velocity(0, 0, 0, 0, 0, 0)

#     def _publish(self):
#         if not self._servo_active:
#             return
#         msg                 = TwistStamped()
#         msg.header.stamp    = self.get_clock().now().to_msg()
#         msg.header.frame_id = COMMAND_FRAME
#         with self._lock:
#             msg.twist.linear.x  = self._vx
#             msg.twist.linear.y  = self._vy
#             msg.twist.linear.z  = self._vz
#             msg.twist.angular.x = self._rx
#             msg.twist.angular.y = self._ry
#             msg.twist.angular.z = self._rz
#         self._pub.publish(msg)

#     # ── Key watchdog ──────────────────────────────────────────────────────────

#     def _start_watchdog(self):
#         """
#         Runs in a background thread. Zeros velocity if no keypress has
#         arrived within KEY_TIMEOUT seconds. This simulates key-release
#         since tty.setcbreak cannot detect when a key is actually released.

#         How it works: every valid keypress updates _last_key_time.
#         The watchdog checks every KEY_TIMEOUT/2 seconds — if the time
#         since the last keypress exceeds KEY_TIMEOUT, the arm stops.
#         When you hold a key, the terminal repeats it fast enough
#         (typically every ~30ms) to keep refreshing _last_key_time
#         before the watchdog fires.
#         """
#         while self._running:
#             time.sleep(KEY_TIMEOUT / 2)
#             if not self._servo_active:
#                 continue
#             elapsed = time.time() - self._last_key_time
#             if elapsed > KEY_TIMEOUT:
#                 self._zero_velocity()

#     # ── Keyboard ──────────────────────────────────────────────────────────────

#     def start_keyboard(self):
#         threading.Thread(target=self._keyboard_loop, daemon=True).start()
#         threading.Thread(target=self._start_watchdog, daemon=True).start()

#     def _keyboard_loop(self):
#         fd  = sys.stdin.fileno()
#         old = termios.tcgetattr(fd)
#         tty.setcbreak(fd)
#         self._print_controls()
#         try:
#             while self._running:
#                 ch = sys.stdin.read(1)

#                 if ch == QUIT_KEY:
#                     self._running = False
#                     self.deactivate_servo()
#                     rclpy.shutdown()
#                     break

#                 if ch == TOGGLE_KEY:
#                     threading.Thread(
#                         target=self.toggle_servo, daemon=True).start()
#                     continue

#                 if ch in KEY_BINDINGS:
#                     if not self._servo_active:
#                         print('Servo paused — press SPACE to enable.')
#                         continue
#                     # Update last keypress time so watchdog doesn't zero us
#                     self._last_key_time = time.time()
#                     self._set_velocity(*KEY_BINDINGS[ch])
#                 else:
#                     # Unrecognised key — stop immediately
#                     self._zero_velocity()

#         finally:
#             self._zero_velocity()
#             termios.tcsetattr(fd, termios.TCSADRAIN, old)

#     @staticmethod
#     def _print_controls():
#         print('\n── Cartesian velocity servo ──────────────────────────')
#         print('  Translation:  w/s=±x   a/d=±y   r/f=±z')
#         print('  Rotation:     i/k=roll  j/l=pitch  u/o=yaw')
#         print('  HOLD key to move — release to stop')
#         print()
#         print('  SPACE  →  toggle servo on/off')
#         print('  q      →  quit')
#         print('──────────────────────────────────────────────────────\n')


# def main(args=None):
#     rclpy.init(args=args)
#     node = KeyboardServoNode()

#     executor = MultiThreadedExecutor()
#     executor.add_node(node)

#     spin_thread = threading.Thread(target=executor.spin, daemon=True)
#     spin_thread.start()

#     node.activate_servo()
#     node.start_keyboard()

#     try:
#         spin_thread.join()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.deactivate_servo()
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
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController

# ── Tuning ────────────────────────────────────────────────────────────────────
LINEAR_SPEED  = 0.05
ANGULAR_SPEED = 0.3
PUBLISH_RATE  = 30.0
KEY_TIMEOUT   = 0.08   # seconds — hold-to-move watchdog threshold

COMMAND_FRAME = 'base_link'

# ── Controller names ──────────────────────────────────────────────────────────
# SERVO_CONTROLLER  — used when servo/keyboard is active
# PLANNER_CONTROLLER — used when RViz motion planner is active
SERVO_CONTROLLER   = 'forward_velocity_controller'
PLANNER_CONTROLLER = 'scaled_joint_trajectory_controller'

SWITCH_CONTROLLER_SERVICE = '/controller_manager/switch_controller'

# ── Servo services ────────────────────────────────────────────────────────────
SERVO_TOPIC         = '/servo_node/delta_twist_cmds'
SERVO_START_SERVICE = '/servo_node/start_servo'
SERVO_STOP_SERVICE  = '/servo_node/stop_servo'
SERVO_UNPAUSE       = '/servo_node/unpause_servo'
SERVO_RESET         = '/servo_node/reset_servo_status'

KEY_BINDINGS = {
    'w': ( 0,  1,  0,  0,  0,  0),
    's': ( 0, -1,  0,  0,  0,  0),
    'a': ( 1,  0,  0,  0,  0,  0),
    'd': (-1,  0,  0,  0,  0,  0),
    'r': ( 0,  0,  1,  0,  0,  0),
    'f': ( 0,  0, -1,  0,  0,  0),
    'i': ( 0,  0,  0,  1,  0,  0),
    'k': ( 0,  0,  0, -1,  0,  0),
    'j': ( 0,  0,  0,  0,  1,  0),
    'l': ( 0,  0,  0,  0, -1,  0),
    'u': ( 0,  0,  0,  0,  0,  1),
    'o': ( 0,  0,  0,  0,  0, -1),
}

TOGGLE_KEY = ' '
QUIT_KEY   = 'q'


class KeyboardServoNode(Node):
    def __init__(self):
        super().__init__('keyboard_servo_node')

        self._cbg = ReentrantCallbackGroup()

        self._pub = self.create_publisher(TwistStamped, SERVO_TOPIC, 10)

        # Servo services
        self._start_client   = self.create_client(
            Trigger, SERVO_START_SERVICE, callback_group=self._cbg)
        self._stop_client    = self.create_client(
            Trigger, SERVO_STOP_SERVICE,  callback_group=self._cbg)
        self._unpause_client = self.create_client(
            Trigger, SERVO_UNPAUSE,       callback_group=self._cbg)
        self._reset_client   = self.create_client(
            Trigger, SERVO_RESET,         callback_group=self._cbg)

        # Controller manager switch service
        self._switch_client  = self.create_client(
            SwitchController, SWITCH_CONTROLLER_SERVICE,
            callback_group=self._cbg)

        self._lock          = threading.Lock()
        self._vx = self._vy = self._vz = 0.0
        self._rx = self._ry = self._rz = 0.0

        self._running       = True
        self._servo_active  = False
        self._last_key_time = 0.0

        self._timer = self.create_timer(
            1.0 / PUBLISH_RATE, self._publish,
            callback_group=self._cbg)

    # ── Safe async wait ───────────────────────────────────────────────────────

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
        """
        Atomically activate one controller and deactivate another.
        Uses BEST_EFFORT strictness so it doesn't fail if a controller
        is already in the target state.
        """
        if not self._switch_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('switch_controller service not available.')
            return False

        req                = SwitchController.Request()
        req.activate_controllers   = [activate]
        req.deactivate_controllers = [deactivate]
        # BEST_EFFORT = 1 — won't fail if controller is already active/inactive
        req.strictness     = SwitchController.Request.BEST_EFFORT
        req.activate_asap  = True
        req.timeout        = rclpy.duration.Duration(seconds=2.0).to_msg()

        result = self._wait(
            self._switch_client.call_async(req), timeout=5.0)

        if result and result.ok:
            self.get_logger().info(
                f'Controllers switched: +{activate}  -{deactivate}')
            return True
        else:
            self.get_logger().error(
                f'Controller switch failed: +{activate}  -{deactivate}')
            return False

    # ── Servo lifecycle ───────────────────────────────────────────────────────

    def activate_servo(self):
        """
        Switch to forward_velocity_controller then activate servo.
        Order matters — switch controller first so servo has somewhere
        to send commands before it starts publishing.
        """
        self.get_logger().info('Switching to servo controller...')
        self._switch_controllers(
            activate=SERVO_CONTROLLER,
            deactivate=PLANNER_CONTROLLER)

        self.get_logger().info('Activating servo...')
        self._call(self._reset_client,   'reset_servo_status')
        self._call(self._start_client,   'start_servo')
        self._call(self._unpause_client, 'unpause_servo')

        self._servo_active = True
        print('\n[SERVO ON]  forward_velocity_controller active.')
        print('            Hold a key to move. SPACE to switch back to planner.\n')

    def deactivate_servo(self):
        """
        Stop servo then switch back to scaled_joint_trajectory_controller
        so the RViz motion planner can command the arm again.
        Order matters — stop servo before switching so it doesn't send
        a final command to the wrong controller mid-switch.
        """
        self._zero_velocity()
        self._call(self._stop_client, 'stop_servo')

        self.get_logger().info('Switching back to planner controller...')
        self._switch_controllers(
            activate=PLANNER_CONTROLLER,
            deactivate=SERVO_CONTROLLER)

        self._servo_active = False
        print('\n[PLANNER ON] scaled_joint_trajectory_controller active.')
        print('             RViz motion planning is free. SPACE to re-enable servo.\n')

    def toggle_servo(self):
        if self._servo_active:
            self.deactivate_servo()
        else:
            self.activate_servo()

    # ── Velocity ──────────────────────────────────────────────────────────────

    def _set_velocity(self, lx, ly, lz, rx, ry, rz):
        with self._lock:
            self._vx = lx * LINEAR_SPEED
            self._vy = ly * LINEAR_SPEED
            self._vz = lz * LINEAR_SPEED
            self._rx = rx * ANGULAR_SPEED
            self._ry = ry * ANGULAR_SPEED
            self._rz = rz * ANGULAR_SPEED

    def _zero_velocity(self):
        self._set_velocity(0, 0, 0, 0, 0, 0)

    def _publish(self):
        if not self._servo_active:
            return
        msg                 = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = COMMAND_FRAME
        with self._lock:
            msg.twist.linear.x  = self._vx
            msg.twist.linear.y  = self._vy
            msg.twist.linear.z  = self._vz
            msg.twist.angular.x = self._rx
            msg.twist.angular.y = self._ry
            msg.twist.angular.z = self._rz
        self._pub.publish(msg)

    # ── Key watchdog ──────────────────────────────────────────────────────────

    def _start_watchdog(self):
        while self._running:
            time.sleep(KEY_TIMEOUT / 2)
            if not self._servo_active:
                continue
            if time.time() - self._last_key_time > KEY_TIMEOUT:
                self._zero_velocity()

    # ── Keyboard ──────────────────────────────────────────────────────────────

    def start_keyboard(self):
        threading.Thread(target=self._keyboard_loop, daemon=True).start()
        threading.Thread(target=self._start_watchdog, daemon=True).start()

    def _keyboard_loop(self):
        fd  = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        self._print_controls()
        try:
            while self._running:
                ch = sys.stdin.read(1)

                if ch == QUIT_KEY:
                    self._running = False
                    if self._servo_active:
                        self.deactivate_servo()
                    rclpy.shutdown()
                    break

                if ch == TOGGLE_KEY:
                    # Run in thread — controller switch takes ~1s,
                    # don't block the keyboard loop while it happens
                    threading.Thread(
                        target=self.toggle_servo, daemon=True).start()
                    continue

                if ch in KEY_BINDINGS:
                    if not self._servo_active:
                        print('Servo off — press SPACE to enable.')
                        continue
                    self._last_key_time = time.time()
                    self._set_velocity(*KEY_BINDINGS[ch])
                else:
                    self._zero_velocity()

        finally:
            self._zero_velocity()
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    @staticmethod
    def _print_controls():
        print('\n── Cartesian velocity servo ──────────────────────────')
        print('  Translation:  w/s=±x   a/d=±y   r/f=±z')
        print('  Rotation:     i/k=roll  j/l=pitch  u/o=yaw')
        print('  HOLD key to move — release to stop')
        print()
        print('  SPACE  →  toggle servo / RViz planner')
        print('  q      →  quit (restores planner controller)')
        print('──────────────────────────────────────────────────────\n')


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardServoNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Start in planner mode — servo activate is triggered by SPACE
    print('Starting in planner mode (scaled_joint_trajectory_controller).')
    print('Press SPACE to switch to servo mode.')
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
