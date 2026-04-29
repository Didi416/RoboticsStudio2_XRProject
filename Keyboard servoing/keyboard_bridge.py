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
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController

# ── Tuning ────────────────────────────────────────────────────────────────────
LINEAR_SPEED  = 0.05
ANGULAR_SPEED = 0.3
PUBLISH_RATE  = 30.0
KEY_TIMEOUT   = 0.08   # seconds — hold-to-move watchdog threshold

COMMAND_FRAME = 'base_link'

# ── Controller names ──────────────────────────────────────────────────────────
SERVO_CONTROLLER   = 'forward_position_controller'
PLANNER_CONTROLLER = 'scaled_joint_trajectory_controller'

SWITCH_CONTROLLER_SERVICE = '/controller_manager/switch_controller'

# Gripper controller swap — finger_width_controller conflicts with
# finger_width_trajectory_controller so we deactivate the latter on startup
GRIPPER_ACTIVATE   = 'finger_width_controller'
GRIPPER_DEACTIVATE = 'finger_width_trajectory_controller'

# ── Servo services ────────────────────────────────────────────────────────────
SERVO_TOPIC         = '/servo_node/delta_twist_cmds'
SERVO_START_SERVICE = '/servo_node/start_servo'
SERVO_STOP_SERVICE  = '/servo_node/stop_servo'
SERVO_UNPAUSE       = '/servo_node/unpause_servo'
SERVO_RESET         = '/servo_node/reset_servo_status'

# ── Gripper ───────────────────────────────────────────────────────────────────
# finger_width_trajectory_controller is active by default — use it directly.
# finger_width_controller (Float64MultiArray) conflicts with it and requires
# a controller switch which can cause brief interruptions.
GRIPPER_TOPIC  = '/finger_width_controller/commands'
GRIPPER_OPEN   = 0.10   # metres — RG2 max aperture ~110 mm
GRIPPER_CLOSE  = 0.0    # metres — fully closed
GRIPPER_STEP   = 0.01   # metres per incremental keypress

# ── Key bindings ──────────────────────────────────────────────────────────────
KEY_BINDINGS = {
    'w': ( 1,  0,  0,  0,  0,  0),
    's': (-1,  0,  0,  0,  0,  0),
    'a': ( 0,  1,  0,  0,  0,  0),
    'd': ( 0, -1,  0,  0,  0,  0),
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

        # Arm publisher
        self._pub = self.create_publisher(TwistStamped, SERVO_TOPIC, 10)

        # Gripper publisher
        self._gripper_pub = self.create_publisher(
            Float64MultiArray, GRIPPER_TOPIC, 10)

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
        self._finger_width  = GRIPPER_OPEN   # assume open on start

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
        if not self._switch_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('switch_controller service not available.')
            return False

        req                        = SwitchController.Request()
        req.activate_controllers   = [activate]
        req.deactivate_controllers = [deactivate]
        req.strictness             = SwitchController.Request.BEST_EFFORT
        req.activate_asap          = True
        req.timeout                = rclpy.duration.Duration(seconds=2.0).to_msg()

        result = self._wait(self._switch_client.call_async(req), timeout=5.0)

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
        self.get_logger().info('Switching to servo controller...')
        self._switch_controllers(
            activate=SERVO_CONTROLLER,
            deactivate=PLANNER_CONTROLLER)

        self.get_logger().info('Activating servo...')
        self._call(self._start_client,   'start_servo')
        self._call(self._reset_client,   'reset_servo_status')
        self._call(self._unpause_client, 'unpause_servo')

        self._servo_active = True
        print('\n[SERVO ON]  forward_position_controller active.')
        print('            Hold a key to move. SPACE to switch back to planner.\n')

    def deactivate_servo(self):
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

    # ── Arm velocity ──────────────────────────────────────────────────────────

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

    # ── Gripper ───────────────────────────────────────────────────────────────

    def _set_gripper(self, width):
        """Send a finger width command to the gripper (metres)."""
        width = max(GRIPPER_CLOSE, min(GRIPPER_OPEN, width))
        self._finger_width = width
        msg      = Float64MultiArray()
        msg.data = [width]
        self._gripper_pub.publish(msg)
        self.get_logger().info(f'Gripper → {width * 1000:.1f} mm')

    def gripper_open(self):
        self._set_gripper(GRIPPER_OPEN)

    def gripper_close(self):
        self._set_gripper(GRIPPER_CLOSE)

    def gripper_increment(self, delta):
        self._set_gripper(self._finger_width + delta)

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
                    threading.Thread(
                        target=self.toggle_servo, daemon=True).start()
                    continue

                # Gripper — works in both servo and planner mode
                if ch == 'e':
                    self.gripper_increment(GRIPPER_STEP)
                    continue
                if ch == 'c':
                    self.gripper_increment(-GRIPPER_STEP)
                    continue
                if ch == 'g':
                    self.gripper_open()
                    continue
                if ch == 'b':
                    self.gripper_close()
                    continue

                # Arm — servo mode only
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
        print('\n── UR3e + RG2 keyboard control ───────────────────────')
        print('  Arm (servo mode):')
        print('    w/s=±x   a/d=±y   r/f=±z')
        print('    i/k=roll  j/l=pitch  u/o=yaw')
        print('    HOLD key to move — release to stop')
        print()
        print('  Gripper (always active):')
        print('    e / c  →  open / close  10 mm per press')
        print('    g / b  →  fully open / fully close')
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

    # Switch gripper controller on launch so finger_width_controller
    # is active from the start — no manual ros2 control command needed
    print('Switching gripper controller...')
    node._switch_controllers(
        activate=GRIPPER_ACTIVATE,
        deactivate=GRIPPER_DEACTIVATE)
    print('Gripper controller ready.')

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