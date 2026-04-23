import sys
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger

# ── Tuning ────────────────────────────────────────────────────────────────────

LINEAR_SPEED  = 0.05   # m/s
ANGULAR_SPEED = 0.3    # rad/s
PUBLISH_RATE  = 50.0   # Hz

COMMAND_FRAME = 'base_link'

SERVO_TOPIC         = '/servo_node/delta_twist_cmds'
SERVO_START_SERVICE = '/servo_node/start_servo'
SERVO_STOP_SERVICE  = '/servo_node/stop_servo'

# ── Key bindings ──────────────────────────────────────────────────────────────
KEY_BINDINGS = {
    'w': ( 1,  0,  0,  0,  0,  0),   # +x forward
    's': (-1,  0,  0,  0,  0,  0),   # -x backward
    'a': ( 0,  1,  0,  0,  0,  0),   # +y left
    'd': ( 0, -1,  0,  0,  0,  0),   # -y right
    'r': ( 0,  0,  1,  0,  0,  0),   # +z up
    'f': ( 0,  0, -1,  0,  0,  0),   # -z down
    'i': ( 0,  0,  0,  1,  0,  0),   # roll +
    'k': ( 0,  0,  0, -1,  0,  0),   # roll -
    'j': ( 0,  0,  0,  0,  1,  0),   # pitch +
    'l': ( 0,  0,  0,  0, -1,  0),   # pitch -
    'u': ( 0,  0,  0,  0,  0,  1),   # yaw +
    'o': ( 0,  0,  0,  0,  0, -1),   # yaw -
}

TOGGLE_KEY = ' '   # spacebar — pause/resume servo so RViz planner can move arm
QUIT_KEY   = 'q'


class KeyboardServoNode(Node):
    def __init__(self):
        super().__init__('keyboard_servo_node')

        self._pub          = self.create_publisher(TwistStamped, SERVO_TOPIC, 10)
        self._start_client = self.create_client(Trigger, SERVO_START_SERVICE)
        self._stop_client  = self.create_client(Trigger, SERVO_STOP_SERVICE)

        self._lock    = threading.Lock()
        self._vx = self._vy = self._vz = 0.0
        self._rx = self._ry = self._rz = 0.0

        self._running       = True
        self._servo_active  = False   # tracks whether servo is currently running

        self._timer = self.create_timer(1.0 / PUBLISH_RATE, self._publish)

    # ── Servo lifecycle ───────────────────────────────────────────────────────

    def _call_service(self, client, label, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f'{label} service not available.')
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        self.get_logger().info(f'{label} called.')
        return True

    def start_servo(self):
        if self._servo_active:
            return
        if self._call_service(self._start_client, 'start_servo'):
            self._servo_active = True
            print('\n[SERVO ON]  Hold a key to move the arm.')

    def stop_servo(self):
        if not self._servo_active:
            return
        self._zero_velocity()
        if self._call_service(self._stop_client, 'stop_servo'):
            self._servo_active = False
            print('\n[SERVO OFF] You can now use RViz motion planning freely.')
            print('            Press SPACE to re-enable servo.\n')

    def toggle_servo(self):
        if self._servo_active:
            self.stop_servo()
        else:
            self.start_servo()

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
        # Only publish when servo is active — keeps the topic silent
        # when RViz planner is in control so there's no conflict
        if not self._servo_active:
            return
        msg                   = TwistStamped()
        msg.header.stamp      = self.get_clock().now().to_msg()
        msg.header.frame_id   = COMMAND_FRAME
        with self._lock:
            msg.twist.linear.x  = self._vx
            msg.twist.linear.y  = self._vy
            msg.twist.linear.z  = self._vz
            msg.twist.angular.x = self._rx
            msg.twist.angular.y = self._ry
            msg.twist.angular.z = self._rz
        self._pub.publish(msg)

    # ── Keyboard ──────────────────────────────────────────────────────────────

    def start_keyboard(self):
        threading.Thread(target=self._keyboard_loop, daemon=True).start()

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
                    self.stop_servo()
                    rclpy.shutdown()
                    break

                if ch == TOGGLE_KEY:
                    self.toggle_servo()
                    continue

                if ch in KEY_BINDINGS:
                    if not self._servo_active:
                        print('Servo is paused — press SPACE to enable.')
                        continue
                    self._set_velocity(*KEY_BINDINGS[ch])
                else:
                    self._zero_velocity()

        finally:
            self._zero_velocity()
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    @staticmethod
    def _print_controls():
        print('\n── Cartesian velocity servo ─────────────────────────')
        print('  Translation:  w/s=x   a/d=y   r/f=z')
        print('  Rotation:     i/k=roll  j/l=pitch  u/o=yaw')
        print('  Hold key to move, release to stop')
        print()
        print('  SPACE  →  toggle servo on/off')
        print('             OFF = RViz planner works freely')
        print('             ON  = keyboard controls the arm')
        print()
        print('  q  →  quit')
        print('─────────────────────────────────────────────────────\n')


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardServoNode()

    # Start servo and keyboard
    node.start_servo()
    node.start_keyboard()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_servo()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()