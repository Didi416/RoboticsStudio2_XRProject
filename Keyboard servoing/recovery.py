"""
Servo recovery script.

Run this whenever the arm hits a singularity and servo emergency stops:
    python3 recover.py

It will:
  1. Stop servo
  2. Clear the emergency stop latch
  3. Switch to scaled_joint_trajectory_controller
  4. Move the arm to the safe home pose
  5. Switch back to forward_position_controller
  6. Restart and unpause servo

After this script completes, your keyboard_bridge.py should work normally.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController
import threading


# ── Safe home pose ────────────────────────────────────────────────────────────
# Elbow up, wrist neutral — well away from all UR3e singularities.
# Adjust these if your workspace needs a different home.
HOME_JOINTS = [0.0, -1.0, 0.5, -1.0, -1.57, 0.0]
HOME_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]
HOME_MOVE_TIME = 5.0   # seconds — slow and safe

# ── Controller names ──────────────────────────────────────────────────────────
SERVO_CONTROLLER   = 'forward_velocity_controller'
PLANNER_CONTROLLER = 'scaled_joint_trajectory_controller'

# ── Servo services ────────────────────────────────────────────────────────────
SERVO_STOP    = '/servo_node/stop_servo'
SERVO_START   = '/servo_node/start_servo'
SERVO_RESET   = '/servo_node/reset_servo_status'
SERVO_UNPAUSE = '/servo_node/unpause_servo'


class RecoveryNode(Node):
    def __init__(self):
        super().__init__('servo_recovery_node')

        from rclpy.callback_groups import ReentrantCallbackGroup
        self._cbg = ReentrantCallbackGroup()

        self._stop_client    = self.create_client(
            Trigger, SERVO_STOP,    callback_group=self._cbg)
        self._start_client   = self.create_client(
            Trigger, SERVO_START,   callback_group=self._cbg)
        self._reset_client   = self.create_client(
            Trigger, SERVO_RESET,   callback_group=self._cbg)
        self._unpause_client = self.create_client(
            Trigger, SERVO_UNPAUSE, callback_group=self._cbg)
        self._switch_client  = self.create_client(
            SwitchController, '/controller_manager/switch_controller',
            callback_group=self._cbg)

        self._traj_pub = self.create_publisher(
            JointTrajectory,
            f'/{PLANNER_CONTROLLER}/joint_trajectory',
            10)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _wait(self, future, timeout=10.0):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        done.wait(timeout=timeout)
        return future.result()

    def _call(self, client, label, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f'{label} not available — skipping.')
            return False
        result = self._wait(client.call_async(Trigger.Request()), timeout)
        ok = result is not None
        status = 'OK' if ok else 'FAILED'
        print(f'  {status}  {label}')
        return ok

    def _switch(self, activate, deactivate):
        if not self._switch_client.wait_for_service(timeout_sec=5.0):
            print('  FAILED  switch_controller not available')
            return False
        req                        = SwitchController.Request()
        req.activate_controllers   = [activate]
        req.deactivate_controllers = [deactivate]
        req.strictness             = SwitchController.Request.BEST_EFFORT
        req.activate_asap          = True
        req.timeout                = rclpy.duration.Duration(seconds=2.0).to_msg()
        result = self._wait(self._switch_client.call_async(req), timeout=5.0)
        ok = result is not None and result.ok
        print(f'  {"OK" if ok else "FAILED"}  switch +{activate}  -{deactivate}')
        return ok

    def _move_home(self):
        msg              = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names  = HOME_JOINT_NAMES

        pt               = JointTrajectoryPoint()
        pt.positions     = HOME_JOINTS
        pt.velocities    = [0.0] * 6
        pt.time_from_start = Duration(
            sec=int(HOME_MOVE_TIME),
            nanosec=int((HOME_MOVE_TIME % 1) * 1e9))
        msg.points       = [pt]

        self._traj_pub.publish(msg)
        print(f'  OK   trajectory sent — waiting {HOME_MOVE_TIME:.0f}s for arm to arrive...')
        time.sleep(HOME_MOVE_TIME + 1.0)

    # ── Recovery sequence ─────────────────────────────────────────────────────

    def recover(self):
        print('\n── Servo recovery ───────────────────────────────────')

        print('\n[1/5] Stopping servo...')
        self._call(self._stop_client, SERVO_STOP)

        print('\n[2/5] Clearing emergency stop latch...')
        self._call(self._reset_client, SERVO_RESET)

        print('\n[3/5] Switching to planner controller...')
        self._switch(
            activate=PLANNER_CONTROLLER,
            deactivate=SERVO_CONTROLLER)

        print('\n[4/5] Moving arm to safe home pose...')
        self._move_home()

        print('\n[5/5] Restarting servo from safe pose...')
        self._switch(
            activate=SERVO_CONTROLLER,
            deactivate=PLANNER_CONTROLLER)
        self._call(self._start_client,   SERVO_START)
        self._call(self._reset_client,   SERVO_RESET)
        self._call(self._unpause_client, SERVO_UNPAUSE)

        print('\n── Recovery complete ─────────────────────────────────')
        print('Servo is running from safe home pose.')
        print('Your keyboard_bridge.py should now respond normally.\n')


def main():
    rclpy.init()
    node = RecoveryNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Brief pause to let executor start and publishers register
    time.sleep(0.5)

    try:
        node.recover()
    except KeyboardInterrupt:
        print('\nRecovery interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()