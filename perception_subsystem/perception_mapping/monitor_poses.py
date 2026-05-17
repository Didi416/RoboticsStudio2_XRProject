#!/usr/bin/env python3
"""
monitor_poses.py

Subscribes to all ArUco marker pose topics simultaneously and prints a
clean, labelled readout whenever any marker's pose updates.

Run in Terminal 3 instead of: ros2 topic echo /aruco/marker_0/pose

Usage:
    python3 monitor_poses.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

PUZZLE_MARKERS = {
    0:  'Frog Call Console',
    1:  'Lily Pad Maze',
    2:  'Frog Egg Sorting',
    3:  'Containment Dial',
    10: 'Calibration Target',
}

# Only print a pose update if position changed by more than this (metres).
# Prevents flooding the terminal with identical values between frames.
MIN_CHANGE_THRESHOLD = 0.002


class PoseMonitor(Node):

    def __init__(self):
        super().__init__('pose_monitor')

        # Track last printed position per marker to filter noise
        self._last_pos: dict[int, tuple] = {}

        for marker_id, label in PUZZLE_MARKERS.items():
            topic = f'/aruco/marker_{marker_id}/pose'
            # Use a closure to capture marker_id correctly in the lambda
            self.create_subscription(
                PoseStamped,
                topic,
                self._make_callback(marker_id, label),
                10
            )

        self.get_logger().info(
            'Monitoring poses for markers: '
            + ', '.join(f'ID {k}' for k in PUZZLE_MARKERS)
        )
        print('\n  Waiting for markers...\n')

    def _make_callback(self, marker_id: int, label: str):
        def callback(msg: PoseStamped):
            p = msg.pose.position
            o = msg.pose.orientation

            # Skip if position hasn't meaningfully changed since last print
            last = self._last_pos.get(marker_id)
            current = (round(p.x, 4), round(p.y, 4), round(p.z, 4))
            if last is not None:
                delta = max(abs(current[i] - last[i]) for i in range(3))
                if delta < MIN_CHANGE_THRESHOLD:
                    return

            self._last_pos[marker_id] = current

            print(
                f'┌─ Marker ID {marker_id} — {label}\n'
                f'│  Position   x: {p.x:+.4f} m   '
                f'y: {p.y:+.4f} m   '
                f'z: {p.z:+.4f} m\n'
                f'│  Orientation  '
                f'x: {o.x:+.4f}  '
                f'y: {o.y:+.4f}  '
                f'z: {o.z:+.4f}  '
                f'w: {o.w:+.4f}\n'
                f'└─ stamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec // 1_000_000:03d}s\n'
            )

        return callback


def main():
    rclpy.init()
    node = PoseMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()