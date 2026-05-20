#!/usr/bin/env python3
"""
pose_filter_node.py
Subscribes to /perception/puzzle_wall_pose (PoseStamped from aruco_detector_node),
applies an N-frame moving average on XYZ and quaternion separately,
and republishes filtered pose.

Inputs:
  /perception/puzzle_wall_pose  (geometry_msgs/PoseStamped)  — from aruco_detector_node

Outputs:
  /perception/puzzle_poses         (geometry_msgs/PoseStamped)  — filtered
  /perception/puzzle_poses_raw     (geometry_msgs/PoseStamped)  — raw pass-through for RViz comparison
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from collections import deque
import numpy as np


def normalize_quaternion(q):
    norm = np.linalg.norm(q)
    if norm < 1e-6:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / norm


def average_quaternions(quats):
    """
    Average quaternions using eigenvalue method.
    quats: list of [x, y, z, w] arrays
    """
    M = np.zeros((4, 4))
    for q in quats:
        q = np.array(q).reshape(4, 1)
        M += q @ q.T
    M /= len(quats)
    eigenvalues, eigenvectors = np.linalg.eigh(M)
    return eigenvectors[:, np.argmax(eigenvalues)]


class PoseFilterNode(Node):

    def __init__(self):
        super().__init__('pose_filter_node')

        # --- Parameters ---
        self.declare_parameter('window_size',   7)
        self.declare_parameter('input_topic',   '/perception/puzzle_wall_pose')
        self.declare_parameter('output_topic',  '/perception/puzzle_poses')
        self.declare_parameter('debug_topic',   '/perception/puzzle_poses_raw')

        self.window_size = self.get_parameter('window_size').value
        input_topic      = self.get_parameter('input_topic').value
        output_topic     = self.get_parameter('output_topic').value
        debug_topic      = self.get_parameter('debug_topic').value

        # Sliding windows for position
        self.pos_x = deque(maxlen=self.window_size)
        self.pos_y = deque(maxlen=self.window_size)
        self.pos_z = deque(maxlen=self.window_size)

        # Sliding window for quaternion [x, y, z, w]
        self.quat  = deque(maxlen=self.window_size)

        # --- ROS interfaces ---
        self.sub = self.create_subscription(
            PoseStamped,
            input_topic,
            self.pose_callback,
            10
        )
        self.pub_filtered = self.create_publisher(PoseStamped, output_topic, 10)
        self.pub_raw      = self.create_publisher(PoseStamped, debug_topic,  10)

        self.get_logger().info(
            f'PoseFilterNode ready | window={self.window_size} | '
            f'{input_topic} → {output_topic}'
        )

    def pose_callback(self, msg: PoseStamped):

        # --- Pass raw straight through for RViz comparison ---
        self.pub_raw.publish(msg)

        # --- Update sliding windows ---
        self.pos_x.append(msg.pose.position.x)
        self.pos_y.append(msg.pose.position.y)
        self.pos_z.append(msg.pose.position.z)
        self.quat.append([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ])

        # --- Compute averages ---
        filtered = PoseStamped()
        filtered.header = msg.header  # keep same frame_id and timestamp

        filtered.pose.position.x = float(np.mean(self.pos_x))
        filtered.pose.position.y = float(np.mean(self.pos_y))
        filtered.pose.position.z = float(np.mean(self.pos_z))

        avg_q = average_quaternions(list(self.quat))
        avg_q = normalize_quaternion(avg_q)
        filtered.pose.orientation.x = float(avg_q[0])
        filtered.pose.orientation.y = float(avg_q[1])
        filtered.pose.orientation.z = float(avg_q[2])
        filtered.pose.orientation.w = float(avg_q[3])

        self.pub_filtered.publish(filtered)

        self.get_logger().debug(
            f'raw=({msg.pose.position.x:.4f}, {msg.pose.position.y:.4f}, {msg.pose.position.z:.4f}) '
            f'filtered=({filtered.pose.position.x:.4f}, {filtered.pose.position.y:.4f}, {filtered.pose.position.z:.4f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()