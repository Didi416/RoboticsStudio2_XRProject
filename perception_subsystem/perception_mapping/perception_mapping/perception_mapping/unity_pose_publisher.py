#!/usr/bin/env python3
"""
unity_pose_publisher.py
Subscribes to ArUco marker pose topics and republishes as a
JSON string on /perception/unity_sync for Unity to consume.
Wall pose (IDs 5-8) is computed from corner markers.
Egg poses (IDs 1-4) are published individually.
"""

import json
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

WALL_CORNER_IDS = [5, 6, 7, 8]
EGG_IDS         = [1, 2, 3, 4]
ALL_IDS         = WALL_CORNER_IDS + EGG_IDS

# ROS (right-hand Z-forward) → Unity (left-hand Y-up) remapping
def ros_to_unity_pos(x, y, z):
    return {'x': float(z), 'y': float(-y), 'z': float(x)}

def ros_to_unity_rot(rx, ry, rz, rw):
    return {'x': float(-rz), 'y': float(ry), 'z': float(-rx), 'w': float(rw)}


class UnityPosePublisher(Node):

    def __init__(self):
        super().__init__('unity_pose_publisher')

        # Store latest pose for each marker ID
        self.poses: dict[int, PoseStamped] = {}

        # Subscribe to all marker topics
        for mid in ALL_IDS:
            self.create_subscription(
                PoseStamped,
                f'/aruco/marker_{mid}/pose',
                lambda msg, m=mid: self._pose_cb(m, msg),
                10)

        # Publish unity_sync at 30 Hz
        self.unity_pub = self.create_publisher(String, '/perception/unity_sync', 10)
        self.create_timer(1.0 / 30.0, self._publish_unity_sync)

        self.get_logger().info('unity_pose_publisher ready.')

    def _pose_cb(self, marker_id: int, msg: PoseStamped):
        self.poses[marker_id] = msg

    def _publish_unity_sync(self):
        visible_wall = [i for i in WALL_CORNER_IDS if i in self.poses]
        visible_eggs = [i for i in EGG_IDS        if i in self.poses]

        # Compute wall centre from average of visible corners (need ≥2)
        if len(visible_wall) >= 2:
            positions = np.array([
                [self.poses[i].pose.position.x,
                 self.poses[i].pose.position.y,
                 self.poses[i].pose.position.z]
                for i in visible_wall])
            centre = positions.mean(axis=0)
            # Use orientation from first visible corner as wall orientation
            o = self.poses[visible_wall[0]].pose.orientation
            wall_status = 'OK'
            confidence  = len(visible_wall) / len(WALL_CORNER_IDS)
        else:
            centre = np.zeros(3)
            o = type('O', (), {'x':0,'y':0,'z':0,'w':1})()
            wall_status = 'LOST'
            confidence  = 0.0

        # Build egg list
        eggs = []
        for mid in EGG_IDS:
            if mid in self.poses:
                p = self.poses[mid].pose.position
                r = self.poses[mid].pose.orientation
                eggs.append({
                    'id':       mid,
                    'status':   'visible',
                    'position': ros_to_unity_pos(p.x, p.y, p.z),
                    'rotation': ros_to_unity_rot(r.x, r.y, r.z, r.w),
                })
            else:
                eggs.append({'id': mid, 'status': 'lost'})

        payload = {
            'wall': {
                'status':     wall_status,
                'confidence': round(confidence, 2),
                'position':   ros_to_unity_pos(*centre),
                'rotation':   ros_to_unity_rot(o.x, o.y, o.z, o.w),
                'corners_visible': visible_wall,
            },
            'eggs': eggs,
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.unity_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UnityPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()