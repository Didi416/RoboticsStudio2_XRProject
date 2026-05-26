#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

class CameraRelay(Node):
    def __init__(self):
        super().__init__('camera_relay')

        # Match camera's QoS exactly for subscribing
        camera_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        # Match what Unity ROS-TCP-Connector expects
        unity_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.relay_callback,
            camera_qos
        )

        self.pub = self.create_publisher(
            Image,
            '/camera/color/image_raw_unity',
            unity_qos
        )

        self.get_logger().info('Camera relay node started.')
        self.get_logger().info('Subscribing: /camera/camera/color/image_raw')
        self.get_logger().info('Publishing:  /camera/color/image_raw_unity')
        self.get_logger().info('QoS: RELIABLE + VOLATILE (Unity compatible)')

    def relay_callback(self, msg):
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CameraRelay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()