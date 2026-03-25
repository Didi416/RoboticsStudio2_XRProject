# import rclpy
# import numpy as np
# from rclpy.node import Node
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point
# from std_msgs.msg import ColorRGBA

# class Reachability(Node):

#     def __init__(self):
#         super().__init__('reachability')

#         self.pub = self.create_publisher(Marker, 'reachability', 10)

#         self.marker = Marker()
#         self.marker.header.frame_id = "base_link"
#         self.marker.ns = "reachability"
#         self.marker.id = 0
#         self.marker.type = Marker.POINTS
#         self.marker.action = Marker.ADD

#         self.marker.scale.x = 0.01
#         self.marker.scale.y = 0.01

#         self.generate_points()

#     # def generate_points(self):
#     #     R = 0.5  

#     #     for _ in range(20000):
#     #         x = np.random.uniform(0.0, R)
#     #         y = np.random.uniform(-R, R)
#     #         z = np.random.uniform(0.0, R)

#     #         if x**2 + y**2 + z**2 <= R**2:
#     #             p = Point()
#     #             p.x = x
#     #             p.y = y
#     #             p.z = z
#     #             self.marker.points.append(p)

#     #             c = ColorRGBA()
#     #             c.r = 0.0
#     #             c.g = 1.0
#     #             c.b = 0.0
#     #             c.a = 1.0
#     #             self.marker.colors.append(c)

#     def generate_points(self):

#         R = 0.5  # UR3e reach

#         # Board parameters (EDIT THESE)
#         board_distance = 0.35   # distance from base
#         board_width = 0.4
#         board_height = 0.4
#         resolution = 0.02

#         for y in np.arange(-board_width/2, board_width/2, resolution):
#             for z in np.arange(0.0, board_height, resolution):

#                 x = board_distance

#                 p = Point()
#                 p.x = float(x)
#                 p.y = float(y)
#                 p.z = float(z)
#                 self.marker.points.append(p)

#                 # Distance from base
#                 dist = np.sqrt(x**2 + y**2 + z**2)

#                 c = ColorRGBA()

#                 if dist <= R:
#                     c.r = 0.0
#                     c.g = 1.0
#                     c.b = 0.0   # reachable = green
#                 else:
#                     c.r = 1.0
#                     c.g = 0.0
#                     c.b = 0.0   # unreachable = red

#                 c.a = 1.0
#                 self.marker.colors.append(c)

# def main():
#     rclpy.init()
#     node = Reachability()

#     node.get_logger().info("Reachability node running")

#     while rclpy.ok():
#         node.marker.header.stamp = node.get_clock().now().to_msg()
#         node.pub.publish(node.marker)
#         rclpy.spin_once(node, timeout_sec=0.1)

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#new
import rclpy
import numpy as np
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class Reachability(Node):

    def __init__(self):
        super().__init__('reachability')

        self.pub = self.create_publisher(Marker, 'reachability', 10)

        # Board marker
        self.board_marker = Marker()
        self.board_marker.header.frame_id = "base_link"
        self.board_marker.ns = "board"
        self.board_marker.id = 0
        self.board_marker.type = Marker.POINTS
        self.board_marker.action = Marker.ADD
        self.board_marker.scale.x = 0.01
        self.board_marker.scale.y = 0.01

        # Floor marker
        self.floor_marker = Marker()
        self.floor_marker.header.frame_id = "base_link"
        self.floor_marker.ns = "floor"
        self.floor_marker.id = 1
        self.floor_marker.type = Marker.POINTS
        self.floor_marker.action = Marker.ADD
        self.floor_marker.scale.x = 0.01
        self.floor_marker.scale.y = 0.01

        self.generate_board_points()
        self.generate_floor_points()

        # Timer to continuously publish
        self.timer = self.create_timer(0.2, self.publish_markers)

    def generate_board_points(self):
        R = 0.5  # UR3e reach
        board_distance = 0.35
        board_width = 0.4
        board_height = 0.4
        resolution = 0.02

        for y in np.arange(-board_width/2, board_width/2, resolution):
            for z in np.arange(0.0, board_height, resolution):
                x = board_distance
                p = Point(x=float(x), y=float(y), z=float(z))
                self.board_marker.points.append(p)

                dist = np.sqrt(x**2 + y**2 + z**2)
                c = ColorRGBA()
                if dist <= R:
                    c.r, c.g, c.b, c.a = 0.0, 1.0, 0.0, 1.0  # reachable = green
                else:
                    c.r, c.g, c.b, c.a = 1.0, 0.0, 0.0, 1.0  # unreachable = red
                self.board_marker.colors.append(c)

    def generate_floor_points(self):
        R = 0.5
        floor_distance = 0.1
        floor_width = 0.6
        floor_length = 0.6
        resolution = 0.02

        for xi in np.arange(floor_distance, floor_distance + floor_length, resolution):
            for y in np.arange(-floor_width/2, floor_width/2, resolution):
                z = 0.0
                p = Point(x=float(xi), y=float(y), z=float(z))
                self.floor_marker.points.append(p)

                dist = np.sqrt(xi**2 + y**2 + z**2)
                c = ColorRGBA()
                if dist <= R:
                    c.r, c.g, c.b, c.a = 0.0, 1.0, 0.0, 1.0
                else:
                    c.r, c.g, c.b, c.a = 1.0, 0.0, 0.0, 1.0
                self.floor_marker.colors.append(c)

    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        self.board_marker.header.stamp = now
        self.floor_marker.header.stamp = now

        self.pub.publish(self.board_marker)
        self.pub.publish(self.floor_marker)


def main():
    rclpy.init()
    node = Reachability()
    node.get_logger().info("Reachability node running")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()