#!/usr/bin/env python3
"""Publish a box Marker representing the golf cart body for RViz visualization."""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class CartMarker(Node):
    def __init__(self):
        super().__init__('cart_marker')

        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('topic', '/cart_marker')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('length_m', 2.8)
        self.declare_parameter('width_m', 1.3)
        self.declare_parameter('height_m', 1.8)
        # Center of the cart body relative to frame_id (lidar is usually
        # roof-mounted, so the body sits below and may be offset fore/aft).
        self.declare_parameter('offset_x', 0.0)
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('offset_z', -0.9)
        self.declare_parameter('color_rgba', [1.0, 0.65, 0.0, 0.35])

        frame_id = str(self.get_parameter('frame_id').value)
        topic = str(self.get_parameter('topic').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.marker = Marker()
        self.marker.header.frame_id = frame_id
        self.marker.ns = 'golf_cart'
        self.marker.id = 0
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = float(self.get_parameter('offset_x').value)
        self.marker.pose.position.y = float(self.get_parameter('offset_y').value)
        self.marker.pose.position.z = float(self.get_parameter('offset_z').value)
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = float(self.get_parameter('length_m').value)
        self.marker.scale.y = float(self.get_parameter('width_m').value)
        self.marker.scale.z = float(self.get_parameter('height_m').value)

        r, g, b, a = [float(c) for c in self.get_parameter('color_rgba').value]
        self.marker.color.r = r
        self.marker.color.g = g
        self.marker.color.b = b
        self.marker.color.a = a

        self.pub = self.create_publisher(Marker, topic, 10)
        self.create_timer(1.0 / rate_hz, self._timer_cb)

        self.get_logger().info(
            f'publishing cart marker on {topic} in frame "{frame_id}" '
            f'({self.marker.scale.x:.2f} x {self.marker.scale.y:.2f} x '
            f'{self.marker.scale.z:.2f} m)')

    def _timer_cb(self):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.marker)


def main(args=None):
    rclpy.init(args=args)
    node = CartMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
