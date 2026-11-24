#!/usr/bin/env python3
"""Convert the manager's latest start/goal pair into labeled RViz markers."""

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class EndpointMarkers(Node):
    def __init__(self):
        super().__init__('nav3_endpoint_markers')
        endpoint_topic = self.declare_parameter(
            'endpoint_topic', '/nav3/navigation_endpoints').value
        marker_topic = self.declare_parameter(
            'marker_topic', '/nav3/viz/navigation_endpoints').value
        self.marker_scale = float(self.declare_parameter('marker_scale', 0.35).value)
        self.text_height = float(self.declare_parameter('text_height', 0.28).value)
        self.text_z_offset = float(self.declare_parameter('text_z_offset', 0.50).value)
        if self.marker_scale <= 0.0 or self.text_height <= 0.0:
            raise ValueError('marker_scale and text_height must be positive')

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(MarkerArray, marker_topic, qos)
        self.subscription = self.create_subscription(
            PoseArray, endpoint_topic, self.on_endpoints, qos)
        self.get_logger().info(
            f'Navigation endpoint markers: {endpoint_topic} -> {marker_topic}')

    def sphere(self, message, marker_id, pose, color):
        marker = Marker()
        marker.header = message.header
        marker.ns = 'nav3_navigation_endpoints'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y
        marker.pose.position.z = pose.position.z
        marker.pose.orientation = pose.orientation
        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale
        marker.scale.z = self.marker_scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def label(self, message, marker_id, pose, text, color):
        marker = Marker()
        marker.header = message.header
        marker.ns = 'nav3_navigation_endpoint_labels'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y
        marker.pose.position.z = pose.position.z + self.text_z_offset
        marker.pose.orientation.w = 1.0
        marker.scale.z = self.text_height
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.text = text
        return marker

    def on_endpoints(self, message):
        if not message.header.frame_id or len(message.poses) != 2:
            self.get_logger().warn(
                'Ignore navigation endpoints: expected a frame and exactly two poses')
            return
        start, goal = message.poses
        start_color = (0.10, 1.00, 0.20, 0.95)
        goal_color = (1.00, 0.15, 0.10, 0.95)
        markers = MarkerArray()
        markers.markers = [
            self.sphere(message, 0, start, start_color),
            self.sphere(message, 1, goal, goal_color),
            self.label(message, 10, start, '起点', start_color),
            self.label(message, 11, goal, '目标', goal_color),
        ]
        self.publisher.publish(markers)


def main():
    rclpy.init()
    node = EndpointMarkers()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
