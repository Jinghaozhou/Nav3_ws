#!/usr/bin/env python3
"""Freeze a gravity-aligned local planning frame after FAST-LIO initialization."""
import math
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


def gravity_rotation(gravity):
    """Quaternion mapping child downward Z to the observed parent gravity."""
    gravity = np.asarray(gravity, dtype=float)
    norm = np.linalg.norm(gravity)
    if not np.isfinite(gravity).all() or not 5.0 < norm < 15.0:
        raise ValueError('Invalid gravity vector')
    direction = gravity / norm
    down = np.array([0., 0., -1.])
    quat = np.r_[np.cross(down, direction), 1. + np.dot(down, direction)]
    if np.linalg.norm(quat) < 1e-8:
        return np.array([1., 0., 0., 0.])
    return quat / np.linalg.norm(quat)


class GravityFrame(Node):
    def __init__(self):
        super().__init__('nav3_gravity_frame')
        self.child = self.declare_parameter('child_frame', 'nav_odom').value
        self.samples = []
        self.published = False
        self.broadcaster = StaticTransformBroadcaster(self)
        self.subscription = self.create_subscription(Vector3Stamped, '/fast_lio/gravity', self.on_gravity, 20)

    def on_gravity(self, message):
        if self.published or message.header.frame_id != 'odom':
            return
        vector = np.array([message.vector.x, message.vector.y, message.vector.z])
        try:
            gravity_rotation(vector)
        except ValueError:
            return
        self.samples.append(vector)
        if len(self.samples) < 20:
            return
        gravity = np.mean(self.samples, axis=0)
        quat = gravity_rotation(gravity)
        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = self.child
        q = transform.transform.rotation
        q.x, q.y, q.z, q.w = map(float, quat)
        self.broadcaster.sendTransform(transform)
        self.published = True
        tilt = math.degrees(math.acos(float(np.clip(-gravity[2]/np.linalg.norm(gravity), -1., 1.))))
        self.get_logger().info(f'Fixed odom -> {self.child}; gravity tilt={tilt:.2f} deg. Restart with FAST-LIO after an odometry reset.')


def main():
    rclpy.init()
    node = GravityFrame()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
