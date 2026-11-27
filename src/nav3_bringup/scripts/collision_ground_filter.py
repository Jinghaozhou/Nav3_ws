#!/usr/bin/env python3
"""Separate a validated, locally tracked flat support plane from obstacles."""
import json
import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def fit_floor(points, robot, minimum=120, fit_radius=4.0):
    """Return z=ax+by+c only for a broad, gravity-aligned support surface."""
    points = np.asarray(points, dtype=float)
    robot = np.asarray(robot, dtype=float)
    if not math.isfinite(fit_radius) or fit_radius <= 0:
        raise ValueError('Floor fit radius must be finite and positive')
    select = (np.isfinite(points).all(axis=1)
              & (np.linalg.norm(points[:, :2]-robot[:2], axis=1) < fit_radius)
              & (points[:, 2] > robot[2]-.65) & (points[:, 2] < robot[2]-.30))
    candidates = points[select]
    if len(candidates) < minimum:
        raise ValueError('Insufficient support-plane points')
    design = np.c_[candidates[:, :2], np.ones(len(candidates))]
    rng = np.random.default_rng(42)
    best = None
    for _ in range(100):
        ids = rng.choice(len(candidates), 3, replace=False)
        if abs(np.linalg.det(design[ids])) < 1e-5:
            continue
        plane = np.linalg.solve(design[ids], candidates[ids, 2])
        if np.linalg.norm(plane[:2]) > math.tan(math.radians(5)):
            continue
        if abs(np.dot(np.r_[robot[:2], 1.], plane)-(robot[2]-.44)) > .10:
            continue
        inliers = np.abs(design@plane-candidates[:, 2]) < .0125
        if best is None or inliers.sum() > best.sum():
            best = inliers
    if best is None or best.sum() < minimum or best.mean() < .60:
        raise ValueError('No dominant floor plane')
    floor = candidates[best]
    if np.any(np.ptp(floor[:, :2], axis=0) < 1.5):
        raise ValueError('Floor support does not span both horizontal axes')
    plane = np.linalg.lstsq(design[best], floor[:, 2], rcond=None)[0]
    residual = design[best]@plane-floor[:, 2]
    if np.linalg.norm(plane[:2]) > math.tan(math.radians(5)) or np.sqrt(np.mean(residual**2)) > .01:
        raise ValueError('Floor fit is tilted or inconsistent')
    return plane


def obstacle_mask(points, plane, tolerance=.025):
    return np.abs(points[:, 2]-(points[:, :2]@plane[:2]+plane[2])) > tolerance


def track_floor(previous, candidate, robot):
    # Compare predictions locally, not the intercept at an increasingly distant
    # odometry origin. Abrupt changes fail closed; accepted changes are smoothed.
    xy = np.asarray(robot)[:2]+np.array([[0, 0], [1, 0], [-1, 0], [0, 1], [0, -1]])
    changes = np.c_[xy, np.ones(5)]@(candidate-previous)
    if np.max(np.abs(changes)) > .025:
        raise ValueError('Local support plane changed abruptly')
    return .8*previous+.2*candidate


def transform_xyz(points, transform):
    t = transform.transform.translation
    q = transform.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return points@rotation.T+np.array([t.x, t.y, t.z])


class GroundFilter(Node):
    def __init__(self):
        super().__init__('collision_ground_filter')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.floor_reference_frame = self.declare_parameter('floor_reference_frame', 'base_link').value
        self.declare_parameter('floor_fit_radius', 4.0)
        self.plane = None
        self.confirmations = []
        self.last_warning = 0
        self.frames = 0
        self.qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(PointCloud2, '/fast_lio/collision_obstacles', self.qos)
        self.status = self.create_publisher(String, '/fast_lio/ground_filter_status', 10)
        self.subscription = self.create_subscription(PointCloud2, '/fast_lio/collision_map', self.on_cloud, self.qos)

    def on_cloud(self, message):
        try:
            stamp = Time.from_msg(message.header.stamp)
            tf = self.buffer.lookup_transform('nav_odom', message.header.frame_id, stamp,
                                              timeout=Duration(seconds=.03))
            base = self.buffer.lookup_transform('nav_odom', self.floor_reference_frame, stamp,
                                                timeout=Duration(seconds=.03))
            translation = base.transform.translation
            robot = np.array([translation.x, translation.y, translation.z])
            data = point_cloud2.read_points_numpy(message, field_names=('x', 'y', 'z'), skip_nans=True)
            points = transform_xyz(np.asarray(data, dtype=float), tf)
            fit_radius = self.get_parameter('floor_fit_radius').value
            if self.plane is None:
                plane = fit_floor(points, robot, fit_radius=fit_radius)
                if self.confirmations and np.max(np.abs(plane-self.confirmations[-1])) > .01:
                    self.confirmations.clear()
                self.confirmations.append(plane)
                if len(self.confirmations) < 3:
                    return
                self.plane = np.median(self.confirmations, axis=0)
                self.get_logger().info(f'Initialized nav_odom floor z=ax+by+c: {self.plane.tolist()}; separation band=0.025 m')
            else:
                try:
                    candidate = fit_floor(points, robot, fit_radius=fit_radius)
                except ValueError:
                    # Clutter may prevent a new dominant fit. Keep the previous
                    # model only if it still has live support below.
                    candidate = None
                if candidate is not None:
                    self.plane = track_floor(self.plane, candidate, robot)
            # Missing live support suppresses this snapshot. Elevated points
            # are never filtered as outliers, even when model updates pause.
            residual = points[:, 2]-(points[:, :2]@self.plane[:2]+self.plane[2])
            if np.count_nonzero(np.abs(residual) < .025) < 120:
                raise ValueError('Fixed floor plane no longer has sufficient live support')
            keep = obstacle_mask(points, self.plane)
            header = message.header
            header.frame_id = 'nav_odom'
            output = point_cloud2.create_cloud_xyz32(header, points[keep].astype(np.float32))
            self.publisher.publish(output)
            self.frames += 1
            if self.frames % 5 == 0:
                self.status.publish(String(data=json.dumps({'plane': self.plane.tolist(),
                    'input': len(points), 'ground': int((~keep).sum()), 'obstacles': int(keep.sum())})))
        except Exception as error:
            now = self.get_clock().now().nanoseconds
            if now-self.last_warning > 2_000_000_000:
                self.get_logger().warning(f'No filtered collision snapshot: {error}')
                self.last_warning = now


def main():
    rclpy.init()
    node = GroundFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
