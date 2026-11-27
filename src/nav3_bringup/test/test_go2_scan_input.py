"""Check the compiled SCAN nodes accept IMU odometry via the Go2 mount TF.

ROS_DOMAIN_ID=91 /usr/bin/python3 -m unittest discover -s src/nav3_bringup/test \
    -p test_go2_scan_input.py -v
"""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from scan_planner_msgs.msg import PlannerStatus
from scipy.spatial.transform import Rotation
from tf2_ros import StaticTransformBroadcaster
from mid360_bringup.mount_tf import go2_transforms


@unittest.skipUnless(os.environ.get('ROS_DOMAIN_ID') == '91', 'requires isolated ROS_DOMAIN_ID=91')
class Go2ScanInput(unittest.TestCase):
    def test_imu_odometry_is_accepted_and_no_goal_keeps_zero_velocity(self):
        rclpy.init()
        node = Node('test_go2_scan_input')
        statuses, commands = [], []
        qos = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(PlannerStatus, '/scan_planner/status', statuses.append, qos)
        node.create_subscription(Twist, '/cmd_vel', commands.append, 100)
        odom_pub = node.create_publisher(Odometry, '/fast_lio/odom', 20)
        cloud_pub = node.create_publisher(PointCloud2, '/fast_lio/local_map', 5)
        broadcaster = StaticTransformBroadcaster(node)
        imu_base, base_lidar = go2_transforms([-.011, -.02329, .04412], np.eye(3),
            {'xyz': [.207993, 0., .152959], 'rpy': [0., .34906585, 0.]})
        transforms = []
        for parent, child, mat in [('imu_link', 'base_link', imu_base), ('base_link', 'lidar', base_lidar)]:
            tf = TransformStamped(); tf.header.frame_id = parent; tf.child_frame_id = child
            t, q = tf.transform.translation, tf.transform.rotation
            t.x, t.y, t.z = map(float, mat[:3, 3])
            q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(mat[:3, :3]).as_quat())
            transforms.append(tf)
        broadcaster.sendTransform(transforms)
        with tempfile.TemporaryFile(mode='w+') as log:
            process = subprocess.Popen(['ros2', 'launch', 'scan_planner', 'scan_planner.launch.py'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                deadline = time.monotonic()+8
                while time.monotonic() < deadline:
                    self.assertIsNone(process.poll())
                    stamp = node.get_clock().now().to_msg()
                    odom = Odometry(); odom.header.stamp = stamp; odom.header.frame_id = 'odom'
                    odom.child_frame_id = 'imu_link'; odom.pose.pose.orientation.w = 1.
                    odom_pub.publish(odom)
                    cloud = PointCloud2(); cloud.header = odom.header
                    cloud.height = 1; cloud.width = 2; cloud.point_step = 12; cloud.row_step = 24
                    cloud.fields = [PointField(name=axis, offset=i*4, datatype=PointField.FLOAT32, count=1)
                                    for i, axis in enumerate('xyz')]
                    cloud.data = np.array([[-2, -2, -1], [2, 2, 1]], dtype='<f4').tobytes()
                    cloud_pub.publish(cloud)
                    # Keep the synthetic sensor at 10 Hz regardless of queued
                    # controller/status callbacks. Unthrottled input can fall
                    # below the estimator's intentional 1 ms minimum interval.
                    next_input = time.monotonic()+.1
                    while time.monotonic() < next_input:
                        rclpy.spin_once(node, timeout_sec=.01)
                ready = [s for s in statuses if s.state == PlannerStatus.WAITING_PATH]
                self.assertGreater(len(ready), 20, [(s.state, s.reason) for s in statuses[-5:]])
                self.assertTrue(commands)
                self.assertTrue(all(c.linear.x == c.linear.y == c.angular.z == 0 for c in commands))
                self.assertFalse(any(s.motion_allowed for s in statuses))
            finally:
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait()
                log.seek(0)
                if process.returncode not in (0, -2, 130): print(log.read())
                node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
