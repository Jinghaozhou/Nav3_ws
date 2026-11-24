#!/usr/bin/env python3
"""Observe sensor/FAST-LIO stamps without publishing or changing the ROS graph."""
import argparse
import csv
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from tf2_msgs.msg import TFMessage


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def percentile(values, fraction):
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lo, hi = math.floor(position), math.ceil(position)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (position - lo)


class TimingProbe(Node):
    def __init__(self, output, warmup):
        super().__init__('nav3_sensor_timing_probe')
        self.start = time.monotonic()
        self.warmup = warmup
        self.samples = {}
        self.stream = output.open('x', newline='')
        self.writer = csv.writer(self.stream)
        self.writer.writerow(['topic', 'receive_ros_ns', 'receive_steady_ns',
                              'stamp_ns', 'age_ms', 'frame_id', 'points'])
        self.subscriptions_ = []
        for topic, message_type in [('/lidar_points', CustomMsg), ('/imu', Imu),
                                    ('/fast_lio/odom', Odometry),
                                    ('/fast_lio/local_cloud', PointCloud2),
                                    ('/fast_lio/local_map', PointCloud2),
                                    ('/fast_lio/collision_map', PointCloud2)]:
            self.subscriptions_.append(self.create_subscription(
                message_type, topic, lambda msg, topic=topic: self.record(topic, msg),
                # Livox publishes large fragmented CustomMsg reliably. Match its
                # QoS to avoid mistaking best-effort fragment loss for sensor gaps.
                QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
                if message_type is CustomMsg else qos_profile_sensor_data))
        self.subscriptions_.append(self.create_subscription(
            TFMessage, '/tf', self.on_tf, qos_profile_sensor_data))

    def on_tf(self, message):
        for transform in message.transforms:
            if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_link':
                self.record('/tf:odom->base_link', transform)

    def record(self, topic, message):
        ros_ns = self.get_clock().now().nanoseconds
        steady_ns = time.monotonic_ns()
        source_ns = stamp_ns(message.header.stamp)
        age_ms = (ros_ns - source_ns) / 1e6
        points = getattr(message, 'point_num', getattr(message, 'width', 0))
        self.writer.writerow([topic, ros_ns, steady_ns, source_ns,
                              age_ms, message.header.frame_id, points])
        if time.monotonic() - self.start >= self.warmup:
            self.samples.setdefault(topic, []).append((steady_ns, source_ns, age_ms))

    def finish(self):
        self.stream.close()
        summary = {}
        for topic, rows in self.samples.items():
            arrivals = [(b[0] - a[0]) / 1e6 for a, b in zip(rows, rows[1:])]
            stamps = [(b[1] - a[1]) / 1e6 for a, b in zip(rows, rows[1:])]
            span = (rows[-1][0] - rows[0][0]) / 1e9
            ages = [row[2] for row in rows]
            summary[topic] = {
                'samples_after_warmup': len(rows),
                'receive_hz': (len(rows) - 1) / span if span > 0 else None,
                'age_ms_p50': percentile(ages, .5),
                'age_ms_p95': percentile(ages, .95),
                'age_ms_max': max(ages),
                'arrival_gap_ms_max': max(arrivals) if arrivals else None,
                'stamp_gap_ms_max': max(stamps) if stamps else None,
                'stamp_regressions': sum(delta < 0 for delta in stamps),
            }
        return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=60)
    parser.add_argument('--warmup', type=float, default=10)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 0 <= args.warmup < args.duration:
        parser.error('Require 0 <= warmup < duration')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    probe = TimingProbe(args.output, args.warmup)
    try:
        until = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(probe, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        summary = probe.finish()
        probe.destroy_node()
        rclpy.try_shutdown()
        result = json.dumps(summary, indent=2, ensure_ascii=False)
        args.output.with_suffix('.summary.json').write_text(result + '\n')
        print(result)
    return 0 if all(topic in summary for topic in (
        '/lidar_points', '/imu', '/fast_lio/odom', '/fast_lio/local_cloud',
        '/fast_lio/local_map', '/tf:odom->base_link')) else 1


if __name__ == '__main__':
    raise SystemExit(main())
