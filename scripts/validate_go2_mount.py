#!/usr/bin/env python3
"""Replay raw inputs in an isolated domain and compare IMU/Go2 TF profiles.

Source scripts/source_env.bash, then use system Python to run this file.
No driver, chassis, navigation target, or recorded TF is started/published.
"""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation
import yaml
from ament_index_python.packages import get_package_share_directory
from mid360_bringup.mount_tf import go2_transforms


def matrix(transform):
    t, q = transform.translation, transform.rotation
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    result[:3, 3] = [t.x, t.y, t.z]
    return result


def pose_matrix(pose):
    p, q = pose.position, pose.orientation
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    result[:3, 3] = [p.x, p.y, p.z]
    return result


def stamp(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


def points(message):
    fields = {f.name: f.offset for f in message.fields}
    return np.column_stack([np.ndarray((message.width,), dtype='<f4', buffer=message.data,
        offset=fields[axis], strides=(message.point_step,))[:100] for axis in 'xyz'])


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def run_profile(args, profile, config, label=None):
    label = label or profile
    body = 'imu_link' if profile == 'go2' else 'base_link'
    node = Node('validate_go2_mount_' + profile)
    buffer = Buffer(); listener = TransformListener(buffer, node)
    odom, clouds, body_clouds, static, counts = {}, {}, {}, {}, Counter()
    static_qos = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def on_odom(m):
        assert (m.header.frame_id, m.child_frame_id) == ('odom', body)
        odom[stamp(m)] = pose_matrix(m.pose.pose)

    def on_cloud(m, store, frame):
        assert m.header.frame_id == frame
        store[stamp(m)] = points(m)

    def on_static(m):
        for t in m.transforms:
            static[t.child_frame_id] = (t.header.frame_id, matrix(t.transform))

    node.create_subscription(Odometry, '/fast_lio/odom', on_odom, 100)
    node.create_subscription(PointCloud2, '/fast_lio/local_cloud',
                             lambda m: on_cloud(m, clouds, 'odom'), qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/fast_lio/body_cloud',
                             lambda m: on_cloud(m, body_clouds, body), qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/fast_lio/local_map',
                             lambda m: counts.update(['local_map']), qos_profile_sensor_data)
    node.create_subscription(TFMessage, '/tf_static', on_static, static_qos)
    processes, streams = [], []
    try:
        commands = [
            ['ros2', 'launch', 'mid360_bringup', 'mid360_mapping.launch.py',
             'driver:=false', 'use_sim_time:=true', 'rviz:=false', 'monitor:=false',
             'robot_model:=' + profile, 'config_file:=' + str(config),
             'map_output_dir:=' + str(args.output)],
        ]
        for index, command in enumerate(commands):
            stream = (args.output / f'{label}_launch_{index}.log').open('w')
            streams.append(stream)
            processes.append(subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                               start_new_session=True))
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        stream = (args.output / f'{label}_bag.log').open('w'); streams.append(stream)
        player = subprocess.Popen(['ros2', 'bag', 'play', str(args.bag), '--clock', '100',
                                   '--topics', '/lidar_points', '/imu'],
                                  stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(player)
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            assert processes[0].poll() is None, 'sensor launch exited; inspect logs'
            rclpy.spin_once(node, timeout_sec=.02)
        assert len(odom) >= 50 and counts['local_map'] >= 3, (len(odom), counts)
        expected_children = {'base_link', 'lidar'} if profile == 'go2' else {'imu_link', 'lidar'}
        assert set(static) == expected_children
        if profile == 'go2':
            assert static['base_link'][0] == 'imu_link' and static['lidar'][0] == 'base_link'
            mount = yaml.safe_load((Path(get_package_share_directory('mid360_bringup'))/'config/go2_mount.yaml').read_text())
            imu_base, base_lidar = go2_transforms([-.011, -.02329, .04412], np.eye(3), mount)
            np.testing.assert_allclose(static['base_link'][1], imu_base, atol=1e-12)
            np.testing.assert_allclose(static['lidar'][1], base_lidar, atol=1e-12)
        keys = sorted(set(odom) & set(clouds) & set(body_clouds))
        assert len(keys) >= 50, 'missing synchronized clouds'
        error = 0.
        for key in keys:
            expected = body_clouds[key] @ odom[key][:3, :3].T + odom[key][:3, 3]
            error = max(error, float(np.max(np.abs(expected-clouds[key]))))
        assert error < 1e-4, f'body/world point inconsistency: {error}'
        latest = buffer.lookup_transform('odom', 'lidar', Time())
        latest_body = buffer.lookup_transform('odom', body, Time())
        imu_lidar = np.eye(4); imu_lidar[:3, 3] = [-.011, -.02329, .04412]
        np.testing.assert_allclose(matrix(latest.transform), matrix(latest_body.transform) @ imu_lidar, atol=1e-9)
        summary = dict(odom=len(odom), synchronized_clouds=len(keys), local_maps=counts['local_map'],
                       max_world_cloud_error_m=error,
                       static_edges=[parent+' -> '+child for child,(parent,_) in static.items()])
        np.savez(args.output/f'{label}_imu_poses.npz', stamps=list(odom), poses=list(odom.values()))
        print(label, json.dumps(summary), flush=True)
        return odom, summary
    finally:
        for process in reversed(processes): stop(process)
        for stream in streams: stream.close()
        del listener
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, default=Path('data/bags/wide_joint_repeat_20261124'))
    parser.add_argument('--output', type=Path, default=Path('log/go2_mount_validation'))
    parser.add_argument('--seconds', type=float, default=25.)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '89':
        raise SystemExit('Use ROS_DOMAIN_ID=89 to isolate replay from live systems')
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'result.json').unlink(missing_ok=True)
    share = Path(get_package_share_directory('mid360_bringup'))
    config = yaml.safe_load((share/'config/mid360_robot.yaml').read_text())
    params = config['/**']['ros__parameters']
    params['publish'].update(dense_publish_en=True, map_en=False, path_en=False)
    params['pcd_save']['pcd_save_en'] = False
    params['runtime_pos_log_enable'] = False
    config_file = args.output.resolve()/'replay_config.yaml'
    config_file.write_text(yaml.safe_dump(config))
    rclpy.init()
    try:
        old, baseline = run_profile(args, 'imu', config_file)
        repeat, repeated_baseline = run_profile(args, 'imu', config_file, 'imu_repeat')
        new, go2 = run_profile(args, 'go2', config_file)

        def difference(first, second):
            common = sorted(set(first) & set(second))
            assert len(common) >= 50, 'too few matching replay timestamps'
            position = max(float(np.linalg.norm(first[k][:3, 3]-second[k][:3, 3])) for k in common)
            angle = max(float(Rotation.from_matrix(first[k][:3, :3].T @ second[k][:3, :3]).magnitude()) for k in common)
            return dict(common_stamps=len(common), max_position_m=position, max_angle_rad=angle)

        replay_noise = difference(old, repeat)
        changed = difference(old, new)
        # FAST-LIO uses parallel map updates; repeated replay is not bit-identical.
        # 2 mm / 0.05 degrees are well below the navigation grid resolution.
        for comparison in (replay_noise, changed):
            assert comparison['max_position_m'] < .002, comparison
            assert comparison['max_angle_rad'] < np.deg2rad(.05), comparison
        summary = dict(baseline=baseline, repeated_baseline=repeated_baseline, go2=go2,
                       baseline_repeat_difference=replay_noise, profile_difference=changed,
                       tolerances=dict(position_m=.002, angle_deg=.05), result='PASS')
        (args.output/'result.json').write_text(json.dumps(summary, indent=2)+'\n')
        print(json.dumps(summary, indent=2), flush=True)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
