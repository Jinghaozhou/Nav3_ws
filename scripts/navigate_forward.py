#!/usr/bin/env python3
"""Send a short Go2 navigation task, with cancellation and motion bounds."""
import argparse
import json
import math
import signal
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from nav3_navigation_manager.action import NavigateToPose
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--distance', type=float, default=1., help='Forward metres, 0 < distance <= 1')
    parser.add_argument('--target', type=float, nargs=3, metavar=('X', 'Y', 'Z'), help='Resume a map goal within 1 metre')
    parser.add_argument('--timeout', type=float, default=45.)
    args = parser.parse_args()
    if not 0 < args.distance <= 1 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('Require 0 < distance <= 1 and a finite positive timeout')
    if args.target and not all(math.isfinite(v) for v in args.target):
        parser.error('Target must be finite')
    stopped = [False]
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for sig in previous:
        signal.signal(sig, lambda *_: stopped.__setitem__(0, True))
    node = rclpy.create_node('nav3_short_navigation_client')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    state = ['']
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscription = node.create_subscription(String, '/nav3/navigation_state',
                                             lambda m: state.__setitem__(0, m.data), qos)
    client = ActionClient(node, NavigateToPose, '/nav3/navigate_to_pose')
    handle = result = None

    def pump_until(predicate, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and not stopped[0] and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                return True
        return False

    try:
        if not pump_until(lambda: state[0] == 'READY' and buffer.can_transform('map', 'base_link', Time()), 30):
            raise RuntimeError('Navigation did not become READY: ' + state[0])
        transform = buffer.lookup_transform('map', 'base_link', Time())
        stamp = transform.header.stamp
        age = (node.get_clock().now().nanoseconds - stamp.sec*10**9 - stamp.nanosec)/1e9
        if not -.1 <= age <= .5:
            raise RuntimeError(f'Robot TF is stale: age={age:.3f}s')
        p, q = transform.transform.translation, transform.transform.rotation
        start = [p.x, p.y, p.z]
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        target = args.target or [p.x+args.distance*math.cos(yaw), p.y+args.distance*math.sin(yaw), p.z]
        if math.dist(start, target) > 1.05:
            raise RuntimeError('This short-test client only accepts goals within 1 metre')
        goal = NavigateToPose.Goal()
        goal.target.header.frame_id = 'map'
        goal.target.header.stamp = node.get_clock().now().to_msg()
        goal.target.pose.position.x, goal.target.pose.position.y, goal.target.pose.position.z = target
        goal.target.pose.orientation.z, goal.target.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        print(json.dumps({'start': start, 'target': target}), flush=True)
        if not client.wait_for_server(timeout_sec=5):
            raise RuntimeError('Navigation action is unavailable')
        accepted = client.send_goal_async(goal)
        if not pump_until(accepted.done, 5):
            raise RuntimeError('No goal acceptance response')
        handle = accepted.result()
        if not handle.accepted:
            raise RuntimeError('Navigation goal was rejected')
        result = handle.get_result_async()
        deadline = time.monotonic() + args.timeout
        reason = ''
        while rclpy.ok() and not stopped[0] and not result.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            try:
                current = buffer.lookup_transform('map', 'base_link', Time()).transform.translation
                dx, dy = current.x-start[0], current.y-start[1]
                forward = dx*math.cos(yaw)+dy*math.sin(yaw)
                lateral = -dx*math.sin(yaw)+dy*math.cos(yaw)
                if forward < -.08 or abs(lateral) > .20 or math.hypot(dx, dy) > 1.25:
                    reason = 'Unexpected motion exceeded short-test bounds'
                    break
            except Exception:
                # The manager and SCAN independently stop on stale/missing TF.
                pass
        if result.done():
            response = result.result()
            print(json.dumps({'status': response.status, 'success': response.result.success,
                              'message': response.result.message}, ensure_ascii=False), flush=True)
            return 0 if response.result.success else 1
        raise RuntimeError(reason or ('Interrupted' if stopped[0] else 'Short navigation timed out'))
    except (Exception, KeyboardInterrupt) as error:
        print(str(error), flush=True)
        return 1
    finally:
        if handle and handle.accepted and (result is None or not result.done()):
            cancel = handle.cancel_goal_async()
            deadline = time.monotonic()+3
            while rclpy.ok() and not cancel.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
        node.destroy_node()
        rclpy.try_shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
