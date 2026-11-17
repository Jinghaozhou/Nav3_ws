#!/usr/bin/env python3
"""Initialize PCT before the navigation manager is allowed to start."""

import argparse
import sys
import time

import rclpy
from pct_planner_interfaces.srv import PublishCostMap
from rclpy.utilities import remove_ros_args


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    if args.timeout <= 0:
        parser.error("--timeout must be positive")

    rclpy.init()
    node = rclpy.create_node("nav3_pct_prewarmer")
    deadline = time.monotonic() + args.timeout
    try:
        client = node.create_client(PublishCostMap, "/pct/publish_cost_map")
        while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
            if time.monotonic() >= deadline:
                raise TimeoutError("等待 /pct/publish_cost_map 超时")

        node.get_logger().info("正在准备 PCT 全局地图")
        future = client.call_async(PublishCostMap.Request())
        remaining = max(0.0, deadline - time.monotonic())
        rclpy.spin_until_future_complete(node, future, timeout_sec=remaining)
        if not future.done():
            raise TimeoutError("PCT 地图准备超时")
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)
        node.get_logger().info(
            f"PCT 地图已准备：{response.map_signature}；{response.message}"
        )
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        node.get_logger().error(f"PCT 启动检查失败：{exc}")
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
