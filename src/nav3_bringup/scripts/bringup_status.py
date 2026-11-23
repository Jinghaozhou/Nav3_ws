#!/usr/bin/env python3
"""Print one concise readiness line per Nav3 module."""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--driver", choices=("true", "false"), required=True)
    parser.add_argument("--initialization-source", choices=("bbs", "seeds"), required=True)
    parser.add_argument("--chassis", choices=("true", "false"), required=True)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    expected = {
        "传感器": {"/laser_mapping", "/local_map_window"}
        | ({"/livox_lidar_publisher"} if args.driver == "true" else set()),
        "定位": {"/localization/navigation_localizer"}
        | ({"/bbs_retriever"} if args.initialization_source == "bbs" else set()),
        "PCT 全局规划": {"/pct_planner"},
        "SCAN 局部规划": {"/scan_planner", "/scan_controller"},
        "导航任务管理": {"/nav3_navigation_manager"},
    }
    if args.chassis == "true":
        expected["Go2 底盘桥接"] = {"/go2_cmd_vel_bridge"}

    rclpy.init()
    node = Node("nav3_bringup_status")
    permission = [False]
    ever_allowed = [False]

    def on_permission(message):
        if message.data and not permission[0]:
            print("[Nav3] 定位：首次定位成功，导航许可已开启" if not ever_allowed[0]
                  else "[Nav3] 定位：导航许可已恢复", flush=True)
            ever_allowed[0] = True
        elif not message.data and permission[0]:
            print("[Nav3] 定位：导航许可已撤销", flush=True)
        permission[0] = message.data

    node.create_subscription(Bool, "/localization/navigation_allowed", on_permission, 10)
    print("[Nav3] 正在加载传感器、定位、PCT、SCAN 和导航任务管理器", flush=True)
    if args.chassis == "false":
        print("[Nav3] Go2 底盘桥接：未启用", flush=True)
    state = {}
    start = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            names = {namespace.rstrip("/") + "/" + name
                     for name, namespace in node.get_node_names_and_namespaces()}
            for label, required in expected.items():
                missing = required - names
                if not missing:
                    next_state = "loaded"
                elif state.get(label) == "loaded":
                    next_state = "exited"
                elif time.monotonic() - start > (200 if label == "导航任务管理" else 30):
                    next_state = "missing"
                else:
                    next_state = "waiting"
                if next_state == state.get(label):
                    continue
                state[label] = next_state
                if next_state == "loaded":
                    suffix = "（雷达驱动关闭）" if label == "传感器" and args.driver == "false" else ""
                    print(f"[Nav3] {label}：节点已加载{suffix}", flush=True)
                elif next_state in ("exited", "missing"):
                    print(f"[Nav3] {label}：节点未就绪，缺少 {', '.join(sorted(missing))}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
