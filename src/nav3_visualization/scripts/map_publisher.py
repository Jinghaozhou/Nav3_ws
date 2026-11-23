#!/usr/bin/env python3
"""Publish the small offline visualization PCD as a latched map cloud."""

from pathlib import Path
import signal
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


DTYPE = np.dtype([
    ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
    ("rgb", "<f4"), ("region_id", "<u4"),
])


def read_pcd(path):
    with path.open("rb") as stream:
        values = {}
        for _ in range(100):
            raw = stream.readline(4096)
            if not raw or len(raw) == 4096 and not raw.endswith(b"\n"):
                raise ValueError("无效的可视化 PCD 文件头")
            if raw.startswith(b"#") or not raw.strip():
                continue
            parts = raw.decode("ascii").split()
            values[parts[0].upper()] = parts[1:]
            if parts[0].upper() == "DATA":
                if values.get("FIELDS") != ["x", "y", "z", "rgb", "region_id"] or \
                   values.get("SIZE") != ["4"] * 5 or \
                   values.get("TYPE") != ["F", "F", "F", "F", "U"] or \
                   values.get("DATA") != ["binary"]:
                    raise ValueError("可视化 PCD 格式与生成脚本不匹配")
                count = int(values["POINTS"][0])
                if count < 1 or count > 1000000:
                    raise ValueError("可视化地图点数必须在 1 至 100 万之间")
                data = stream.read(count * DTYPE.itemsize)
                if len(data) != count * DTYPE.itemsize:
                    raise ValueError("可视化 PCD 数据不完整")
                return count, data
    raise ValueError("缺少 PCD DATA 字段")


class MapPublisher(Node):
    def __init__(self):
        super().__init__("nav3_visual_map_publisher")
        path = Path(self.declare_parameter("map_pcd", "").value).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"可视化地图不存在：{path}；先运行 prepare_visual_map.py")
        count, data = read_pcd(path)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(PointCloud2, "/nav3/viz/global_map", qos)
        self.message = PointCloud2()
        self.message.header.frame_id = "map"
        self.message.height = 1
        self.message.width = count
        self.message.fields = [
            PointField(name=name, offset=offset, datatype=datatype, count=1)
            for name, offset, datatype in (
                ("x", 0, PointField.FLOAT32), ("y", 4, PointField.FLOAT32),
                ("z", 8, PointField.FLOAT32), ("rgb", 12, PointField.FLOAT32),
                ("region_id", 16, PointField.UINT32))
        ]
        self.message.is_bigendian = False
        self.message.point_step = DTYPE.itemsize
        self.message.row_step = count * DTYPE.itemsize
        self.message.is_dense = True
        self.message.data = data
        self.timer = self.create_timer(1.0, self.publish_once)
        self.get_logger().info(f"Loaded {count} visualization points from {path}")

    def publish_once(self):
        self.message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.message)
        self.timer.cancel()
        self.get_logger().info("Published visualization map in map frame")


def main():
    stopped = threading.Event()
    signals = (signal.SIGINT, signal.SIGTERM)
    previous = {number: signal.getsignal(number) for number in signals}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for number in signals:
        signal.signal(number, lambda _signum, _frame: stopped.set())
    node = None
    try:
        node = MapPublisher()
        while rclpy.ok() and not stopped.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        for number, handler in previous.items():
            signal.signal(number, handler)


if __name__ == "__main__":
    main()
