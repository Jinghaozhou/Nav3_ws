"""Record selected Nav3 topics without recording large derived global maps."""

from datetime import datetime
from pathlib import Path
import re

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def _start_recording(context):
    value = lambda name: LaunchConfiguration(name).perform(context).strip()
    config_path = Path(value("topics_file")).expanduser().resolve()
    if not config_path.is_file():
        raise RuntimeError(f"录包话题配置不存在：{config_path}")
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise RuntimeError(f"录包话题配置格式无效：{config_path}")
    groups, profiles = config.get("groups"), config.get("profiles")
    if not isinstance(groups, dict) or not isinstance(profiles, dict):
        raise RuntimeError("录包话题配置需要 groups 和 profiles 映射")
    profile = value("profile")
    selected = profiles.get(profile)
    if not isinstance(selected, list) or not selected:
        raise RuntimeError(f"未知或空的录包配置：{profile}")
    topics = []
    for group_name in selected:
        group_topics = groups.get(group_name)
        if not isinstance(group_topics, list) or not group_topics:
            raise RuntimeError(f"录包话题组无效：{group_name}")
        for topic in group_topics:
            if not isinstance(topic, str) or not re.fullmatch(r"/[A-Za-z0-9_/]+", topic):
                raise RuntimeError(f"录包话题名无效：{topic!r}")
            if topic not in topics:
                topics.append(topic)

    split_size_mib = value("split_size_mib")
    if not re.fullmatch(r"[1-9][0-9]*", split_size_mib):
        raise RuntimeError("split_size_mib 必须为正整数")
    output_root = Path(value("output_dir")).expanduser().resolve()
    if output_root.exists() and not output_root.is_dir():
        raise RuntimeError(f"录包输出路径不是目录：{output_root}")
    name = value("bag_name")
    if name == "auto":
        name = "nav3_" + datetime.now().strftime("%Y%m%d_%H%M%S")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]*", name) or name in (".", ".."):
        raise RuntimeError("bag_name 只能包含字母、数字、点、下划线和短横线")
    output = output_root / name
    if output.exists():
        raise RuntimeError(f"录包目录已存在，请换 bag_name：{output}")
    output_root.mkdir(parents=True, exist_ok=True)

    cmd = [
        "ros2", "bag", "record", "--output", str(output),
        "--storage", "sqlite3",
        "--max-bag-size", str(int(split_size_mib) * 1024 * 1024),
        "--include-hidden-topics",
    ]
    if value("use_sim_time") == "true":
        cmd.append("--use-sim-time")
    cmd.extend(topics)
    recorder = ExecuteProcess(cmd=cmd, output="screen")
    return [
        LogInfo(msg=f"录包配置={profile}；输出={output}；话题数={len(topics)}"),
        RegisterEventHandler(OnProcessExit(
            target_action=recorder,
            on_exit=[EmitEvent(event=Shutdown(reason="录包进程已退出"))],
        )),
        recorder,
    ]


def generate_launch_description():
    root = Path(get_package_prefix("nav3_bag_recorder")).parent
    share = Path(get_package_share_directory("nav3_bag_recorder"))
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="integration",
                              choices=["sensor", "integration", "diagnostic"]),
        DeclareLaunchArgument("topics_file", default_value=str(share / "config/topics.yaml")),
        DeclareLaunchArgument("output_dir", default_value=str(root / "data/bags")),
        DeclareLaunchArgument("bag_name", default_value="auto"),
        DeclareLaunchArgument("split_size_mib", default_value="2048"),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=_start_recording),
    ])
