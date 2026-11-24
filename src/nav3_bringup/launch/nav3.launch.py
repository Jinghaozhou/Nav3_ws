"""Start the Nav3 navigation modules, manager, and optional Go2 bridge."""

import csv
from datetime import datetime
import hashlib
import logging
import math
import os
from pathlib import Path
import sys

import yaml
import launch.logging
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


class _BringupConsoleFilter(logging.Filter):
    """Show readiness and localization while retaining detailed ROS logs."""

    stopping = False

    def filter(self, record):
        name = record.name
        if name.endswith(("-stdout", "-stderr")):
            if any(level in record.getMessage() for level in ("[ERROR]", "[FATAL]")):
                return not self.stopping
            return name.startswith((
                "navigation_localizer-", "bbs_retriever-",
                "bringup_status.py-",
            ))
        return record.levelno >= logging.ERROR and not self.stopping


def _configure_console():
    _BringupConsoleFilter.stopping = False
    handler = launch.logging.launch_config.get_screen_handler()
    if not any(isinstance(item, _BringupConsoleFilter) for item in handler.filters):
        handler.addFilter(_BringupConsoleFilter())


def _on_shutdown(_event, _context):
    _BringupConsoleFilter.stopping = True
    return []


def _digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def _launch(context):
    value = lambda name: LaunchConfiguration(name).perform(context).strip()
    map_file = Path(value("map_pcd")).expanduser().resolve()
    if not map_file.is_file() or map_file.stat().st_size == 0:
        raise RuntimeError(f"统一全局地图不存在或为空：{map_file}")

    region = value("initial_region")
    initialization_source = value("initialization_source")
    if initialization_source not in ("bbs", "seeds"):
        raise RuntimeError("initialization_source 只能是 bbs 或 seeds")
    seed_setting = value("region_seed_file")
    mode = value("region_multistart_enabled").lower()
    if mode not in ("auto", "true", "false"):
        raise RuntimeError("region_multistart_enabled 只能是 auto、true 或 false")
    localization_share = Path(get_package_share_directory("kiss_matcher_localization"))
    sensor_tools = (Path(get_package_prefix("nav3_bringup")).parent
                    / "modules/Nav3_Sensor_ws/tools/mid360_lidar")
    if seed_setting == "auto":
        if region in ("space_10", "10") and initialization_source == "bbs":
            seed_file = sensor_tools / "config/region_initialization_bbs_space_10.yaml"
        elif region in ("space_2", "2"):
            seed_file = localization_share / "config/region_initialization_space.yaml"
        else:
            seed_file = None
    else:
        seed_file = Path(seed_setting).expanduser().resolve() if seed_setting else None
    multistart = (seed_file is not None) if mode == "auto" else mode == "true"
    if initialization_source == "bbs" and not multistart:
        raise RuntimeError("BBS 初始化需要匹配区域的 region_seed_file 和启用多候选初始化")
    if multistart:
        if seed_file is None or not seed_file.is_file():
            raise RuntimeError("已启用区域候选，但 region_seed_file 不存在")
        profile = yaml.safe_load(seed_file.read_text(encoding="utf-8"))
        if not isinstance(profile, dict):
            raise RuntimeError(f"区域候选配置无效：{seed_file}")
        regions = profile.get("regions", [])
        if not region or not any(region in (str(item.get("region_name")),
                                          str(item.get("region_id")))
                                 for item in regions if isinstance(item, dict)):
            raise RuntimeError(f"区域 {region!r} 与候选文件 {seed_file} 不匹配")
        expected_hash = profile.get("map_sha256")
        if expected_hash and _digest(map_file) != expected_hash:
            raise RuntimeError(f"地图哈希与区域候选文件不匹配：{seed_file}")

    labels_file = localization_share / "maps/region_labels.csv"
    localization_config = localization_share / "config/continuous_localization.yaml"
    for required in (labels_file, localization_config):
        if not required.is_file():
            raise RuntimeError(f"定位配置缺失：{required}")
    if region:
        with labels_file.open(newline="", encoding="utf-8") as stream:
            known_regions = {
                name for row in csv.DictReader(stream)
                for name in (row["region_id"], row["region_name"])
            }
        if region not in known_regions:
            raise RuntimeError(f"未知首次启动区域：{region}；可使用区域名称或编号")

    timeout = float(value("pct_startup_timeout_sec"))
    if not math.isfinite(timeout) or timeout <= 0:
        raise RuntimeError("pct_startup_timeout_sec 必须为正数")
    if value("driver") == "true" and value("use_sim_time") == "true":
        raise RuntimeError("实体 MID360 驱动不可与 use_sim_time:=true 同时启动")
    chassis_enabled = value("chassis") == "true"
    chassis_dry_run = value("chassis_dry_run") == "true"
    if chassis_enabled and not chassis_dry_run and value("use_sim_time") == "true":
        raise RuntimeError("回放仿真时间时不可连接实体底盘；使用 chassis_dry_run:=true")
    visual_map = Path(value("visualization_map_pcd")).expanduser().resolve()
    if value("visualization") == "true" and not visual_map.is_file():
        raise RuntimeError(f"可视化地图不存在：{visual_map}；先运行 nav3_visualization 的降采样脚本")

    def include(package, file, arguments, yaml_launch=False):
        source = Path(get_package_share_directory(package)) / "launch" / file
        return IncludeLaunchDescription(
            AnyLaunchDescriptionSource(str(source)) if yaml_launch
            else PythonLaunchDescriptionSource(str(source)),
            launch_arguments=arguments.items(),
        )

    common_time = value("use_sim_time")
    timing_dir = None
    if value("timing_diagnostics") == "true":
        root = Path(get_package_prefix("nav3_bringup")).parent
        timing_dir = root / "log" / "timing" / (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + f"_{os.getpid()}")
        timing_dir.mkdir(parents=True, exist_ok=False)
        print(f"[Nav3] 时间诊断记录：{timing_dir}", flush=True)
    sensor = include("mid360_bringup", "mid360_mapping.launch.py", {
        "driver": value("driver"), "use_sim_time": common_time,
        "rviz": value("rviz"), "keyframe_candidates": "false",
        "local_map_timing_csv": str(timing_dir / "local_map_publish.csv") if timing_dir else "",
    })
    if initialization_source == "bbs":
        bbs_setting = value("bbs_config")
        if bbs_setting == "auto":
            if region in ("space_10", "10"):
                bbs_config = sensor_tools / "config/bbs_initialization_space_10.json"
            elif region in ("space_2", "2"):
                bbs_config = sensor_tools / "config/bbs_initialization.json"
            else:
                raise RuntimeError(f"区域 {region!r} 没有默认 BBS 配置；请指定 bbs_config")
        else:
            bbs_config = Path(bbs_setting).expanduser().resolve()
        retriever_script = sensor_tools / "scripts/bbs_retriever.py"
        sensor_config = sensor_tools / "config/continuous_localization.yaml"
        for required in (bbs_config, retriever_script, sensor_config):
            if not required.is_file():
                raise RuntimeError(f"BBS 初始化文件不存在：{required}")
        sys.path.insert(0, str(sensor_tools / "scripts"))
        from bbs_database import BbsDatabase
        from keyframe_database import validate_sensor_config
        from prepare_race_map_cache import ensure_cache

        database = BbsDatabase(map_file, bbs_config)
        try:
            validate_sensor_config(sensor_tools / "mid360_robot.yaml", database)
            bbs_parameters = {
                "map_cache_directory": str(ensure_cache(map_file, labels_file)),
                "keyframe_library_sha256": database.library_sha,
                "keyframe_profile_sha256": database.profile_sha,
                "keyframe_region_id": str(database.region_id),
                "keyframe_expected_candidates": str(len(database.index)),
                "hierarchical_min_initial_overlap": str(
                    database.search_config["minimum_initial_overlap_percent"]),
            }
        finally:
            database.close()
        retriever = ExecuteProcess(
            cmd=["/usr/bin/python3", str(retriever_script),
                 "--config", str(bbs_config), "--map", str(map_file),
                 "--ros-args", "-p", f"use_sim_time:={common_time}"],
            name="bbs_retriever",
            output="screen",
            on_exit=[EmitEvent(event=Shutdown(reason="BBS 检索器退出"))],
        )
        localization = include("kiss_matcher_localization",
                               "navigation_localization.launch.yaml", {
            "map_pcd": str(map_file), "region_labels_csv": str(labels_file),
            "config_path": str(sensor_config),
            "local_map_topic": "/fast_lio/local_map",
            "local_map_reliability": "reliable",
            "initial_region": region,
            "region_multistart_enabled": "true",
            "region_seed_file": str(seed_file),
            "candidate_source": "bbs",
            "keyframe_initialization_enabled": "true",
            "record_inputs": "false",
            "timing_diagnostics_csv": str(timing_dir / "localization_timing.csv") if timing_dir else "",
            "navigation_permission_allow_after_initialization": value("localization_permission_after_initialization"),
            **bbs_parameters,
        }, yaml_launch=True)
        localization_actions = [retriever, localization]
    else:
        localization = include("kiss_matcher_localization",
                               "navigation_localization.launch.yaml", {
            "map_pcd": str(map_file), "region_labels_csv": str(labels_file),
            "config_path": str(localization_config),
            "local_map_topic": "/fast_lio/local_map",
            "initial_region": region,
            "region_multistart_enabled": str(multistart).lower(),
            "region_seed_file": str(seed_file) if multistart else "",
            "timing_diagnostics_csv": str(timing_dir / "localization_timing.csv") if timing_dir else "",
            "navigation_permission_allow_after_initialization": value("localization_permission_after_initialization"),
        }, yaml_launch=True)
        localization_actions = [localization]
    # The PCT package YAML has a node-specific map setting. A later wildcard
    # parameter file from its standalone launch does not override that setting.
    pct_config = Path(get_package_share_directory("pct_planner")) / "config/pct_planner.yaml"
    with pct_config.open(encoding="utf-8") as stream:
        pct_parameters = yaml.safe_load(stream)["pct_planner"]["ros__parameters"]
    pct_parameters.update({
        "map": map_file.name,
        "map_dir": str(map_file.parent),
        "use_sim_time": common_time == "true",
    })
    pct = Node(
        package="pct_planner", executable="pct_planner_node",
        name="pct_planner", output="screen", parameters=[pct_parameters],
    )
    collision_window = float(value("scan_collision_window_sec"))
    if not math.isfinite(collision_window) or collision_window < 0:
        raise RuntimeError("scan_collision_window_sec must be finite and nonnegative")
    if collision_window > 0:
        localization_actions.append(Node(package="mid360_bringup", executable="local_map_window",
            name="collision_map_window", output="screen", parameters=[{"use_sim_time": common_time == "true"}],
            arguments=["--window-time", str(collision_window), "--tracking-window-time", str(collision_window),
                       "--window-frames", str(max(30, math.ceil(collision_window * 15))),
                       "--voxel", "0.1", "--rate", "5.0",
                       "--in-topic", "/fast_lio/local_cloud", "--out-topic", "/fast_lio/collision_map",
                       "--global-topic", "/fast_lio/collision_map_global",
                       "--window-status-topic", "/fast_lio/collision_window_status"]))
    scan = include("scan_planner", "scan_planner.launch.py", {
        "use_sim_time": common_time,
        "path_topic": "/nav3/managed_path",
        "local_map_topic": "/fast_lio/collision_map" if collision_window > 0 else "/fast_lio/local_map",
        "planning_frame": "nav_odom" if value("scan_gravity_aligned") == "true" else "odom",
        "cmd_vel_topic": "/cmd_vel",
        "unknown_space_policy": value("scan_unknown_space_policy"),
        "check_local_collision": value("scan_check_local_collision"),
        "map_timeout": value("scan_map_timeout"),
        "local_map_reliability": "reliable",
        "override_config": value("scan_override_config"),
        "require_task_heartbeat": "true",
        "path_z_offset": value("scan_path_z_offset"),
        "require_localization_permission": "true",
        "localization_permission_topic": "/localization/navigation_allowed",
    })
    if value("scan_gravity_aligned") == "true":
        localization_actions.append(Node(package="nav3_bringup", executable="gravity_frame.py",
                                         parameters=[{"use_sim_time": common_time == "true"}], output="screen"))
    manager = include("nav3_navigation_manager", "navigation_manager.launch.py", {
        "use_sim_time": common_time,
        "bypass_checks": value("manager_bypass_checks"),
        "start_z_override_enabled": value("manager_start_z_override_enabled"),
        "start_z_override_m": value("manager_start_z_override_m"),
        "config_file": str(Path(get_package_share_directory("nav3_navigation_manager"))
                           / "config/navigation_manager.yaml"),
    })
    prewarm = Node(
        package="nav3_bringup", executable="prewarm_pct.py",
        output="screen", arguments=["--timeout", value("pct_startup_timeout_sec")],
    )
    status = Node(
        package="nav3_bringup", executable="bringup_status.py",
        output="screen", arguments=[
            "--driver", value("driver"),
            "--initialization-source", initialization_source,
            "--chassis", value("chassis"),
        ],
    )

    chassis_actions = []
    if chassis_enabled:
        chassis_config = Path(value("chassis_config")).expanduser().resolve()
        if not chassis_config.is_file():
            raise RuntimeError(f"底盘配置文件不存在：{chassis_config}")
        chassis_sdk_root = Path(value("chassis_sdk_root")).expanduser().resolve()
        if not chassis_dry_run and not (chassis_sdk_root / "vendor/unitree_sdk2_python").is_dir():
            raise RuntimeError(f"底盘 SDK 目录不存在：{chassis_sdk_root}")
        bridge = Node(
            package="go2control", executable="cmd_vel_bridge",
            name="go2_cmd_vel_bridge", output="screen",
            parameters=[str(chassis_config), {
                "interface": value("chassis_interface"),
                "sdk_root": str(chassis_sdk_root),
                "dry_run": chassis_dry_run,
            }],
            remappings=[("cmd_vel", "/cmd_vel")],
        )
        chassis_actions = [bridge]

    def after_prewarm(event, _context):
        if event.returncode == 0:
            return [GroupAction(scoped=True, actions=[manager, *chassis_actions])]
        print("[Nav3] PCT 全局规划：地图准备失败", flush=True)
        return [EmitEvent(event=Shutdown(reason="PCT 地图准备失败"))]

    actions = [
        SetParameter(name="use_sim_time", value=common_time == "true"),
        status,
        GroupAction(scoped=True, actions=[sensor]),
        GroupAction(scoped=True, actions=localization_actions),
        pct,
        GroupAction(scoped=True, actions=[scan]),
    ]
    if value("visualization") == "true":
        actions.append(GroupAction(scoped=True, actions=[include(
            "nav3_visualization", "navigation_visualization.launch.py",
            {"map_pcd": str(visual_map), "rviz": value("visualization_rviz")},
        )]))
    if chassis_actions:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=chassis_actions[0],
            on_exit=[EmitEvent(event=Shutdown(reason="Go2 底盘桥接退出"))],
        )))
    actions.extend([
        RegisterEventHandler(OnProcessExit(
            target_action=prewarm, on_exit=after_prewarm)),
        prewarm,
    ])
    return actions


def generate_launch_description():
    _configure_console()
    install_prefix = Path(get_package_prefix("nav3_bringup"))
    # The root build uses --merge-install; source_env.bash exports PCT_MAP_DIR.
    fallback_map_dir = install_prefix.parent / "data/maps"
    default_map_dir = Path(os.environ.get("PCT_MAP_DIR", fallback_map_dir))
    chassis_root = install_prefix.parent / "modules/Nav3_Chassis_ws"
    return LaunchDescription([
        RegisterEventHandler(OnShutdown(on_shutdown=_on_shutdown)),
        DeclareLaunchArgument("map_pcd", default_value=str(default_map_dir / "building_split_space.pcd")),
        DeclareLaunchArgument("initial_region", default_value="space_10"),
        DeclareLaunchArgument("initialization_source", default_value="bbs", choices=["bbs", "seeds"]),
        DeclareLaunchArgument("bbs_config", default_value="auto"),
        DeclareLaunchArgument("region_seed_file", default_value="auto"),
        DeclareLaunchArgument("region_multistart_enabled", default_value="auto"),
        DeclareLaunchArgument("driver", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("timing_diagnostics", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("localization_permission_after_initialization", default_value="true",
                              choices=["true", "false"]),
        DeclareLaunchArgument("manager_bypass_checks", default_value="false",
                              choices=["true", "false"]),
        DeclareLaunchArgument("manager_start_z_override_enabled", default_value="false",
                              choices=["true", "false"]),
        DeclareLaunchArgument("manager_start_z_override_m", default_value="0.0"),
        DeclareLaunchArgument("rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("visualization", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("visualization_rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("visualization_map_pcd", default_value=str(
            default_map_dir / "building_split_space_viz_space10_open.pcd")),
        DeclareLaunchArgument("chassis", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("chassis_dry_run", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("chassis_interface", default_value="eno2"),
        DeclareLaunchArgument("scan_unknown_space_policy", default_value="free_inside_bounds",
                              choices=["occupied", "free_inside_bounds"]),
        DeclareLaunchArgument("scan_check_local_collision", default_value="true",
                              choices=["true", "false"]),
        DeclareLaunchArgument("scan_map_timeout", default_value="1.5"),
        DeclareLaunchArgument("scan_override_config", default_value=""),
        DeclareLaunchArgument("scan_collision_window_sec", default_value="0.0"),
        DeclareLaunchArgument("scan_gravity_aligned", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("scan_path_z_offset", default_value="0.0"),
        DeclareLaunchArgument("chassis_sdk_root", default_value=str(chassis_root)),
        DeclareLaunchArgument("chassis_config", default_value=str(
            chassis_root / "src/go2control/config/go2control.yaml")),
        DeclareLaunchArgument("pct_startup_timeout_sec", default_value="180"),
        OpaqueFunction(function=_launch),
    ])
