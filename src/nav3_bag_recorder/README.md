# Nav3 录包功能包

`nav3_bag_recorder` 使用本机 ROS 2 Humble 的 `ros2 bag record`，按固定话题清单录制联调数据。它独立于 `nav3_bringup` 启动，建议先开录包，再启动传感器或统一 bringup，便于保留首次定位过程。默认输出到主工作空间 `data/bags/`；该目录被 Git 忽略。

## 录哪些话题

话题清单在 [`config/topics.yaml`](config/topics.yaml)，分为三个档位：

| 档位 | 录制内容 | 用途 |
| --- | --- | --- |
| `sensor` | `/lidar_points`、`/imu`、`/tf`、`/tf_static`、FAST-LIO 里程计、重力、局部地图、健康状态和窗口状态，另含 `/clock`。 | 保留原始输入与传感器链路，支持之后重算里程计和定位。 |
| `integration`（默认） | `sensor` 的全部话题，加定位结果与许可、PCT 路径、任务管理路径与状态、Action 反馈与状态、SCAN 状态/轨迹及 `/cmd_vel`、Go2 桥接处理后的速度。 | 排查整条导航链路在何处失效。 |
| `diagnostic` | `integration` 的全部话题，加当前帧点云、定位选用点云、定位尝试输入、可选关键帧候选及其状态。 | 深入分析首次定位或关键帧问题；数据量更大。 |

原始 MID360 点云必须录 `/lidar_points` 的 `livox_ros_driver2/msg/CustomMsg`，不能只录 FAST-LIO 产生的普通 `PointCloud2`。`/fast_lio/local_map` 和 `/fast_lio/local_map_global` 都保留，因为定位分别使用当前窗口和长窗口。`/tf_static` 用于还原雷达安装外参；`/tf` 用于还原 `map → odom → base_link`。默认不录 `/Laser_map` 和 `/pct/tomogram_layers` 这类可重新生成的大型全局数据。

Action 的目标请求和 `/pct/plan` 服务调用不是普通 Topic，本包只录 Action 反馈/状态及已经发布的路径。需要完整复现操作者发送的目标时，另行记录目标坐标和发送时刻。`/localization/input_snapshot` 仅在定位模块开启 `publish_attempt_inputs` 后才会发布；未启用或尚未启动的话题不会阻止录包。

## 使用

先统一构建，并在新终端加载环境：

```bash
scripts/build_ros.sh --jobs 4
source /home/jinghao/Nav3_ws/scripts/source_env.bash
ros2 launch nav3_bag_recorder record.launch.py
```

然后在另一个已加载同一 ROS 环境、同一 `ROS_DOMAIN_ID` 的终端启动 `nav3_bringup`。录制终端按 `Ctrl+C` 正常结束，待写盘完成后检查：

```bash
ros2 bag info /home/jinghao/Nav3_ws/data/bags/<实际目录名>
```

常用参数：

```bash
ros2 launch nav3_bag_recorder record.launch.py profile:=sensor
ros2 launch nav3_bag_recorder record.launch.py profile:=diagnostic bag_name:=space2_initial
ros2 launch nav3_bag_recorder record.launch.py output_dir:=/另一块磁盘/nav3_bags split_size_mib:=4096
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `profile` | `integration` | `sensor`、`integration` 或 `diagnostic`。 |
| `topics_file` | 本包 `config/topics.yaml` | 自定义话题组文件。 |
| `output_dir` | 主工作空间 `data/bags/` | bag 目录的父目录。 |
| `bag_name` | `auto` | 自动使用本机时间生成 `nav3_年月日_时分秒`；自定义名称只能用 ASCII 字母、数字、点、下划线和短横线。 |
| `split_size_mib` | `2048` | 单个 SQLite bag 文件的分片阈值，单位 MiB。 |
| `use_sim_time` | `false` | 录制仿真或回放数据时设为 `true`，必须有 `/clock`，否则不会写入消息。 |

录包默认不压缩，避免在线采集时增加 CPU 负担。预计体积以实际点云频率和大小为准；现场先检查磁盘剩余空间。自定义 `bag_name` 已存在时启动会拒绝覆盖。话题自动发现保持开启，因此可以在各模块启动前开始录制。

要用原始输入重跑导航链路，可在隔离 ROS 域启动 `nav3_bringup` 并设置 `driver:=false use_sim_time:=true chassis:=false`，另一个终端只回放原始输入：

```bash
ros2 bag play /home/jinghao/Nav3_ws/data/bags/<实际目录名> \
  --clock 100 --topics /lidar_points /imu
```

回放前仍须在主工作空间准备相同的全局 PCD、定位候选和各模块版本。录包中的派生话题用于检查结果；重算时不要同时回放它们，否则会与新运行的节点重复发布。
