# Nav3 三维导航系统

Nav3 将传感器里程计、全局定位、三维全局规划和局部规划组织为四个独立的 ROS 2 Humble 工作空间。模块间通过 ROS 2 Topic、Service 和 TF 交换数据，不直接调用彼此的算法源码。本仓库用 Git 子模块固定各模块版本，并保存集成时使用的数据目录。

**当前状态：**四个模块的代码已经纳入子模块；`scripts/prepare_non_ros.sh` 和 `scripts/build_ros.sh` 分别提供非 ROS 依赖准备与主工作空间统一 ROS 构建入口。`src/nav3_bringup` 仍是预留目录，尚无总启动文件或 Navigation Manager。整机闭环仍需联调验证。

## 代码结构

```text
Nav3_ws/
├── modules/
│   ├── Nav3_Sensor_ws/          # Livox MID360、FAST-LIO、mid360_bringup
│   ├── Nav3_Localization_ws/    # 导航定位、可视化、轨迹记录和 PCD 调试发布
│   ├── Nav3_Global_planner_ws/  # PCT 核心、ROS 节点、接口包和可视化
│   └── Nav3_Local_Planner_ws/   # SCAN 核心、ROS 节点、消息包和演示
├── src/nav3_bringup/            # 预留：统一 launch 与集成配置
├── data/maps/                   # 本机全局地图，不纳入 Git
├── data/bags/                   # 本机回放数据，不纳入 Git
└── scripts/                     # 非 ROS 依赖准备、ROS 统一构建和环境加载
```

四个 `modules/*` 都是 Git 子模块，各自保留源码结构和 Git 历史。子模块内部的 `src/` 是其 ROS 2 包目录，部分模块还保留 `vendor/`、`legacy/` 或演示资源。包级细节见[传感器](modules/Nav3_Sensor_ws/README.md)、[定位](modules/Nav3_Localization_ws/README.md)、[PCT](modules/Nav3_Global_planner_ws/README.md)和[SCAN](modules/Nav3_Local_Planner_ws/README.md)说明。

## 模块划分与数据流

| 模块 | 主要 ROS 2 包 | 职责 |
| --- | --- | --- |
| 传感器与里程计 | `livox_ros_driver2`、`fast_lio`、`mid360_bringup` | 接收 MID360 点云和 IMU，输出连续里程计、当前帧点云和滑动窗口局部地图；发布 `odom → base_link`。 |
| 全局定位 | `kiss_matcher_localization`，另有可视化与记录包 | 将 `odom` 中的局部地图与全局 PCD 配准，成功应用后发布 `map → odom`。当前持续导航入口使用区域或关键帧候选加 VGICP；KISS 粗配准保留在历史演示入口。 |
| 全局规划 | `pct_planner`、`pct_planner_interfaces`、`pct_planner_viz` | 从离线 PCD 建立 Tomogram，按服务请求输出 `map` 中的三维路径。 |
| 局部规划与控制 | `scan_planner`、`scan_planner_msgs`、`scan_planner_demo` | 消费里程计、局部地图、TF 和全局路径，输出局部轨迹、状态与平面速度。 |

```text
MID360 + IMU ──> FAST-LIO ── odom / local_map ──> 定位 ── map→odom TF
                          └───────────────> SCAN <── global_path ── PCT <── 全局 PCD
                                               └── cmd_vel / status
```

核心 TF 链为 `map → odom → base_link → lidar`：定位发布 `map → odom`，FAST-LIO 发布 `odom → base_link`，传感器启动包发布机体到 IMU、雷达的静态外参。每段 TF 只应有一个发布者。当前 FAST-LIO 的 `base_link` 位于 **IMU 原点**；若机器人碰撞或底盘控制参考点不同，需要真实安装外参及相应的 SCAN `robot_frame`、`control_frame` 配置。

## 跨模块接口

下表是当前组合启动的默认接口。话题名、坐标帧和部分 QoS 可通过各模块参数调整。

| 接口 | ROS 类型 | 发布者 → 消费者 | 内容与约定 |
| --- | --- | --- | --- |
| `/lidar_points` | `livox_ros_driver2/msg/CustomMsg` | MID360 → FAST-LIO | 带逐点时间的原始点云，不是标准 `PointCloud2`。 |
| `/imu` | `sensor_msgs/msg/Imu` | MID360 → FAST-LIO | `imu_link` 中的惯性测量。 |
| `/fast_lio/odom` | `nav_msgs/msg/Odometry` | FAST-LIO → SCAN | `header.frame_id=odom`、`child_frame_id=base_link`。 |
| `/fast_lio/local_cloud` | `sensor_msgs/msg/PointCloud2` | FAST-LIO → 可选关键帧检索器 | `odom` 中配准后的**当前帧**，不能代替完整局部地图。 |
| `/fast_lio/local_map` | `sensor_msgs/msg/PointCloud2` | FAST-LIO → 定位、SCAN | `odom` 中的完整滑动窗口快照，默认约 1 Hz。 |
| `/fast_lio/local_map_global` | `sensor_msgs/msg/PointCloud2` | FAST-LIO → 定位 | 始终使用较长窗口，供可选初始化与恢复。 |
| `/localization/status` | `kiss_matcher_localization/msg/LocalizationResult` | 定位 → 监控、可选窗口切换 | 区分本次 `matched` 与已更新 TF 的 `applied`；Reliable、Transient Local。 |
| `/localization/keyframe_candidates` | `kiss_matcher_localization/msg/KeyframeCandidates` | 可选检索器 → 定位 | 短时点云与多个 `map → odom` 候选，含地图和库的哈希。 |
| `/pct/global_path` | `nav_msgs/msg/Path` | PCT → SCAN | `map` 中的三维路径；Reliable、Transient Local；空路径撤销旧任务。 |
| `/scan_planner/status` | `scan_planner_msgs/msg/PlannerStatus` | SCAN → 监控/任务层 | 20 Hz 状态心跳，含运动许可和失败原因。 |
| `/scan_planner/local_path` | `nav_msgs/msg/Path` | SCAN → 可视化 | `odom` 中的局部轨迹，失效时清空。 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | SCAN → 底盘接口 | `control_frame` 中的平面 `vx/vy/wz`；底盘仍需自身的命令超时停车。 |
| `/localization/relocalize` | `kiss_matcher_localization/srv/Relocalize` | 操作者/任务层 → 定位 | 对最新局部地图尝试定位；`apply_if_safe` 控制合格结果是否应用。 |
| `/pct/plan` | `pct_planner_interfaces/srv/Plan` | 操作者/任务层 → PCT | 接收 `map` 中的起终点，返回成功标志、路径及说明。 |

定位另提供 `/localization/set_region` 和 `/localization/accept_candidate`；PCT 提供 `/pct/publish_cost_map`；传感器提供 `/map_save`。SCAN 可选订阅 `/scan_planner/localization_permission`（`std_msgs/msg/Bool`），由独立适配层根据定位质量控制运行许可。完整接口与 QoS 见[定位协议](modules/Nav3_Localization_ws/src/localization/NAVIGATION_INTERFACE.md)、[PCT 说明](modules/Nav3_Global_planner_ws/README.md)及[SCAN 协议](modules/Nav3_Local_Planner_ws/src/scan_planner/docs/interfaces.md)。

### 关键自定义消息结构

| 类型 | 关键字段 | 用途 |
| --- | --- | --- |
| `LocalizationResult` | `matched`、`applied`、`reason`、`candidate_map_to_odom`、`last_success_stamp`、`overlap_percent`、`candidate_id` | 区分候选匹配、TF 应用与上一次成功时间；只有 `delta_valid` 为真时位姿变化量才有效。 |
| `KeyframeCandidates` | `cloud`、`map_to_odom[]`、`ids[]`、`scores[]`、`region_id`、`map_sha256`、`library_sha256`、`profile_sha256` | 可选关键帧检索输入，定位节点校验地图、库和配置版本。 |
| `Plan` 服务 | 请求 `start`、`goal`；响应 `success`、`path`、`message` | 起终点均为 `PoseStamped`，`frame_id` 必须为 `map`。 |
| `PlannerStatus` | `path_id`、`traj_id`、`state`、`motion_allowed`、`reason`、`goal` | 状态包括等待、运行、到达、规划失败、定位不可用、数据过期和 TF 大幅修正。 |

精确定义分别位于 [`localization/msg`、`srv`](modules/Nav3_Localization_ws/src/localization/)、[`pct_planner_interfaces`](modules/Nav3_Global_planner_ws/src/pct_planner_interfaces/)和[`scan_planner_msgs`](modules/Nav3_Local_Planner_ws/src/scan_planner_msgs/)。

## 地图与数据

`data/maps/building_split_space.pcd` 是本机当前选定的统一全局参考地图，含 `region_id` 标签；文件不纳入 Git。其 SHA-256 为 `c84fba59d8c6342aa72c5c53599152295e27948ff0e3317d95fd9f3c12c1ab55`，与定位模块的 `space_2` 区域初始候选及传感器包内 40 帧关键帧库记录的地图哈希一致。区域名称表和 15 个初始候选已在定位子模块中；候选基于特定安装姿态的实验数据，仍需现场核验。

定位 launch 可通过 `map_pcd` 指向主项目地图；传感器启用关键帧候选时，可通过 `keyframe_map` 指向同一文件；PCT 通过 `map_dir` 与 `map` 选择 PCD。SCAN 正式运行不直接读取全局 PCD。**尚未验证 PCT 对这份约 880 MiB 地图的加载时间、内存占用及生成路径与定位的几何对齐。**如需规划用抽稀地图，必须保留同一 `map` 坐标系并记录其与原图的派生关系。子模块内的 `building2_9.pcd`、`3_part.pcd` 等属于独立演示或历史场景，不应与当前定位候选混用。

`data/bags/` 用于本机回放数据。地图、bag、构建结果均不随主仓库提交；换机部署时需要单独准备，并核对地图哈希和坐标定义。

## 参数与启动入口

| 模块 | 配置文件及启动入口 | 集成时优先核对的参数 |
| --- | --- | --- |
| 传感器 | [`mid360_robot.yaml`](modules/Nav3_Sensor_ws/src/mid360_bringup/config/mid360_robot.yaml)、[`MID360_config.json`](modules/Nav3_Sensor_ws/src/livox_ros_driver2/config/MID360_config.json)、[`mid360_mapping.launch.py`](modules/Nav3_Sensor_ws/src/mid360_bringup/launch/mid360_mapping.launch.py) | `driver`、`use_sim_time`、`config_file`、`driver_config`、`window_time`（默认 120 s）、`tracking_window_time`（默认 30 s）、`adaptive_window`、`keyframe_candidates`、`keyframe_map`。组合启动话题名在 YAML 的 `mid360_bringup.ros__parameters.topics` 中设置。 |
| 定位 | [`continuous_localization.yaml`](modules/Nav3_Localization_ws/src/localization/config/continuous_localization.yaml)、[`navigation_localization.launch.yaml`](modules/Nav3_Localization_ws/src/localization/launch/navigation_localization.launch.yaml) | `map_pcd`、`region_labels_csv`、`initial_region`、`region_multistart_enabled`、`region_seed_file`、`local_map_topic`、`local_map_reliability`、`config_path`。连续配置含重叠率、重试周期和 TF 修正门槛。 |
| PCT | [`pct_planner.yaml`](modules/Nav3_Global_planner_ws/src/pct_planner/config/pct_planner.yaml)、[`pct_planner.launch.py`](modules/Nav3_Global_planner_ws/src/pct_planner/launch/pct_planner.launch.py) | `map`、`map_dir`、`cache_dir`、`resolution`、`ground_h`、`slice_dh`、机器人净空、坡度、台阶与安全膨胀参数。默认地图仍是 `building2_9.pcd`，集成时须显式覆盖。 |
| SCAN | [`integration.yaml`](modules/Nav3_Local_Planner_ws/src/scan_planner/config/integration.yaml)、`planner.yaml`、`controller.yaml`、[`scan_planner.launch.py`](modules/Nav3_Local_Planner_ws/src/scan_planner/launch/scan_planner.launch.py) | `odom_topic`、`local_map_topic`、`path_topic`、`cmd_vel_topic`、四个 frame、`velocity_source`、`path_height_mode`、`unknown_space_policy`、`require_localization_permission`。 |

SCAN 的 `unknown_space_policy` 默认为 `occupied`；只有点云占据证据而没有自由空间证据时，保守配置会保持零速度。`free_inside_bounds` 需在确认地图覆盖、可通行性和停车能力后显式选择。回放时所有参与节点应统一设置 `use_sim_time`，并使用相同的 `ROS_DOMAIN_ID`。

## 非 ROS 依赖准备

在主工作空间根目录运行：

```bash
git submodule update --init --recursive
scripts/prepare_non_ros.sh --jobs 4
scripts/prepare_non_ros.sh --check
```

默认模式按顺序增量构建 Livox SDK2、定位算法库 ROBIN / KISS Matcher / small_gicp，以及 PCT 的 GTSAM、OSQP 和 Python 虚拟环境。产物保留在各子模块被 Git 忽略的目录中。`--jobs 4` 指定并行编译数；`--check` 只读检查产物和固定版本的 Python 依赖，全部就绪时返回 0。也可以在构建前单独运行 `--check` 查看缺项，此时因依赖未就绪而返回非零状态是正常的。

脚本不执行 `sudo`、不安装系统 apt 包，也不构建 ROS 包。若提示缺少 `ensurepip`，先安装 `python3.10-venv`，然后重新运行脚本。完整选项、产物位置和系统依赖检查命令见[脚本说明](scripts/README.md)。

统一构建四个子模块的 ROS 包并加载主工作空间环境：

```bash
scripts/build_ros.sh --check
scripts/build_ros.sh --jobs 4
# 在新终端中执行：
source /home/jinghao/Nav3_ws/scripts/source_env.bash
```

脚本限定四个子模块的 `src/` 为包发现目录，把所有 ROS 包安装到主目录的 `install/`，不会加载子模块的旧 `install/`。具体用法、默认构建范围和验收命令见[脚本说明](scripts/README.md)。统一参数传递和联合启动尚待在 `src/nav3_bringup` 实现。

## 仓库与部署说明

`.gitmodules` 已配置为 `Jinghaozhou` 账户下的四个 GitHub 仓库。各仓库推送完成后，可使用 `git clone --recurse-submodules` 获取主仓库及固定版本的子模块。子模块固定的是各原仓库当时已提交的版本；原工作空间中未提交的修改、被忽略的地图及构建结果不会自动进入本仓库。地图和 bag 仍须单独准备。
