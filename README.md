# Nav3 三维导航系统

Nav3 将传感器里程计、全局定位、三维全局规划和局部规划组织为四个独立的 ROS 2 Humble 工作空间，另以独立工作空间保存 Go2 底盘控制模块。模块间通过 ROS 2 Topic、Service 和 TF 交换数据，不直接调用彼此的算法源码。本仓库用 Git 子模块固定各模块版本，并保存集成时使用的数据目录。

**当前状态：**四个导航模块和 Go2 底盘工作空间已纳入子模块；主仓库已有独立的 `nav3_navigation_manager` 任务管理包和 `nav3_bringup` 联合启动包。`scripts/prepare_non_ros.sh` 准备导航模块原生依赖，`scripts/build_ros.sh` 统一构建五个子模块和主仓库的 ROS 包。bringup 支持按参数启动 Go2 速度桥接，实机闭环仍需联调验证。

联调现状见[联调记录](report/联调记录.md)和[问题排查与解决建议](report/三维导航系统联调问题排查与解决建议.md)。面向汇报的概览见[主仓库架构与联调进度说明](report/Nav3主仓库架构与联调进度说明.md)，团队交接方式见[模块协作与主仓库联调流程](report/Nav3模块协作与主仓库联调流程.md)。

## 代码结构

```text
Nav3_ws/
├── modules/
│   ├── Nav3_Sensor_ws/          # Livox MID360、FAST-LIO、mid360_bringup
│   ├── Nav3_Localization_ws/    # 导航定位、可视化、轨迹记录和 PCD 调试发布
│   ├── Nav3_Global_planner_ws/  # PCT 核心、ROS 节点、接口包和可视化
│   ├── Nav3_Local_Planner_ws/   # SCAN 核心、ROS 节点、消息包和演示
│   └── Nav3_Chassis_ws/         # Go2 速度桥接与底盘控制
├── src/nav3_bringup/            # 导航模块、任务管理器与可选底盘桥接的联合启动
├── src/nav3_navigation_manager/ # 导航目标 Action、任务状态和受管理路径
├── data/maps/                   # 本机全局地图，不纳入 Git
├── data/bags/                   # 本机回放数据，不纳入 Git
└── scripts/                     # 非 ROS 依赖准备、ROS 统一构建和环境加载
```

五个 `modules/*` 都是 Git 子模块，各自保留源码结构和 Git 历史。子模块内部的 `src/` 是其 ROS 2 包目录，部分模块还保留 `vendor/`、`legacy/` 或演示资源。包级细节见[传感器](modules/Nav3_Sensor_ws/README.md)、[定位](modules/Nav3_Localization_ws/README.md)、[PCT](modules/Nav3_Global_planner_ws/README.md)、[SCAN](modules/Nav3_Local_Planner_ws/README.md)和[底盘](modules/Nav3_Chassis_ws/README.md)说明。

## 模块划分与数据流

| 模块 | 主要 ROS 2 包 | 职责 |
| --- | --- | --- |
| 传感器与里程计 | `livox_ros_driver2`、`fast_lio`、`mid360_bringup` | 接收 MID360 点云和 IMU，输出连续里程计、当前帧点云和滑动窗口局部地图；发布 `odom → base_link`。 |
| 全局定位 | `kiss_matcher_localization`，另有可视化与记录包 | 将 `odom` 中的局部地图与全局 PCD 配准，成功应用后发布 `map → odom`。当前持续导航入口使用区域或关键帧候选加 VGICP；KISS 粗配准保留在历史演示入口。 |
| 全局规划 | `pct_planner`、`pct_planner_interfaces`、`pct_planner_viz` | 从离线 PCD 建立 Tomogram，按服务请求输出 `map` 中的三维路径。 |
| 局部规划与控制 | `scan_planner`、`scan_planner_msgs`、`scan_planner_demo` | 消费里程计、局部地图、TF 和全局路径，输出局部轨迹、状态与平面速度。 |
| 导航任务管理 | `nav3_navigation_manager` | 接收 `map` 中的三维目标，调用 PCT，发布受管理路径，依据 SCAN 状态完成、取消或中止任务。 |
| 统一启动 | `nav3_bringup` | 连接四个导航模块的启动参数，检查统一地图与区域候选，预热 PCT 后启动任务管理器与可选的底盘桥接。 |
| 底盘控制 | `go2control` | 订阅 `/cmd_vel` 并转为 Go2 的运动指令；统一启动默认关闭，可先空跑检查。 |

```text
MID360 + IMU ──> FAST-LIO ── odom / local_map ──> 定位 ── map→odom TF
                          └───────────────> SCAN <── managed_path ── 任务管理器 <── 目标
                                               └── cmd_vel / status       └── PCT 规划服务 <── 全局 PCD
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
| `/localization/navigation_allowed` | `std_msgs/msg/Bool` | 定位 → SCAN（联合启动时接入） | 定位模块按最近成功时间、连续失败、局部地图和里程计 TF 新鲜度持续发布；Reliable、Volatile。 |
| `/localization/keyframe_candidates` | `kiss_matcher_localization/msg/KeyframeCandidates` | 可选检索器 → 定位 | 短时点云与多个 `map → odom` 候选，含地图和库的哈希。 |
| `/pct/global_path` | `nav_msgs/msg/Path` | PCT → 诊断/可视化 | PCT 保留的最近一次规划结果；整机任务运行时 SCAN 不直接订阅。 |
| `/nav3/managed_path` | `nav_msgs/msg/Path` | 任务管理器 → SCAN（联合启动时接入） | 验证后的 `map` 中三维路径；Reliable、Transient Local；空路径撤销任务。 |
| `/nav3/navigate_to_pose` | `nav3_navigation_manager/action/NavigateToPose` | 操作者/上层任务 → 任务管理器 | 接收明确 XYZ 的 `map` 目标，返回任务反馈与结果；暂不执行终点朝向。 |
| `/nav3/navigation_state` | `std_msgs/msg/String` | 任务管理器 → 监控 | 就绪等待原因和任务阶段。 |
| `/scan_planner/status` | `scan_planner_msgs/msg/PlannerStatus` | SCAN → 任务管理器/监控 | 20 Hz 状态心跳，含运动许可和失败原因。 |
| `/scan_planner/execution_state` | `scan_planner_msgs/msg/ExecutionState` | SCAN 控制器 → 任务管理器/监控 | 控制器的路径编号、轨迹编号和实际跟踪状态。 |
| `/scan_planner/local_path` | `nav_msgs/msg/Path` | SCAN → 可视化 | `odom` 中的局部轨迹，失效时清空。 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | SCAN → 底盘接口 | `control_frame` 中的平面 `vx/vy/wz`；底盘仍需自身的命令超时停车。 |
| `/localization/relocalize` | `kiss_matcher_localization/srv/Relocalize` | 操作者/任务层 → 定位 | 对最新局部地图尝试定位；`apply_if_safe` 控制合格结果是否应用。 |
| `/pct/plan` | `pct_planner_interfaces/srv/Plan` | 任务管理器 → PCT | 接收 `map` 中的起终点，返回成功标志、路径及说明。 |

定位另提供 `/localization/set_region` 和 `/localization/accept_candidate`；PCT 提供 `/pct/publish_cost_map`；传感器提供 `/map_save`。联合启动时须设置 SCAN 的 `path_topic:=/nav3/managed_path`、`require_localization_permission:=true`、`localization_permission_topic:=/localization/navigation_allowed`。管理器的模拟成功、取消、规划失败、许可撤销和控制器未跟踪测试已通过，实机联动尚待验证。完整接口与 QoS 见[任务管理器](src/nav3_navigation_manager/README.md)、[定位协议](modules/Nav3_Localization_ws/src/localization/NAVIGATION_INTERFACE.md)、[PCT 说明](modules/Nav3_Global_planner_ws/README.md)及[SCAN 协议](modules/Nav3_Local_Planner_ws/src/scan_planner/docs/interfaces.md)。

### 关键自定义消息结构

| 类型 | 关键字段 | 用途 |
| --- | --- | --- |
| `LocalizationResult` | `matched`、`applied`、`reason`、`candidate_map_to_odom`、`last_success_stamp`、`overlap_percent`、`candidate_id` | 区分候选匹配、TF 应用与上一次成功时间；只有 `delta_valid` 为真时位姿变化量才有效。 |
| `KeyframeCandidates` | `cloud`、`map_to_odom[]`、`ids[]`、`scores[]`、`region_id`、`map_sha256`、`library_sha256`、`profile_sha256` | 可选关键帧检索输入，定位节点校验地图、库和配置版本。 |
| `Plan` 服务 | 请求 `start`、`goal`；响应 `success`、`path`、`message` | 起终点均为 `PoseStamped`，`frame_id` 必须为 `map`。 |
| `PlannerStatus` | `path_id`、`traj_id`、`state`、`motion_allowed`、`reason`、`goal` | 状态包括等待、运行、到达、规划失败、定位不可用、数据过期和 TF 大幅修正。 |
| `NavigateToPose` Action | `target`；结果 `success`、`message`；反馈 `phase`、`detail`、`current_pose`、`distance_remaining` | 管理一次三维目标的规划、执行和取消。 |

精确定义分别位于 [`localization/msg`、`srv`](modules/Nav3_Localization_ws/src/localization/)、[`pct_planner_interfaces`](modules/Nav3_Global_planner_ws/src/pct_planner_interfaces/)、[`scan_planner_msgs`](modules/Nav3_Local_Planner_ws/src/scan_planner_msgs/)和[管理器 Action](src/nav3_navigation_manager/action/NavigateToPose.action)。

## 地图与数据

`data/maps/building_split_space.pcd` 是本机当前选定的统一全局参考地图，含 `region_id` 标签；文件不纳入 Git。其 SHA-256 为 `c84fba59d8c6342aa72c5c53599152295e27948ff0e3317d95fd9f3c12c1ab55`，与定位模块的 `space_2` 区域初始候选及传感器包内 40 帧关键帧库记录的地图哈希一致。区域名称表和 15 个初始候选已在定位子模块中；候选基于特定安装姿态的实验数据，仍需现场核验。

定位 launch 可通过 `map_pcd` 指向主项目地图；传感器启用关键帧候选时，可通过 `keyframe_map` 指向同一文件；PCT 通过 `map_dir` 与 `map` 选择 PCD。SCAN 正式运行不直接读取全局 PCD。**本机已用 RTX 4060 对这份约 880 MiB 地图生成 Tomogram，并从缓存规划出一条 10 点测试路径；峰值内存占用、更多起终点的可行性及路径与定位的几何对齐仍待验证。**如需规划用抽稀地图，必须保留同一 `map` 坐标系并记录其与原图的派生关系。子模块内的 `building2_9.pcd`、`3_part.pcd` 等属于独立演示或历史场景，不应与当前定位候选混用。

`data/bags/` 用于本机回放数据。地图、bag、构建结果均不随主仓库提交；换机部署时需要单独准备，并核对地图哈希和坐标定义。

## 参数与启动入口

| 模块 | 配置文件及启动入口 | 集成时优先核对的参数 |
| --- | --- | --- |
| 传感器 | [`mid360_robot.yaml`](modules/Nav3_Sensor_ws/src/mid360_bringup/config/mid360_robot.yaml)、[`MID360_config.json`](modules/Nav3_Sensor_ws/src/livox_ros_driver2/config/MID360_config.json)、[`mid360_mapping.launch.py`](modules/Nav3_Sensor_ws/src/mid360_bringup/launch/mid360_mapping.launch.py) | `driver`、`use_sim_time`、`config_file`、`driver_config`、`window_time`（默认 120 s）、`tracking_window_time`（默认 30 s）、`adaptive_window`、`keyframe_candidates`、`keyframe_map`。组合启动话题名在 YAML 的 `mid360_bringup.ros__parameters.topics` 中设置。 |
| 定位 | [`continuous_localization.yaml`](modules/Nav3_Localization_ws/src/localization/config/continuous_localization.yaml)、[`navigation_localization.launch.yaml`](modules/Nav3_Localization_ws/src/localization/launch/navigation_localization.launch.yaml) | `map_pcd`、`region_labels_csv`、`initial_region`、`region_multistart_enabled`、`region_seed_file`、`local_map_topic`、`local_map_reliability`、`config_path`。连续配置含重叠率、重试周期、TF 修正门槛和 `navigation_permission.*` 许可阈值。 |
| PCT | [`pct_planner.yaml`](modules/Nav3_Global_planner_ws/src/pct_planner/config/pct_planner.yaml)、[`pct_planner.launch.py`](modules/Nav3_Global_planner_ws/src/pct_planner/launch/pct_planner.launch.py) | `map`、`map_dir`、`cache_dir`、`resolution`、`ground_h`、`slice_dh`、机器人净空、坡度、台阶与安全膨胀参数。默认地图仍是 `building2_9.pcd`，集成时须显式覆盖。 |
| SCAN | [`integration.yaml`](modules/Nav3_Local_Planner_ws/src/scan_planner/config/integration.yaml)、`planner.yaml`、`controller.yaml`、[`scan_planner.launch.py`](modules/Nav3_Local_Planner_ws/src/scan_planner/launch/scan_planner.launch.py) | `odom_topic`、`local_map_topic`、`path_topic`、`cmd_vel_topic`、四个 frame、`velocity_source`、`path_height_mode`、`unknown_space_policy`、`require_localization_permission`。 |
| 任务管理器 | [`navigation_manager.yaml`](src/nav3_navigation_manager/config/navigation_manager.yaml)、[`navigation_manager.launch.py`](src/nav3_navigation_manager/launch/navigation_manager.launch.py) | 目标 Action、定位许可、SCAN 状态、PCT 服务和受管理路径话题；许可、TF、规划和任务超时阈值。 |
| 统一启动 | [`nav3.launch.py`](src/nav3_bringup/launch/nav3.launch.py) | `map_pcd`、`initial_region`（默认 `space_2`）、`region_seed_file`、`region_multistart_enabled`、`driver`、`use_sim_time`、`rviz` 和 PCT 预热时限。SCAN 固定使用 `/nav3/managed_path` 和 `/cmd_vel`。 |

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

统一构建主仓库任务管理器和五个子模块的 ROS 包，并加载主工作空间环境：

```bash
scripts/build_ros.sh --check
scripts/build_ros.sh --jobs 4
# 在新终端中执行：
source /home/jinghao/Nav3_ws/scripts/source_env.bash
```

脚本限定主仓库及五个子模块的 `src/` 为包发现目录，把 16 个 ROS 包安装到主目录的 `install/`，不会加载子模块的旧 `install/`。具体用法、默认构建范围和验收命令见[脚本说明](scripts/README.md)。

联合启动入口：

```bash
ros2 launch nav3_bringup nav3.launch.py
```

默认使用实体 MID360、统一全局地图和 `space_2` 区域候选；首次启动区域可用 `initial_region:=space_3` 等参数修改。更换区域时默认不沿用 `space_2` 候选，需另传对应的 `region_seed_file` 才启用多候选。底盘桥接默认关闭；`chassis:=true` 先以空跑模式订阅 SCAN 的 `/cmd_vel`，实机连接还需显式传入 `chassis_dry_run:=false`。PCT 地图预热完成后才启动管理器和已启用的底盘桥接；管理器仍需等待首次定位和 SCAN 状态才能接收目标。参数、示例和启动条件见[统一启动包说明](src/nav3_bringup/README.md)。

## 仓库与部署说明

`.gitmodules` 已配置为 `Jinghaozhou` 账户下的五个 GitHub 仓库。各仓库推送完成后，可使用 `git clone --recurse-submodules` 获取主仓库及固定版本的子模块。子模块固定的是各原仓库当时已提交的版本；原工作空间中未提交的修改、被忽略的地图及构建结果不会自动进入本仓库。地图和 bag 仍须单独准备。

## 2026-11-23 配套模块同步分支

`wyn/race-auto-library-20261123` 固定以下配套版本，供审查和复现：

| 模块 | 提交 | 更新范围 |
| --- | --- | --- |
| Sensor | `83ff7b3c86c22804425597dd54e70032c1fe9fd6` | `tools/mid360_lidar` 独立实验工具：BBS/实采库竞争、一键录包、版本化补库 |
| Localization | `c34fc31823ad3c136e442b0ec0361665949a2927` | 双路权威应用服务、异步查询等待、精确地图缓存，保留导航许可保护 |

其他三个子模块指针保持原版本。两个模块PR 1已审查并合入各自main，上表固定其合并提交；本主仓库沿用相同代码树的验证结果。标准 `nav3_bringup` 的启动默认不切换；使用独立实验入口时，不得同时启动标准传感器/定位节点。

```bash
# 在按现有脚本完成依赖和ROS构建、并补齐地图/库/BBS数据后
bash modules/Nav3_Sensor_ws/tools/mid360_lidar/scripts/start_scan.sh --mode race --region space_2
```

实验工具默认在录包封口后严格筛选生成下一轮活动库；冻结40帧不改，使用 `--no-auto-library-update` 保持独立实验。仅权威定位节点发布全局TF与 `/localization/navigation_allowed`，私有验证器不发布；成功后先验跟踪，失配重新双路竞争，不恢复KISS粗配准。原始地图与点云库等二进制数据继续单独传输。

验证范围：定位包独立编译、初始化状态机、5项启动/结束生命周期、3项路径解析、隔离域历史点云的双路初始化/先验/失配恢复，以及许可初始拒绝→定位后允许→输入过期撤销。未执行Go2硬件导航或重新标定安装外参。
