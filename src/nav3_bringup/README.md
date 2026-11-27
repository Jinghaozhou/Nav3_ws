# Nav3 统一启动

`nav3_bringup` 统一启动传感器、定位、PCT、SCAN 和独立的导航任务管理器，也可以启动 `go2control` 底盘速度桥接。它不处理导航目标，也不接管各模块的算法参数。SCAN 发布 `/cmd_vel`，启用底盘后由桥接订阅该话题。

后续 space_10 导航联调的固定目标为 `map` 坐标 `(-4.0, 30.0, 5.542)`；请求文件与手动发送方法见[导航任务管理器说明](../nav3_navigation_manager/README.md#space_10-固定测试目标)。联合启动不会自动发送该目标。

默认 `robot_model:=go2`，TF 链为 `map → odom → imu_link → base_link → lidar`。
`base_link` 使用 Unitree Go2 机体原点；雷达安装参数位于传感器包 `config/go2_mount.yaml`，
初值为 `[0.207993, 0, 0.152959]` m 和俯仰 `+20°`，尚未实物标定。
FAST-LIO 输出的里程计 child 为 `imu_link`，SCAN 通过 TF 换算到机体原点，
BBS 检索器自动接入新帧名。`robot_model:=imu` 可恢复历史 IMU 原点模式。
旧 bag 回放时只提供 `/lidar_points` 和 `/imu`，由新节点重新生成里程计和 TF。
修改参考点后，实机测试还需复核碰撞包络、路径高度及历史固定定位候选的物理参考点。

## 使用

`/home/zwz/Nav3_ws` 本机实机部署使用 `scripts/start_go2_nav3.sh`，并在其他终端
加载 `scripts/source_go2_env.bash`，具体命令见[脚本说明](../../scripts/README.md)。
该主机入口使用 `enp130s0`、严格定位许可、管理器心跳、重力对齐规划帧及近期碰撞窗口；
本页其余命令保留通用默认参数。

先在主工作空间完成 `scripts/prepare_non_ros.sh` 和 `scripts/build_ros.sh`，然后在新终端执行：

```bash
source /home/jinghao/Nav3_ws/scripts/source_env.bash
ros2 launch nav3_bringup nav3.launch.py
```

需要同时打开统一导航可视化时，先按 [nav3_visualization 说明](../nav3_visualization/README.md)生成小地图，再启动 `ros2 launch nav3_bringup nav3.launch.py visualization:=true`。可视化显示去除 space_10 天花板的降采样地图、局部地图窗口、受管理全局路径和 SCAN 局部轨迹；默认不启动。原 `rviz` 参数仅控制传感器模块自带的 RViz。

终端默认只显示各模块的节点加载状态、定位节点与 BBS 检索器的输出，以及启动失败信息。`节点已加载` 表示预期 ROS 节点已出现；定位是否真正成功以终端中的定位输出和 `导航许可已开启` 为准。其他节点的详细日志可在启动时打印的 ROS 日志目录中查看。

排查定位许可短暂撤销时，启动前增加 `timing_diagnostics:=true`。终端会打印本次诊断目录，默认位于主工作空间被 Git 忽略的 `log/timing/时间_进程号/`。`local_map_publish.csv` 记录局部地图发布或跳过的时刻、地图消息时间戳及原因；`localization_timing.csv` 记录定位节点实际接收地图的时刻，并以 5 Hz 记录许可检查时的最近地图接收时刻、地图时间戳和 `odom → base_link` TF 时间戳。CSV 的 `ros_ns` 使用 ROS 时钟，`steady_ns` 使用本机单调时钟，单位均为纳秒；缺失的时间戳填 `-1`。发布和接收记录可按 `map_stamp_ns` 对齐，同一时间戳重复发布时按顺序取最近的一条。诊断关闭时不创建文件，也不额外查询许可检查中的 TF。

```bash
ros2 launch nav3_bringup nav3.launch.py timing_diagnostics:=true
```

局部地图的 `header.stamp` 来自最新输入点云，不是发布时刻。若许可日志出现 `map_fresh=false`，比较两份 CSV 中的发布、接收和消息时间戳，便可区分发布暂停、接收延迟和输入时间戳过旧。诊断模式只增加日志，不改变许可阈值或导航控制。

定位许可默认在首次成功应用全局定位后持续通过，由 `localization_permission_after_initialization:=true` 控制；未完成首次定位或定位状态被重置时仍为 false。需要恢复按匹配、局部地图和里程计新鲜度持续撤销许可的严格模式时，传入 `localization_permission_after_initialization:=false`。两种模式都不改变导航管理器对当前 `map → base_link` TF 的 0.75 秒检查，也不改变 SCAN 的局部输入超时和底盘速度超时。时间诊断 CSV 仍记录各项新鲜度，并新增 `allowed` 列表示实际发布的许可。

若只为联调管理器的拒绝/故障路径，可显式传入 `manager_bypass_checks:=true`；默认是 `false`。此模式跳过管理器的就绪、新鲜度、路径距离一致性与执行故障判断，并以 45 秒任务上限撤销路径。它不改变 SCAN 与底盘的独立保护，具体保留项见[管理器说明](../nav3_navigation_manager/README.md#显式联调放行模式)。

仅当需要复现 PCT 起点高度问题时，可额外传入 `manager_start_z_override_enabled:=true manager_start_z_override_m:=5.542`；这只覆盖发给 PCT 的起点 z，默认关闭，不修改 TF 或固定目标高度。

```bash
ros2 launch nav3_bringup nav3.launch.py manager_bypass_checks:=true
```

联合启动给 SCAN 传入 `scan_unknown_space_policy:=free_inside_bounds`：在局部地图快照边界内，未被点云标为占据的位置按自由空间处理；已观测的障碍和地图边界仍会阻止规划。需要恢复保守策略时传入 `scan_unknown_space_policy:=occupied`。该设置针对点云没有自由空间射线证据的现状，实机使用前应确认局部地图覆盖和全局路径可通行。

仅在已确认前方通畅的实机实验中，可显式传入 `scan_check_local_collision:=false`，让 SCAN 不使用局部点云做障碍碰撞判定；局部地图输入、地图窗口边界、参考路径走廊、动态可行性、定位许可、轨迹有效期及底盘限速仍生效。默认值为 `true`，实验后重新启动即恢复碰撞检查。

局部地图以约 1 Hz 发布。联调时若 SCAN 因发布抖动报告局部地图超时，可临时传入 `scan_map_timeout:=3.0`；默认仍为 1.5 秒。

默认启动实体 MID360，地图为主工作空间 `data/maps/building_split_space.pcd`，首次区域为 `space_10`，使用 BBS 动态候选做初定位，RViz 不启动。bringup 复用传感器模块的 BBS 检索脚本与定位配置：先校验统一地图、`space_10` 区域配置和 Git 忽略目录中的 BBS 模型，再启动检索器与权威定位节点；两个窗口确认成功后才应用 `map → odom`。默认不录 rosbag，底盘也不启动。BBS 模型需要按传感器模块说明在本机单独准备。雷达网口仍须按传感器模块的说明事先配置。只检查无雷达场景的启动结构时，可以使用 `driver:=false`；此时不会有真实局部地图和里程计，导航管理器应保持等待状态。回放时还要设 `use_sim_time:=true` 并提供 `/clock`、雷达和 IMU 数据。

底盘默认不启动。先用空跑模式检查统一启动的 `/cmd_vel` 订阅，不连接机器人 SDK：

```bash
ros2 launch nav3_bringup nav3.launch.py driver:=false chassis:=true
```

实机联调时，确认 Go2 已通过遥控器/App 进入正常高层运动模式、网卡已连接、现场可立即停车，并停止其他底盘控制程序，再显式连接底盘：

```bash
ros2 launch nav3_bringup nav3.launch.py chassis:=true chassis_dry_run:=false chassis_interface:=eno2
```

桥接在 PCT 地图预热成功后启动；启动失败或运行中退出会结束联合启动。桥接只接收 SCAN 的 `/cmd_vel`，不会自动站起或切换运动模式。`chassis_dry_run:=true` 时仍会发布 `/go2_cmd_vel_bridge/commanded_velocity` 供观察，但不会向机器人发送指令。仿真时间模式只允许底盘空跑。当前 MID360 使用 USB 网卡的 `192.168.1.5/24`，Go2 使用 `eno2` 的 `192.168.123.5/24`；启动前分别检查两条链路。底盘的速度限幅和输入超时配置见[底盘功能包说明](../../modules/Nav3_Chassis_ws/src/go2control/README.md)。

```bash
ros2 launch nav3_bringup nav3.launch.py initial_region:=space_2
ros2 launch nav3_bringup nav3.launch.py initialization_source:=seeds initial_region:=space_3
ros2 launch nav3_bringup nav3.launch.py initialization_source:=seeds initial_region:=space_3 \
  region_seed_file:=/绝对路径/space_3_candidates.yaml
ros2 launch nav3_bringup nav3.launch.py initialization_source:=seeds initial_region:=""
```

`initial_region` 接受 `region_labels.csv` 内的名称或编号。默认 `initialization_source:=bbs` 与 `space_10` 会选用 `region_initialization_bbs_space_10.yaml` 和 `bbs_initialization_space_10.json`，并启动 BBS 检索器；切换到 `space_2` 时自动选择对应的 BBS 配置。目前 BBS 模型仅支持 `space_10` 与 `space_2`。`initialization_source:=seeds` 保留原有的固定候选入口：`space_2` 自动使用定位子模块的 15 个候选，其他区域可显式指定候选文件；空区域名也仅在此模式下可用。BBS 模式需要 `region_multistart_enabled=true`，因此不能将其关闭。启动前会核对候选文件的区域名和地图 SHA-256，随后校验 BBS 模型清单、配置及地图缓存。

主要启动参数：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `map_pcd` | 主工作空间 `data/maps/building_split_space.pcd` | 定位和 PCT 共用的全局地图。 |
| `initial_region` | `space_10` | 首次定位区域。 |
| `initialization_source` | `bbs` | `bbs` 启动动态候选检索；`seeds` 保留原固定候选方式。 |
| `bbs_config` | `auto` | BBS 模式按 `space_10` 或 `space_2` 选择区域配置；目前模型仅支持这两个区域。 |
| `region_seed_file` | `auto` | BBS 模式下 `space_10` 自动使用动态候选区域配置；`space_2` 使用现有区域配置。 |
| `region_multistart_enabled` | `auto` | 自动根据候选文件启用；也可强制开启或关闭。 |
| `driver` | `true` | 是否启动实体 MID360 驱动。 |
| `robot_model` | `go2` | `go2` 使用 Go2 机体原点和安装外参；`imu` 恢复历史帧定义。 |
| `use_sim_time` | `false` | 统一传给各节点的仿真时间参数。 |
| `timing_diagnostics` | `false` | 记录局部地图发布、定位接收及导航许可检查的时间诊断 CSV。 |
| `localization_permission_after_initialization` | `true` | 首次定位成功后持续许可；`false` 恢复严格的新鲜度检查。 |
| `manager_bypass_checks` | `false` | 显式跳过管理器的就绪、路径一致性与执行故障判断；仅用于联调。 |
| `manager_start_z_override_enabled` | `false` | 是否覆盖发送给 PCT 的规划起点高度。 |
| `manager_start_z_override_m` | `0.0` | 起点高度覆盖值，只有上项为 `true` 时生效。 |
| `rviz` | `false` | 是否启动传感器 RViz。 |
| `visualization` | `false` | 是否启动主仓库导航可视化包和 RViz。 |
| `visualization_rviz` | `true` | 开启可视化时是否同时打开 RViz 窗口；设为 `false` 只发布显示地图。 |
| `visualization_map_pcd` | `data/maps/building_split_space_viz_space10_open.pcd` | RViz 专用降采样地图。 |
| `chassis` | `false` | 是否在 PCT 预热后启动 Go2 速度桥接。 |
| `chassis_dry_run` | `true` | 空跑时只处理 ROS 消息；设为 `false` 才连接实机。 |
| `chassis_interface` | `eno2` | 实机桥接使用的有线网卡。 |
| `scan_unknown_space_policy` | `free_inside_bounds` | SCAN 在局部地图边界内把未观测空间视为自由；`occupied` 将其视为占据。 |
| `scan_check_local_collision` | `true` | SCAN 局部点云碰撞检查；`false` 仅用于已确认通畅的实机联调。 |
| `scan_map_timeout` | `1.5` | SCAN 局部地图输入超时秒数。 |
| `scan_override_config` | 空 | SCAN 规划器和控制器的附加参数文件。 |
| `scan_gravity_aligned` | `false` | 从 FAST-LIO 重力向量固定 `odom → nav_odom`，SCAN 改在 `nav_odom` 中规划。 |
| `scan_collision_window_sec` | `0.0` | 大于零时另建近期碰撞窗口，5 Hz 发布 `/fast_lio/collision_map`，不改变定位长窗口。 |
| `scan_path_z_offset` | `0.0` | SCAN 对参考路径显式施加的高度校准量，默认不加偏移。 |
| `pct_path_z_offset` | `0.0` | 在 PCT 输出端将原生地面上方 0.5 m 的路径参考平移到机器人参考点；不改变地图或楼层选择，不与 SCAN 偏移重复叠加。 |
| `scan_ground_filter` | `false` | 本次平地场景使用的地面分离；要求重力帧和近期碰撞窗口。支撑平面经三帧确认后局部跟踪，仅移除其附近 2.5 cm 点，其他点全部保留。 |
| `scan_ground_fit_radius` | `4.0` | 地面拟合候选邻域半径（米）；本轮 Go2 TF 实机测试显式使用 `3.0`。不限制障碍输出范围。 |
| `chassis_sdk_root` | 主工作空间 `modules/Nav3_Chassis_ws` | 含 `vendor/` 的 SDK 工作空间根目录。 |
| `chassis_config` | 底盘子模块 `config/go2control.yaml` | 速度限幅、超时等桥接参数。 |
| `pct_startup_timeout_sec` | `180` | 等待 PCT 服务及地图预热的总时限。 |

`scan_ground_filter:=true` 增加 `/fast_lio/collision_obstacles` 和
`/fast_lio/ground_filter_status`，SCAN 使用前者，原始碰撞点云与定位点云保留。
平面拟合要求足够点数、水平覆盖、残差和重力方向一致；初始化失败或后续支撑点不足时不输出新快照，
由原有输入超时保护停车。此模式针对本轮近似平地，尚不作为坡道、台阶或多楼层地面分割的验收结果。
地面拟合的 0.44 m 高度先验沿用原 MID360 IMU 参考点：Go2 模式自动给滤波器设置
`floor_reference_frame=imu_link`，旧模式使用 `base_link`。SCAN 的碰撞参考点仍为 Go2 `base_link`。
局部平面更新有突变门限并平滑处理；新拟合质量不足时，仅在旧平面仍有足够实时支撑的条件下保留旧模型。
手动验证：`python3 -m unittest discover -s src/nav3_bringup/test -v`。

启动包将定位局部地图设为 `/fast_lio/local_map`；BBS 模式使用传感器实验入口中的连续定位配置，固定候选模式沿用定位模块配置。PCT 使用相同 PCD；SCAN 的路径输入固定为 `/nav3/managed_path`，定位许可输入为 `/localization/navigation_allowed`，速度输出为 `/cmd_vel`；底盘桥接启用时订阅 `/cmd_vel`。传感器自身的旧关键帧候选发布器保持关闭，BBS 检索器由定位实验入口单独启动。PCT 的 `/pct/publish_cost_map` 成功后才启动导航任务管理器和已启用的底盘桥接；预热失败则结束联合启动。管理器仍需等待首次定位许可、有效 TF 和 SCAN 状态，才会在 `/nav3/navigation_state` 报告 `READY` 并接受 `/nav3/navigate_to_pose` 目标。

space_10 已在 Go2 名义安装 TF 下完成一次前方 1 米实机闭环；参数与复现命令见
[脚本说明](../../scripts/README.md#jh-主机-go2-tf-实机测试)，测量与限制见主仓库实验 29。
本轮保留严格定位许可、局部碰撞检查及底盘超时停车；重复运行、安装标定和长期稳定性仍待验证。
