# Nav3 导航任务管理器

本功能包位于主仓库，管理一次三维导航任务。它不运行雷达、定位、PCT 或 SCAN，也不直接发布速度；总启动包后续负责启动这些节点并传入统一参数。

## 接口

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| `/nav3/navigate_to_pose` | `nav3_navigation_manager/action/NavigateToPose` | 接收 `map` 中的三维目标，提供阶段、当前位置、距离反馈和终止结果。 |
| `/localization/navigation_allowed` | `std_msgs/msg/Bool` | 定位模块的持续导航许可，超过 1 秒未更新或为 false 时拒绝/终止任务。 |
| `/scan_planner/status` | `scan_planner_msgs/msg/PlannerStatus` | SCAN 任务状态；按新的 `path_id` 对应当前任务。 |
| `/scan_planner/execution_state` | `scan_planner_msgs/msg/ExecutionState` | 控制器心跳和实际轨迹跟踪状态；超时或持续未跟踪当前任务时撤销路径。 |
| `/pct/plan` | `pct_planner_interfaces/srv/Plan` | 用当前 `map → base_link` 位姿和用户目标请求三维路径。 |
| `/nav3/managed_path` | `nav_msgs/msg/Path` | 管理器独占的任务路径，Reliable + Transient Local；启动、取消、失败或到达时发布空路径。 |
| `/nav3/navigation_state` | `std_msgs/msg/String` | `READY`、等待原因或活动阶段；Reliable + Transient Local。 |

目标必须为 `header.frame_id=map` 且 XYZ 有限。第一版仅使用目标位置，不执行最终朝向调整，也不从 RViz 二维点击推断高度。只允许一个活动任务；新目标在旧任务结束前被拒绝。PCT 服务可以在首次请求时初始化完整地图，因此 `PLANNING` 阶段可能较长。超时阈值见 `config/navigation_manager.yaml`。

管理器先检查定位许可、SCAN 规划器与控制器状态、PCT 服务及当前 TF，再接受目标。规划成功后重新核对当前位置，拒绝从旧位置生成或起终点偏差过大的路径，再发布到 `/nav3/managed_path`。SCAN 的 `path_topic` 必须指向这个话题，不能直接订阅 PCT 保留的 `/pct/global_path`。管理器等待 SCAN 的新 `path_id`，仅用该编号对应的状态判断到达或失败；控制器必须在限定时间内跟踪同一编号。取消或故障后发布空路径并等待新指令；第一版不自动重新规划。Action 的取消结果表示撤销路径已发布，实体停车仍需在 SCAN 与底盘联调时验收。

## 独立启动与联调

```bash
source /home/jinghao/Nav3_ws/scripts/source_env.bash
ros2 launch nav3_navigation_manager navigation_manager.launch.py
```

在总启动文件中将 SCAN 配置为：

```text
path_topic:=/nav3/managed_path
require_localization_permission:=true
localization_permission_topic:=/localization/navigation_allowed
```

实体底盘联调前，SCAN 的 `cmd_vel_topic` 应先指向隔离话题。当前版本未实现任务管理器失联心跳保护：若管理器进程突然退出，SCAN 可能继续执行已收到的路径；接入底盘前须单独解决并验收这一场景。速度指令仍由 SCAN 产生，底盘驱动需具备命令超时停车能力。
