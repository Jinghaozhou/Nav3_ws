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

## 显式联调放行模式

默认 `bypass_checks=false`，保持上述判断。显式传入 `bypass_checks:=true` 时，管理器跳过接收目标和执行期间的定位许可、SCAN 状态/控制器心跳、PCT 服务就绪、TF 时间戳新鲜度、路径起终点距离一致性、SCAN 故障状态、路径接收/跟踪超时等判断。管理器仍需要可查询的 `map → base_link` TF 来构造 PCT 起点，并需要 PCT 返回成功且格式有效的非空路径；目标与路径的坐标系和坐标有效性、单任务占用、SCAN `path_id` 归属、主动取消和空路径撤销仍保留。此模式将单次任务最长时间限制为 `bypass_max_task_duration_sec=45.0` 秒，超时撤销路径。

此参数只作用于导航管理器；SCAN 自身的定位许可、局部输入超时、碰撞检查和底盘命令超时仍按各自配置执行。联调时可独立启动：

```bash
ros2 launch nav3_navigation_manager navigation_manager.launch.py bypass_checks:=true
```

本次高度诊断可额外显式设置 `start_z_override_enabled:=true start_z_override_m:=5.542`，仅将管理器发给 PCT 的起点高度改为 5.542 m；实时 TF、目标高度和其他节点的位姿均不改变。默认关闭，使用时管理器会同时记录原始 TF 高度和请求高度。该参数可能使规划路径起点与机器人实际高度不一致，仅用于定位高度问题的隔离实验。

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

## space_10 固定测试目标

后续导航联调统一使用 [`config/space10_fixed_goal.yaml`](config/space10_fixed_goal.yaml) 中的 `map` 坐标目标 `(-4.0, 30.0, 5.542)`。这是固定的全局位置，不会随机器狗当前站位重新计算“前方一米”；发令前应确认机器人所在位置、到目标的路线和定位状态。当前管理器只使用目标位置，不执行文件中的最终朝向。该文件仅保存请求，联合启动不会自动发送目标。

联合启动完成且 `/nav3/navigation_state` 为 `READY` 后，在主工作空间终端手动发送：

```bash
source scripts/source_env.bash
ros2 action send_goal /nav3/navigate_to_pose \
  nav3_navigation_manager/action/NavigateToPose \
  "$(cat src/nav3_navigation_manager/config/space10_fixed_goal.yaml)" --feedback
```
