# Nav3 统一启动

`nav3_bringup` 统一启动传感器、定位、PCT、SCAN 和独立的导航任务管理器。它不处理导航目标，也不接管各模块的算法参数。当前尚未接入底盘；SCAN 直接发布 `/cmd_vel`，该话题当前没有底盘订阅者。

## 使用

先在主工作空间完成 `scripts/prepare_non_ros.sh` 和 `scripts/build_ros.sh`，然后在新终端执行：

```bash
source /home/jinghao/Nav3_ws/scripts/source_env.bash
ros2 launch nav3_bringup nav3.launch.py
```

默认启动实体 MID360，地图为主工作空间 `data/maps/building_split_space.pcd`，首次区域为 `space_2`，RViz 不启动。雷达网口仍须按传感器模块的说明事先配置。只检查无雷达场景的启动结构时，可以使用 `driver:=false`；此时不会有真实局部地图和里程计，导航管理器应保持等待状态。回放时还要设 `use_sim_time:=true` 并提供 `/clock`、雷达和 IMU 数据。

```bash
ros2 launch nav3_bringup nav3.launch.py initial_region:=space_3
ros2 launch nav3_bringup nav3.launch.py initial_region:=space_3 \
  region_seed_file:=/绝对路径/space_3_candidates.yaml
ros2 launch nav3_bringup nav3.launch.py initial_region:=""
```

`initial_region` 接受 `region_labels.csv` 内的名称或编号，空值表示不限制区域。`space_2` 默认启用定位子模块的 15 个区域候选；改为其他区域时自动关闭这组候选。提供匹配该区域的 `region_seed_file` 后会自动启用多候选初始化；也可以显式设置 `region_multistart_enabled:=true|false`。启用候选时，启动前核对区域名；候选文件记录了地图 SHA-256 时也核对地图内容，不匹配就停止启动。现有 `space_2` 候选针对特定雷达安装姿态和入口位置，现场首次定位仍需验证。

主要启动参数：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `map_pcd` | 主工作空间 `data/maps/building_split_space.pcd` | 定位和 PCT 共用的全局地图。 |
| `initial_region` | `space_2` | 首次定位区域。 |
| `region_seed_file` | `auto` | `space_2` 自动使用现有候选；其他区域需显式指定候选文件。 |
| `region_multistart_enabled` | `auto` | 自动根据候选文件启用；也可强制开启或关闭。 |
| `driver` | `true` | 是否启动实体 MID360 驱动。 |
| `use_sim_time` | `false` | 统一传给各节点的仿真时间参数。 |
| `rviz` | `false` | 是否启动传感器 RViz。 |
| `pct_startup_timeout_sec` | `180` | 等待 PCT 服务及地图预热的总时限。 |

启动包将定位局部地图设为 `/fast_lio/local_map`，连续定位配置指向定位模块的 `continuous_localization.yaml`；PCT 使用相同 PCD；SCAN 的路径输入固定为 `/nav3/managed_path`，定位许可输入为 `/localization/navigation_allowed`，速度输出为 `/cmd_vel`。关键帧候选保持关闭。PCT 的 `/pct/publish_cost_map` 成功后才启动导航任务管理器；预热失败则结束联合启动。管理器仍需等待首次定位许可、有效 TF 和 SCAN 状态，才会在 `/nav3/navigation_state` 报告 `READY` 并接受 `/nav3/navigate_to_pose` 目标。

当前尚未完成真实首次定位、四模块闭环或实体底盘验收。没有底盘时，`/cmd_vel` 有消息也不会驱动机器人。接入底盘前需要核对坐标参考点、速度方向和底盘自身的命令超时停车机制。
