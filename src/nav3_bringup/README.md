# Nav3 统一启动

`nav3_bringup` 统一启动传感器、定位、PCT、SCAN 和独立的导航任务管理器，也可以启动 `go2control` 底盘速度桥接。它不处理导航目标，也不接管各模块的算法参数。SCAN 发布 `/cmd_vel`，启用底盘后由桥接订阅该话题。

## 使用

先在主工作空间完成 `scripts/prepare_non_ros.sh` 和 `scripts/build_ros.sh`，然后在新终端执行：

```bash
source /home/jinghao/Nav3_ws/scripts/source_env.bash
ros2 launch nav3_bringup nav3.launch.py
```

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
| `use_sim_time` | `false` | 统一传给各节点的仿真时间参数。 |
| `rviz` | `false` | 是否启动传感器 RViz。 |
| `chassis` | `false` | 是否在 PCT 预热后启动 Go2 速度桥接。 |
| `chassis_dry_run` | `true` | 空跑时只处理 ROS 消息；设为 `false` 才连接实机。 |
| `chassis_interface` | `eno2` | 实机桥接使用的有线网卡。 |
| `chassis_sdk_root` | 主工作空间 `modules/Nav3_Chassis_ws` | 含 `vendor/` 的 SDK 工作空间根目录。 |
| `chassis_config` | 底盘子模块 `config/go2control.yaml` | 速度限幅、超时等桥接参数。 |
| `pct_startup_timeout_sec` | `180` | 等待 PCT 服务及地图预热的总时限。 |

启动包将定位局部地图设为 `/fast_lio/local_map`；BBS 模式使用传感器实验入口中的连续定位配置，固定候选模式沿用定位模块配置。PCT 使用相同 PCD；SCAN 的路径输入固定为 `/nav3/managed_path`，定位许可输入为 `/localization/navigation_allowed`，速度输出为 `/cmd_vel`；底盘桥接启用时订阅 `/cmd_vel`。传感器自身的旧关键帧候选发布器保持关闭，BBS 检索器由定位实验入口单独启动。PCT 的 `/pct/publish_cost_map` 成功后才启动导航任务管理器和已启用的底盘桥接；预热失败则结束联合启动。管理器仍需等待首次定位许可、有效 TF 和 SCAN 状态，才会在 `/nav3/navigation_state` 报告 `READY` 并接受 `/nav3/navigate_to_pose` 目标。

space_10 的独立 BBS 实验已完成首次定位；本次接入标准 bringup 后仍需验证联合启动、四模块闭环和实体底盘。实机接入前还需要核对坐标参考点、速度方向和底盘自身的命令超时停车机制。
