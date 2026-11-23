# Nav3 导航可视化

`nav3_visualization` 在 RViz2 中同时显示降采样全局地图、局部地图窗口、实际交给 SCAN 的全局路径和 SCAN 局部轨迹。它只负责显示，不向导航管理器发送目标，也不修改定位或规划使用的完整地图。

## 准备可视化地图

从主工作空间执行一次离线采样：

```bash
python3 src/nav3_visualization/scripts/prepare_visual_map.py \
  --source data/maps/building_split_space.pcd \
  --output data/maps/building_split_space_viz_space10_open.pcd
```

默认保留原地图每个 `0.20 m` 体素的一个原始点，最多 30 万点；仅在 `region_id=10` 去除 `z ≥ 11.55 m` 的点。这个高度根据 space_10 的顶部密集点层选定，可以用 `--space10-ceiling-z` 调整。脚本不改变任何点的 XYZ、RGB 或 `region_id`，输出 `map` 坐标系的二进制 PCD 和同名 JSON 参数记录。输出文件位于主仓库已被 Git 忽略的 `data/maps/`，需要在部署机器上自行生成或传输。当前脚本只接受 `building_split_space.pcd` 的二进制 `x y z rgb region_id` 字段布局。

**完整的 `building_split_space.pcd` 仍供定位和 PCT 使用。**可视化文件去顶后不能用于碰撞检测、定位或全局规划；删除的只是 RViz 显示点。修改高度或原地图后需重新运行脚本，生成过程不会自动发生在联合启动期间。

## 启动与话题

完成主工作空间 ROS 构建并加载环境后，独立启动：

```bash
source scripts/source_env.bash
ros2 launch nav3_visualization navigation_visualization.launch.py
```

也可以随联合启动打开，默认不打开：

```bash
ros2 launch nav3_bringup nav3.launch.py visualization:=true
```

| RViz 显示 | 话题 | 坐标系 | 说明 |
| --- | --- | --- | --- |
| 降采样全局地图 | `/nav3/viz/global_map` | `map` | 本包读取小 PCD 后发布一次，Reliable + Transient Local。 |
| 局部地图窗口 | `/fast_lio/local_map` | `odom` | 直接订阅传感器模块发布的完整窗口快照。 |
| 执行中的全局路径 | `/nav3/managed_path` | `map` | 管理器取消或结束任务时清空。 |
| SCAN 局部轨迹 | `/scan_planner/local_path` | `odom` | 局部轨迹失效时清空。 |
| PCT 原始全局路径 | `/pct/global_path` | `map` | 默认隐藏，可在 RViz 打开；其旧路径不代表当前任务仍有效。 |

RViz 固定坐标系为 `map`；局部地图和局部轨迹需要有效的 `map → odom` TF 才能与全局地图叠加。未定位时，可暂时将 RViz Fixed Frame 改为 `odom` 单独查看局部输入。PCT 的代价地图可视化仍由独立的 `pct_planner_viz` 提供，本包启动时不加载 Tomogram。
