# Nav3 导航系统

本仓库通过 Git 子模块组合四个独立的 ROS 2 Humble 模块：

- `modules/Nav3_Sensor_ws`：Livox MID360 驱动、FAST-LIO 与传感器启动。
- `modules/Nav3_Localization_ws`：全局定位及辅助功能包。
- `modules/Nav3_Global_planner_ws`：PCT 全局规划及可视化。
- `modules/Nav3_Local_Planner_ws`：SCAN 局部规划及演示。

`src/nav3_bringup` 预留总系统启动与接口协调；`data/maps` 和 `data/bags` 保存本机部署数据；`scripts` 预留统一构建脚本。各模块原有源码结构和 Git 历史保持独立。当前仅完成代码集成布局，尚未实现统一构建和联合启动。

四个子模块目前从本机 `/home/jinghao/Nav3_*_ws` 克隆，原目录保留不变。`.gitmodules` 中的本机路径只适合这台机器；跨机器使用前需为各子模块配置可访问的远端 URL。

每个子模块使用原仓库当时已提交的版本。原仓库中未提交的修改、被忽略的地图以及构建结果不会复制到这里。地图须按各模块说明准备，并核对全局规划与定位地图的坐标系。
