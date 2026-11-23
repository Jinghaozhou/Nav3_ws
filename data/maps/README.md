# 部署地图

本目录保存本机地图文件。PCD 不纳入 Git；使用前需核对定位地图与全局规划地图的坐标系和版本。

# 可视化地图

`building_split_space_viz_space10_open.pcd` 是从统一全局地图离线生成的 RViz 专用小地图，见 [nav3_visualization 使用说明](../../src/nav3_visualization/README.md)。该文件和同名 JSON 已被主仓库 Git 忽略；定位、PCT 与区域初始化仍使用完整的 `building_split_space.pcd`。
