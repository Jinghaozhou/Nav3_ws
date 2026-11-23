# 构建与运行脚本

## 非 ROS 依赖准备

在主工作空间根目录执行：

```bash
git submodule update --init --recursive
scripts/prepare_non_ros.sh --check
scripts/prepare_non_ros.sh --jobs 4
scripts/prepare_non_ros.sh --check
```

`--check` 只读检查工具、源码、CMake 缓存路径、原生库产物和 PCT 虚拟环境中固定版本的 Python 包；依赖尚未构建时返回非零状态。默认模式进行增量配置、编译和安装，不自动清空已有构建目录，也不运行 `colcon`。可用 `JOBS=4` 指定默认并行数；`--jobs` 优先于环境变量。

| 依赖 | 源码及产物位置 |
| --- | --- |
| Livox SDK2 | `modules/Nav3_Sensor_ws/third_party/Livox-SDK2` → `modules/Nav3_Sensor_ws/external/` |
| ROBIN、KISS Matcher、small_gicp | `modules/Nav3_Localization_ws/vendor/` → `modules/Nav3_Localization_ws/external/` |
| PCT Python 依赖 | `requirements-ros2.txt` → `modules/Nav3_Global_planner_ws/.venv_pct/` |
| GTSAM、OSQP | PCT 的 `native/3rdparty/` → 各自的 `install_ros2/` |

这些产物都被对应子模块的 Git 忽略。脚本以自身位置计算路径，不依赖当前工作目录。定位子模块的 `build_from_source.sh --native-only` 只准备原生库；不带参数运行该脚本仍保留原有独立工作空间构建流程。

系统依赖由用户在构建前准备，脚本不会执行 `sudo` 或安装 apt 包。可在初始化 rosdep 后，从主目录检查所有 ROS 包的系统依赖：

```bash
source /opt/ros/humble/setup.bash
rosdep check --from-paths \
  modules/Nav3_Sensor_ws/src modules/Nav3_Localization_ws/src \
  modules/Nav3_Global_planner_ws/src modules/Nav3_Local_Planner_ws/src \
  modules/Nav3_Chassis_ws/src src \
  --ignore-src --rosdistro humble
```

若 rosdep 尚未初始化，运行上述检查前需按 ROS 2 安装说明执行 `sudo rosdep init` 和 `rosdep update --rosdistro humble`。原生库的其他系统依赖由 CMake 配置阶段检查，PCT Python 依赖由 pip 检查并安装。

若提示缺少 `ensurepip`，先在 Ubuntu 22.04 安装 `python3.10-venv`。首次创建虚拟环境失败可能留下不完整的 `.venv_pct`；安装系统包后直接重运行准备脚本，它会尝试补全该目录。

移动工作空间后，若 CMake 缓存仍指向旧源码路径，脚本会停止并提示对应目录；请先移走该构建目录再重试。PCT 虚拟环境的激活脚本也含绝对路径，移动工作空间后应重新创建 `.venv_pct`。脚本只检查 CuPy 安装，不验证 GPU 驱动或地图生成。

运行本脚本只完成统一 ROS 构建的前置依赖准备。

## 统一构建 ROS 包

先准备上述非 ROS 依赖，再在主工作空间根目录执行：

```bash
scripts/build_ros.sh --check
scripts/build_ros.sh --jobs 4
```

`--check` 只读检查非 ROS 依赖和 ROS 包发现情况；默认构建从主仓库 `src/` 和五个子模块的 `src/` 中发现的全部 ROS 包，包括 `nav3_bringup`、`nav3_navigation_manager`、`nav3_bag_recorder`、`scan_planner_demo` 与 `go2control`，不会扫描 `legacy/`、`vendor/` 或 PCT 原生库源码。脚本从 ROS 2 Humble 和 PCT 虚拟环境构建，使用定位子模块的 `external/` 查找算法库。`--jobs` 控制单包编译并行数；为了限制内存峰值，`colcon` 一次构建一个包。构建期测试默认关闭，测试与 Gazebo 演示运行需另行执行。

全部 ROS 包统一安装到主目录的 `install/`，构建过程写入主目录的 `build/`、`log/`；不会使用子模块已有的 `install/`。构建脚本结束前还会检查每个发现的包均指向主目录 `install/`。构建成功后，在**新终端**加载环境：

```bash
source /home/jinghao/Nav3_ws/scripts/source_env.bash
ros2 pkg prefix fast_lio
ros2 pkg prefix kiss_matcher_localization
ros2 pkg prefix pct_planner
ros2 pkg prefix scan_planner
ros2 pkg prefix nav3_navigation_manager
ros2 pkg prefix nav3_bag_recorder
ros2 pkg prefix go2control
```

脚本本身按所在位置计算主目录；如果工作空间被移动，修改上面示例命令的路径即可。统一环境加载 ROS 2、PCT 虚拟环境、主目录的 `install/` 和运行时原生库路径，并将 `PCT_MAP_DIR` 设为主目录的 `data/maps/`。联合启动由 `nav3_bringup` 显式传入地图路径，使用方式见[统一启动说明](../src/nav3_bringup/README.md)。
