# 构建与运行脚本

## JH 主机 Go2 TF 实机测试

新 TF 下已完成一次前方 1 米闭环（实验 29）。以下参数只对应本轮安装状态和近似平地，
路径输出高度补偿为实机预检测得的 `-0.333 m`，地面拟合邻域为 3 m。
实测使用的本机细栅格配置及其配套 SCAN 搜索范围修改未纳入本轮提交，
因此下列命令只用于静止检查 TF 和地面滤波，不作为完整运动测试的复现命令：

```bash
source scripts/source_go2_env.bash
OPENBLAS_NUM_THREADS=1 python3 scripts/run_nav3.py \
  chassis:=false robot_model:=go2 \
  localization_permission_after_initialization:=false manager_bypass_checks:=false \
  scan_check_local_collision:=true scan_gravity_aligned:=true scan_collision_window_sec:=2.0 \
  pct_path_z_offset:=-0.333 scan_ground_filter:=true scan_ground_fit_radius:=3.0 \
  timing_diagnostics:=true
```

`OPENBLAS_NUM_THREADS=1` 限制本轮 Python 地面拟合等小矩阵计算的 BLAS 线程数；
3 m 参数只限定拟合候选邻域，所有输入点仍参与后续障碍输出，原有拟合质量门限保留。
定位与 SDK 位移仍有差异，地面滤波存在短暂拒绝快照，不能据一次到达认定全部联调问题已解决。
本机复核文件为 `log/go2_tf_full_20261126/verification.json`，不随仓库发布。
可共享的实测结果与已知问题见[联调记录](../report/联调记录.md)实验 29。

## zwz 主机 Go2 部署入口

本机网卡为 `enp130s0=192.168.123.5/24`（Go2）和
`enxe466e592e70a=192.168.1.5/24`（MID360）。两条持久连接分别为
`nav3-go2`、`nav3-devices`，默认互联网路由仍由 Wi-Fi 提供。
板载 RTL8125 使用带 Secure Boot 签名的 `r8125 9.016.01` DKMS 模块。

```bash
# 启动完整实机链路；不会自动发送导航目标
/home/zwz/Nav3_ws/scripts/start_go2_nav3.sh visualization:=true

# 每个录包、诊断、发目标的终端都加载此环境
source /home/zwz/Nav3_ws/scripts/source_go2_env.bash
ros2 launch nav3_bag_recorder record.launch.py profile:=diagnostic

# 另一个同环境终端：当前位置前方 1 米，最多 45 秒
python3 /home/zwz/Nav3_ws/scripts/navigate_forward.py --distance 1 --timeout 45
```

短任务客户端等待 `READY`，用新鲜 TF 生成 `map` 目标；超时、Ctrl+C、反向或明显侧偏会请求取消。
`--target X Y Z` 可继续一个距离当前位置不超过 1 米的既有终点。
实机链路保留碰撞检查、严格定位许可、输入超时和管理器心跳撤销保护；
控制限幅为 `(vx,vy,wz)=(0.15,0.10,0.25)`，单位为 m/s、m/s、rad/s。
规划帧为重力对齐的 `nav_odom`，定位用长窗口，碰撞用 2 秒/5 Hz 窗口。
固定规划帧须与 FAST-LIO 一起重启，设备断电或里程计重置后不要复用旧帧。

本机环境统一为 ROS 域 193、仅本机 ROS 通信，并使用 `fastdds_local.xml`
的回环 UDP 配置与较大的发现端口范围；Go2 SDK 独立绑定实体网卡，不使用 ROS 域。
这些值可通过 `NAV3_ROS_DOMAIN_ID`、`NAV3_LOCALHOST_ONLY`、`NAV3_DDS_PROFILE` 显式覆盖。
只设 `ROS_DOMAIN_ID` 不能覆盖此主机专用入口。系统 socket 缓冲上限配置见
`/etc/sysctl.d/90-nav3-dds.conf`（两项上限均为 8 MiB）。

录包完成后先 Ctrl+C，确认 `Recording stopped` 再读取 SQLite；运行中直接查询数据库会阻塞写入。
原始点云、地图、模型和构建结果仍被 Git 忽略，部署改动及实测边界见主仓库 `report/联调记录.md`。

## 带进程组清理的统一启动

```bash
python3 scripts/run_nav3.py driver:=false chassis:=true
```

启动器自动加载本工作空间环境，把后续参数原样传给 `nav3.launch.py`。
`Ctrl+C`、向启动器发送 `SIGTERM`/`SIGHUP`，或 launch 自行结束时，
均清理本次启动的整个进程组；先给 ROS 15 秒正常退出，再对剩余进程发送
TERM/KILL。只清理本次进程组，不按名称结束其他 ROS 进程。
上例关闭雷达驱动、启用底盘空跑；实机参数仍须显式指定。
若后台运行，应把停止信号发给这个启动器的 PID，不能仅结束其 launch 子进程。
强制杀死启动器本身（SIGKILL）不经过清理逻辑。

## 传感器时间戳诊断

在已加载主工作空间环境、传感器正在运行的同一 ROS 域中执行：

```bash
source /home/zwz/Nav3_ws/scripts/source_env.bash
python3 scripts/diagnose_sensor_timing.py \
  --duration 60 --warmup 10 --output log/timing/sensor_chain.csv
```

脚本只订阅，不发布控制消息。CSV 逐条记录原始雷达、IMU、FAST-LIO
里程计、配准点云、局部地图和 FAST-LIO 动态 TF 的消息时间与接收时间；
Go2 模式记录 `odom → imu_link`，历史模式记录 `odom → base_link`。
同名 `.summary.json` 汇总预热后的接收频率、消息年龄中位数/P95/最大值、
最大接收间隔和时间戳回退次数。输出 CSV 必须使用新文件名。
缺少任一链路的样本时退出码为 1，仍保留已有数据。

雷达点云时间为扫描起点，FAST-LIO 输出通常为扫描终点，因此二者消息年龄
会有约一个扫描周期的正常差异。将原始雷达/IMU 与里程计、TF 的年龄对比，
再结合 `timing_diagnostics:=true` 产生的局部地图发布日志，可定位延迟出现在哪一段。
接收频率反映此探针回调观测，不能单独作为硬件采样频率的结论。

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

`--check` 只读检查非 ROS 依赖和 ROS 包发现情况；默认构建从主仓库 `src/` 和五个子模块的 `src/` 中发现的全部 ROS 包，包括 `nav3_bringup`、`nav3_navigation_manager`、`nav3_visualization`、`nav3_bag_recorder`、`scan_planner_demo` 与 `go2control`，不会扫描 `legacy/`、`vendor/` 或 PCT 原生库源码。脚本从 ROS 2 Humble 和 PCT 虚拟环境构建，使用定位子模块的 `external/` 查找算法库。`--jobs` 控制单包编译并行数；为了限制内存峰值，`colcon` 一次构建一个包。构建期测试默认关闭，测试与 Gazebo 演示运行需另行执行。

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

## MID360–Go2 TF 回放验收

主仓库联合启动默认使用 Go2 原点。用已有原始输入比较旧 IMU 模式与新 Go2 模式：

```bash
source scripts/source_env.bash
ROS_DOMAIN_ID=89 /usr/bin/python3 scripts/validate_go2_mount.py \
  --bag data/bags/wide_joint_repeat_20261124 --seconds 25
/usr/bin/python3 -m unittest discover \
  -s modules/Nav3_Sensor_ws/src/mid360_bringup/test -v
ROS_DOMAIN_ID=90 /usr/bin/python3 -m unittest discover \
  -s modules/Nav3_Sensor_ws/tools/mid360_lidar/tests -p test_bbs_async.py -v
ROS_DOMAIN_ID=91 /usr/bin/python3 -m unittest discover \
  -s src/nav3_bringup/test -p test_go2_scan_input.py -v
```

脚本顺序启动三次无驱动的传感器回放（旧模式两次、新模式一次），每次默认 25 秒，不启动底盘、不发送导航目标。
只播放 `/lidar_points` 和 `/imu`，重建 `/clock`，不播放 bag 中的旧 TF 与派生数据。
临时配置开启完整当前帧点云以核对机体/世界点一致性，关闭累计地图与 PCD 保存以减少开销；
不修改原传感器配置。检查 IMU 轨迹差异小于 2 mm / 0.05°（同时量化旧模式自身的重复回放差异）、静态 TF 父子关系正确、
世界点云无重复变换、局部地图持续发布。结果及进程日志保存在 Git 忽略的
`log/go2_mount_validation/`，成功时生成 `result.json`。
这项测试验证软件几何与数据接口，实物安装精度和机器人运动方向需现场验证。
