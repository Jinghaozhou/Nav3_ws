#!/usr/bin/env bash
# Build the ROS packages from all five submodules into the root workspace.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
sensor="$root/modules/Nav3_Sensor_ws"
localization="$root/modules/Nav3_Localization_ws"
global_planner="$root/modules/Nav3_Global_planner_ws"
local_planner="$root/modules/Nav3_Local_Planner_ws"
chassis="$root/modules/Nav3_Chassis_ws"
pct_venv="$global_planner/.venv_pct"
ros_setup=/opt/ros/humble/setup.bash
base_paths=("$root/src" "$sensor/src" "$localization/src" "$global_planner/src" "$local_planner/src" "$chassis/src")
required_packages=(
  nav3_bringup nav3_navigation_manager nav3_bag_recorder
  livox_ros_driver2 fast_lio mid360_bringup
  kiss_matcher_localization pcd_cloud_publisher localization_visualizer localization_path_recorder
  pct_planner_interfaces pct_planner pct_planner_viz
  scan_planner_msgs scan_planner scan_planner_demo
  go2control
)
jobs="${JOBS:-4}"
mode=build

usage() {
  cat <<'EOF'
用法：scripts/build_ros.sh [--check] [--jobs 数量]

  --check       只检查依赖和 ROS 包发现情况，不构建
  --jobs 数量   单个包的编译并行数，默认读取 JOBS，未设置时为 4
  -h, --help    显示帮助
EOF
}

die() {
  echo "错误：$*" >&2
  exit 1
}

while (( $# > 0 )); do
  case "$1" in
    --check) mode=check; shift ;;
    --jobs)
      (( $# >= 2 )) || die "--jobs 需要一个正整数"
      jobs="$2"
      shift 2
      ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; die "未知参数：$1" ;;
  esac
done
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || die "并行编译数必须是正整数：$jobs"
[[ -f "$ros_setup" ]] || die "找不到 ROS 2 Humble：$ros_setup"
command -v colcon >/dev/null || die "找不到 colcon 命令"

for source_dir in "${base_paths[@]}"; do
  [[ -d "$source_dir" ]] || die "缺少子模块源码目录：$source_dir；先运行 git submodule update --init --recursive"
done
"$root/scripts/prepare_non_ros.sh" --check || die "非 ROS 依赖尚未就绪"

# Limit discovery to the root and five module ROS source trees. In particular, legacy/, vendor/
# and the PCT native third-party sources must not become colcon packages.
package_names="$(colcon list --base-paths "${base_paths[@]}" --names-only)" || die "colcon 无法列出 ROS 包"
for package in "${required_packages[@]}"; do
  if ! grep -Fxq "$package" <<< "$package_names"; then
    die "colcon 未发现预期 ROS 包：$package"
  fi
done
echo "已发现 $(wc -l <<< "$package_names") 个 ROS 包："
printf '%s\n' "$package_names"
if [[ "$mode" == check ]]; then
  echo "ROS 构建前检查通过。"
  exit 0
fi

if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  die "请先退出当前 Python 虚拟环境，再运行本脚本：$VIRTUAL_ENV"
fi

# A child process can clear old overlays without changing the user's shell.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
set +u
source "$ros_setup"
source "$pct_venv/bin/activate"
set -u
export CMAKE_PREFIX_PATH="$localization/external${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$sensor/external/lib:$localization/external/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

echo "开始统一构建；产物目录：$root/install"
cd "$root"
MAKEFLAGS="-j$jobs -l$jobs" CMAKE_BUILD_PARALLEL_LEVEL="$jobs" \
  colcon --log-base "$root/log" build \
    --base-paths "${base_paths[@]}" \
    --build-base "$root/build" --install-base "$root/install" \
    --merge-install --symlink-install --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      -DPython3_EXECUTABLE="$pct_venv/bin/python"

set +u
source "$root/scripts/source_env.bash" || die "无法加载统一运行环境"
set -u
while IFS= read -r package; do
  prefix="$(ros2 pkg prefix "$package")" || die "构建后无法找到 ROS 包：$package"
  [[ "$prefix" == "$root/install" ]] || die "$package 指向了其他安装目录：$prefix"
done <<< "$package_names"
echo "全部 ROS 包的安装前缀均指向主工作空间。"
echo "ROS 包构建完成。新终端运行：source $root/scripts/source_env.bash"
