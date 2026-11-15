#!/usr/bin/env bash
# Prepare native libraries and the PCT Python environment before the root colcon build.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
sensor="$root/modules/Nav3_Sensor_ws"
localization="$root/modules/Nav3_Localization_ws"
global_planner="$root/modules/Nav3_Global_planner_ws"
pct_native="$global_planner/src/pct_planner/native"
pct_venv="$global_planner/.venv_pct"
pct_requirements="$global_planner/src/pct_planner/requirements-ros2.txt"
jobs="${JOBS:-4}"
mode=build

usage() {
  cat <<'EOF'
用法：scripts/prepare_non_ros.sh [--check] [--jobs 数量]

默认执行增量构建，将产物保留在各子模块现有的 Git 忽略目录。
  --check       只检查源码、工具和构建产物，不修改文件
  --jobs 数量   原生库并行编译数，默认读取 JOBS，未设置时为 4
  -h, --help    显示帮助
EOF
}

die() {
  echo "错误：$*" >&2
  exit 1
}

while (( $# > 0 )); do
  case "$1" in
    --check)
      mode=check
      shift
      ;;
    --jobs)
      (( $# >= 2 )) || die "--jobs 需要一个正整数"
      jobs="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      die "未知参数：$1"
      ;;
  esac
done

[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || die "并行编译数必须是正整数：$jobs"

preflight() {
  local tool source_file
  for tool in cmake python3 c++ make; do
    command -v "$tool" >/dev/null || die "缺少命令 $tool；请先安装系统编译依赖"
  done
  python3 - <<'PY' || die "当前 Python 或 CMake 版本不符合 ROS 2 Humble 的构建要求"
import re
import subprocess
import sys

cmake_line = subprocess.check_output(["cmake", "--version"], text=True).splitlines()[0]
match = re.search(r"(\d+)\.(\d+)", cmake_line)
if not match or tuple(map(int, match.groups())) < (3, 22):
    print("需要 CMake 3.22 或更新版本", file=sys.stderr)
    raise SystemExit(1)
if sys.version_info[:2] != (3, 10):
    print("需要与 Ubuntu 22.04 / ROS 2 Humble 一致的 Python 3.10", file=sys.stderr)
    raise SystemExit(1)
PY
  for source_file in \
    "$sensor/third_party/Livox-SDK2/CMakeLists.txt" \
    "$localization/vendor/robin/CMakeLists.txt" \
    "$localization/vendor/pmc/CMakeLists.txt" \
    "$localization/vendor/xenium/CMakeLists.txt" \
    "$localization/vendor/kiss_matcher/CMakeLists.txt" \
    "$localization/vendor/small_gicp/CMakeLists.txt" \
    "$localization/src/localization/deployment/build_from_source.sh" \
    "$pct_native/3rdparty/gtsam-4.1.1/CMakeLists.txt" \
    "$pct_native/3rdparty/osqp/CMakeLists.txt" \
    "$global_planner/src/pct_planner/scripts/build_native_deps.sh" \
    "$pct_requirements"; do
    [[ -f "$source_file" ]] || die "缺少源码或脚本：$source_file；先运行 git submodule update --init --recursive"
  done
}

check_cache_source() {
  local build_dir="$1" expected_source="$2" cache_source
  [[ -f "$build_dir/CMakeCache.txt" ]] || return 0
  cache_source="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$build_dir/CMakeCache.txt")"
  if [[ -n "$cache_source" && "$cache_source" != "$expected_source" ]]; then
    die "$build_dir 的 CMake 缓存仍指向 $cache_source；请先移走该旧构建目录，再重新运行"
  fi
}

check_caches() {
  check_cache_source "$sensor/build/livox_sdk2" "$sensor/third_party/Livox-SDK2"
  check_cache_source "$localization/build_algorithms/robin" "$localization/vendor/robin"
  check_cache_source "$localization/build_algorithms/kiss_matcher" "$localization/vendor/kiss_matcher"
  check_cache_source "$localization/build_algorithms/small_gicp" "$localization/vendor/small_gicp"
  check_cache_source "$global_planner/build/pct_gtsam" "$pct_native/3rdparty/gtsam-4.1.1"
  check_cache_source "$global_planner/build/pct_osqp" "$pct_native/3rdparty/osqp"
}

check_artifacts() {
  local missing=0 artifact
  local artifacts=(
    "$sensor/external/lib/liblivox_lidar_sdk_shared.so"
    "$sensor/external/include/livox_lidar_api.h"
    "$localization/external/lib/cmake/robin/robinConfig.cmake"
    "$localization/external/lib/cmake/kiss_matcher/kiss_matcherConfig.cmake"
    "$localization/external/lib/cmake/small_gicp/small_gicp-config.cmake"
    "$pct_native/3rdparty/gtsam-4.1.1/install_ros2/lib/cmake/GTSAM/GTSAMConfig.cmake"
    "$pct_native/3rdparty/gtsam-4.1.1/install_ros2/lib/libgtsam.so.4.1.1"
    "$pct_native/3rdparty/osqp/install_ros2/lib/cmake/osqp/osqp-config.cmake"
    "$pct_native/3rdparty/osqp/install_ros2/lib/libosqp.a"
  )

  for artifact in "${artifacts[@]}"; do
    if [[ -f "$artifact" ]]; then
      echo "已就绪：${artifact#"$root"/}"
    else
      echo "未准备：${artifact#"$root"/}" >&2
      missing=$((missing + 1))
    fi
  done

  if [[ -x "$pct_venv/bin/python" && -f "$pct_venv/bin/activate" ]]; then
    if ! grep -Fq "$pct_venv" "$pct_venv/bin/activate"; then
      echo "虚拟环境路径已变化：$pct_venv；请重新创建 .venv_pct" >&2
      missing=$((missing + 1))
    elif PYTHONDONTWRITEBYTECODE=1 "$pct_venv/bin/python" - "$pct_requirements" <<'PY'
from importlib import metadata
from pathlib import Path
import sys

missing = []
for line in Path(sys.argv[1]).read_text().splitlines():
    line = line.strip()
    if not line or line.startswith("#"):
        continue
    name, expected = line.split("==", 1)
    try:
        actual = metadata.version(name)
    except metadata.PackageNotFoundError:
        actual = "未安装"
    if actual != expected:
        missing.append(f"{name}：需要 {expected}，当前 {actual}")
if missing:
    for item in missing:
        print(f"Python 依赖未就绪：{item}", file=sys.stderr)
    raise SystemExit(1)
print("已就绪：PCT Python 虚拟环境及固定版本依赖")
PY
    then
      :
    else
      missing=$((missing + 1))
    fi
  else
    echo "未准备：${pct_venv#"$root"/} 虚拟环境" >&2
    missing=$((missing + 1))
  fi

  if (( missing > 0 )); then
    echo "尚有 $missing 项未准备完成。" >&2
    return 1
  fi
  echo "非 ROS 依赖检查通过。"
}

preflight
check_caches
if ! python3 -c 'import ensurepip' >/dev/null 2>&1; then
  echo "系统缺少 Python venv 的 ensurepip；Ubuntu 22.04 需安装 python3.10-venv" >&2
  if [[ "$mode" == build ]]; then
    die "先安装 python3.10-venv，再运行依赖准备脚本"
  fi
fi
if [[ "$mode" == check ]]; then
  check_artifacts
  exit $?
fi

# Native builds must not accidentally pick up libraries from older ROS overlays.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  die "请先退出当前 Python 虚拟环境，再运行本脚本：$VIRTUAL_ENV"
fi

echo "[1/3] 构建 Livox SDK2"
cmake -S "$sensor/third_party/Livox-SDK2" -B "$sensor/build/livox_sdk2" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$sensor/external"
cmake --build "$sensor/build/livox_sdk2" --parallel "$jobs"
cmake --install "$sensor/build/livox_sdk2"

echo "[2/3] 构建定位算法库 ROBIN、KISS Matcher、small_gicp"
JOBS="$jobs" bash "$localization/src/localization/deployment/build_from_source.sh" --native-only

echo "[3/3] 准备 PCT Python 环境、GTSAM 和 OSQP"
if [[ ! -x "$pct_venv/bin/python" || ! -f "$pct_venv/bin/activate" ]] || \
   ! "$pct_venv/bin/python" -m pip --version >/dev/null 2>&1; then
  python3 -m venv --system-site-packages "$pct_venv" || die "创建或修复虚拟环境失败；请检查 python3.10-venv"
fi
[[ -x "$pct_venv/bin/python" ]] || die "虚拟环境不完整：$pct_venv；请先移走该目录再重试"
"$pct_venv/bin/python" -m pip install -r "$pct_requirements"
CMAKE_BUILD_PARALLEL_LEVEL="$jobs" bash "$global_planner/src/pct_planner/scripts/build_native_deps.sh"

check_artifacts
