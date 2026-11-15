#!/usr/bin/env bash
# Source from a new Bash terminal to run the root Nav3 workspace.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "请用 source scripts/source_env.bash 加载到当前 Bash 终端" >&2
  exit 2
fi

_nav3_source_env() {
  local root sensor localization global_planner pct_venv pct_site prefix
  local -a active_prefixes
  root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)" || return 1
  sensor="$root/modules/Nav3_Sensor_ws"
  localization="$root/modules/Nav3_Localization_ws"
  global_planner="$root/modules/Nav3_Global_planner_ws"
  pct_venv="$global_planner/.venv_pct"
  pct_site="$pct_venv/lib/python3.10/site-packages"

  if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$root/install/local_setup.bash" ||
        ! -f "$pct_venv/bin/activate" ]]; then
    echo "Nav3 环境未就绪：先准备依赖并运行 scripts/build_ros.sh" >&2
    return 1
  fi
  if [[ -n "${VIRTUAL_ENV:-}" && "$VIRTUAL_ENV" != "$pct_venv" ]]; then
    echo "请先退出其他 Python 虚拟环境：$VIRTUAL_ENV" >&2
    return 1
  fi
  IFS=: read -r -a active_prefixes <<< "${AMENT_PREFIX_PATH:-}"
  for prefix in "${active_prefixes[@]}"; do
    if [[ -n "$prefix" && "$prefix" != /opt/ros/humble && "$prefix" != "$root/install" ]]; then
      echo "当前终端已加载其他 ROS 工作空间：$prefix；请在新终端加载 Nav3" >&2
      return 1
    fi
  done

  source /opt/ros/humble/setup.bash || return 1
  source "$pct_venv/bin/activate" || return 1
  source "$root/install/local_setup.bash" || return 1

  export LD_LIBRARY_PATH="$sensor/external/lib:$localization/external/lib:$pct_site/nvidia/cuda_nvrtc/lib:$pct_site/nvidia/cuda_runtime/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export CUPY_CACHE_DIR="$pct_venv/cupy_cache"
  export PCT_MAP_DIR="$root/data/maps"
}

_nav3_source_env
_nav3_result=$?
unset -f _nav3_source_env
return "$_nav3_result" 2>/dev/null || exit "$_nav3_result"
