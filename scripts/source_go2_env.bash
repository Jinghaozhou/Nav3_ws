#!/usr/bin/env bash
# Source this in every terminal participating in the zwz Go2 deployment.
_nav3_go2_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$_nav3_go2_root/scripts/source_env.bash" || return $?
export ROS_DOMAIN_ID="${NAV3_ROS_DOMAIN_ID:-193}"
export ROS_LOCALHOST_ONLY="${NAV3_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${NAV3_DDS_PROFILE:-$_nav3_go2_root/src/nav3_bringup/config/fastdds_local.xml}"
unset _nav3_go2_root
