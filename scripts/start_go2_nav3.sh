#!/usr/bin/env bash
# Deployment entry for the zwz host; no navigation target is sent automatically.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
set +u
source "$root/scripts/source_go2_env.bash"
set -u
exec python3 "$root/scripts/run_nav3.py" \
  chassis:=true chassis_dry_run:=false chassis_interface:=enp130s0 \
  localization_permission_after_initialization:=false manager_bypass_checks:=false \
  scan_check_local_collision:=true scan_gravity_aligned:=true scan_collision_window_sec:=2.0 \
  scan_override_config:="$root/src/nav3_bringup/config/go2_zwz_scan.yaml" \
  timing_diagnostics:=true "$@"
