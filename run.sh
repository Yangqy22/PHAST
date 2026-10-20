#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/setenv.sh" "${PHAST_ROS_PREFIX:-}"
experiment="${1:-exp_cpu_phast}"
if (( $# )); then shift; fi
case "$experiment" in
  exp_baseline_robospike|exp_baseline_ros2|exp_cpu_robospike|exp_cpu_ros2|exp_cpu_picas|exp_cpu_phast|exp_config_priority|exp_config_subcallback|executor_stability_test|exp_gpu_phast|exp_gpu_robospike|exp_gpu_ros2|exp_gpu_picas) ;;
  *) printf 'Unknown experiment: %s\n' "$experiment" >&2; exit 1 ;;
esac
executable="$PHAST_ROOT/install/lib/phast_experiments/$experiment"
[[ -x "$executable" ]] || { printf 'Target not built: %s; run bash build.sh first.\n' "$experiment" >&2; exit 1; }
exec "$executable" "$@"
