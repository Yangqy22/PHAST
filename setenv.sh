#!/usr/bin/env bash
# Source after activating your Python environment; optionally pass a Dashing prefix.
PHAST_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
export PHAST_ROOT
mkdir -p "$PHAST_ROOT/.cache"/{tmp,config,data,state,colcon,ros} "$PHAST_ROOT/log"
export XDG_CACHE_HOME="$PHAST_ROOT/.cache" XDG_CONFIG_HOME="$PHAST_ROOT/.cache/config"
export XDG_DATA_HOME="$PHAST_ROOT/.cache/data" XDG_STATE_HOME="$PHAST_ROOT/.cache/state"
export TMPDIR="$PHAST_ROOT/.cache/tmp"
export TMP="$TMPDIR" TEMP="$TMPDIR"
export COLCON_HOME="$PHAST_ROOT/.cache/colcon" ROS_HOME="$PHAST_ROOT/.cache/ros"
export ROS_LOG_DIR="$PHAST_ROOT/log/ros" PYTHONDONTWRITEBYTECODE=1
export PIP_CACHE_DIR="$PHAST_ROOT/.cache/pip" CUDA_CACHE_PATH="$PHAST_ROOT/.cache/cuda"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export RCL_LOGGING_IMPLEMENTATION="${RCL_LOGGING_IMPLEMENTATION:-rcl_logging_noop}"
export RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED=1
phast_underlay="${1:-${PHAST_ROS_PREFIX:-}}"
if [[ -n "$phast_underlay" ]]; then
  if [[ ! -f "$phast_underlay/local_setup.bash" ]]; then
    printf 'Dashing prefix not found: %s\n' "$phast_underlay" >&2
    return 1
  fi
  source "$phast_underlay/local_setup.bash"
fi
if [[ "${PHAST_SKIP_OVERLAY:-0}" != 1 && -f "$PHAST_ROOT/install/local_setup.bash" ]]; then
  source "$PHAST_ROOT/install/local_setup.bash"
fi
unset phast_underlay
