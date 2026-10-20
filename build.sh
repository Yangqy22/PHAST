#!/usr/bin/env bash
set -eo pipefail
PHAST_SKIP_OVERLAY=1 source "$(dirname -- "${BASH_SOURCE[0]}")/setenv.sh" "${PHAST_ROS_PREFIX:-}"
[[ -n "${AMENT_PREFIX_PATH:-}" ]] || { echo 'Source ROS 2 Dashing first (see README).' >&2; exit 1; }
export MAKEFLAGS="-j${PHAST_BUILD_JOBS:-2}" CMAKE_BUILD_PARALLEL_LEVEL="${PHAST_BUILD_JOBS:-2}"
cd "$PHAST_ROOT"
exec colcon --log-base "$PHAST_ROOT/log" build \
  --base-paths "$PHAST_ROOT/src/rclcpp_spk" "$PHAST_ROOT/src/phast_experiments" \
  --build-base "$PHAST_ROOT/build" --install-base "$PHAST_ROOT/install" \
  --merge-install --executor sequential --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPHAST_BUILD_GPU=OFF -DPHAST_STABILITY_TESTS=ON \
  -DCMAKE_C_COMPILER="${CC:-gcc}" -DCMAKE_CXX_COMPILER="${CXX:-g++}" \
  -DPYTHON_EXECUTABLE="${PHAST_PYTHON_EXECUTABLE:-$(command -v python3)}" "$@"
