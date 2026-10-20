# PHAST: Priority-Driven Hierarchical Scheduling for Real-Time ROS2 Executor on Heterogeneous Robotic Platforms 

Qingyuan Yang*, Hongyi Li*, Leshan Li, Rong Zhao, Xinglong Ji (*equal contribution)  
2026 IEEE/RSJ International Conference on Intelligent Robots and Systems  
[Project Page](https://yangqy22.github.io/phast-project/) · [Paper PDF](https://events.infovaya.com/uploads/documents/pdfviewer/17/d3/235499-0858.pdf) · [Code](https://github.com/Yangqy22/PHAST)

PHAST adds coroutine-based hierarchical scheduling and callback priorities to the
ROS 2 Dashing executor. This repository contains the executor, experiment sources
and build/run helpers. 

## Requirements

Validated on Ubuntu 22.04.5 / aarch64, Jetson Orin NX Super: **ROS 2 Dashing
built from source**, GCC 11.4 (C++20), CMake 3.22.3, Python 3.6.13,
colcon-core 0.12.1, vcstool 0.3.0, empy 3.3.4, lark-parser 0.8.9,
NumPy 1.19.5, PyYAML 5.4.1 and setuptools 58.0.4.
The tested middleware is Cyclone DDS 0.7.0 (`rmw_cyclonedds_cpp`) with
`rcl_logging_noop`; prepare these in your Dashing underlay.

Before building Dashing, apply the allocator compatibility patch in its `rosidl`
source checkout, then regenerate messages and build the underlay:

```bash
git -C /path/to/ros2_dashing_ws/src/ros2/rosidl apply "$PWD/patches/rosidl-cxx20-allocator.patch"
```

## Build and run

Activate your prepared Python environment first. From this repository root:

```bash
source setenv.sh /path/to/ros2_dashing_ws/install
bash build.sh
bash run.sh exp_cpu_phast              
bash run.sh executor_stability_test    
```

Results depend on hardware and load; paper timings are not guaranteed.

## Source layout

- `src/rclcpp_spk`: executor and coroutine runtime.
- `src/phast_experiments`: experiment package and executor regression checks.

| Directory in `src/phast_experiments/src/` | Purpose | Executable targets |
|---|---|---|
| `exp_baseline/` | Basic scheduling comparison | `exp_baseline_ros2`, `exp_baseline_robospike` |
| `exp_cpu/` | CPU execution-order comparison (Fig.9) | `exp_cpu_phast`, `exp_cpu_ros2`, `exp_cpu_robospike`, `exp_cpu_picas` |
| `exp_config/` | Priority and subcallback configuration (Fig.7) | `exp_config_priority`, `exp_config_subcallback` |
| `exp_gpu/` | CPU/GPU scheduling comparison (Fig.10) | `exp_gpu_phast`, `exp_gpu_ros2`, `exp_gpu_robospike`, `exp_gpu_picas` |

Each experiment source has the same name as its executable (for example,
`exp_gpu/exp_gpu_phast.cpp`). GPU helpers are `gpu_workload.cu` and
`gpu_workload.cuh`. Regression checks are in `test/executor_stability.cpp`.

GPU experiments have not been revalidated here. CUDA targets are optional:
`bash build.sh -DPHAST_BUILD_GPU=ON` (requires CUDA Toolkit).
Builds, installs, logs and caches are generated locally in ignored `build/`,
`install/`, `log/` and `.cache/`. Environments and downloaded dependencies are not bundled.
The scripts only set variables for the current shell/process and do not change system settings.

Licensing: executor **Apache-2.0**; experiments and project scripts **MIT**.
See [LICENSE](LICENSE) and [NOTICE](NOTICE).
