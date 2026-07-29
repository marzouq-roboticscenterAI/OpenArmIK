# OpenArmIK

`openarm_ik_ros` is a hardware-free ROS 2 Lyrical visualization adapter for the checked-in OpenArm v1.0 bimanual C model.

**It is position-only IK: orientation is free. It performs no self, body, inter-arm, or environment collision checking. It is not motion authorization and has no CAN, ros2_control, MoveIt, or hardware backend.**

The repository also contains:

- [UPSTREAM_SOURCES.md](UPSTREAM_SOURCES.md): exact commits and licenses for all ten
  repositories from Enactic's canonical OpenArm repository index. The complete
  clones live under ignored `upstream/` and can be reproduced with
  `scripts/fetch_upstreams.sh`.
- [model/README.md](model/README.md): the dependency-free C11 FK, all-joint frame,
  geometric-Jacobian, and bounded claw/TCP XYZ IK interface for both arms.
- [can/README.md](can/README.md): the C11 DaMiao codec, commissioned-manifest
  validator, fresh-disabled probe, fake transport, and read-only SocketCAN
  interface discovery API.

Automatic CAN discovery cannot establish physical joint identity, side, sign,
zero, gearing, firmware, or a safe motor mapping. Consequently this revision
does not configure or move physical arms. A physical backend must remain
disarmed until those facts are commissioned and an independent E-stop and
watchdog are installed.

## Run

```bash
./scripts/fetch_upstreams.sh
./scripts/build.sh
./scripts/launch_rviz.sh
```

The launch starts only `robot_state_publisher`, the virtual adapter, and RViz. `robot_state_publisher` is the only TF authority. The pinned `openarm_description` package supplies mesh URI resolution; the installed robot description is copied from [model/generated/openarm_v10_bimanual.urdf](model/generated/openarm_v10_bimanual.urdf).

In another sourced terminal, send one atomic paired request:

```bash
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
./scripts/send_paired_xyz.py 0.20 0.30 0.85 0.20 -0.30 0.85
```

The only input is `/openarm_ik/paired_xyz` (`geometry_msgs/PoseArray`, reliable depth 10): its header must have frame `world`, a fresh nonzero stamp (one-second default expiry, configurable only from 1 to 60000 ms), and exactly two poses ordered **left, right**. Only `position.x/y/z` is used. The helper waits for discovery, publishes exactly once, and waits for its diagnostic acknowledgement. If either position-IK solve fails, the adapter keeps the previous complete two-arm state.

The immutable redundant-DOF policy is `continuity-v1`: both arms seed and use posture reference equal to their last committed seven-joint posture; all posture weights are `1`; tolerance is `0.0001 m`; maximum joint step is `0.15 rad`; damping is `[0.0001, 0.3]`; maximum iterations are `300`; and `limit_margin_rad` is `0`. This policy is deliberately position-only: orientation remains free. It is reported in every diagnostic, so callers can reproduce the model result. No per-request seed/options override exists in this visualization-only API.

Results are structured `diagnostic_msgs/DiagnosticArray` messages on `/openarm_ik/diagnostics`. Every message includes `sequence`, `committed`, `achieved_available`, both statuses, the `continuity-v1` policy, `backend=virtual`, and `collision_checked=false`. Residual and achieved TCP pose/matrix fields are present only when the full pair committed; rejected input never advertises zero residuals or a fake pose.

RViz may report four unrealistic finger-inertia errors from the pinned canonical generated URDF. They are inherited model data; meshes and TF still load, and this adapter neither edits nor reinterprets that URDF.

`model/` is installed as an ordinary CMake package (`openarm_model::openarm_model`); the ROS package finds that export instead of compiling monorepo-relative sources. `scripts/install_ros_dependencies.sh` is review-only unless explicitly passed `--apply`; it is never run automatically. After `scripts/build.sh`, test only the authored adapter with:

```bash
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
colcon --log-base ros2_ws/log test \
  --base-paths ros2_ws/src \
  --packages-select openarm_ik_ros \
  --build-base ros2_ws/build \
  --install-base ros2_ws/install
colcon test-result --test-result-base ros2_ws/build/openarm_ik_ros --verbose
```

The explicit package scope prevents the pinned upstream repositories from being mistaken for this workspace's test targets. `scripts/test_ros_coverage.sh` rebuilds the authored adapter with gcov instrumentation and writes a reproducible text report under `ros2_ws/coverage/`.
