# OpenArmIK

`openarm_ik_ros` is a hardware-free ROS 2 Lyrical measured-feedback adapter for the OpenArm v1.0 virtual controller.

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

On this Wayland hybrid-GPU laptop the launcher uses Mesa software OpenGL with
the XWayland/GLX backend required by this RViz/Ogre build. Hardware GLX remains
available with `OPENARM_RVIZ_RENDERER=nvidia` or `integrated`, but it flickers
during live window resize on this host. The launcher also disables HiDPI render
target scaling for RViz only, closes the RViz window before stopping ROS so
`Ctrl+C` shuts down cleanly, and holds a single-instance lock to prevent
duplicate joint-state/TF publishers. The window-close path is a compiled C++17
Xlib utility installed with the ROS package; it has no Python runtime dependency.

The launch starts only `robot_state_publisher`, the virtual adapter, and RViz. `robot_state_publisher` is the only TF authority. The pinned `openarm_description` package supplies mesh URI resolution; the installed robot description is copied from [model/generated/openarm_v10_bimanual.urdf](model/generated/openarm_v10_bimanual.urdf).

In another sourced terminal, inspect the measured virtual controller or send an
action goal with the compiled client:

```bash
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
ros2 run openarm_ik_ros openarm_control_cli status
ros2 run openarm_ik_ros openarm_control_cli move-joint openarm_left_joint4 0.2
ros2 run openarm_ik_ros openarm_control_cli move-paired-tcp \
  openarm_body_link0 0.20 0.30 0.85 0.20 -0.30 0.85
```

The production inputs are `/openarm_ik/move_joint` and
`/openarm_ik/move_paired_tcp`. Both use action goal/result correlation and finish
only from measured controller feedback. The paired action has explicit named
left and right points in a common stamped source frame. The old
`/openarm_ik/paired_xyz` PoseArray topic remains for one compatibility cycle as
a deprecated validated shim through the same reject-new arbiter.

Action results are authoritative. Periodic and event-driven
`diagnostic_msgs/DiagnosticArray` messages on `/openarm_ik/diagnostics` report
the virtual backend, collision warning, adapter/controller lifecycle, executing
state, coherent feedback masks, sequences, timestamps, ages, and skew. Healthy
unchecked operation is WARN, never OK.

RViz may report four unrealistic finger-inertia errors from the pinned canonical generated URDF. They are inherited model data; meshes and TF still load, and this adapter neither edits nor reinterprets that URDF.

`model/` is installed as an ordinary CMake package (`openarm_model::openarm_model`); the ROS package finds that export instead of compiling monorepo-relative sources. `scripts/install_ros_dependencies.sh` is review-only unless explicitly passed `--apply`; it is never run automatically. After `scripts/build.sh`, test only the authored adapter with:

```bash
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
colcon --log-base ros2_ws/log test \
  --base-paths ros2_ws/src \
  --packages-select openarm_control_msgs openarm_ik_ros \
  --build-base ros2_ws/build \
  --install-base ros2_ws/install
colcon test-result --test-result-base ros2_ws/build/openarm_ik_ros --verbose
```

The explicit package scope prevents the pinned upstream repositories from being mistaken for this workspace's test targets. `scripts/test_ros_coverage.sh` rebuilds the authored adapter with gcov instrumentation and writes a reproducible text report under `ros2_ws/coverage/`.
