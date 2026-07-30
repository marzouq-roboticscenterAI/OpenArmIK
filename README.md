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
- [transport/README.md](transport/README.md): the query-only transport library.
- [commission/README.md](commission/README.md): manual and supervised
  commissioning building blocks.
- [control/README.md](control/README.md): the virtual controller core.
- [runtime/README.md](runtime/README.md): the installed product facade and
  immutable identity/capability authority used by ROS.

Automatic CAN discovery cannot establish physical joint identity, side, sign,
zero, gearing, firmware, or a safe motor mapping. Consequently this revision
does not query, configure, or move physical arms through the runtime. A
physical backend must remain
disarmed until those facts are commissioned and an independent E-stop and
watchdog are installed.

## Run

```bash
./scripts/fetch_upstreams.sh
./scripts/build.sh
./scripts/launch_rviz.sh
```

The build command performs a clean Release build in dependency order (CAN,
model, commission, transport, control, runtime, then ROS), installs everything under
`ros2_ws/install`, and never uses sudo or configures an interface. To compile
and run every registered hardware-free native CTest while verifying that all
13 ROS tests are freshly registered, use:

```bash
./scripts/build.sh --tests
```

The transport `vcan0` smoke test is not registered in this hardware-free
profile. Standalone transport builds retain that test by default. Use
`--incremental` only when deliberately reusing prior output. A disposable build
can be isolated with `--output-root /tmp/openarmik-build`; its setup file is
then `/tmp/openarmik-build/install/setup.bash`. Output roots may contain spaces,
but `:` and `;` are rejected because they are prefix-list delimiters used by
ROS and CMake.

The reusable native helper overwrites and verifies each component's cached
dependency directory on every invocation. Its two-prefix reuse regression can
be run without ROS:

```bash
reuse_root=$(mktemp -d /tmp/openarmik-prefix-reuse.XXXXXX)
./tests/test_native_prefix_reuse.sh "$reuse_root"
```

On this Wayland hybrid-GPU laptop the launcher uses Mesa software OpenGL with
the XWayland/GLX backend required by this RViz/Ogre build. Hardware GLX remains
available with `OPENARM_RVIZ_RENDERER=nvidia` or `integrated`, but it flickers
during live window resize on this host. The launcher also disables HiDPI render
target scaling for RViz only, closes the RViz window before stopping ROS so
`Ctrl+C` shuts down cleanly, and holds a single-instance lock to prevent
duplicate joint-state/TF publishers. The window-close path is a compiled C++17
Xlib utility installed with the ROS package; it has no Python runtime dependency.

The launch starts only `robot_state_publisher`, the virtual adapter, and RViz.
`robot_state_publisher` is the only TF authority. The pinned
`openarm_description` package supplies mesh URI resolution. Launch uses a
derived visualization-only URDF with fixed, explicitly unmeasured fingers;
the canonical generated URDF remains installed unchanged for model identity.

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
the runtime backend, immutable manifest/model/TCP/scene identity, exact virtual
inventory, capability/persistence/calibration exposure, collision warning,
adapter lifecycle, coherent feedback masks, sequences, timestamps, ages, and
skew. Healthy unchecked operation is WARN, never OK.

The native install exports CMake targets for dependency-safe consumption:
`OpenArm::Can`, `OpenArm::Model`, `OpenArm::Transport`, `OpenArm::Commission`,
`OpenArm::Control`, and `OpenArm::Runtime`. Existing model and transport target names remain
available. Control discovers the installed model package instead of compiling a
second model copy, and transport links the installed CAN target instead of
embedding CAN objects. The production ROS session links only the installed
`OpenArm::Runtime` facade; only the separate portal geometry helper consumes
the model target directly. Production archives do
not contain the native suites' fault-injection hooks.

`scripts/install_ros_dependencies.sh` is review-only unless explicitly passed
`--apply`; it is never run automatically. `./scripts/build.sh --tests` is the
preferred hardware-free native test and ROS registration path. To explicitly
execute the already-built authored adapter tests (which start ROS middleware):

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
