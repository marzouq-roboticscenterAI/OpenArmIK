# Unified native and ROS build evidence

Date: 2026-07-29 (America/Los_Angeles)

Branch: `feat/unified-native-build`

Disposition: **DONE**

## Result

`scripts/build.sh` now provides one clean Release build/install entry point for
CAN, model, commission, transport, control, `openarm_description`, and
`openarm_ik_ros`. Native components are configured in dependency order with an
explicit build directory and install prefix. The default output remains
`ros2_ws` for launch compatibility; `--output-root` permits an isolated build,
and `--incremental` is the explicit opt-in to stale-output reuse.

`--tests` enables and runs every CTest registered by the hardware-free native
profile. The transport `vcan0` smoke test is excluded from that profile at
configure time, so the three registered transport tests all run without opening
a CAN socket. Standalone transport builds retain the prior default and register
all four tests, including the smoke test. ROS tests are compiled and enumerated,
but not executed by the hardware-free entry point because ROS runtime tests
start DDS middleware. The script fails unless the current eight ROS drivers are
freshly registered.

CAN now installs its archive and public header. No public API, controller logic,
network configuration, GUI path, or hardware behavior changed. No sudo command
or Python runtime dependency was added; Python remains a build/test interpreter
for the existing model and ROS checks.

## Clean Release build and test

Command:

```bash
./scripts/build.sh --tests \
  --output-root /tmp/openarmik-unified-final \
  --build-type Release
```

The output root did not exist before this invocation. Result: exit 0.
`/tmp/openarmik-unified-final.log` records:

- CAN: 1/1 CTest passed.
- Model: 4/4 CTests passed, including generator determinism.
- Commission: 2/2 CTests passed.
- Transport: 3/3 registered hardware-free CTests passed.
- Control: 3/3 CTests passed.
- Native total: 13/13 passed.
- ROS: `openarm_description` and `openarm_ik_ros` built and installed from clean
  build directories; colcon reported `2 packages finished`.
- Fresh `openarm_ik_ros` CTest enumeration: `Total Tests: 8`.

Every native `CMakeCache.txt` records `CMAKE_BUILD_TYPE:STRING=Release`.
The install prefix contains:

```text
libopenarm_can.a            openarm_can.h
libopenarm_model.a          openarm_model.h
libopenarm_commission.a     openarm_commission.h
libopenarm_transport.a      openarm_transport.h
libopenarm_control.a        openarm_control.h
```

The earlier clean run at `/tmp/openarmik-unified-validation` produced the same
13 native passes, two-package ROS rebuild, eight registered ROS tests, and five
installed native products.

## Safety and compatibility checks

```bash
bash -n scripts/build.sh scripts/build_native.sh scripts/test_ros_coverage.sh
git diff --check
```

Both passed.

Unsafe root checks reject `/`, `/home`, `/tmp`, the repository, its parent, and
the user's home as output/build/install roots. A cleanup escape test placed
`output/install` as a symlink to a sibling directory containing a sentinel.
`scripts/build.sh` exited 2 with:

```text
Refusing cleanup outside output root: .../victim
```

The sentinel remained unchanged. A relative native build root also exits 2.
Cleanup only targets the resolved `native_build`, `build`, `install`, and `log`
children after verifying each lies below the selected output root.

A standalone transport configure with ordinary defaults reported four tests,
including `openarm_transport_vcan_smoke`, confirming that the new option
preserves standalone behavior. The unified build explicitly sets it off.

## Remaining boundaries

- This unifies build, test, and install orchestration; it does not claim that the
  five native libraries form one physical controller runtime.
- The hardware-free path verifies ROS compilation and all eight registrations,
  but intentionally does not execute tests that start DDS. The documented ROS
  and coverage commands remain available when middleware access is authorized.
- The pinned upstream `openarm_description` emits its existing CMake deprecation
  warning and an unused package-specific coverage-variable warning; both ROS
  packages still build successfully.
