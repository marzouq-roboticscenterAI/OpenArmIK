# Independent integration-state verification

Date: 2026-07-29 (America/Los_Angeles)  
Verified tree: `main` at `19a92de`  
Disposition: **DONE_WITH_CONCERNS**

## Bottom line

The repository contains five independently compiled native libraries plus one
ROS package, and the fresh hardware-free native builds tested below pass.  It
does **not** yet form one integrated controller product.  In particular, the
normal controller ABI implements virtual single-joint and paired-TCP motion from
encoder-decoded simulator state, but it neither exposes detection/configuration
nor connects to the transport, CAN, or commissioning libraries.  Manual and
supervised calibration building blocks exist only in the separate
`openarm_commission` ABI.  ROS remains a separate model-only visualization path.

`scripts/build.sh` does **not** build all native libraries.  It configures and
installs only `openarm_model` (with model tests disabled), then builds
`openarm_description` and `openarm_ik_ros`.  It omits `openarm_can`,
`openarm_transport`, `openarm_commission`, and `openarm_control`, and it does not
run tests.

## Git evidence

- `git status --short --branch` before this report: clean `## main`.
- `git diff --check`: clean.
- `git log -8 --oneline --decorate` starts with:
  - `19a92de (HEAD -> main) docs: record clean controller review`
  - `6d698e6 fix: gate watchdog holds on coherent feedback`
  - `5c4f908 fix: materialize cause-aware fault stops`
  - `5589115 fix: close controller ABI and handle safety gaps`
  - `e28357e fix: harden encoder controller safety contracts`
  - `9f19ce7 feat: add encoder-driven controller core`

The only workspace write made by this verification is this requested report.
All fresh configure/build/test output was placed under `/tmp`.

## Build-system and test inventory

There is no root `CMakeLists.txt`, CMake preset, or other unified native build.

| Area | Build/export | Registered tests | Fresh verification |
|---|---|---|---|
| CAN | Standalone CMake, static `openarm_can`; no install/export rules | `openarm_can_tests` | Release build, **1/1 passed** |
| Model | Standalone CMake, static `openarm_model`; installs `openarm_model::openarm_model` package | model unit, ABI-v1 canary, Python URDF reference; optional generator determinism | Release build, **3/3 passed**; generator determinism was not enabled in this fresh run |
| Commission | Standalone CMake, static `openarm_commission`; installs `OpenArm::Commission` package | C++ suite, strict C11 consumer | Release/no-sanitizer build, **2/2 passed** |
| Transport | Standalone CMake, static `openarm_transport`; installs `OpenArm::openarm_transport`; compiles `can/src/openarm_can.c` into an object library | C++ suite, C11 consumer, `vcan0` smoke, install consumer | Release build; non-vcan selection **3/3 passed**; vcan executable built and was deliberately not run |
| Control | Standalone CMake, static `openarm_control`; also builds model as a subdirectory; installs archive/header but no CMake package export | C++ suite, C11 ABI consumer, frozen original-v1 ABI consumer | Release build, **3/3 passed** |
| ROS | Ament/colcon package `openarm_ik_ros`, plus pinned `openarm_description`; links model only | current source registers paired transaction, URDF, no-CAN-linkage, ROS contract, invalid expiry, and three close-helper tests | Existing result reports **10 cases, 0 failures**, but its CTest file has only the older five drivers |

Fresh temporary build directories:

- `/tmp/openarmik-verify-can.kpMDrX`
- `/tmp/openarmik-verify-model.72q3n7`
- `/tmp/openarmik-verify-commission.UBmMjO`
- `/tmp/openarmik-verify-transport.vavgxs`
- `/tmp/openarmik-verify-control.xUdsyp`

The ROS evidence is stale for current main: source
`ros2_ws/src/openarm_ik_ros/CMakeLists.txt` is from commit `0ed82c4` at
12:40:17, while `ros2_ws/build/openarm_ik_ros/CTestTestfile.cmake` was generated
at 12:08:31.  `ctest -N` therefore lists only five tests and omits the three
current `close_rviz_window` argument tests.  A fresh ROS rebuild/test is needed.

## Requested modular-interface audit

| Requested capability | What exists | Verdict |
|---|---|---|
| Virtual detection/configuration | `oa_manifest_create` accepts a complete compiled two-arm configuration; `OA_BACKEND_VIRTUAL` constructs two in-process simulators. `open_and_verify` initializes both simulated arms at zero and reports both verified. | **Partial.** Configuration records exist, but there is no detection/enumeration/candidate API, no custom transport binding, and virtual verification does not discover or compare an external simulated device. |
| Manual calibration building blocks | `openarm_commission.h` exposes create/sample/review/commit/abort/report for stable disabled encoder-reference capture. | **Implemented separately, absent from control ABI.** |
| Supervised calibration building blocks | `openarm_commission.h` exposes a caller-driven recipe state machine returning bounded next-action records and mapping patches. | **Implemented separately, absent from control ABI.** It consumes caller samples/interlocks and emits requests; it does not own a controller or execute actions. |
| Individual joints | `oa_controller_plan_joint` selects side and joint, binds a fresh feedback sequence, targets one joint, and holds the other six at their measured start pose. | **Yes, virtual Stage A.** |
| Paired TCP XYZ IK | `oa_controller_plan_paired_tcp` accepts left/right XYZ, measured sequence requirements, collision-scene revision, branch-step and singularity policy; both arms are planned as one 17-waypoint transaction. | **Yes, virtual Stage A.** It is position-only, orientation-free, and only allowed with the explicitly collision-unchecked virtual policy. |
| Encoder-authoritative state | Simulator plant state is quantized into DaMiao-layout feedback bytes and decoded into raw/mapped snapshots; FK, IK seeds, trajectory starts, and completion use measured snapshots. Frozen feedback does not complete motion. | **Yes for the in-process virtual controller.** There is no physical transport integration. `oa_controller_sim_set_state` is an explicit simulator injection entry point and also passes through quantize/decode. |

The public `openarm_control.h` symbol set confirms the boundary: it contains
manifest creation, controller lifecycle/snapshot/kinematics, joint and paired-TCP
planning, execution, simulator fault/state injection, watchdog/interlock/stop,
and events.  It has no `detect`, `discover`, `interface`, `transport`,
`commission`, `calibration`, or configuration-write operation.

## Remaining integration and product gaps

1. **No unified build or test.** Each native component is a separate CMake
   project.  There is no target that links CAN + transport + commission +
   control + model, and no end-to-end C consumer exercises the combined API.

2. **The controller is not connected to CAN or transport.**
   `openarm_control` links only `openarm_model`.  Its `FakeTransport` and DaMiao
   simulator are private implementations, and it reimplements feedback
   quantization/decoding rather than consuming `openarm_can` or
   `openarm_transport`.  `OA_BACKEND_PHYSICAL` returns `OA_EUNSUPPORTED` before
   verification.

3. **Detection/configuration is not a control capability.** `openarm_can` can
   list Linux CAN interfaces read-only and probe a caller-supplied expected-ID
   manifest; it explicitly cannot identify joint, side, serial, motor family,
   sign, zero, firmware, or configuration.  `openarm_transport` can open an
   already configured interface and publicly transmit only strict queries.
   There is no broad candidate discovery, virtual-device discovery, stable
   adapter matching, baud probing, serial/register verification orchestration,
   isolated motor configuration workflow, or host-link configuration CLI/script.

4. **Commissioning is disconnected.** Manual and supervised recipe sessions do
   not bind exclusive ownership of an `oa_controller`, source samples from its
   coherent snapshots, drive simulator/transport actions, apply a patch to a
   manifest, reload it, or reverify it.  The returned mapping patch is only a
   data record.  There is no integrated simulated calibration test spanning
   controller + commission.

5. **Manifest persistence is reserved, not implemented.**
   `oa_manifest_load(path, sha256_path, ...)` always returns
   `OA_EUNSUPPORTED`; there is no strict schema/digest loader, atomic patch
   writer, draft/armable distinction, or calibration-patch application path.

6. **Packaging is incomplete.** CAN has no install/export rules. Control
   installs an archive and header but no exported target, config, or version
   file. Commission, transport, and model do export packages, but naming is not
   uniform. Transport compiles its own copy of `can/src/openarm_can.c` instead
   of linking the CAN target/package; its archive consequently exports many
   `oa_can_*` symbols but omits `oa_can_linux_list_interfaces`, creating a
   divergent and potentially confusing duplicate codec boundary.

7. **ROS does not exercise the controller ABI.** `openarm_ik_ros` links only
   `openarm_model` and keeps its own last-committed 14-joint virtual posture.
   It does not consume `openarm_control` snapshots/events, the simulated encoder
   plant, the joint-plan API, detection, transport, or commissioning.  Thus the
   currently documented paired-XYZ ROS demo is not an adapter over the new
   controller.

8. **No current full test result.** The fresh component results are good, but
   no fresh sanitizer matrix, model generator-determinism test, current ROS
   build, all-four transport test run, package-install consumer for CAN/control,
   or cross-library integration run was performed here.  The vcan smoke and ROS
   runtime tests were intentionally not run because this audit was forbidden to
   touch CAN/network or launch GUI processes.

9. **Physical functionality remains intentionally incomplete.** There is no
   physical control path, collision engine, commissioned hardware evidence,
   E-stop/deadman hardware integration, qualified limits/recipes, or staged
   hardware acceptance.  This is correct fail-closed behavior, but it means the
   interface cannot yet control physical arms.

10. **Top-level documentation/build entry point is behind the source tree.** The
    README describes model/CAN and the model-only ROS adapter, but does not make
    the new transport/commission/control build products or their disconnected
    status visible through the main build workflow.

11. **Test fault-injection hooks ship in the production archives.** The Release
    `libopenarm_control.a` globally exports four undeclared `oa_control_test_*`
    C symbols, including allocation-failure injection.  Release
    `libopenarm_commission.a` likewise contains externally linkable C++ test
    hooks that force allocation failure/exception or exhaust handle tokens.
    These should be compile-gated into test-only objects rather than linked into
    the installed runtime libraries.

## Exact commands for a complete current-tree build/test sweep

The following is a complete **sequential component sweep**, not a true
integrated product test (none exists). It does not launch RViz or transmit motor
frames. The transport vcan smoke opens/closes a verified `vcan0` if present, so
omit that one test when CAN access is prohibited.

```bash
cd /home/signalprocessing-dev/OpenArmIK
verify_root=$(mktemp -d /tmp/openarmik-full.XXXXXX)

cmake -S can -B "$verify_root/can" -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON
cmake --build "$verify_root/can" --parallel
ctest --test-dir "$verify_root/can" --output-on-failure

cmake -S model -B "$verify_root/model" -DCMAKE_BUILD_TYPE=Release \
  -DOA_MODEL_BUILD_TESTS=ON -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOA_DESCRIPTION_ROOT="$PWD/upstream/openarm_description" \
  -DOA_XACRO_EXECUTABLE="$PWD/.deps/xacro/root/opt/ros/lyrical/bin/xacro" \
  -DOA_XACRO_PYTHONPATH="$PWD/.deps/xacro/root/opt/ros/lyrical/lib/python3.14/site-packages:/opt/ros/lyrical/lib/python3.14/site-packages" \
  -DOA_XACRO_AMENT_PREFIX=/opt/ros/lyrical \
  -DCMAKE_INSTALL_PREFIX="$verify_root/install"
cmake --build "$verify_root/model" --parallel
ctest --test-dir "$verify_root/model" --output-on-failure
cmake --install "$verify_root/model"

cmake -S commission -B "$verify_root/commission" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DOA_COMMISSION_ENABLE_SANITIZERS=OFF
cmake --build "$verify_root/commission" --parallel
ctest --test-dir "$verify_root/commission" --output-on-failure

cmake -S transport -B "$verify_root/transport" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build "$verify_root/transport" --parallel
ctest --test-dir "$verify_root/transport" --output-on-failure
# No-CAN alternative:
# ctest --test-dir "$verify_root/transport" \
#   -R 'openarm_transport_(tests|c11_consumer|install_consumer)$' \
#   --output-on-failure

cmake -S control -B "$verify_root/control" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DOA_CONTROL_BUILD_TESTS=ON
cmake --build "$verify_root/control" --parallel
ctest --test-dir "$verify_root/control" --output-on-failure

source /opt/ros/lyrical/setup.bash
export CMAKE_PREFIX_PATH="$verify_root/install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
colcon --log-base "$verify_root/ros-log" build \
  --base-paths "$PWD/upstream/openarm_description" "$PWD/ros2_ws/src" \
  --packages-select openarm_description openarm_ik_ros \
  --build-base "$verify_root/ros-build" \
  --install-base "$verify_root/install" \
  --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DOPENARM_IK_ROS_COVERAGE=OFF
source "$verify_root/install/setup.bash"
colcon --log-base "$verify_root/ros-test-log" test \
  --base-paths "$PWD/ros2_ws/src" \
  --packages-select openarm_ik_ros \
  --build-base "$verify_root/ros-build" \
  --install-base "$verify_root/install" \
  --event-handlers console_direct+
colcon test-result --test-result-base \
  "$verify_root/ros-build/openarm_ik_ros" --verbose
```

Before this can become one integrated build/test, implementation work is
required: add a root build/export graph, make control depend on explicit codec
and injectable transport interfaces, bind commissioning sessions to controller
snapshots/exclusive lifecycle, add manifest patch persistence/reload, make ROS a
control-ABI adapter, and add a compiled cross-library C integration test.
