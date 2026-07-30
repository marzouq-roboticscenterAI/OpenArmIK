# Final C API / device acceptance

Verdict: **CLEAN** for the `openarm_runtime` acceptance boundary tested on 2026-07-29 at `main` commit `e0d06c86d3fe3cba7c2fc78c4c0b4926de384ad9`. No CAN frame was transmitted, no interface/link state was changed, and no physical device was configured, calibrated, armed, or moved. This verifier changed no product source; it created this report and used only `/var/tmp` for builds, installed artifacts, examples, logs, and V2 persistence data.

## Artifact locations

- Native verification root: `/var/tmp/openarmik-final-accept.U5tFTw`
- Installed native prefix: `/var/tmp/openarmik-final-accept.U5tFTw/prefix`
- External examples: `/var/tmp/openarmik-final-accept.U5tFTw/consumer/acceptance.c`, `/var/tmp/openarmik-final-accept.U5tFTw/consumer/acceptance.cpp`, and their `CMakeLists.txt`
- External example binaries: `/var/tmp/openarmik-final-accept.U5tFTw/consumer-build/acceptance_c11` and `acceptance_cxx17`
- Full ROS/CLI build root: `/var/tmp/openarmik-final-ros.5YzkVY`
- Installed CLI: `/var/tmp/openarmik-final-ros.5YzkVY/install/openarm_ik_ros/lib/openarm_ik_ros/openarm_control_cli`

## Build and strict external consumers

Native build/install/test command:

```sh
./scripts/build_native.sh \
  --build-root /var/tmp/openarmik-final-accept.U5tFTw/build \
  --install-prefix /var/tmp/openarmik-final-accept.U5tFTw/prefix \
  --build-type Release --tests
```

Result: all 16 registered native component tests passed: CAN 1/1, model 4/4, commission 2/2, transport 3/3, control 4/4, runtime 2/2. The runtime test suite passed in 17.90 s. The build script also compiled and ran its installed all-header C11 and C++17 consumers. Full log: `/var/tmp/openarmik-final-accept.U5tFTw/build.log`.

Independent installed-package consumer commands:

```sh
cmake -S /var/tmp/openarmik-final-accept.U5tFTw/consumer \
  -B /var/tmp/openarmik-final-accept.U5tFTw/consumer-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/var/tmp/openarmik-final-accept.U5tFTw/prefix \
  -Dopenarm_runtime_DIR=/var/tmp/openarmik-final-accept.U5tFTw/prefix/lib/cmake/openarm_runtime
cmake --build /var/tmp/openarmik-final-accept.U5tFTw/consumer-build --parallel
/var/tmp/openarmik-final-accept.U5tFTw/consumer-build/acceptance_cxx17
/var/tmp/openarmik-final-accept.U5tFTw/consumer-build/acceptance_c11 \
  /var/tmp/openarmik-final-accept.U5tFTw/persist-rerun
```

Both passed. Generated flags prove that the examples were compiled as strict `-std=c11` and `-std=c++17`, with `-Wall -Wextra -Wpedantic -Werror`, and the only include directory was the installed prefix. They found and linked the installed `openarm_runtime` CMake package, not repository-private headers. Logs: `consumer-configure.log`, `consumer-build.log`, `acceptance-cxx17.log`, and `acceptance-c11-rerun.log` in the native verification root.

## Virtual facade acceptance

The external strict C11 program exercised these conditions end to end:

- Exact capability word `0x0df8`: virtual coordinates, individual joint motion, paired XYZ motion, manual calibration, supervised calibration, interface enumeration, manifest preview, and persistence. Reserved standalone FK/IK and every physical actuation/collision-validation capability were absent.
- Exact virtual inventory: 2 interfaces and 14 motors. Every motor mapped deterministically as `side=index/7`, `joint=index%7`, with `OA_RUNTIME_EVIDENCE_VIRTUAL_EXACT`, no unknown/ambiguous/conflict masks, and no unresolved assignment.
- Armable manifest revision 1 and its 64-character content digest; side-specific model/TCP identities and their model-data, flattened-URDF, source, and coordinate digests.
- Both coherent arm snapshots had expected/fresh masks `0x7f`, SI units, monotonic clock, body-link0 frame, and matching coordinate identity. Both side TCP kinematics were bound to the measured feedback sequence; joint coordinates, TCP transform translation, and TCP XYZ agreed.
- The facade monotonic clock advanced. A foreign clock ID returned `OA_RUNTIME_EINVAL`, and `oa_runtime_get_last_error` reported the runtime facility.
- Manual virtual calibration collected two distinct fresh measured samples, entered review, and committed a one-motor immutable patch at manifest revision 2. The original manifest retained its original evidence record/revision; the new handle alone contained `external-fixture-verified`, manual-fixture evidence, the changed mapping, and a different content digest.
- An unqualified arm-joint recipe was rejected with `OA_RUNTIME_EUNSUPPORTED`. A fully qualified recipe was accepted, but its initial unsafe/not-ready posture produced only `OA_RECIPE_ACTION_HOLD_DISABLED`; abort succeeded and a repeated abort returned `OA_RUNTIME_ESTATE`.
- A measured left joint-0 plan moved by 0.05 rad. The test observed fresh measured feedback still lagging the requested target during execution, then required a matching `STARTED` and `COMPLETED` event and final measured position within 0.001 rad. Observed facade elapsed time was 928,306,339 ns; the final measured timestamp was fresh.
- Named paired body-frame targets were planned and executed for `left_tcp_m=(0.20, 0.30, 0.85)` and `right_tcp_m=(0.20, -0.30, 0.85)`. Both plan residuals and final measured TCP FK errors were at most 0.001 m. A matching `STARTED` and `COMPLETED` event was required; observed facade elapsed time was 17,245,041,613 ns.
- Completion event evidence followed the documented lower-event contract: monotonic event timestamp, nonzero source feedback sequence, and no fabricated per-arm feedback/timestamp validity.
- Every opaque handle kind used by the program was cleaned up. Idempotent/stale-handle destruction was exercised for plan, calibration, inventory, runtime, manifest, and persistence-authority handles without a crash.

Identity values reported by the installed C++17 consumer:

```text
manifest_sha256=ad86f4713ae3a8ead0fc7dde28de78676d3c771845eaee4310b52fc751e0d8c7
coordinate_sha256=d15128a05538ba533f1d8045b44d72bbf47a5501f767a7c46eb2bcedabcaf32d
left model=openarm-v1.0-bimanual-left-body-to-hand_tcp tcp=openarm_left_hand_tcp
right model=openarm-v1.0-bimanual-right-body-to-hand_tcp tcp=openarm_right_hand_tcp
model_data_sha256=9844be1eff37a801b4c48372bc35d8e96c4b872bb9d45c77a9787c4b50774354
flattened_urdf_sha256=dd4b5d5fa0c050b78922bc95850ea65bb15721cb259030cccf106a953f171c55
source_sha256=3f48ffec1598bebca34f90419521d5e320787746b66bf54937c3faeb7c6cb5fc
```

## V2 persistence and replay rejection

The external C11 consumer opened a V2 authority over `/var/tmp/openarmik-final-accept.U5tFTw/persist-rerun` with a caller-owned zero provisioning checkpoint, saved manifest revision 1, then performed an exact-checkpoint CAS to revision 2. Checked load of revision 2 returned an authenticated, checkpoint-authorized manifest and successfully created a virtual runtime. Reusing the stale revision-1 external checkpoint returned `OA_RUNTIME_ESTALE`, returned no manifest, and cleared the observed-checkpoint output.

The current artifact has revision 2 and content digest `0fa11d0b5044e2c47309a78729b1c76228772bed42f697b2c6d3177ecbf8f1c4`; `.previous` has revision 1 and digest `ad86f4713ae3a8ead0fc7dde28de78676d3c771845eaee4310b52fc751e0d8c7`. They are separate mode-0600 inodes. Thus an older authentic artifact cannot satisfy the newer caller-owned checkpoint.

## Read-only device discovery and physical gates

Read-only commands used:

```sh
ip -details link show
lsusb
find/readlink/cat under /sys/class/net, /sys/class/tty, and /sys/bus/usb/devices
```

Observed network links were `lo`, PCI Ethernet `enp4s0` (down/no carrier), PCI Wi-Fi `wlo1` (up), and virtual `tailscale0`. None was an ARPHRD_CAN/SocketCAN link. `oa_runtime_list_interfaces` independently returned zero physical CAN interfaces.

USB enumeration showed Linux root hubs, Genesys hubs, MediaTek wireless, ITE device, Intel RealSense D405, and an integrated camera. There was no USB CAN adapter identity and no `ttyUSB*` or `ttyACM*` device. Captures: `ip-details-link.txt`, `lsusb.txt`, `net-sysfs-summary.txt`, and `usb-sysfs-summary.txt` in the native verification root.

The external program created the SocketCAN-query runtime but queried only the guaranteed-nonexistent interface name `oa_absent`, with zero candidates. The implementation therefore returned immutable empty evidence before opening a transport or sending anything: 0 interfaces, 0 motors, no ambiguity, and no assignment. Physical configuration preview was invalid/not-armable.

The following physical-runtime operations all returned exactly `OA_RUNTIME_EUNSUPPORTED`: snapshot/control state, interlock, arm, configuration apply, manual calibration, recipe calibration, individual joint planning, paired XYZ planning, stop, and disarm. Capability bits for physical configuration, calibration motion, motion, and collision-validated motion were absent.

Source/API inspection found no transmit frame, register write, raw socket, or write-builder entry point in installed `openarm_runtime.h`. The runtime implementation references exactly the typed `oa_can_make_register_query_typed` builder and sends that classified query through the query-only transport. It has no reference to register-write, enable/disable, zero, save, or motion builders. The lower CAN codec intentionally exposes offline encoders, but it has no socket authority; the public transport handle accepts query/status frames only and the passing transport tests verify dangerous classes return `OA_TRANSPORT_EPERMISSION` without reaching the backend.

No live physical evidence could be tested because this device has no CAN interface or CAN/serial USB adapter. Specifically untestable here: actual motor register responses, present-but-ambiguous candidate evidence, duplicate/stale/mismatch/fault evidence, interface bitrate/FD/up metadata for a CAN link, query timeout behavior on a real bus, and correlation of any physical serial to a side/joint. No side/joint assignment was inferred from absence.

## Modular implementation, stable C ABI, and Python independence

The installed native prefix contains six distinct archives and CMake packages: CAN, model, commission, transport, control, and runtime. The implementation is C++ OOP internally (`Controller`, `Transport`, `SocketCanBackend`, manual/recipe calibration session classes, and runtime manifest/inventory/runtime/calibration state types) behind versioned fixed-width ISO-C headers and `extern "C"` translation units. `nm` showed the runtime API as unmangled global `oa_runtime_*` symbols; installed archives contain no test-hook runtime symbols. The strict external C11 consumer is direct evidence that the installed ABI is usable from C, while the C++17 consumer verifies the same installed target from C++.

No controller or runtime archive has Python symbol references. A complete ROS/CLI install was also built under `/var/tmp`:

```sh
./scripts/build.sh --output-root /var/tmp/openarmik-final-ros.5YzkVY \
  --build-type Release
```

All three ROS packages built. `openarm_control_cli` is a native C++ ELF binary; its ELF `NEEDED` entries contain ROS C/C++ libraries and the C/C++ runtime only, with no `libpython` or Python symbols. Executing it with no arguments under the installed environment produced its native usage text and the expected status 1. Python remains a build/test/code-generation and `.launch.py` concern, but is not a runtime dependency of the controller library or CLI executable. Evidence: `/var/tmp/openarmik-final-accept.U5tFTw/api-symbol-audit.log`, `architecture-audit.log`, `cli-dependency-audit.log`, and `/var/tmp/openarmik-final-ros.5YzkVY/build.log`.

## Repository state note

The worktree was already dirty and continued to receive unrelated `.swarm` artifacts during this concurrent verification. `transport/tests/test_transport.cpp` was modified before this verifier began. Those changes were preserved and not edited. No product-source file was changed by this verifier.
