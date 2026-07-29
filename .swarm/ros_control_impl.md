# ROS Stage-A measured virtual control implementation

Date: 2026-07-29  
Branch: `feat/ros-control-sim`  
Status: **DONE_WITH_CONCERNS**

## Implemented contract

- Replaced the model-only `PairedTransactionProcessor` with a C++17
  `VirtualControlSession` that is the sole caller of the stable control C ABI.
- Added `oa_manifest_create_openarm_v10_virtual`, backed by the canonical 2x7
  OpenArm v1.0 virtual manifest with explicit in-process identities.
- The session creates, verifies, arms, advances, snapshots, heartbeats, polls,
  stops, disarms, and destroys the controller on one dedicated steady-clock
  owner thread. Controller time is real elapsed steady time at a 5 ms cadence.
- Added shared reject-new arbitration across joint actions, paired actions, and
  the deprecated PoseArray shim. Cancellation uses `OA_STOP_DISABLE` and leaves
  the adapter in `stopped_requires_restart`.
- Added the separate `openarm_control_msgs` package with explicit `MoveJoint`
  and `MovePairedTcp` actions. Left and right paired targets are named fields,
  transformed together at the request timestamp into `openarm_body_link0`, and
  action results carry UUID, command ID, seed/terminal sequences, lifecycle,
  event, cause, collision status, and reason.
- Replaced command-derived state with coherent decoded `oa_snapshot` state.
  `/joint_states` contains exactly the fourteen arm names and measured q, dq,
  and tau. Measurement time is backdated from the oldest coherent controller
  timestamp. No finger state or TF is published by the adapter.
- Added a UID/domain-scoped nonblocking authority lock. The launch remains one
  adapter plus one `robot_state_publisher`; the latter is the only TF and static
  TF publisher.
- Added periodic and event-driven diagnostics. Healthy virtual unchecked
  operation is WARN, never OK, and reports backend/capabilities, lifecycle,
  executing state, revisions, masks, sequences, timestamps, ages, and skew.
- Added the compiled `openarm_control_cli` with `status`, `move-joint`, and
  `move-paired-tcp`; removed the Python command helper.
- Kept no backend selector, CAN/transport/config/calibration/write endpoint, or
  physical motion path. `/use_sim_time=true` is rejected.

## Verification evidence

- Native control clean local build/install:
  `/tmp/openarmik-ros-control-native-build` and
  `/tmp/openarmik-ros-control-native-install`.
- Native CTest: 4/4 passed, including controller regression, C11 ABI, frozen V1
  ABI, and installed public-header consumer.
- Fresh ROS 2 Lyrical colcon build from empty build/install directories:
  `/tmp/openarmik-ros-fresh-build` and `/tmp/openarmik-ros-fresh-install`;
  `openarm_control_msgs` and `openarm_ik_ros` both built and installed.
- Fresh colcon test: 12 tests, 0 errors, 0 failures, 0 skipped. The four C++
  session tests cover canonical mapping/limits, single reservation, multiple
  encoder-derived intermediate samples, measured q/dq terminal completion,
  paired measured completion, cancellation/disable, restart gating, and
  bounded idempotent close.
- The headless isolated-domain contract test verifies exactly one JointState
  publisher, one TF publisher, one static TF publisher, fourteen state names,
  q/dq/tau arrays, WARN diagnostics, feedback sequence advancement, invalid
  name/frame rejection, measured action completion, duplicate-authority
  rejection, and shutdown within two seconds.
- Live isolated-domain checks completed both compiled CLI motion commands:
  joint command ID 1 and paired TCP command ID 2. Ctrl+C stopped both launch
  processes cleanly.
- `test_no_can_linkage` passed both dependency and socket tracing checks. `file`
  identifies the CLI as an ELF executable; `ldd` shows no Python, CAN,
  transport, or commission dependency.
- `git diff --check` passes. No GUI, CAN, external network, hardware, or
  privileged action was performed.

## Remaining concerns

- The exhaustive sanitizer/TSan, injected feedback fault matrix, randomized
  executor race campaign, and million-cycle soak from the design synthesis were
  not run in this implementation pass. The underlying control suite already
  covers bounded feedback delay provenance, stale/frozen/faulted generations,
  skew, plan invalidation, measured dwell completion, and watchdog behavior;
  the ROS session tests exercise the production owner-thread path without
  exposing simulator injection through ROS.
- RViz was intentionally not launched. Headless launch, state/TF graph, command,
  and graceful shutdown behavior were verified.
