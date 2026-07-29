# ROS Stage-A measured virtual control implementation

Date: 2026-07-29
Branch: `feat/ros-control-sim`
Status: **DONE**

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

## Second-review fault closure

- State, feedback, and terminal callbacks now report success to the session.
  A callback rejection or exception is converted to a non-success control
  status and latches `AdapterState::fault`; it is no longer swallowed at a ROS
  publication boundary.
- The worker gates both measured-state and feedback publication immediately
  after a fault. Fault terminalization moves active/pending authority exactly
  once, clears the owner, preserves the first cause, and prevents later
  reservations.
- Fault handling always refreshes an authoritative native snapshot after its
  best-effort disable stop. This includes native deadline faults for which the
  core is already in `OA_LIFECYCLE_FAULT` and rejects stop with
  `OA_CONTROL_ESTATE`.
- Cancellation stops if event draining has already faulted and terminalized the
  command. Shutdown also preserves an already-latched fault lifecycle, event,
  and cause rather than relabeling it as an ordinary abort.

## Verification evidence

- A clean unified RelWithDebInfo build from empty output directories built and
  installed all native components plus `openarm_description`,
  `openarm_control_msgs`, and `openarm_ik_ros` under
  `/home/signalprocessing-dev/openarmik-ros-control-unified-clean`.
- Native CTest: 14/14 passed across CAN, model, commission, transport, control,
  ABI, and installed-consumer coverage.
- Installed ROS/runtime CTest: 9/9 passed sequentially. The production session
  suite now contains 13 cases, including throwing state/feedback/terminal
  callbacks, idle and active 35 ms native deadline overruns, authoritative
  lifecycle/cause reporting, publication freeze, exact-once terminalization,
  and authority-leak rejection.
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
- ASan+UBSan with halt-on-error and leak detection passed the native control
  suite 3/3 and the production session suite 13/13. TSan with halt-on-error
  passed the same native 3/3 and session 13/13 suites.
- `git diff --check` passes. No GUI, CAN, external network, hardware, or
  privileged action was performed.

## Scope limits

- The targeted sanitizer suites above were run; a randomized executor race
  campaign, exhaustive injected fault matrix, and million-cycle soak were not.
- RViz was intentionally not launched. Headless launch, state/TF graph, command,
  and graceful shutdown behavior were verified.
