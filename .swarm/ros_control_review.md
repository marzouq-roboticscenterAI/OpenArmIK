# Independent ROS/control review of `b3993ba`

Date: 2026-07-29 (America/Los_Angeles)
Compared: `b3993ba85fe34ed1af971eec01bc3af4a94b0663` against `29657f7`
Disposition: **FINDINGS**

No GUI, CAN, external network, hardware, commissioning, or physical-control
operation was performed. The implementation was not modified; this report is
the only worktree write.

## Critical

None.

## Important

### I1. SIGINT during an active action aborts the node and loses the terminal result

`VirtualControlSession::shutdown_on_owner_thread()` disable-stops an active
command and then invokes its terminal callback
(`virtual_control_session.cpp:679-706`). The ROS callback calls
`goal->abort(result)` without an exception boundary
(`openarm_ik_ros_node.cpp:365-393`). During SIGINT, `rclcpp::spin()` has already
returned and action teardown can remove the goal before the session is closed
from the node destructor (`openarm_ik_ros_node.cpp:188-193,637-649`). The callback
then throws through the session's `run() noexcept`, causing `std::terminate`
rather than bounded graceful shutdown.

Isolated-domain reproduction against the fresh install:

1. Start `openarm_ik_ros_node` in domain 197.
2. Start the compiled CLI goal
   `move-joint openarm_left_joint4 1.5` and allow execution to begin.
3. Send SIGINT to the node.

Observed node exit was abnormal (`ros2run` reported process failure 250) with:

```text
terminate called after throwing an instance of 'std::runtime_error'
  what():  Asked to publish result for goal that does not exist
```

The client received no terminal action result and exited 5 with `terminal
result timeout`. The node stopped within about 1.14 s, and the controller
disable-stop is attempted before the throwing callback, but the required clean,
exactly-once terminalization and exception-safe destruction are not met.

### I2. The documented clean build path cannot build this commit

The new ROS target requires both `openarm_control` and
`openarm_control_msgs` (`openarm_ik_ros/CMakeLists.txt:14-15`), but
`scripts/build.sh` still builds/installs only `model` (`scripts/build.sh:10-15`)
and its colcon selection omits `openarm_control_msgs`
(`scripts/build.sh:23-29`). On a clean checkout it therefore has neither an
installed `openarm_controlConfig.cmake` nor a built message package. The review's
fresh colcon build succeeded only after separately building/installing model and
control and explicitly selecting `openarm_control_msgs`; the advertised
`./scripts/build.sh` workflow is not self-contained.

### I3. Cancellation is not correct for the reserved/pending phases

`cancel()` accepts any matching reserved owner (`virtual_control_session.cpp:
125-133`), but `process_cancel()` calls `OA_STOP_DISABLE` only when `active_`
exists (`593-617`). If cancellation wins while merely reserved, no command
object exists, so no terminal callback is issued; nevertheless the adapter
clears ownership and reports `stopped_requires_restart` (`618-642`) while the
controller remains armed.

A direct test of the production session performed `reserve("reserved")` then
`cancel("reserved")`; both returned true, after 30 ms the adapter was
`stopped_requires_restart`, but the snapshot lifecycle was 4
(`OA_LIFECYCLE_ARMED_IDLE`), not disabled/disarmed. If the ROS accepted callback
subsequently tries to submit, it instead aborts the goal as a reservation
mismatch, despite cancellation having been accepted. This violates cancellation
coverage for reserved/planning phases and makes the restart gate disagree with
the underlying lifecycle.

For an executing cancellation, stop does occur, but no post-stop snapshot is
taken. Diagnostics continue to publish the pre-stop snapshot lifecycle via
`openarm_ik_ros_node.cpp:546-594`, so they can report controller lifecycle
`EXECUTING` while `executing=false` and the adapter says
`stopped_requires_restart`.

### I4. The deprecated PoseArray terminal acknowledgement can be attributed to a different request

The compatibility topic has no action result, so its terminal diagnostic must
retain the exact legacy request identity. Instead all ingress paths share the
single `last_action_`, `last_owner_`, and `last_request_stamp_ns_` record
(`openarm_ik_ros_node.cpp:514-533`). The legacy terminal callback updates only
`last_committed_` and `last_reason_` (`453-458`). A concurrent rejected request
can therefore replace the action/owner while leaving the legacy stamp, after
which legacy completion sets `committed=true` on that mixed record.

This was reproduced in isolated domain 232: a valid legacy paired command began
as owner `legacy:1785361972535783345`; an invalid `MoveJoint` goal was then
rejected. The terminal diagnostic reported:

```text
last_action=move_joint
last_goal_id=3f531388795f4fcc9caea21b4f3ed9ea
request_stamp_ns=1785361972535783345
committed=true
reason=completed_measured_feedback
```

Thus a legacy client can observe a successful-looking acknowledgement whose
identity belongs partly to a rejected action goal. The compatibility shim is
not safely correlated for its promised migration cycle.

### I5. The completion arbiter is released before terminal result publication

On `OA_EVENT_COMPLETED`, `complete_active()` clears `active_`, owner, and the
reservation under the mutex (`virtual_control_session.cpp:526-540`) before it
invokes the action terminal callback (`541-556`). A concurrent ROS goal can
therefore reserve and begin planning before the prior goal's terminal result has
been published. This contradicts the specified reject-new reservation spanning
terminal result publication and creates a result/diagnostic race, even though
the controller itself has already emitted completion.

## Minor

### M1. CLI timeout does not ensure cancellation or stop

On terminal timeout the compiled CLI calls `async_cancel_goal()` and immediately
returns (`openarm_control_cli.cpp:83-88`); `main()` then shuts down ROS
(`110-143`). It does not spin for the cancel response or terminal cancellation,
so a goal can continue server-side after the CLI reports timeout. Discovery,
rejection, cancellation, and ordinary completion exit codes are otherwise
distinct and matching action-handle correlation is used.

### M2. Manifest identity and ROS acceptance tables have duplicate sources of truth

The production manifest fixes the current 2x7 names, sides, limits, motor
families, affine sign/zero mapping, SI units, and virtual identities in
`control/src/standard_manifest.cpp:18-82`. The current values match the model
limits and canonical left/right names. However, ROS independently hard-codes
the names and limits in `virtual_control_session.cpp:28-35,784-817` rather than
querying or validating against the created manifest. The only new manifest test
checks successful creation (`control/tests/test_control.cpp` near
`test_manifest_validation`), not every name/side/index/limit/motor/sign/zero
field. Current mapping is correct, but future manifest drift can silently make
ROS validation target a different numeric side/joint contract.

### M3. Diagnostics omit part of the promised stable provenance

Periodic diagnostics correctly remain WARN for healthy unchecked virtual
execution and expose source, masks, sequences, timestamps, ages, skew,
revisions, owner, event, and cause (`openarm_ik_ros_node.cpp:544-608`). They do
not expose capability bits, plan seed sequences, plan duration, or terminal
measured sequences. Fault/shutdown action results also omit the active plan's
seed sequences (`virtual_control_session.cpp:645-706`). This weakens exact fault
and invalidation provenance, though normal completed action results contain the
seed and terminal sequences.

## Verified clean properties

- Launch contains exactly one adapter and one `robot_state_publisher`; runtime
  graph inspection found one `/joint_states`, one `/tf`, and one `/tf_static`
  publisher. The adapter has no TF broadcaster. A duplicate local adapter was
  rejected by the UID/domain authority lock.
- Every observed `JointState` contained exactly the 14 canonical arm names and
  14 q/dq/tau values copied from coherent `oa_snapshot` feedback. No finger or
  command-target state is published. Multiple decoded intermediate samples
  precede measured q/dq dwell completion.
- Named `left_tcp_m`/`right_tcp_m` action fields, a common stamped source frame,
  transactional body-frame transformation, finite checks, joint-name/limit
  checks, measured planning seeds, unchecked-collision result flags, action
  UUIDs, command IDs, reject-new arbitration, heartbeat, steady controller
  time, and `/use_sim_time=true` rejection are present.
- Runtime arbitration race in domain 199 accepted exactly one of simultaneous
  valid joint and paired goals; the loser was rejected and executing cancel
  produced a canceled result. Invalid names and missing frames were rejected.
- Healthy diagnostics are WARN, fault/closing diagnostics are ERROR, and
  `collision_checked=false`, `physical_motion_authorized=false`, and
  `state_source=oa_snapshot_encoder_feedback` are explicit.
- No ROS backend/CAN/interface/calibration/commission/persistence endpoint is
  present. Dependency/symbol inspection and the syscall test found no CAN,
  transport, commission, Python runtime, or physical-control linkage in the node
  or compiled CLI. DDS network sockets are the expected ROS transport.
- The action interface package builds and exports generated Lyrical type
  support. Python additions are limited to launch, tests, and generated
  interface support; node and CLI are ELF C++ executables.

## Test evidence

- Fresh model native tests: 3/3 passed.
- Fresh control native tests/ABI/install consumer: 4/4 passed.
- Fresh Lyrical colcon build: `openarm_description`,
  `openarm_control_msgs`, and `openarm_ik_ros` all built and installed.
- Authored `openarm_ik_ros` tests: 8/8 passed, including the session suite,
  no-CAN syscall check, generated URDF, expiry validation, runtime graph/action
  contract, and bounded idle shutdown.
- Full three-package colcon test aggregate was red only in the unchanged pinned
  upstream `openarm_description` lint suite (flake8/pep257 and an xmllint
  timeout); authored message/adapter tests passed.
- ASan/UBSan: native control tests 3/3 and production session tests 4/4 passed
  with leak/error halting enabled.
- TSan: native control tests 3/3 and production session tests 4/4 passed with
  halt-on-error enabled.
- `git diff --check 29657f7 b3993ba` reports two trailing-space lines in the
  implementation report only; no executable source whitespace error was found.

Not claimed: exhaustive injected freeze/drop/skew/feedback-delay ROS fault
matrix, million-cycle soak, randomized multi-threaded executor campaign, or GUI
rendering. Simulator fault injection is intentionally absent from the ROS API;
the underlying native control suite covers those controller mechanisms, but the
adapter fault-to-action/diagnostic boundary is not exhaustively exercised.
