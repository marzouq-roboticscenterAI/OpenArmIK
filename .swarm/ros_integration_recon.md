# ROS 2 measured-simulator integration reconnaissance

Status: **DONE_WITH_CONCERNS**  
Inspected: clean `main` at `19a92de` (2026-07-29). No runtime, GUI, CAN, or
network action was taken.

## Finding

The present `openarm_ik_ros_node` cannot be extended by merely changing its
publisher. It owns a separate `PairedTransactionProcessor`, solves directly
with `openarm_model`, replaces its stored joints with the IK result, and
immediately publishes that command result. Replace that state owner with one
C++17 RAII wrapper around `oa_manifest`/`oa_controller`; do not retain the old
processor beside it. The only arm positions ever placed in `/joint_states`
must be `oa_snapshot.arm[side].q` (and likewise `dq`, `tau`) obtained after
`oa_controller_advance`. Never publish `oa_motion_plan_report.target_q` or an
incoming target as state.

The control API already provides all required motion semantics:

- startup: `oa_manifest_create` -> `oa_controller_create` with
  `OA_BACKEND_VIRTUAL` -> `oa_controller_open_and_verify` ->
  `oa_controller_get_arm_challenge` -> `oa_controller_arm`;
- state: `oa_controller_advance`, `oa_controller_snapshot`, and optionally
  `oa_controller_get_kinematics` bound to the snapshot feedback sequence;
- one joint: `oa_controller_plan_joint`;
- paired claw/TCP XYZ: `oa_controller_plan_paired_tcp` (the “claw” is exactly
  `openarm_{left,right}_hand_tcp`, position-only/free orientation);
- execution: `oa_motion_plan_get_report`, `oa_controller_execute`, periodic
  `oa_controller_heartbeat`, `oa_controller_poll_event`;
- fail closed: `oa_controller_stop(..., OA_STOP_DISABLE)` and destruction.

Both plans bind fresh measured sequence(s), measured start pose, manifest,
model, controller, verification epoch, and collision-scene revision. Completion
is already based on new measured q/dq (plus measured FK for TCP) for a three-cycle
dwell. The adapter must not recreate those decisions.

## Smallest correct adapter

Keep one node/executable and replace `paired_transaction.*` with a small
`VirtualControlSession` library used by the ROS node and unit tests.

1. Construct the exact 2x7 virtual manifest now used by
   `control/tests/test_control.cpp::valid_config` (joint limits must match the
   generated URDF; J1/J2 DM8009, J3/J4 DM4340, J5-J7 DM4310, unique virtual
   buses/IDs/serials and validated affine mappings). This duplication is a drift
   risk; preferably move the production simulator-manifest builder into
   `control` rather than copying a test helper into ROS. `oa_manifest_load` is
   reserved and currently returns `OA_EUNSUPPORTED`.
2. Use virtual options only, with
   `collision_policy=OA_COLLISION_VIRTUAL_UNCHECKED` and scene revision 1.
   Report `collision_checked=false` on every acknowledgement. Do not expose a
   backend selector or link the CAN library. A reasonable non-RT ROS policy is
   a 5 ms wall timer with a 20 ms controller maximum cycle and 100 ms feedback
   timeout: pass actual elapsed `steady_clock` nanoseconds to `advance`; a gap
   over the configured cycle must fault rather than be hidden by synthetic
   fixed time. Run callbacks in the current single-threaded executor.
3. On every successful tick: snapshot, require both `fresh_mask==expected_mask`,
   zero fault masks, allowed lifecycle, and cross-bus skew within the configured
   bound; then publish exactly the 14 measured arm joints. Poll events and renew
   the active command heartbeat. On any advance/integrity/heartbeat failure,
   attempt disable-stop, latch the adapter fault, reject later commands, emit an
   ERROR diagnostic, and cease publishing apparently-live joint states. Never
   auto-reset, reverify, or rearm.
4. For each command, validate the ROS envelope first, take one fresh snapshot,
   populate `required_feedback_seq`, plan, inspect duration/report, create
   overflow-checked absolute controller deadlines, execute, destroy the plan,
   and retain only the returned command ID. No queue or preemption: reject a new
   request while lifecycle is executing. Use `OA_STOP_DISABLE` for expiry,
   explicit stop, shutdown, and unexpected adapter errors.
5. Stamp measured state, not receipt time. Map controller sample age into ROS
   time: `JointState.header.stamp = ros_now - (controller_now - arm.t_ns)` after
   confirming the two arm timestamps are coherent. Include feedback sequences,
   sample age, lifecycle, event, cause, command ID, request stamp, and
   `state_source=oa_snapshot_encoder_feedback` in diagnostics.

The controller has no gripper DOF. Publishing two zero finger-source joints
would not be encoder authority. Prefer publishing the 14 measured arm joints;
verify how `robot_state_publisher`/RViz renders the two prismatic/mimic finger
chains. If complete finger TFs require their source joints, label the two zero
values explicitly as immutable visualization constants, not measured state.

## Exact ROS contract (minimal and backward-compatible)

- Subscribe `/openarm_ik/paired_xyz`, `geometry_msgs/msg/PoseArray`, reliable
  volatile depth 10. Require a nonzero fresh stamp, frame `world`, exactly two
  poses ordered left/right, and finite XYZ. Ignore orientation explicitly.
- Add `/openarm_ik/joint_target`,
  `trajectory_msgs/msg/JointTrajectory`, same QoS. Require exactly one of the 14
  canonical arm joint names, exactly one point and one finite position, no
  velocity/acceleration/effort arrays, zero `time_from_start`, and a nonzero
  fresh header stamp. Joint indexes passed to the C API are zero-based.
- Publish `/joint_states`, `sensor_msgs/msg/JointState`, reliable volatile depth
  10, only from coherent controller snapshots.
- Publish `/openarm_ik/diagnostics`,
  `diagnostic_msgs/msg/DiagnosticArray`, for `rejected`, `accepted`, `started`,
  `settling`, `completed`, `aborted`, and `faulted`. Echo the request stamp so
  the CLI can correlate an acknowledgement; include the C `oa_status` and
  `command_id`.
- Optional but useful safe operation: `/openarm_ik/stop_disable`,
  `std_srvs/srv/Trigger`, mapped only to `OA_STOP_DISABLE` (never controlled
  hold). There are otherwise no services/actions in the minimal design.

Retain `scripts/send_paired_xyz.py`, but make it ignore unrelated diagnostics
and wait for the matching request stamp and terminal event. Add
`scripts/send_joint_target.py JOINT_NAME TARGET_RAD [--timeout ...]` with the
same discovery, one-shot publish, correlation, terminal-result wait, and
nonzero exit on rejection/fault/timeout. Merely receiving an “accepted” plan is
not success.

## Build, launch, and authority changes

`control` currently installs a static archive/header but no CMake package
export. Add the same export pattern used by `model`: build/install interfaces,
`openarm_control::openarm_control`, `openarm_controlConfig.cmake`, version file,
and `find_dependency(openarm_model)`. Then:

- `scripts/build.sh`: install `model`, then install `control` with
  `OA_CONTROL_BUILD_TESTS=OFF` into `ros2_ws/install`, then run colcon. Linking
  monorepo-relative control/model sources from the ROS package is brittle and
  can create a second model target.
- ROS CMake: `find_package(openarm_control CONFIG REQUIRED)` and link the session
  to `openarm_control::openarm_control`; add `trajectory_msgs` and, if the stop
  service is included, `std_srvs`. Remove direct `openarm_model` use and the old
  transaction target/tests. Keep `sensor_msgs`, `geometry_msgs`,
  `diagnostic_msgs`, `rclcpp`, description, X11 helper, URDF installation, and
  no-CAN linkage test.
- `package.xml`: add ROS dependencies for `trajectory_msgs`/`std_srvs`; document
  the locally installed non-ament `openarm_control` dependency as is already
  done operationally for `openarm_model`.
- `openarm_ik_rviz.launch.py`: retain exactly one adapter and one
  `robot_state_publisher`; the adapter publishes no TF. Retain optional RViz.
  Do not add `joint_state_publisher`, `static_transform_publisher`,
  `ros2_control`, or a second state node.
- The repository has `scripts/launch_rviz.sh`, not `scripts/run_rviz.sh`.
  Update that existing script/README names only. Its host-local `flock` already
  guards the GUI workflow; ideally the adapter itself holds a lock keyed by UID
  and `ROS_DOMAIN_ID` before advertising `/joint_states`, so direct headless
  launches also reject a second authority. ROS/DDS offers no portable exclusive
  publisher guarantee, so multi-host deployments still require domain/namespace
  isolation and graph monitoring.

`robot_state_publisher` remains the sole `/tf` and `/tf_static` authority. Its
fixed `world -> openarm_body_link0` comes from the generated URDF; its dynamic
TF is driven only by the adapter's measured `/joint_states`. Tests must assert
one `/joint_states` publisher, one `/tf` publisher, no joint-state-publisher
process, and failure of a second adapter instance.

## Required proof/tests

1. Preserve all `control` strict/sanitizer tests. Add session unit tests for
   startup/RAII, exact names/config, one-joint mapping and bounds, paired atomic
   rejection, busy/stale/NaN/expiry/deadline rejection, command-event mapping,
   heartbeat, disable on shutdown/error, and no post-fault command acceptance.
2. Replace the current live ROS contract test. It must see an initial measured
   snapshot; after a joint target it must observe multiple intermediate q values
   before the target and only later a measured `COMPLETED` event. Other joints
   must remain at their measured starting values. Repeat for paired XYZ and
   verify both TF TCPs converge together from measured JointState.
3. Launch with a small `oa_sim_fault.feedback_delay_ns` on both arms (test-only
   or a bounded virtual-only parameter below the feedback timeout). Assert
   JointState/TF stamps carry that sample age, feedback sequence advances, q is
   not replaced by the plan target at acceptance, intermediate measured q is
   visible, and completion occurs only after final measured q/dq/FK dwell. This
   is the decisive RViz provenance test. Note: the present fault injection
   backdates feedback timestamps; it does **not** queue old plant values. Plant
   lag plus decoded snapshot-only publication proves command/state separation,
   while the header age proves timestamp propagation—do not claim a value-delay
   queue that the simulator does not implement.
4. Fault/freeze tests must show frozen encoders never complete, stale/incoherent
   feedback stops new JointState publication, and fault diagnostics carry the
   core cause. Keep `test_no_can_linkage.py`; check both `ldd` and `strace`.
5. CLI tests must inject unrelated diagnostics and prove correlation, terminal
   wait, timeout, and nonzero failure exit. Keep URDF/mesh validation and invalid
   parameter bounds. Run the full headless launch under ROS 2 Lyrical.

## Integration risks

- `feedback_delay_ns` is timestamp age, not delayed-value buffering; overstating
  it would invalidate the evidence claim.
- ROS wall-timer scheduling is not real time. A timer period equal to the control
  maximum will nuisance-fault; a faster timer with the actual-time watchdog is
  required, and load-induced faults remain intentionally fail-closed.
- `OA_COLLISION_VIRTUAL_UNCHECKED` is necessary for current motion because no
  collision engine exists. The adapter remains virtual visualization, not
  physical motion authorization.
- The compiled manifest helper and generated URDF limits can drift unless one
  production config source is shared.
- Topic-plus-diagnostic acknowledgement is not transactionally unique without
  echoed request stamps; concurrent producers remain unsupported. A later custom
  service/action is preferable if multi-client command arbitration is required.
- Process locks do not prevent a publisher on another host/domain participant;
  deployment must enforce one authority beyond the single-host demo.
