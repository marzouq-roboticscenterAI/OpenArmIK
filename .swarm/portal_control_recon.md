# Local portal control reconciliation

Date: 2026-07-29 (America/Los_Angeles)  
Tree inspected: `main` at `7c0398a`  
Status: **DESIGN COMPLETE; VIRTUAL PORTAL IMPLEMENTABLE AFTER ONE CONTROL-ABI ADDITION; PHYSICAL MOTION/CALIBRATION UNAVAILABLE**

This was a read-only inspection of the current control, model, commissioning,
ROS package/launch, tests, and relevant `.swarm` design reports. The only write
is this requested report. No build, executable, GUI, network, CAN, or hardware
operation was performed.

## Bottom line

The current ROS node is not a controller adapter. It calls `openarm_model`
directly, instantly stores IK output as state, publishes two fabricated zero
finger joints, and accepts only a paired `PoseArray`. It never constructs,
advances, snapshots, heartbeats, stops, resets, or polls an `oa_controller`.
The launch correctly has one `/joint_states` publisher and leaves TF to
`robot_state_publisher`, but the published arm state is commanded IK output,
not decoded simulator feedback.

A correct local portal should be a compiled C++ ROS client of one replacement
`openarm_ik_ros_node`. The node should own one `oa_controller` through a
single-owner C++17 session, publish measured joint/TCP state, and expose ROS 2
actions/services. The portal must never own a model, controller, joint-state
publisher, or TF broadcaster.

One control-ABI addition is mandatory for honest independent left/right XYZ
buttons: a single-arm TCP plan. The existing ABI has only
`oa_controller_plan_paired_tcp`. Supplying the other target from an editable or
cached portal field guesses operator intent. Supplying its freshly measured TCP
does not guess position, but still does not provide independent-arm semantics:
the paired planner re-solves both chains, sets `active_arm_mask=0x3`, may change
the other arm's redundant posture, trajectories both arms, and requires both
arms to settle. That is not an acceptable production shim.

The portal can be fully functional against the virtual backend after the new
single-TCP symbol and a production virtual-manifest builder are added. It must
always show `collision_checked=false`; the current core has no collision engine
and permits motion only under `OA_COLLISION_VIRTUAL_UNCHECKED`. Therefore
"safe bimanual" currently means coherent measured-start binding, one-command
arbitration, exact inactive-arm joint hold, watchdogs, and stop-both fault
policy. It does **not** mean collision-safe motion. Physical verification
already fails with `OA_CONTROL_EUNSUPPORTED`, and physical auto-calibration is
not present.

## Current compiled capability and gaps

What is usable now in `openarm_control`:

- a virtual two-arm encoder-decoded plant with coherent snapshots;
- `oa_controller_get_kinematics` bound to an exact feedback sequence;
- measured-seed single-joint and paired-TCP planning;
- plan identity/revision/sequence/start-pose binding;
- measured q/dq and measured-FK completion with a three-feedback-interval dwell;
- heartbeat, control-cycle, feedback, skew, fault, and event overflow handling;
- stop-both, latched `FAULT`/`ESTOP`, nonce-based reset, and re-verification; and
- a fail-closed physical backend.

Gaps that directly affect the portal:

1. There is no single-arm Cartesian planning call.
2. The fixed virtual 2x7 manifest exists only as test construction in
   `control/tests/test_control.cpp::valid_config`; `oa_manifest_load` is
   unsupported. ROS must not copy a test fixture.
3. There is no real collision validator. `OA_COLLISION_REJECT_ALL` blocks all
   plans; `OA_COLLISION_VIRTUAL_UNCHECKED` is virtual-only and reports
   unchecked.
4. `openarm_commission` is a disconnected, transport-free session library. It
   consumes samples and emits mapping patches or abstract bounded next actions;
   it cannot operate the controller, write/apply a manifest, or calibrate an
   arm. Arm-joint recipes require explicit hardware qualification. Only a
   gripper recipe may carry simulation-only evidence, and the controller has no
   gripper plant.
5. The current ROS target links only `openarm_model`; `scripts/build.sh` does not
   install control. ROS has no actions/services, controller worker, E-stop,
   reset, calibration, or completion correlation.
6. The control snapshot has no gripper feedback. The current two zero finger
   positions cannot be called measured.

## Required native ABI addition

Add a new record and symbol without changing any existing V1 layout or symbol:

```c
#define OA_PLAN_TCP UINT32_C(3)

typedef struct oa_tcp_move {
    uint32_t struct_size;
    uint32_t abi_version;                 /* OA_CONTROL_ABI_V1 */
    uint64_t expiry_ns;
    uint64_t required_feedback_seq[2];   /* bind one coherent bimanual snapshot */
    oa_side side;                        /* OA_LEFT or OA_RIGHT */
    uint32_t reserved0;
    double tcp_m[3];                     /* openarm_body_link0, metres */
    double velocity_scale;
    double acceleration_scale;
    double jerk_scale;
    double tcp_tol_m;
    uint64_t collision_scene_revision;
    double max_branch_step_rad;
    double min_singular_value;
} oa_tcp_move;

oa_control_status oa_controller_plan_tcp(
    oa_controller *controller,
    const oa_tcp_move *request,
    oa_motion_plan **out);
```

Exact core semantics:

- require `ARMED_IDLE`, healthy coherent fresh feedback, both requested
  sequences equal to the current sequences, unexpired input, valid scales and
  finite values, and the current scene revision;
- snapshot both measured joint vectors once;
- set `active_arm_mask = 1U << side`, `kind = OA_PLAN_TCP`, and bind both start
  vectors/sequences/revisions exactly as paired planning does;
- solve only the selected chain through predecessor-seeded Cartesian waypoints;
- copy the inactive measured `start_q` into every inactive waypoint and its
  `target_q`, without invoking inactive-arm IK;
- run the collision policy over the complete two-arm path when a real validator
  eventually exists; until then allow only explicitly unchecked virtual mode;
- retain the existing execute-time all-joint start-drift check, so movement of
  either arm invalidates the plan;
- command both seven-joint sets during execution, holding the inactive arm at
  its measured start vector; and
- apply TCP completion only to the active side. The existing
  `measured_at_goal()` already honors `active_arm_mask`.

The existing `oa_motion_plan_report` is sufficient for the first integration:
the adapter verifies `kind==OA_PLAN_TCP`, both seed sequences, and that every
inactive `target_q` equals its captured measured start. Exposing
`active_arm_mask` in a new report version would improve auditing, but is not a
prerequisite and the existing V1 report must not be grown in place.

Also add the production helper previously specified in
`.swarm/ros_design_synthesis.md`:

```c
oa_control_status oa_manifest_create_openarm_v10_virtual(oa_manifest **out);
```

It must be the one source for the standard virtual manifest used by control
tests and ROS. It fixes canonical names, revisions, URDF limits, motor families,
unique virtual identities, affine mappings, and watchdog values. It never opens
or probes a CAN interface.

## Process architecture

```text
compiled local portal (Qt/RViz panel or standalone rclcpp client)
  subscribes: /openarm_ik/portal_state, /openarm_ik/diagnostics
  clients: MoveTcp, MovePairedTcp, AutoCalibrate, EStop/Reset/Enable
                         |
                         v
openarm_ik_ros_node (only controller/state authority)
  ingress callbacks -> one shared reject-new arbiter -> immutable work queue
  urgent E-stop latch -------------------------------> owner worker
  owner worker -> VirtualControlSession -> openarm_control C ABI
  publishes measured /joint_states and atomic measured portal state
                         |
                         v
robot_state_publisher (only /tf and /tf_static authority) -> RViz
```

All control C calls run on one owner thread. ROS callbacks do only bounded
validation, reservation, and queueing. The worker advances against elapsed
`steady_clock` time at 5 ms with a 20 ms maximum core cycle, drains events with
poll deadline zero, renews the active heartbeat, snapshots, validates both
complete fresh masks/fault masks/skew, and publishes only new coherent feedback.
Any timing, feedback, event, heartbeat, or controller failure attempts
`OA_STOP_DISABLE`, latches the adapter fault, terminalizes the action once,
rejects later motion, and stops publishing apparently live state. There is no
automatic fault reset or re-arm.

Startup should create the standard virtual manifest and controller, call
open/verify, and remain `DISARMED`. The portal exposes an explicit **Enable
virtual motion** control. Enable obtains an expiring challenge and arms only
from healthy, fresh, disabled `DISARMED`. Startup and safety reset must not
silently arm.

## Exact ROS 2 Lyrical interface

Generate these interfaces inside `openarm_ik_ros` with
`rosidl_default_generators`. The node and portal are compiled C++17 using
`rclcpp_action`; no Python or rosbridge belongs in the command path.

### Atomic measured state

`msg/ArmMeasuredState.msg`:

```text
uint8 LEFT=0
uint8 RIGHT=1
uint8 side
string tcp_frame
uint64 feedback_seq
uint64 controller_time_ns
uint32 expected_mask
uint32 fresh_mask
uint32 fault_mask
float64[7] q
float64[7] dq
float64[7] tau
geometry_msgs/Point tcp_m
```

`msg/PortalState.msg`:

```text
uint8 BACKEND_VIRTUAL=1
uint8 BACKEND_PHYSICAL=2
std_msgs/Header header
uint64 state_sequence
uint8 backend
uint32 lifecycle
bool state_valid
bool motion_enabled
bool motion_busy
bool software_estop_latched
bool fault_latched
bool collision_checked
bool physical_motion_authorized
bool physical_calibration_available
ArmMeasuredState left
ArmMeasuredState right
```

Publish `/openarm_ik/portal_state` reliable, transient-local, depth 1 after each
new coherent snapshot. Its frame is always `openarm_body_link0`; `left` and
`right` are obtained by two `oa_controller_get_kinematics` calls using the exact
sequences in the same snapshot, with no `advance` between them. The outer stamp
represents the older arm sample:

```text
ros_stamp = sampled_ros_now - (controller_now - min(left.t_ns, right.t_ns))
```

Checked arithmetic, positive time, and coherent skew are mandatory. Repeated
publication never refreshes a measurement timestamp.

Publish `/joint_states` reliable, volatile, depth 10 from the same snapshot and
stamp, containing exactly the 14 canonical arm names and measured q/dq/tau.
Publish no fingers until measured gripper state exists.

### Independent Cartesian movement

`action/MoveTcp.action`:

```text
# Goal
geometry_msgs/PointStamped target
string tip_frame
---
# Result
uint8 COMPLETED=1
uint8 REJECTED=2
uint8 CANCELED=3
uint8 ABORTED=4
uint8 ESTOPPED=5
uint8 outcome
unique_identifier_msgs/UUID goal_id
uint32 control_status
uint64 command_id
uint64[2] seed_feedback_seq
uint64[2] terminal_feedback_seq
uint32 final_lifecycle
uint32 final_event
uint32 final_cause
bool collision_checked
geometry_msgs/Point measured_tcp_m
float64 position_error_m
string reason
---
# Feedback
uint32 lifecycle
uint32 event
uint64 command_id
uint64[2] feedback_seq
geometry_msgs/Point measured_tcp_m
float64 position_error_m
```

`tip_frame` must be exactly `openarm_left_hand_tcp` or
`openarm_right_hand_tcp`; no numeric side enters ROS. `target.header.stamp` must
be nonzero and fresh, and all coordinates finite. Transform the point at that
stamp into `openarm_body_link0` or reject the whole goal for missing, stale, or
extrapolated TF. Orientation is absent, not silently ignored. The left and
right portal buttons each send only their own field set through this one action.

Keep a named `MovePairedTcp.action` for deliberate simultaneous moves, with one
common stamped frame plus explicit `left_tcp_m` and `right_tcp_m` fields. It is
the only ROS path that calls `oa_controller_plan_paired_tcp`. The old
`/openarm_ik/paired_xyz` `PoseArray` may remain for one migration cycle only as
a deprecated shim through the same arbiter; it must never regain a separate
processor or state store.

The action result is the authoritative completion/error acknowledgement.
Diagnostics are health telemetry, not command correlation. `COMPLETED` is sent
only after the core's measured completion event, never after IK or plan
acceptance.

### Software E-stop, reset, and enable

`srv/EmergencyStop.srv`:

```text
string reason
---
bool accepted
bool already_latched
uint32 control_status
uint32 lifecycle
string detail
```

`srv/ResetSafety.srv`:

```text
bool operator_confirmed
---
bool accepted
bool requires_enable
uint32 control_status
uint32 lifecycle
string detail
```

`srv/SetMotionEnabled.srv`:

```text
bool enable
bool operator_confirmed
---
bool accepted
uint32 control_status
uint32 lifecycle
string detail
```

E-stop is privileged over the motion arbiter. Its callback first atomically
latches the adapter and rejects new work, then sends urgent owner-thread work
to call `oa_controller_set_interlock(controller, 1, 0)`. The active action ends
`ESTOPPED`; both arms disable. Repeated requests are idempotent at the adapter
and must not repeatedly advance the core nonce/event ring.

Reset never means re-arm. In virtual mode, after explicit confirmation and only
after the software input is clear, the owner calls:

1. `oa_controller_set_interlock(controller, 0, 1)`;
2. `oa_controller_get_arm_challenge()` while in `ESTOP`/`FAULT`;
3. `oa_controller_reset_fault()` with that epoch/nonce;
4. `oa_controller_open_and_verify()`; and
5. validates a fresh coherent disabled snapshot.

The result is `DISARMED`, `requires_enable=true`. A separate confirmed enable
gets a new arm challenge and calls `oa_controller_arm`. Disable always performs
`OA_STOP_DISABLE` if needed and disarms. A portal/software E-stop is not a
physical power E-stop; diagnostics and UI must state that plainly. A future
physical backend must require the independent hardwired interlock to be clear,
and software reset must never clear it.

### Truthful Auto Calibrate

`action/AutoCalibrate.action`:

```text
# Goal
uint8 AUTO=0
uint8 VIRTUAL_VERIFY=1
uint8 PHYSICAL=2
uint8 requested_mode
bool operator_confirmed
---
# Result
uint8 VIRTUAL_VERIFIED=1
uint8 NOT_REQUIRED=2
uint8 UNAVAILABLE=3
uint8 FAILED=4
uint8 CANCELED=5
uint8 outcome
bool hardware_calibrated
bool mapping_changed
uint32 status
uint64 evidence_revision
string reason
---
# Feedback
uint8 CHECKING_MANIFEST=1
uint8 CHECKING_FEEDBACK=2
uint8 CHECKING_KINEMATICS=3
uint8 stage
float32 progress
string detail
```

The same arbiter makes calibration exclusive with motion. With the current
fixed virtual backend, `AUTO` resolves to `VIRTUAL_VERIFY`: validate the compiled
standard manifest/revisions, complete fresh fault-free feedback, finite FK/TCP,
and mapping consistency without moving or changing a manifest. Return
`VIRTUAL_VERIFIED` (or `NOT_REQUIRED`), always with
`hardware_calibrated=false`, `mapping_changed=false`, and zero hardware evidence
revision. The UI should label this **Verify simulation** once capabilities are
known; if the product keeps the words “Auto Calibrate,” its terminal text must
say “Simulation verified; no physical calibration was performed.”

`PHYSICAL` returns `UNAVAILABLE` before any session starts because
`OA_BACKEND_PHYSICAL` verification is unsupported, ROS has no physical
transport, and commissioning is disconnected. Do not link `openarm_commission`
merely to make the button appear implemented. Its arm-joint recipe requires
external hardware qualification and returns abstract next-action requests; it
does not create safe actuator commands or apply patches. A future physical
calibration action requires a separately reviewed controller/commission
orchestrator, immutable qualified recipe, fixture/posture checks, real
E-stop/deadman, supervised review, durable manifest patching, re-probe, and
staged hardware acceptance.

## Portal field and arbitration behavior

On the first valid transient `PortalState`, the client copies measured left TCP
XYZ into the left edit fields and measured right TCP XYZ into the right edit
fields. It does so again only on explicit **Use current** or a deliberate
reconnect/reset policy; live state must not overwrite a field the user is
editing. Each Move button serializes only its own three values and canonical tip
name. The other edit field is neither read nor sent.

One mutex-protected reservation spans `MoveTcp`, deliberate paired movement,
legacy paired input, calibration, enable/disable transitions, and reset. Policy:
**reject new, no queue, no preemption**. Reservation happens in the action goal
callback before acceptance and remains through TF validation, planning,
execution, settling, and terminal result. Simultaneous left/right clicks have
one winner and one explicit `busy` rejection; they can never produce two plans
from different snapshots. E-stop alone bypasses and cancels the owner.

Cancellation calls `OA_STOP_DISABLE`, reports `CANCELED` once, leaves the core
`DISARMED`, and requires explicit enable. Shutdown uses the same policy.

## Diagnostics and RViz authority

Publish `/openarm_ik/diagnostics` periodically and on transitions with stable
fields:

- `backend=virtual`, `state_source=oa_snapshot_encoder_feedback`,
  `physical_motion_authorized=false`,
  `physical_calibration_available=false`;
- `collision_policy=virtual_unchecked`, `collision_checked=false`,
  `orientation_constrained=false`;
- controller lifecycle, adapter state, motion-enabled/busy, software-E-stop and
  fault latches;
- active action type/UUID/command ID, last event, control cause, and terminal
  reason;
- manifest/model/scene/verification revisions;
- per-arm feedback sequence/time/age, expected/fresh/fault masks, and pair skew;
- request stamp, plan seed sequences/duration, and terminal measured sequences;
  and
- calibration mode/outcome plus `hardware_calibrated=false`.

Unchecked successful virtual motion is WARN, malformed/busy requests are WARN,
and stale/coherence/watchdog/controller/E-stop faults are ERROR. Omit unavailable
achieved values instead of publishing zeros.

The launch must contain exactly one adapter and one `robot_state_publisher`, plus
optional portal and RViz processes. It must contain no `joint_state_publisher`,
ros2_control state publisher, static-transform publisher, or second adapter.
The adapter and portal publish no TF. `robot_state_publisher` alone owns `/tf`
and `/tf_static`, driven by the adapter's measured 14-joint state. A host-local
authority lock keyed by UID and `ROS_DOMAIN_ID` should be acquired before
advertising `/joint_states`; deployments still need ROS domain/namespace
isolation because a local lock cannot exclude a remote publisher.

## Build/package/launch changes

- `scripts/build.sh`: install `openarm_model`, then `openarm_control` with tests
  disabled into `ros2_ws/install`, then run colcon.
- ROS CMake: `find_package(openarm_control CONFIG REQUIRED)`, add
  `rosidl_default_generators`, `rclcpp_action`, `builtin_interfaces`,
  `unique_identifier_msgs`, `std_msgs`, `tf2_ros`, and `tf2_geometry_msgs`;
  export `rosidl_default_runtime`; remove the direct model/old transaction
  target from the node.
- `package.xml`: declare those generated/action/TF dependencies and retain
  `sensor_msgs`, `geometry_msgs`, `diagnostic_msgs`, description, RSP, and RViz.
- Replace `PairedTransactionProcessor` with `VirtualControlSession`; there must
  be no compatibility switch back to instantaneous command-as-state behavior.
- Keep `openarm_ik_rviz.launch.py` names for compatibility, add optional compiled
  portal launch, and preserve one state/TF authority.

## Required tests

1. **ABI:** strict C11/C++17 compile and installed-consumer tests for the new
   record/symbol; all old layout/frozen-header consumers remain unchanged.
2. **Single TCP semantics:** every side/axis, limits, NaN/Inf, expiry and
   sequence boundaries; inactive q is identical at every waypoint and in every
   measured sample; moving either arm after plan creation makes execute stale.
3. **Portal prepopulation:** first coherent state seeds both fields; dirty fields
   are not overwritten; left/right buttons serialize only their named side.
4. **Arbitration race:** barrier-race many left, right, paired, legacy, and
   calibration goals under multithreaded executors. Exactly one reserves; all
   others explicitly return busy with no queue or mixed request.
5. **Measured provenance:** observe multiple decoded intermediate q/TCP samples;
   neither acceptance nor plan report changes state; completion follows the
   measured dwell only. Frozen feedback never completes.
6. **Action truth:** results correlate by action UUID; accepted/planned/started/
   settling are not success; reject, abort, cancel, E-stop, timeout, shutdown,
   and completed each terminalize exactly once.
7. **E-stop/reset:** E-stop at every lifecycle/command phase disables both and
   latches; repeated E-stop is idempotent; reset with bad confirmation/nonce,
   uncleared input, stale feedback, or fault fails; successful reset ends
   `DISARMED` and cannot move until explicit enable.
8. **Calibration truth table:** AUTO in virtual runs only the non-motion
   verification and never claims hardware/mapping evidence; PHYSICAL is
   unavailable and emits no control/commission/transport call; cancel/failure is
   terminal and leaves state unchanged.
9. **Freshness/coherence/watchdogs:** independently freeze/drop/fault each joint,
   test feedback timeout and skew boundaries, late control cycles, heartbeat
   expiry, and event overflow. No new state is published after invalid feedback.
10. **Joint/TCP/TF authority:** one `/joint_states`, `/tf`, and `/tf_static`
    publisher; exact 14 names and no fingers; both displayed TCPs match FK from
    the same measured JointState; second adapter fails before advertising.
11. **No physical path:** ROS/portal `ldd` and `nm` have no CAN, transport, or
    commission dependency; tracing shows no AF_CAN/PF_CAN or interface-changing
    process. Direct physical verify remains unsupported.
12. **Shutdown:** SIGINT/SIGTERM/context shutdown during every reservation phase
    rejects ingress, terminalizes once, disable-stops, joins the owner, destroys
    plan/controller/manifest once, and exits within a fixed bound.
13. **Sanitizers/soak:** ASan/UBSan plus TSan session/arbiter tests and a long
    deterministic virtual campaign with randomized actions, cancellations,
    faults, resets, and shutdown.

## Risks and release blockers

- **Blocker:** no `oa_controller_plan_tcp`; independent XYZ must not be built by
  repurposing the paired ABI.
- **Blocker:** no production standard virtual manifest helper; copying the test
  manifest into ROS creates configuration drift.
- **Claim boundary:** there is no collision engine. Virtual motion remains
  collision-unchecked and cannot be described as collision-safe bimanual motion.
- **Physical blocker:** physical controller verification, transport integration,
  commissioned manifest persistence, E-stop hardware integration, and staged
  qualification do not exist.
- **Calibration blocker:** current commissioning sessions do not operate a
  controller or apply a mapping; arm auto-calibration is not available.
- A software E-stop and process lock are convenience/safety layers, not a
  hardwired stop or multi-host authority guarantee.
- ROS scheduling is not real time. The 5 ms worker must pass actual elapsed time;
  load-induced gaps over 20 ms intentionally fault rather than being hidden.
- Removing fabricated finger state leaves finger dynamic TF absent until a real
  gripper plant/measurement is integrated; that is more truthful than zero-state
  authority.

## Acceptance decision

Proceed with the compiled **virtual** portal only after the two native additions
(`oa_controller_plan_tcp` and the standard virtual-manifest builder) and the
measured-state/action adapter are implemented and tested. Label independent
motion as coordinated inactive-arm hold with collision checking unavailable.
Expose simulation verification instead of claiming hardware calibration. Do
not expose physical motion, physical reset, or physical Auto Calibrate until the
separate transport, collision, commissioning, persistence, interlock, and
hardware-qualification gates are genuinely implemented.
