# Portal control design: independent critic

Date: 2026-07-29 (America/Los_Angeles)  
Status: **DONE_WITH_CONCERNS**  
Scope: read-only inspection except for this requested report. No build, test
executable, GUI, network, CAN, or hardware operation was performed.

## Decision

Proceed only with a **virtual-only** measured-state adapter, after two narrow C
ABI additions and motion-envelope corrections. Do not call the result physical,
collision-safe, calibrated, or safety-rated.

The proposed `oa_controller_plan_tcp` is justified. A paired-plan shim cannot
implement independent-arm intent because it runs IK for the other arm, makes it
active for completion, and can select a different redundant posture. The new
planner must actively hold the inactive arm's measured joint posture, while
continuing to collect fresh feedback from it. “Hold” is the correct term;
“freeze” is not. Freezing feedback would fault, and byte-identical measured
samples cannot be promised in a quantized dynamic plant.

The proposal nevertheless overclaims four things that must be corrected:

1. The seventh-order reference is velocity/acceleration/jerk bounded, but the
   current simulated plant is only velocity/acceleration bounded. Its
   acceleration can step, it ignores `max_jerk_rad_s3`, and tracking feedback
   can exceed the requested scaled reference envelope. Smooth UI animation
   cannot repair this.
2. The test fixture is not automatically a production virtual manifest. Its
   `vcan*` bus names, alternating signs, and `0.125` rad offsets are useful
   mapping-test data, but are arbitrary for a canonical model-space simulator.
3. An atomic adapter E-stop latch can reject new commands immediately, but an
   owner-thread work item cannot interrupt a C call already holding the
   controller serialization lock. It is an urgent software stop request, not a
   bounded-latency or safety-rated E-stop.
4. A virtual “AutoCalibrate” action is unnecessary and misleading. The fixed
   virtual manifest can be verified without motion at startup. Publish that
   capability/result; do not manufacture calibration evidence or an action
   lifecycle.

## Evidence from the current tree

- `openarm_ik_ros_node` calls `openarm_model` directly, commits IK output
  immediately, republishes it with new ROS timestamps, and fabricates two zero
  finger positions. It does not use `oa_controller`.
- The C API has single-joint and paired-TCP planning, but no single-arm TCP
  planner and no production standard-manifest constructor.
- Plans already bind controller instance, verification epoch, manifest/model/
  scene revisions, both feedback sequences, and both measured start vectors.
  Execute rechecks those bindings and all-joint start drift.
- Paired TCP uses 17 predecessor-seeded XYZ IK knots. Between knots it executes
  joint-space seventh-order interpolation. It is not a continuously constrained
  Cartesian straight line and does not constrain orientation.
- Completion is measured: q/dq are decoded from simulated DaMiao frames, TCP is
  recomputed by FK, and three new complete feedback intervals in tolerance are
  required.
- The model and controller have no collision engine. Unchecked plans are
  virtual-only and report `collision_checked == 0`.
- `oa_controller_get_kinematics` already supplies the measured q, full row-major
  4x4 TCP transform, and XYZ bound to an exact feedback sequence.
- There is no gripper state in `oa_snapshot` and no gripper plant.
- Physical verification returns `OA_CONTROL_EUNSUPPORTED`. Commissioning does
  not drive this controller or install a manifest.

## Exact stable C ABI additions

Do not change the size, offset, meaning, or symbol of any existing V1 record.
Add only:

```c
#define OA_PLAN_TCP UINT32_C(3)

typedef struct oa_tcp_move {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t expiry_ns;
    uint64_t required_feedback_seq[2];
    oa_side side;
    uint32_t reserved0;                 /* must be zero */
    double tcp_m[3];                    /* openarm_body_link0, metres */
    double velocity_scale;              /* finite, (0, 1] */
    double acceleration_scale;          /* finite, (0, 1] */
    double jerk_scale;                  /* finite, (0, 1] */
    double tcp_tol_m;                   /* finite, > 0 */
    uint64_t collision_scene_revision;
    double max_branch_step_rad;         /* finite, > 0 */
    double min_singular_value;          /* finite, >= 0 */
} oa_tcp_move;

#define OA_TCP_MOVE_V1_PREFIX_SIZE \
    ((uint32_t)offsetof(oa_tcp_move, max_branch_step_rad))

oa_control_status oa_controller_plan_tcp(
    oa_controller *controller,
    const oa_tcp_move *request,
    oa_motion_plan **out);

oa_control_status oa_manifest_create_openarm_v10_virtual(
    oa_manifest **out);
```

The prefix behavior should match paired TCP: an original prefix defaults
`max_branch_step_rad=2.0` and `min_singular_value=0.0`. A nonzero reserved field,
bad ABI, undersized prefix, invalid side, null output, nonfinite value, expired
request, wrong scene revision, or either stale sequence rejects without
publishing a plan or changing `*out`. No new plan-report ABI is required.
`oa_motion_plan_report.kind`, both seed sequences, duration, target q, achieved
TCP/residual, revisions, and collision flag are sufficient. The opaque plan's
existing internal verification epoch is stronger than exposing a caller-set
epoch.

The manifest helper returns a newly owned immutable handle destroyed with
`oa_manifest_destroy`; it performs no I/O or probing. It must use model-space
identity mappings (`q_scale=1`, `q_offset=0`, `direction=1`) unless a documented
simulator-codec reason proves otherwise, in-process bus labels that cannot be
mistaken for kernel `vcan` interfaces, canonical joint names and URDF limits,
fixed unique virtual identities, and pinned nonzero manifest/model revisions.
Keep separate custom-manifest tests for alternating mapping signs and offsets;
do not turn adversarial fixture values into the production preset.

## Required `oa_controller_plan_tcp` semantics

1. Require `ARMED_IDLE`, healthy complete coherent feedback, the current scene
   revision, an unexpired request, and both exact current feedback sequences.
2. Require **both arms to be at rest** at planning time, using the existing TCP
   completion velocity threshold (`abs(dq[j]) <= 2e-2 rad/s`) or a stricter
   documented threshold. The current zero-boundary time law does not correctly
   model a nonzero measured starting velocity. Return `OA_CONTROL_EBUSY` when
   fresh healthy state is still moving. The ROS adapter should apply this gate
   to joint and paired commands too; preferably strengthen all core planners.
3. Capture both measured q vectors exactly once. Set `kind=OA_PLAN_TCP`,
   `active_arm_mask=1U<<side`, and bind both q vectors, sequences, revisions,
   controller identity, and verification epoch.
4. Run FK only to obtain the selected arm's measured starting TCP. Generate 17
   linearly spaced XYZ knots from that measured point to the target and solve
   only the selected chain, predecessor-seeded. Apply residual, joint/model/
   codec limits, singularity, and per-knot branch-step checks.
5. Copy the inactive measured start q to its target and to every waypoint.
   Never run inactive-arm IK. Execute still commands both seven-joint sets, so
   the inactive side is an enabled measured position hold and remains part of
   feedback/coherence/fault monitoring.
6. Calculate each segment duration over the selected arm only. Keep the
   existing seventh-order constants: peak normalized velocity is 2.1875 and
   the existing 2.2, 8, and 60 duration factors conservatively bound reference
   velocity, acceleration, and jerk. Sum segments with checked overflow.
7. Mark `collision_checked=false`. Until a real full two-arm/body/environment
   validator exists, permit only explicit `OA_COLLISION_VIRTUAL_UNCHECKED`.
8. At execution, retain both-sequence, both-start-q, scene, identity, and epoch
   revalidation. Movement of either arm invalidates the plan.
9. At completion, require q/dq tolerance and measured FK TCP tolerance only for
   the active side, plus the existing three-new-generation dwell. The inactive
   side remains subject to feedback health and faults, not target completion.

“Inactive q is identical at every measured sample” is not a valid acceptance
criterion. The correct criterion is: its planned q is exactly constant; its
measured q remains within an explicit hold tolerance, measured dq remains below
the rest threshold after normal quantization, and its feedback sequence keeps
advancing.

## Motion envelope and no-jerk release condition

For the first portal release, make policy fixed rather than browser-configurable:

- canonical virtual hard joint limits: `1.0 rad/s`, `2.0 rad/s^2`, and
  `10.0 rad/s^3` per joint;
- command scales: `0.5` velocity, acceleration, and jerk, giving reference
  ceilings of `0.5 rad/s`, `1.0 rad/s^2`, and `5.0 rad/s^3`;
- TCP plans: 17 measured-start XYZ knots, `1e-3 m` final TCP tolerance;
- joint completion: `5e-4 rad` position and `2e-2 rad/s` velocity; and
- reject a report whose checked `duration_ns` plus settle allowance exceeds the
  fixed command horizon.

These are virtual product-policy values, not qualified physical limits.

The plant must track acceleration as state. On each positive `dt`, clamp the
change in acceleration to `max_jerk_rad_s3 * dt`, then clamp acceleration and
velocity before integrating position. Remove or redesign the present snap-to-
target branches, because directly assigning q/dq to the target also bypasses
acceleration and jerk. A simulator that retains the current acceleration-step
model may claim only bounded velocity/acceleration at the full manifest limits;
it may not claim jerk limiting or the scaled measured envelope.

E-stop/disable is the deliberate exception: stopping both arms and removing
drive authority takes priority over smoothness, so measured finite differences
may violate normal acceleration/jerk limits at that transition. State and tests
must classify those samples as `ESTOP`/disable, never ordinary trajectory
motion.

## ROS state and command flow

Use one C++17 node, one controller owner thread, one `/joint_states` publisher,
and `robot_state_publisher` as the only TF authority. The portal is only an
action/service client and state subscriber.

Startup must be:

1. create the fixed virtual manifest;
2. create a fixed virtual, explicitly collision-unchecked controller;
3. open/verify and require verified mask `0x3`;
4. validate fresh coherent disabled feedback and both sequence-bound FK calls;
5. publish `simulation_verified=true`, `collision_checked=false`; and
6. remain `DISARMED` until an explicit confirmed Enable service call.

Do not arm on startup, reconnect, reset, or simulation verification. Reset ends
`DISARMED`; cancel/disable ends `DISARMED`; later motion requires explicit
Enable. No automatic reset or re-arm is allowed.

Use one reject-new reservation across single TCP, deliberate paired TCP, joint
motion if exposed, enable/reset transitions, and simulation re-verification.
There is no queue and no motion preemption. Action cancellation is owner-only,
calls `OA_STOP_DISABLE` if execution started, terminates exactly once, and
requires Enable before another goal. E-stop alone bypasses the reservation:
its callback atomically latches ingress rejection, signals the owner’s urgent
slot, and returns “accepted for processing.” The owner checks that slot before
and after every bounded C call and must never execute a plan returned after the
latch was set.

Do not claim synchronous hardware stop acknowledgement from the service
callback. The C API serializes controller calls, so an E-stop request cannot
interrupt a planning/advance call already in progress. Measure and publish
`request_to_core_estop_ns`, define a virtual-only test budget, and fault the
adapter if it is exceeded. A hardwired E-stop remains mandatory for any future
hardware product.

### State contract corrections

`PortalState` should include a per-process `unique_identifier_msgs/UUID
session_id` and monotonically increasing `state_sequence`; clients discard old
session state and goals on restart. This is the ROS/server epoch. The C plan
epoch remains internal and is tested by attempting to execute an old opaque
plan after reset/reverify/rearm, which must return `OA_CONTROL_EIDENTITY`.

For each arm publish q/dq/tau, feedback sequence/time/masks, and a
`geometry_msgs/Pose tcp_pose_body` computed from the exact sequence-bound
`oa_arm_kinematics.tcp_transform`. Validate the homogeneous matrix, convert its
rotation to a normalized quaternion, and use frame `openarm_body_link0`.
Targets remain `PointStamped`: orientation is intentionally absent and every
state/result says `orientation_constrained=false`. A target in `world` is
accepted only through timestamped TF, even though the current URDF transform is
identity. The controlled tip name must be exactly the side-specific
`openarm_*_hand_tcp`.

Publish `/joint_states` from the same coherent snapshot, with the conservative
oldest member stamp and exactly 14 arm joints. Publish no finger entries. State
should explicitly say `gripper_state_available=false`; the UI says “Gripper
unavailable — not commanded or measured,” not open/closed/zero.

### Actions, services, and progress

Keep `MoveTcp` and deliberate `MovePairedTcp` actions. Their authoritative
terminal result carries session ID, action UUID, command ID, seed and terminal
sequences, final lifecycle/event/cause, measured TCP, error, duration, and
`collision_checked=false`. Success is emitted only for the matching measured
COMPLETED event.

Feedback must come from the worker/controller, never browser animation. Publish:

- phase: `RESERVED`, `PLANNING`, `EXECUTING`, `SETTLING`;
- `planned_duration_ns` and controller-domain `elapsed_ns`;
- `trajectory_fraction = min(elapsed/duration, 1)` while executing;
- the newest coherent measured TCP and remaining position error; and
- feedback sequences and capture age.

Time fraction is not proof of physical completion. Cap UI completion below
100% until the measured COMPLETED event; settling may have fraction 1 with
nonzero measured error. A browser may interpolate only decorative graphics and
must never use them as state, progress authority, or completion.

Keep three services: Enable/Disable, software Stop/E-stop request, and safety
Reset. Responses distinguish adapter latch acceptance from the later core
transition. Repeated E-stop is adapter-idempotent and must not fill the event
ring. Reset does not enable motion.

Do **not** add `AutoCalibrate.action` in this release. Perform non-moving virtual
verification during startup and expose `simulation_verified` plus a read-only
capability. If a manual refresh is required, use an idle-only
`VerifySimulation.srv`; it validates the fixed manifest revisions, complete
fault-free feedback, and sequence-bound finite FK, changes no mapping, and
returns `hardware_calibrated=false`, `mapping_changed=false`, evidence revision
zero. Do not advertise a PHYSICAL goal that can only return unavailable.

## Exact acceptance tests

### C ABI and manifest

1. Compile strict installed C11/C++17 consumers using the new record, prefix
   macro, symbol, and manifest helper; retain every frozen original V1 layout
   and link test unchanged.
2. Check nulls, bad ABI, every undersize boundary, oversized records, reserved
   nonzero, and allocation failure. Failed calls leave output sentinels and
   registry counts unchanged; destroy remains overlap-safe.
3. Verify the production helper is deterministic, I/O-free, has 14 unique
   canonical names/identities, distinct in-process bus labels, identity
   mappings, pinned limits/revisions/families, and is consumed by ROS and
   integration tests. Preserve separate nonidentity mapping tests.

### Single-TCP semantics

4. Cover both sides, each XYZ nonfinite field, exact expiry and +/-1 ns, both
   sequence mismatches, scene mismatch, invalid scales/tolerances/branch/
   singularity values, unreachable knots, exact limits, and both collision
   policies with no plant state change on rejection.
5. Internal plan tests inspect all 17 knots: inactive q equals captured start
   bit-for-bit, active XYZ knots are the specified linear fractions, only the
   selected chain invokes IK, waypoint times strictly increase, reference
   finite differences respect scaled velocity/acceleration/jerk, and duration
   arithmetic cannot wrap.
6. C-level execution observes multiple new decoded feedback samples. The
   inactive arm’s sequence advances, q stays within a declared hold tolerance,
   dq stays within rest tolerance, and active measured TCP converges. Completion
   follows measured q/dq/FK dwell, not plan acceptance or elapsed duration.
7. Inject nonzero dq into either arm while idle and require planning to return
   busy. Move either arm, change scene revision, reset/reverify, or use another
   controller after plan creation; execute returns stale or identity exactly as
   appropriate.

### Dynamics and reasonable motion

8. For every joint and representative short/long moves, sample the internal
   reference at adversarial sub-cycle times and prove `|dq|<=0.5`,
   `|ddq|<=1.0`, and `|dddq|<=5.0` under the fixed scales, including every TCP
   knot boundary.
9. With the jerk-state plant, test positive/negative reversal, convergence,
   quantization, variable legal `dt`, and all motor mapping signs. Internal
   plant velocity/acceleration/jerk never exceed `1/2/10`; decoded finite-
   difference tests use a documented quantization allowance and never replace
   the internal proof.
10. Verify no ordinary step or snap-to-target violates the plant bounds. Verify
    E-stop/disable may violate the smooth envelope only while lifecycle/event
    explicitly reports the stop exception, with both arms disabled and zero
    reported command authority.

### ROS lifecycle, arbitration, and truth

11. Startup failure injection at manifest/create/verify/FK/publication steps
    leaks nothing and advertises no motion server as ready. Successful startup
    is verified, coherent, collision-unchecked, and DISARMED; no motion occurs
    before confirmed Enable.
12. Barrier-race left, right, paired, joint, enable/reset, and simulation verify
    requests. Exactly one reservation wins; others explicitly reject busy;
    E-stop alone bypasses. No queue, mixed target, double result, or UUID/session
    misattribution occurs.
13. Inject E-stop before reservation, during TF, during every IK knot, after
    planning/before execute, executing, settling, cancel, reset, and shutdown.
    The atomic latch rejects ingress immediately; no plan returned after the
    latch executes; both arms enter ESTOP; the active action terminalizes once;
    repeated requests add no event; measured request-to-core latency is exposed.
14. Test cancel/disable/reset truth: stop-disable, zero measured velocity,
    DISARMED result, explicit re-enable required, bad/stale nonce rejected, and
    no automatic arm at any transition.
15. Freeze/drop/fault every joint, delay at timeout boundaries, skew at the
    exact limit and +1 ns, expire heartbeat, overflow events, and miss a control
    cycle. Invalid feedback never completes or refreshes state; faults stop both
    and reject future goals.
16. Verify action progress solely from controller elapsed time and coherent
    measured feedback. At planned fraction 1 without measured dwell, phase is
    SETTLING and UI remains below complete. Browser timer/animation pause,
    throttle, reconnect, or clock changes cannot alter authoritative progress.
17. Verify current TCP pose equals FK of the exact same published measured q and
    sequence, matrix-to-quaternion conversion is finite/normalized, target
    frames are transformed at their request stamp, and orientation is always
    disclosed as unconstrained.
18. Verify exactly one `/joint_states`, `/tf`, and `/tf_static` authority, exact
    14 names, no finger state, `gripper_state_available=false`, and no command or
    IK report is ever published as measured feedback.
19. Verify every UI/state/result permanently exposes `backend=virtual`, body
    frame/metres, `physical_motion_authorized=false`,
    `hardware_calibrated=false`, `collision_checked=false`, and
    `orientation_constrained=false`.
20. Run sanitizer/thread tests and a deterministic long virtual campaign over
    moves, E-stops, cancels, faults, resets, explicit enables, restart/session
    epoch changes, and shutdown. Queues and handle registries remain bounded.

## Release boundary

The smallest safe deliverable is therefore: production virtual manifest
helper, true single-arm TCP plan, corrected jerk-aware simulator, measured ROS
state/actions, explicit enable/reset/urgent software-stop flow, and startup
simulation verification. Physical motion, physical calibration, gripper state,
collision safety, and a safety-rated E-stop remain unavailable and must not be
represented by disabled-looking placeholders that imply latent capability.
