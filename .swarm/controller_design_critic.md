# Controller design critique: safest minimal compiled physical architecture

Date: 2026-07-29

## Verdict

**CORRECTIONS REQUIRED.** The broad direction—C ABI over C++ objects, one
SocketCAN owner per interface, encoder-derived state, generated FK/IK, explicit
commissioning, and no Python runtime—is sound. It is not yet an approvable
physical-motion design. The minimum safe design must make identity/configuration
verification, calibration, collision authorization, trajectory execution, and
measured completion explicit state machines rather than helper functions.

The existing libraries are strong foundations but intentionally stop short of
those responsibilities:

- `can/include/openarm_can.h:142-147,230-237` says the current probe verifies only
  fresh disabled feedback at expected IDs; serial, model, physical joint,
  firmware, and registers are not verified.
- `can/src/openarm_can.c:267-295` only validates that mapping scales are finite and
  nonzero. It does not enforce derivative consistency or power-consistent effort
  mapping.
- `model/include/openarm_model.h:57-83` exposes bounded position-only IK and
  explicitly reports `collision_checked == 0`.
- The virtual paired adapter seeds from and then overwrites last committed command
  state (`ros2_ws/src/openarm_ik_ros/src/paired_transaction.cpp:66-89`). That is
  appropriate for visualization but must never become physical state handling.
- Upstream hard-stop calibration has no travel/time bound, accepts a single
  velocity/torque threshold, leaves precise-home motion unimplemented, and then
  writes all motor zeros anyway
  (`upstream/openarm_can/setup/openarm-can-zero-position-calibration:107-147,393-403`).

## Ranked design corrections

### 1. Critical: define “detect” as matching a commissioned identity, not inferring an arm

Normal startup may enumerate host CAN interfaces and probe motors, but it must
only match a complete expected serial-number set to an already commissioned left
or right manifest. CAN IDs, motor family, direction, zero, arm side, joint name,
and tool frame must never be inferred from a scan. If there is no manifest, a
serial cannot be read, a response is duplicated, or two candidate interfaces
match, return `UNCOMMISSIONED` or `AMBIGUOUS` and remain torque-disabled.

For standard OpenArm v1.0, require two distinct bus interfaces because both arms
reuse IDs 1–8. A swapped cable is acceptable only if the complete serial sets let
the controller unambiguously rematch buses to manifest roles. Interface names
alone are not physical identities.

Motor and host configuration belongs in a separate compiled commissioning CLI,
not in controller startup. The CLI may:

- list and verify interface state/configuration read-only;
- explicitly apply host CAN settings through netlink when run with the necessary
  privilege and an `--apply`-style confirmation, never by calling a shell or
  `sudo`;
- assign IDs only while exactly one physically isolated motor is connected,
  followed by read-back verification;
- persist motor settings only after a separate confirmation and with flash-write
  accounting.

The normal runtime must contain no automatic bitrate cycling, broad motor scan,
ID rewrite, zero write, or save-to-flash path.

### 2. Critical: make measured encoder snapshots the sole state truth

No joint array may default to a meaningful finite zero. A state is unusable until
all expected motors have fresh, valid feedback after the current open/verify
epoch. Each immutable snapshot needs:

- monotonic receive time, state sequence, per-bus sample window/skew, expected and
  fresh masks;
- raw output-shaft `q/dq/T`, raw status nibble and temperatures;
- mapped model `q/dq/effort`, validity, and the exact manifest/calibration revision;
- bus-off/error/overflow counters and the last complete command cycle.

Capture a monotonic timestamp at dequeue for control freshness; retain kernel
timestamps separately for diagnostics because ordinary SocketCAN timestamping is
not by itself a guaranteed monotonic control clock.

Every individual-joint or XYZ plan starts from a coherent fresh measured
snapshot. Revalidate that snapshot and start-pose drift immediately before
execution. Progress and completion require measured joint error, measured low
velocity for a dwell interval, and—on XYZ moves—FK of measured joints within the
declared TCP tolerance. Elapsed time, requested setpoints, or the previous IK
result can never establish current or achieved angles.

### 3. Critical: split commissioning from normal arming and fix calibration semantics

Commissioning is an exclusive top-level controller mode, not a command available
while normally armed. It blocks all ordinary commands and operates on only one
arm and one joint at a time.

Manual encoder calibration must mean:

1. The arm is mechanically supported and torque-disabled.
2. The operator/fixture supplies an exact named model-joint reference, not “this
   looks like zero.”
3. Stable encoder samples are collected over a configured dwell and tolerance.
4. A known sign plus the measured output angle produces a staged software offset.
   One reference pose cannot determine both sign and offset; sign requires an
   independently commissioned direction or two distinct known fixture poses.
5. The full mapped pose is checked against model and commissioned limits, shown as
   a candidate, and committed atomically only after confirmation.

Default calibration commits a versioned software mapping. It must not issue the
motor `FE` zero command or write flash. Motor-zero persistence is a distinct,
rare commissioning procedure with preview, disable, exact known-zero placement,
read-back, power-cycle verification, and rollback documentation.

Automatic calibration must be a recipe-driven supervised hard-stop state
machine, never a generic “find zero” loop:

```text
PRECHECK -> WAIT_INTERLOCK -> LOW_ENERGY_APPROACH -> CONTACT_DWELL
         -> RETREAT -> REAPPROACH -> REPEATABILITY_CHECK
         -> CANDIDATE_OFFSET -> OPERATOR_REVIEW -> SOFTWARE_COMMIT
any state -> ABORTING -> DISABLED (latched failure, no commit)
```

Each recipe must bind motor serial, joint, side, approach direction, known stop
coordinate, allowed starting corridor, other-joint fixture posture, maximum
speed/current-or-torque/travel/time/contact energy/temperature, consecutive
contact samples, retreat distance, repeatability tolerance, and physical
deadman/E-stop requirements. Low velocity plus estimated feedback torque is only
one contact condition; it must not be the only bound. Failure or interruption
must be unable to write any zero.

The documented gripper closed stop is the only automatic reference justified by
the pinned evidence. Automatic arm-joint calibration must return
`UNSUPPORTED_RECIPE` until each mechanical stop and safe approach has been
qualified on the installed hardware. Encoders still cannot auto-calibrate arm
side, joint identity, TCP, or collision geometry.

### 4. Critical: correct units, gearing, sign, offset, wrap, and effort semantics

For J1–J7, feedback is already output-shaft angle. Never multiply it by the
integrated 9:1/10:1/40:1 gear ratio again. Query and verify `Gr`, direction,
PMAX/VMAX/TMAX, IDs, mode, bitrate, timeout, hardware/software versions, and
serial before using feedback units.

Represent one coherent affine coordinate transform:

```text
q_model  = a * q_output + b
dq_model = a * dq_output
tau_model = tau_output / a       (after separately validating the effort sign/unit)
```

For directly driven arm joints, commissioning should normally constrain
`abs(a) == 1`; any other scale requires explicit mechanical evidence. Do not keep
independently editable position, velocity, and torque scales as a physical
configuration because they can violate derivative and power consistency. The
gripper linkage is separate: motor radians to finger metres/force is approximate
and potentially nonlinear, so it needs its own calibrated curve and uncertainty.

Protocol P/V/T limits are decode parameters, not safety limits. Runtime limits
are the conservative intersection of verified protocol ranges, canonical model
limits, commissioned mechanical margins, and payload/application speed,
acceleration, jerk, effort, and thermal limits. Do not invent physical defaults
for missing acceleration, jerk, payload, or safe gain values.

For the smallest first physical implementation, require queried PMAX/VMAX/TMAX
to agree exactly with one supported codec profile and refuse every mismatch.
Adding instance-specific dynamic codec ranges can come later; silently decoding a
mismatched motor with family defaults is forbidden.

Multi-turn/wrap behavior across power cycles is still unverified. Commissioning
must prove a continuous encoder representation over every legal joint interval.
Runtime must fault on an implausible discontinuity; it must not select a wrap by
nearest command or elapsed motion because that would guess an angle.

### 5. Critical: no motion endpoint is authorization without a collision/path gate

The current model proves endpoint kinematics and URDF bounds only. A legal
single-joint target can hit the body, and a paired position-IK result can collide
with the body, the other arm, the environment, cables, or payload. Position-only
IK also leaves orientation free.

All physical execution paths therefore require a compiled collision validator.
Make it an internal injected interface whose production default is `RejectAll`,
not `AcceptAll`. A plan must bind the collision model/scene revision and report
self, body, inter-arm, environment, payload, and tool checks. A fixture-qualified
joint-space corridor can be an initial conservative validator; sparse waypoint
sampling without a continuous-motion clearance argument is insufficient.

For paired claw XYZ, define “claw” precisely as the canonical named
`openarm_{left,right}_hand_tcp` expressed in the shared
`openarm_body_link0` frame. Do not call an ambiguous `world` target physical
unless a separately validated, timestamped world-to-body transform exists. The
API must return the achieved orientation and reject solutions outside an allowed
orientation/posture/change envelope even though orientation is not commanded.

Plan Cartesian motion as bounded position-IK waypoints seeded first from measured
`q` and then from the preceding planned waypoint. Reject any branch jump,
singularity policy failure, bounds failure, collision failure, or residual
failure. Time-parameterize the resulting joint path with synchronized
velocity/acceleration/jerk limits. Never transmit a final IK vector as one large
MIT setpoint.

### 6. High: define honest individual and paired execution semantics

An individual-joint request snapshots all seven measured joints, changes exactly
one target, and holds the other six through a full-arm trajectory. It still sends
a complete command for every enabled motor on every cycle so the motor watchdog
and feedback masks remain meaningful. The selected target and all held targets
must pass mapping, dynamic, collision, and start-drift validation.

A paired XYZ request is atomic only in planning/acceptance: either both complete
plans are accepted or neither is. SocketCAN cannot make two buses physically
atomic. Execution needs a shared scheduled epoch, measured transmit skew, a
configured maximum skew, and the default coupling `either arm fails => stop both`.
Expose `PLANNED`, `QUEUED`, `STARTED`, `SETTLING`, `COMPLETED`, `ABORTED`, and
`FAULTED`; do not overload “committed” to mean sent or achieved.

Every command has an ID, creation snapshot sequence, absolute monotonic start and
expiry, and completion tolerances. The internal trajectory producer must also
heartbeat. A stale caller, stalled planner/executor, missed bus deadline, partial
send, incomplete feedback mask, or excessive cross-bus skew latches a fault and
initiates the configured stop policy.

### 7. High: use a smaller internal object model and one public owner

Avoid a public forest of controller/arm/motor/trajectory handles and lifetime
races. The minimum useful architecture is:

```text
versioned extern "C" API
        |
oa_controller* (one opaque owner; arms selected by fixed side/joint IDs)
        |
Controller -- authoritative lifecycle, events, command coordinator
  +-- ArmRuntime[2] -- immutable manifest, mapping, limits, measured snapshot
  +-- BusWorker[2]   -- sole SocketCAN FD owner and codec/register sequencer
  +-- MotionPlanner  -- model calls, time parameterization, collision policy
  +-- CalibrationSession -- optional exclusive state, never concurrent motion
```

`MotorState` can be fixed-size data owned by `ArmRuntime`; it does not need its
own thread, polymorphic class, or public handle. `KinematicsAdapter` and
`TrajectoryGenerator` can be ordinary helper/value components. Use exactly one
worker per bus plus one coordinator, fixed-size queues/buffers allocated at
creation, and injected transport/monotonic clock/interlock implementations for
tests.

Keep three build products:

1. Existing pure C `openarm_can` codec/diagnostics and `openarm_model`.
2. A compiled C++ controller library with a C ABI; no Python, ROS, shell, or sudo
   dependency at runtime.
3. A separately installed compiled commissioning CLI/library containing
   destructive register/zero/save operations.

The checked-in model generator may remain a developer-time Python tool because
the generated C model is compiled and Python is not a runtime dependency. All
controller and acceptance tests can be C/C++ executables.

### 8. High: make the C ABI asynchronous, versioned, and fail-closed

Every public record should retain `struct_size`/`abi_version`, fixed-width status
fields, SI units, explicit frame/model/manifest revisions, and caller-owned
buffers. Catch all C++ exceptions at the ABI. Do not print, allocate unexpectedly,
invoke callbacks from the bus thread, or unwind through C. Poll/wait on a bounded
event queue instead.

The minimal surface is conceptually:

```c
oa_status oa_controller_create(const oa_controller_config *, oa_controller **);
oa_status oa_controller_open_and_verify(oa_controller *, oa_verify_report *);
oa_status oa_controller_snapshot(oa_controller *, oa_snapshot *);
oa_status oa_controller_arm(oa_controller *, const oa_arm_request *);
oa_status oa_controller_plan_joint(oa_controller *, const oa_joint_move *, oa_plan_id *);
oa_status oa_controller_plan_paired_xyz(oa_controller *, const oa_paired_xyz *, oa_plan_id *);
oa_status oa_controller_execute(oa_controller *, oa_plan_id, const oa_execute_request *, oa_command_id *);
oa_status oa_controller_stop(oa_controller *, oa_stop_kind);
oa_status oa_controller_disarm(oa_controller *);
oa_status oa_controller_reset_fault(oa_controller *, const oa_reset_request *);
oa_status oa_controller_poll_event(oa_controller *, oa_event *);
oa_status oa_calibration_begin(oa_controller *, const oa_calibration_request *, oa_calibration_id *);
oa_status oa_calibration_step(oa_controller *, oa_calibration_id, const oa_calibration_action *, oa_calibration_report *);
oa_status oa_calibration_abort(oa_controller *, oa_calibration_id);
```

Plans must be immutable and bind measured start sequence, model hash, manifest
revision, safety-limit revision, collision-scene revision, target, path, and
expiry. `execute` rejects any stale or changed dependency. Avoid exposing generic
register writes through this API.

### 9. High: complete the lifecycle and stop contract

Use a single explicit state machine:

```text
CLOSED -> VERIFYING -> DISARMED -> ARMING -> ARMED_IDLE -> EXECUTING
                      |                                       |
                      +-> COMMISSIONING -> DISARMED            v
normal stop: EXECUTING -> STOPPING -> ARMED_IDLE or DISARMED
any violation -> FAULT_LATCHED -> (explicit reset and full reverify) -> DISARMED
physical interlock -> ESTOP_LATCHED -> physical reset + full reverify -> DISARMED
```

Arming requires complete verified identity/config, fresh legal measured state,
healthy buses, verified motor timeout behavior, live host watchdog/producer,
collision validator availability, clear physical interlock, and explicit
operator action. Fault reset never returns directly to armed state.

The stop reaction cannot be one universal “disable”: removing torque may drop a
payload, while holding may be impossible after a hard fault. Define
controlled-decelerate/hold, torque-disable, and external power-cut outcomes from
an installation hazard analysis. CAN disable and a destructor are best effort,
not an E-stop. Hardware must provide an independent accessible E-stop/deadman and
restart interlock; software may monitor it but cannot replace it.

## Testability requirements

The current diagnostics fake intentionally rejects control and should remain so.
Add a separate deterministic simulated motor/transport for controller tests. It
must model registers, enable/disable, command-triggered encoder feedback, timeout,
hard stops, and fault injection without letting command targets masquerade as
measurements.

Minimum compiled tests:

- C-only ABI consumer, record sizes/versions/canaries, handle lifetime, exception
  containment, event-buffer overflow, and no callbacks after close.
- No command before a complete fresh feedback epoch; zero-initialized internal
  memory must never become a valid pose.
- Freeze simulated encoder feedback while commands advance and prove completion
  never occurs; inject measured convergence and prove dwell-based completion.
- Mapping round trips and derivative/power consistency; output-shaft gearing is
  not applied twice; wrap discontinuity faults rather than unwraps.
- Two virtual buses with identical IDs, swapped interfaces, duplicate/missing/
  wrong serials, register drift, wrong P/V/T ranges, stale frames, overflow,
  bus-off, process/producer expiry, and partial paired sends.
- Manual calibration stability, unknown sign rejection, two-reference sign
  validation, preview/abort/atomic commit, corrupted/truncated manifest, and
  power-loss-safe persistence.
- Automatic calibration false contact, noisy contact, no contact, overtravel,
  timeout, thermal/effort ceiling, deadman release, repeated-stop disagreement,
  abort at every state, and proof that no failure can write zero or commit.
- Joint and paired trajectory dynamic bounds, measured start drift, IK branch
  jumps/singularities, collision-policy rejection, scene revision changes,
  cross-bus skew, and stop-both behavior.
- `vcan` filters, CAN error frames, interface down/up, queue overflow, shutdown,
  timing/load distribution, and two-bus isolation.

## What must remain hardware-gated

No CI or simulation result can approve physical motion. Acceptance must progress
through isolated unloaded motor, supported single joint, reduced-limit single
arm, and only then bimanual motion. The following facts remain installation
gates:

1. Exact motor package/firmware/sub-version/serial at every joint, J3/J4 family,
   classic-CAN versus FD/BRS support, and configured P/V/T ranges.
2. Timeout unit, zero behavior, persistence, which frames refresh it, disable
   latency, and clear/re-enable behavior on every installed firmware.
3. Output-angle persistence and wrap behavior; side/joint/sign/offset and effort
   sign; verified no second gear conversion.
4. Manual fixtures and per-joint hard-stop contact safety/coordinates/recipes;
   auto arm-joint calibration stays unavailable until qualified.
5. Physical E-stop/deadman/contactors, measured interruption and restart behavior,
   mounting, gravity/load-drop response, payload, gains, thermal/current limits,
   and safe stop policy.
6. Installed nominal/actual TCP, gripper displacement/force curve, payload/tool
   collision geometry, cable envelope, and environment/inter-arm collision scene.
7. End-to-end trajectory tracking, bus utilization/latency/skew margin, missed
   deadline behavior, process kill, bus-off, encoder loss, and paired fault tests.

Until these gates pass, the implementable and honest scope is compiled
configuration/probing, encoder snapshots, software calibration workflows,
planning/preview, deterministic simulation, and fail-closed execution APIs.
Physical paired claw XYZ execution in particular must remain rejected until a
collision validator and the staged hardware acceptance are complete.

**Final status: CORRECTIONS REQUIRED (not APPROVE).**
