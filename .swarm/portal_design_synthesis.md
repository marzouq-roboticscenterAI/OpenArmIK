# Local OpenArm portal: reconciled implementation design

Date: 2026-07-29 (America/Los_Angeles)  
Tree inspected: `aeffc2b` plus the five untracked portal review reports  
Status: **DONE_WITH_CONCERNS — DESIGN COMPLETE; IMPLEMENTATION REQUIRES A
CALIBRATED SUPPORT-POLE SCENE AND LIVE RVIZ ACCEPTANCE**

This is a read-only synthesis except for this requested report. No build,
test executable, GUI, listener, ROS graph, network, CAN interface, or hardware
operation was started.

## Release decision

Build one virtual-only local portal in four dependency-ordered stages:

1. extend `openarm_control` with a production virtual manifest, a true
   single-arm TCP plan, a jerk-limited measured plant, and a mandatory
   revision-bound collision scene;
2. replace the implementation of the existing compiled ROS node with the one
   controller authority and expose measured state, two motion actions, and
   four safety/verification services;
3. add one compiled C++ loopback HTTP/MJPEG portal which is only a ROS client
   and which captures the actual stock RViz XWayland window; and
4. extend the Bash launcher to own ROS, RViz, and portal process groups and to
   shut them down in authority order.

The portal is not releasable by falling back to
`OA_COLLISION_VIRTUAL_UNCHECKED`. Every accepted portal motion must carry
`collision_checked=true` for the exact current scene revision. The current
flattened URDF has collision meshes for the body, links, hands, and fingers,
but it has no separately named or calibrated support-pole geometry. Its single
body collision STL spans approximately `x=[-0.155,0.095] m`,
`y=[-0.095,0.095] m`, `z=[0,0.773] m` and appears to contain a narrow central
component, but a mesh name and bounding box do not prove that component is the
large metal pole or that its pose and dimensions match the installation.
Therefore Stage A0 below is a hard gate: add surveyed/CAD-bound pole geometry
with a digest and revision, or all motion remains disabled.

Physical control and physical calibration remain unsupported. The ROS node
has no backend selector and must not link transport, CAN, or commission.
`OA_BACKEND_PHYSICAL` remains query/fail-closed only and
`open_and_verify()` must continue to return `OA_CONTROL_EUNSUPPORTED`.

## Reconciliation of the reviews

The following resolves the material disagreements.

- **Independent buttons:** use a new single-arm TCP planner. A paired-TCP shim
  is rejected because it re-solves, activates, trajectories, and settles the
  other arm. The new plan holds the other arm at its measured joint posture and
  continues to monitor its feedback.
- **Collision:** the earlier unchecked-virtual proposals are superseded by the
  explicit arm-arm/pole requirement. Reachability is never motion authority.
  All joint interpolation between the 17 Cartesian IK knots is certified, not
  merely the knots.
- **Auto Calibrate:** the required button remains visible, but it calls a
  non-moving `VerifySimulation` service. Its fixed caption is
  **“Auto Calibrate — simulation verification only”** and its successful
  terminal text is **“Simulation verified; no physical calibration was
  performed.”** It never links commissioning, moves a joint, changes a
  mapping, or emits calibration evidence. This satisfies the visible product
  requirement without making a false hardware claim.
- **Stop:** the path has priority over leases, command reservations, TF, IK,
  JPEG, and normal HTTP work, but is named **“Request stop (not a safety
  E-stop)”**. It is an urgent software interlock request, not a hardwired or
  bounded-latency safety device. Reset never re-enables motion.
- **RViz:** only pixels captured from the launcher-owned stock `rviz2` window
  may be labelled “Live RViz.” A browser renderer, static image, root-window
  crop, or newly embedded `RenderPanel` is not an acceptable substitute.
- **Orientation and gripper:** targets are XYZ only; orientation is free and
  always disclosed. No finger state is published. Collision checking uses a
  conservative full-stroke gripper envelope because gripper position is not
  measured.
- **Speed:** there is no browser speed slider. Product policy fixes joint
  reference scales at 0.5 of the `1 rad/s`, `2 rad/s^2`, `10 rad/s^3`
  production virtual limits: `0.5 rad/s`, `1 rad/s^2`, and `5 rad/s^3`.
  These are virtual joint limits, not a Cartesian-speed or physical-safety
  claim.

## Current repository baseline

The current native status namespaces/build exports are partly landed, but the
portal prerequisites are not:

- `openarm_control.h` has joint and paired-TCP plans, no single-TCP symbol, no
  production manifest builder, and only reject-all/virtual-unchecked collision
  policies.
- the seventh-order reference is jerk bounded, but
  `DamiaoMotorSimulator::capture()` stores no acceleration, steps acceleration,
  and snaps q/dq directly to a target near convergence;
- the standard-looking 2x7 manifest exists only as `valid_config()` in a test;
  its alternating signs, `0.125 rad` offsets, and `vcan*` names are adversarial
  test data, not production simulator identity mappings;
- `openarm_ik_ros_node` calls model IK directly, immediately publishes the IK
  result as state, republishes it with fresh timestamps, and fabricates two
  zero finger positions;
- launch has one adapter and one `robot_state_publisher`, which is the correct
  authority count, but the state provenance is wrong;
- `scripts/launch_rviz.sh` already retains separate ROS/RViz PIDs, forces
  XWayland/GLX, closes RViz by `WM_DELETE_WINDOW`, and has the lifecycle shape
  to preserve; and
- no portal target or listener exists. The installed ROS workspace is also
  stale relative to the checked-in close helper and must be rebuilt before a
  live session.

## Smallest implementation DAG

```text
A0 calibrated scene artifact + generated conservative proxies
 |\
 | +--> A2 true single-TCP planner + continuous collision certification
 |       ^
A1 production virtual manifest -----------------------------+
 |
A3 jerk-state measured plant --------------------------------+
 |                                                           |
+---------------------- A4 native ABI/control gates <---------+
                                |
                 +--------------+---------------+
                 |                              |
          B1 controller session            B0 ROS IDLs
                 +--------------+---------------+
                                |
                     B2 replace existing node
                                |
           +--------------------+--------------------+
           |                                         |
 C0 XComposite/JPEG capture                   C1 HTTP/ROS/UI
           +--------------------+--------------------+
                                |
                       C2 integrated portal
                                |
                       D Bash orchestration
                                |
                   headless + mandatory live gates
```

`A0`, `A1`, `A3`, and `B0` can be developed independently. Nothing in B may
merge against an unchecked controller. C may use fake X and fake ROS adapters
for tests, but command routes stay unregistered until B reports compatible
capabilities. D is last because it names and supervises the installed outputs.

## Stage A — native control ABI and semantics

### A0. Collision scene is a prerequisite, not a portal option

Add a checked-in generated scene source whose inputs are:

1. the exact flattened OpenArm v1.0 URDF and collision-mesh digests;
2. a separately named `openarm_support_pole` fixed collision object with a
   surveyed/CAD pose relative to `openarm_body_link0`, radius or conservative
   cross-section, lower/upper extent, measurement uncertainty, provenance, and
   approval revision; and
3. a fixed release clearance of **0.025 m**, increased by every recorded pose,
   dimension, mesh, and numeric uncertainty. Reducing it is a scene revision
   requiring the full collision suite.

Do not infer pole calibration from `body_link0_symp.stl`. Either add the pole
as a fixed URDF link/collision or as an equally versioned scene object consumed
by both collision generation and RViz. The hashes of the flattened URDF,
meshes, pole record, generator, and generated proxy table form the scene
digest. Missing, zero, non-finite, unapproved, or digest-mismatched data makes
the production scene builder return `OA_CONTROL_EUNSUPPORTED`; it must never
substitute a guessed cylinder.

Generate conservative convex proxies rather than doing runtime STL parsing:

- every body/arm/hand/finger mesh component is enclosed by a verified convex
  hull or tighter collection of convex hulls; generation proves every source
  triangle vertex is inside its proxy within numeric tolerance;
- the support pole is a calibrated capped cylinder/convex prism, expanded by
  its uncertainty;
- each gripper is one conservative link-7-attached union spanning both fingers
  over their complete `0..0.044 m` travel plus the hand mesh;
- add a `0.030 m` radius TCP keep-out sphere to that tool union, then apply the
  scene clearance; and
- all collision math uses double precision, finite checks, checked sizes, a
  deterministic GJK distance implementation, and fail-closed handling of
  non-convergence or near-zero numeric separation.

The only allowed collision exclusions are generated from this explicit list:

- body versus its fixed left/right link0 mounting component;
- within one side, directly connected `linkN`/`linkN+1` pairs; and
- link7 versus its same-side composite hand/fingers/TCP tool.

There is no blanket “same arm,” “adjacent by two,” or disabled-collision-matrix
rule. Check every non-excluded same-arm pair, every left/right pair, every arm
and tool against the non-mount body and support pole, and both tool envelopes
against each other. No inter-arm or pole pair is ever excluded.

Every candidate path is checked at all 17 IK knots and over every joint-space
segment. For each segment, recursively certify each collision pair:

1. evaluate exact proxy distance at the midpoint;
2. bound each proxy's possible motion over the interval from precomputed
   joint-to-proxy radii and the endpoint joint-angle ranges;
3. accept the interval only when midpoint separation exceeds the sum of both
   sweep bounds plus numeric epsilon;
4. otherwise subdivide; and
5. return `OA_CONTROL_ECOLLISION` when an exact sample intersects or when the
   maximum subdivision depth/angle width is reached without proof.

The seventh-order segment is componentwise monotone between its endpoints, so
the endpoint angle interval contains the whole segment. This makes the sweep
bound a continuous-path certificate, unlike fixed-rate sampling. Keep the
certificate summary (minimum clearance, checked pair, segment, subdivision
count) internally and expose its minimum clearance in the new scene/report
record below. Planning must also validate the measured start configuration;
an initially colliding or indeterminate scene disables motion.

### New public scene ABI

Do not resize or reinterpret an existing V1 record. Add opaque scene ownership
and new symbols:

```c
#define OA_COLLISION_OPENARM_V10_POLE UINT32_C(2)

typedef struct oa_collision_scene oa_collision_scene;

typedef struct oa_collision_scene_report {
    uint32_t struct_size;
    uint32_t abi_version;                 /* OA_CONTROL_ABI_V1 */
    uint64_t scene_revision;
    uint64_t model_revision;
    uint32_t calibrated;
    uint32_t includes_body;
    uint32_t includes_inter_arm;
    uint32_t includes_grippers;
    uint32_t includes_tcp_keepout;
    uint32_t includes_support_pole;
    uint32_t reserved0;
    double clearance_margin_m;            /* 0.025 release minimum */
    double pole_center_body_m[3];
    double pole_axis_body[3];              /* finite normalized vector */
    double pole_radius_m;
    double pole_half_length_m;
    double pole_uncertainty_m;
    char geometry_sha256[65];              /* lowercase hex + NUL */
} oa_collision_scene_report;

oa_control_status oa_collision_scene_create_openarm_v10_pole(
    oa_collision_scene **out);
oa_control_status oa_collision_scene_get_report(
    const oa_collision_scene *scene, oa_collision_scene_report *out);
void oa_collision_scene_destroy(oa_collision_scene *scene);

oa_control_status oa_controller_create_with_scene(
    const oa_manifest *manifest,
    const oa_collision_scene *scene,
    const oa_controller_options *options,
    oa_controller **out);
```

The scene handle follows the existing monotonic typed-registry/destroy rules.
`create_with_scene` snapshots immutable scene data; later scene-handle
destruction cannot invalidate the controller. It requires
`options.collision_policy == OA_COLLISION_OPENARM_V10_POLE`, an exact nonzero
`options.collision_scene_revision == scene.scene_revision`, matching model
revision, `calibrated==1`, and every include flag above. The existing
`oa_controller_create` and `OA_COLLISION_VIRTUAL_UNCHECKED` remain exported for
binary compatibility, but the ROS/portal build must never call or accept them.

Append, do not alter, these fields to a new report record rather than growing
`oa_motion_plan_report` in place:

```c
typedef struct oa_motion_plan_collision_report {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t collision_checked;
    uint32_t indeterminate;
    uint64_t collision_scene_revision;
    double required_clearance_m;
    double minimum_clearance_m;
    uint32_t checked_knot_count;
    uint32_t certified_interval_count;
} oa_motion_plan_collision_report;

oa_control_status oa_motion_plan_get_collision_report(
    const oa_motion_plan *plan, oa_motion_plan_collision_report *out);
```

A successful portal plan requires `collision_checked==1`,
`indeterminate==0`, the exact requested scene revision, and finite
`minimum_clearance_m >= required_clearance_m`. The existing V1 plan report and
all current symbols/layouts remain unchanged.

### A1. Production virtual manifest

Add exactly:

```c
oa_control_status oa_manifest_create_openarm_v10_virtual(oa_manifest **out);
```

It is allocation-only and performs no filesystem, CAN, network, or device I/O.
The returned handle is destroyed by `oa_manifest_destroy`. The fixed record
uses:

- `manifest_revision=1`, `model_revision=1` initially, bumped whenever the
  generated model/scene contract changes;
- bus labels `sim_left` and `sim_right`, never `can*`/`vcan*`;
- the 14 canonical `openarm_{left,right}_joint1..7` names;
- motor families J1/J2 DM8009, J3/J4 DM4340, J5-J7 DM4310;
- identity model mappings: `q_scale=1`, `q_offset_rad=0`, `direction=1`;
- unique `SIM-L-J1..J7` and `SIM-R-J1..J7` serials;
- per-bus send IDs `1..7`, receive IDs `0x11..0x17`, embedded IDs `1..7`;
- the exact generated URDF limits;
- product virtual limits `1 rad/s`, `2 rad/s^2`, `10 rad/s^3`;
- existing family codec pmax/vmax/tmax/gear data; and
- fixed nonzero bitrate/version/timeout metadata documented as simulator
  identity, not probed motor evidence.

Move adversarial sign/offset construction to a separately named test fixture.
ROS and integration tests consume only this production builder.

### A2. True single-arm TCP ABI

Add without changing any existing V1 layout or symbol:

```c
#define OA_PLAN_TCP UINT32_C(3)

typedef struct oa_tcp_move {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t expiry_ns;
    uint64_t required_feedback_seq[2];
    oa_side side;
    uint32_t reserved0;                    /* must be zero */
    double tcp_m[3];                       /* openarm_body_link0, metres */
    double velocity_scale;                 /* finite, (0,1] */
    double acceleration_scale;             /* finite, (0,1] */
    double jerk_scale;                     /* finite, (0,1] */
    double tcp_tol_m;                       /* finite, >0 */
    uint64_t collision_scene_revision;
    double max_branch_step_rad;             /* finite, >0 */
    double min_singular_value;              /* finite, >=0 */
} oa_tcp_move;

#define OA_TCP_MOVE_V1_PREFIX_SIZE \
    ((uint32_t)offsetof(oa_tcp_move, max_branch_step_rad))

oa_control_status oa_controller_plan_tcp(
    oa_controller *controller,
    const oa_tcp_move *request,
    oa_motion_plan **out);
```

The original prefix defaults branch step to `2.0 rad` and minimum singular
value to zero. Bad ABI/size/side/reserved data, non-finite values, scale outside
`(0,1]`, expired input, either sequence mismatch, scene mismatch, or non-resting
arm rejects without changing `*out` or plant state. Require both arms' measured
`abs(dq)<=0.02 rad/s` before all joint, single-TCP, and paired-TCP planning;
return `OA_CONTROL_EBUSY` otherwise.

The planner snapshots both measured q vectors once, sets
`active_arm_mask=1U<<side`, solves only the selected chain through 17
predecessor-seeded XYZ knots, and copies the inactive measured start q bit for
bit into every inactive waypoint and target. It commands that constant
inactive reference during execution while continuing to require advancing,
fresh, fault-free feedback. Moving either arm between plan and execute makes
the plan stale. Completion applies q/dq/FK TCP tolerance only to the active
side; inactive measured hold error is monitored as a fault, not an active goal.

Run the full two-arm/body/pole/tool collision certificate after all waypoints
exist and before publishing a plan. The plan remains bound to controller
identity, verification epoch, both start q/sequence values, manifest/model
revision, and collision-scene revision. `OA_CONTROL_ECOLLISION` covers collision
and fail-closed indeterminate geometry; no unchecked plan is returned.

### A3. Jerk-limited measured plant

Store plant acceleration as state for every simulated motor. Integrate in
model-joint coordinates so the configured `1/2/10` limits remain true under
nonidentity codec mappings, then convert to raw motor coordinates only for
encoding. For every positive `dt`:

1. compute the bounded desired acceleration from position/velocity tracking;
2. clamp acceleration change to `max_jerk_rad_s3 * dt`;
3. clamp acceleration to `max_acceleration_rad_s2`;
4. integrate and clamp velocity to `max_velocity_rad_s`;
5. integrate q without overshoot; and
6. converge through a jerk-respecting terminal controller, never by direct
   q/dq assignment.

`oa_controller_sim_set_state()` resets internal acceleration to zero without
growing `oa_sim_state`. Disable/stop and E-stop may zero authority immediately;
that transition is explicitly classified as a stop exception and is not
reported as ordinary jerk-limited motion. Reference and plant bounds are both
tested; JPEG or browser animation has no role in motion truth.

## Stage B — one compiled ROS controller authority

Keep package, executable, node, and launch names. Replace
`PairedTransactionProcessor` in `openarm_ik_ros_node`; do not run old and new
implementations beside each other. One C++17 `VirtualControlSession` owns, in
destruction order, the production scene, manifest, controller, and transient
plan. All C control calls occur on one owner worker at a 5 ms steady cadence
with the core's 20 ms cycle limit. Startup creates the checked scene and
manifest, calls `oa_controller_create_with_scene`, verifies both arms, validates
a fresh coherent collision-free disabled snapshot and sequence-bound FK, sets
`simulation_verified=true`, and remains `DISARMED`. It never auto-arms.

ROS callbacks perform bounded envelope validation and reservation only. One
reject-new/no-queue arbiter covers both actions, legacy paired input,
enable/disable, reset, and simulation verification. Stop alone bypasses it:
the callback atomically latches reject-new before posting one urgent owner
slot. The owner checks that slot before and after every bounded C call and
never executes a plan returned after the latch. Publish
`request_to_core_stop_ns`; the virtual release test budget is 100 ms on this
host under injected capture/HTTP/ROS load. Exceeding it faults the adapter. It
is still not a safety-rated stop latency.

The worker advances using actual elapsed steady time, heartbeats an executing
command, drains events with poll deadline zero, validates both complete fresh
masks, zero faults, advancing sequences, and <=1 ms pair skew, then publishes
only a new coherent measurement. Any timing, heartbeat, feedback, scene,
collision, event-overflow, or control failure attempts `OA_STOP_DISABLE`,
latches fault, terminalizes once, and suppresses apparently fresh state.

### Exact ROS IDLs

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
geometry_msgs/Pose tcp_pose_body
```

`msg/PortalState.msg`:

```text
uint8 BACKEND_VIRTUAL=1
uint8 BACKEND_PHYSICAL=2
std_msgs/Header header
unique_identifier_msgs/UUID session_id
uint64 state_sequence
uint8 backend
uint32 lifecycle
bool state_valid
bool simulation_verified
bool motion_enabled
bool motion_busy
bool software_stop_latched
bool fault_latched
bool collision_checked
uint64 collision_scene_revision
float64 minimum_clearance_m
float64 required_clearance_m
bool orientation_constrained
bool gripper_state_available
bool physical_motion_authorized
bool physical_calibration_available
ArmMeasuredState left
ArmMeasuredState right
```

`action/MoveTcp.action`:

```text
# Goal
unique_identifier_msgs/UUID session_id
uint64 required_state_sequence
geometry_msgs/PointStamped target
string tip_frame
---
# Result
uint8 COMPLETED=1
uint8 REJECTED=2
uint8 CANCELED=3
uint8 ABORTED=4
uint8 STOPPED=5
uint8 outcome
unique_identifier_msgs/UUID session_id
unique_identifier_msgs/UUID goal_id
uint32 control_status
uint64 command_id
uint64[2] seed_feedback_seq
uint64[2] terminal_feedback_seq
uint32 final_lifecycle
uint32 final_event
uint32 final_cause
bool collision_checked
uint64 collision_scene_revision
float64 minimum_clearance_m
bool orientation_constrained
geometry_msgs/Point measured_tcp_m
float64 position_error_m
string reason
---
# Feedback
uint8 RESERVED=1
uint8 PLANNING=2
uint8 EXECUTING=3
uint8 SETTLING=4
uint8 phase
uint64 command_id
uint64 planned_duration_ns
uint64 elapsed_ns
float32 trajectory_fraction
uint64[2] feedback_seq
geometry_msgs/Point measured_tcp_m
float64 position_error_m
```

`tip_frame` is exactly `openarm_left_hand_tcp` or
`openarm_right_hand_tcp`. The stamped point is transformed at its nonzero fresh
stamp into `openarm_body_link0`; missing/stale/extrapolated TF rejects before
planning. Orientation is absent, not ignored.

`action/MovePairedTcp.action`:

```text
# Goal
unique_identifier_msgs/UUID session_id
uint64 required_state_sequence
std_msgs/Header target_header
geometry_msgs/Point left_tcp_m
geometry_msgs/Point right_tcp_m
---
# Result
uint8 COMPLETED=1
uint8 REJECTED=2
uint8 CANCELED=3
uint8 ABORTED=4
uint8 STOPPED=5
uint8 outcome
unique_identifier_msgs/UUID session_id
unique_identifier_msgs/UUID goal_id
uint32 control_status
uint64 command_id
uint64[2] seed_feedback_seq
uint64[2] terminal_feedback_seq
uint32 final_lifecycle
uint32 final_event
uint32 final_cause
bool collision_checked
uint64 collision_scene_revision
float64 minimum_clearance_m
bool orientation_constrained
geometry_msgs/Point measured_left_tcp_m
geometry_msgs/Point measured_right_tcp_m
float64 left_position_error_m
float64 right_position_error_m
string reason
---
# Feedback
uint8 RESERVED=1
uint8 PLANNING=2
uint8 EXECUTING=3
uint8 SETTLING=4
uint8 phase
uint64 command_id
uint64 planned_duration_ns
uint64 elapsed_ns
float32 trajectory_fraction
uint64[2] feedback_seq
geometry_msgs/Point measured_left_tcp_m
geometry_msgs/Point measured_right_tcp_m
float64 left_position_error_m
float64 right_position_error_m
```

The paired action is for deliberate simultaneous motion and is the only new
path to `oa_controller_plan_paired_tcp`.

`srv/SetMotionEnabled.srv`:

```text
unique_identifier_msgs/UUID session_id
uint64 required_state_sequence
bool enable
bool operator_confirmed
---
bool accepted
uint32 control_status
uint32 lifecycle
string detail
```

`srv/RequestStop.srv`:

```text
string reason
---
bool accepted_for_processing
bool already_latched
bool core_transition_observed
uint32 control_status
uint32 lifecycle
uint64 request_to_core_stop_ns
string detail
```

`srv/ResetSafety.srv`:

```text
unique_identifier_msgs/UUID session_id
uint64 required_state_sequence
bool operator_confirmed
---
bool accepted
bool requires_enable
uint32 control_status
uint32 lifecycle
string detail
```

`srv/VerifySimulation.srv`:

```text
unique_identifier_msgs/UUID session_id
uint64 required_state_sequence
bool operator_confirmed
---
uint8 VERIFIED=1
uint8 NOT_REQUIRED=2
uint8 FAILED=3
uint8 outcome
bool hardware_calibrated
bool mapping_changed
uint32 control_status
uint64 evidence_revision
uint64 collision_scene_revision
string detail
```

Verification is idle-only and DISARMED-only. Success always returns
`hardware_calibrated=false`, `mapping_changed=false`, and
`evidence_revision=0`. There is no physical mode or AutoCalibrate action.

### Exact graph and QoS

- action servers: `/openarm_ik/move_tcp`,
  `/openarm_ik/move_paired_tcp`;
- services: `/openarm_ik/set_motion_enabled`, `/openarm_ik/request_stop`,
  `/openarm_ik/reset_safety`, `/openarm_ik/verify_simulation`;
- `/openarm_ik/portal_state`: reliable, transient-local, depth 1, published
  after each new coherent snapshot and every capability/lifecycle transition;
- `/joint_states`: reliable, volatile, depth 10, exactly 14 canonical arm
  names with measured q/dq/tau and no fingers;
- `/openarm_ik/diagnostics`: reliable, volatile, depth 10, periodic and on
  transition; and
- legacy `/openarm_ik/paired_xyz`: reliable, volatile, depth 10 for one
  migration cycle, deprecated, routed through the same arbiter/session and
  never through the old instantaneous processor.

The outer state/joint stamp is the oldest coherent member:

```text
ros_stamp = sampled_ros_now
            - (controller_now - min(left.controller_time_ns,
                                    right.controller_time_ns))
```

Use checked arithmetic; republishing never refreshes measurement time. Two
sequence-bound kinematics calls occur with no advance between them. Convert the
full TCP transform to a finite normalized quaternion. The adapter publishes no
TF. One `robot_state_publisher` is the only `/tf` and `/tf_static` authority.
A UID+`ROS_DOMAIN_ID` host lock is acquired before advertising `/joint_states`.

Reset clears the software/core latch only after explicit confirmation, clear
input, nonce/epoch validation, reverify, fresh coherent collision-free disabled
state, and exact scene revision. It ends DISARMED. Enable obtains a new arm
challenge. Disable/cancel/shutdown use `OA_STOP_DISABLE`, end DISARMED, and
require another explicit enable.

## Stage C — compiled loopback portal and actual RViz

Install one C++17 executable `openarm_portal`. It combines a bounded rclcpp
client/subscriber, Boost.Asio/Beast HTTP service, one X capture/encoder thread,
and compiled-in HTML/CSS/JS. It owns no model, controller, plan, joint-state
publisher, TF broadcaster, or RViz input injector.

Launcher arguments are exactly:

```text
openarm_portal --rviz-pid PID --rviz-start-ticks TICKS --port PORT
```

Port defaults to 8765. V1 has no remote-bind option and binds numeric IPv4
`127.0.0.1` only. Validate PID liveness, `/proc/PID/stat` start ticks (or retain
a pidfd), direct `rviz2` executable identity, `_NET_WM_PID`, mapped normal
top-level membership, unique window, and stable geometry. PID exit permanently
invalidates that launch identity.

One thread exclusively owns `Display*`. Use the compositor's existing
redirection and `XCompositeNameWindowPixmap`, generic XImage mask/stride/byte
order conversion, and libjpeg with a nonfatal `setjmp` boundary. Reacquire on
configure/map/unmap/reparent/destroy and trap asynchronous X errors. Capture
one immutable shared JPEG at 10 fps, quality 80, maximum four viewers, maximum
pixel/dimension caps, and no catch-up bursts. Each client has one async write;
slow clients skip to the latest frame. XDamage/MIT-SHM are optional only after
plain `XGetImage` passes.

The actual-host gate must prove that the parent redirected pixmap includes the
native Ogre child and Qt panels. A black/partial render is failure. Never use a
root crop, because it leaks covering windows. Minimized/unmapped RViz produces
an overlay and no stale-live claim. The stream is display-only; browser mouse,
keyboard, clipboard, and XTest injection are absent.

### HTTP routes and status codes

Static assets are compiled into the binary under exact paths:

```text
GET  /
GET  /assets/app.css
GET  /assets/app.js
GET  /api/bootstrap
GET  /api/health
GET  /api/state
GET  /api/rviz.mjpeg
GET  /api/rviz.jpg
GET  /api/commands/{uuid}
POST /api/lease/acquire
POST /api/lease/release
POST /api/commands/prepare
POST /api/commands
POST /api/stop
```

There are no other routes, directory serving, WebSocket, CORS, query-command,
or GET side effects. `/api/bootstrap` creates a random 128-bit observer session
cookie (`HttpOnly; SameSite=Strict; Path=/`) and returns a separate CSRF token.
Every mutation requires exact `Host: 127.0.0.1:PORT`, exact matching `Origin`,
session cookie, CSRF header, and JSON content type. Normal commands additionally
require the one expiring operator lease and a single-use prepared nonce bound
to portal epoch, controller session, browser session, lease, current state
sequence, exact canonical payload, and short expiry. `/api/stop` requires the
origin/session/CSRF checks but bypasses lease, prepared nonce, and normal
reservation.

Use these HTTP outcomes consistently:

- `200`: reads, already-latched stop, or completed synchronous lease release;
- `201`: session/lease/prepared-nonce created;
- `202`: ROS action/service accepted and immutable command URI returned, or a
  new stop latch accepted for processing;
- `204`: idempotent release with no body;
- `400`: malformed request/JSON, duplicate/unknown fields, bad scalar shape;
- `401`: missing/invalid/expired session;
- `403`: bad Host/Origin/CSRF or observer attempting mutation;
- `404`: unknown exact route or command ID;
- `405`: wrong method (`Allow` included);
- `409`: busy, lease held, stale session/state/scene, replayed nonce, stop/fault
  latch, disabled motion, or unavailable capability; stable JSON `code`
  distinguishes them;
- `413`: target/header/body/pixel limit exceeded;
- `415`: media type or transfer encoding rejected;
- `422`: finite but schema-valid command violates frame/units/range policy;
- `429`: rate, connection, or viewer cap;
- `500`: bounded generic internal failure;
- `503`: controller or actual-RViz dependency unavailable; and
- `504`: bounded ROS admission timeout.

Once a ROS goal is accepted, later planning/collision/execution failure is a
terminal command-ledger result retrieved from the returned URI, not a rewritten
HTTP response. Browser retries query status and never resubmit motion.

Every JSON response has `schema_version`, `request_id`, and either `data` or
`error:{code,message}`. All uint64/steady-time values are decimal strings to
avoid JavaScript precision loss.

`GET /api/state` schema version 1 is:

```json
{
  "schema_version": 1,
  "request_id": "uuid",
  "data": {
    "portal_epoch": "uuid",
    "controller_session_id": "uuid",
    "state_sequence": "uint64-decimal",
    "measurement_stamp_ns": "uint64-decimal",
    "backend": "virtual",
    "frame": {"id":"openarm_body_link0","unit":"m",
              "world_relation":"fixed_identity_model_only"},
    "controller": {"lifecycle":"DISARMED","state_valid":true,
      "simulation_verified":true,"motion_enabled":false,"motion_busy":false},
    "safety": {"software_stop_latched":false,"fault_latched":false,
      "physical_motion_authorized":false,"physical_calibration_available":false},
    "collision": {"checked":true,"scene_revision":"uint64-decimal",
      "required_clearance_m":0.025,"minimum_clearance_m":0.031,
      "includes":["self","inter_arm","body","support_pole","grippers","tcp_keepout"]},
    "kinematics": {"orientation_constrained":false,
                   "gripper_state_available":false},
    "arms": {
      "left":  {"feedback_seq":"uint64-decimal","controller_time_ns":"uint64-decimal",
        "fresh_mask":127,"fault_mask":0,"q":[0,0,0,0,0,0,0],
        "dq":[0,0,0,0,0,0,0],"tau":[0,0,0,0,0,0,0],
        "tcp_pose_body":{"position_m":[0,0,0],"quaternion_xyzw":[0,0,0,1]}},
      "right": {"feedback_seq":"uint64-decimal","controller_time_ns":"uint64-decimal",
        "fresh_mask":127,"fault_mask":0,"q":[0,0,0,0,0,0,0],
        "dq":[0,0,0,0,0,0,0],"tau":[0,0,0,0,0,0,0],
        "tcp_pose_body":{"position_m":[0,0,0],"quaternion_xyzw":[0,0,0,1]}}
    },
    "active_command": null,
    "lease": {"held":false,"mine":false,"expires_ns":null},
    "rviz": {"state":"LIVE","actual_stock_rviz":true,"generation":"uint64-decimal",
      "width":1280,"height":800,"last_capture_age_ms":42,"viewers":1},
    "disclosures": ["virtual_only","orientation_free","physical_motion_unavailable",
                    "software_stop_not_safety_estop","unmodelled_environment_not_checked"]
  }
}
```

Invalid state uses `state_valid=false` in the controller object, null measured
arrays/clearance where unavailable, and an explicit reason; it never fills
unknown data with zero. RViz state is `STARTING`, `LIVE`, `UNMAPPED`, `STALE`,
or `UNAVAILABLE`. A separate state poll drives the overlay because an `<img>`
can retain its last decoded frame after stream failure.

### Required UI behavior

The page has a persistent top banner and two panes. The banner always says:

> Virtual model only — no hardware control. XYZ position target; orientation
> is free. Collision approval covers the versioned robot/body/support-pole
> scene only; unmodelled objects are not checked.

The left pane shows backend, controller session/state age, exact model body
frame and metres, collision scene revision/minimum clearance, fixed joint speed
policy, software-stop/fault status, and operator lease. The right pane is an
`<img>` labelled **Live RViz capture** only after the actual-RViz gate; otherwise
it says Starting/Unmapped/Stale/Unavailable over the last image or blank pane.

On the first valid coherent state, seed left and right XYZ fields from their
own measured TCP. Never overwrite a dirty field. **Use current** explicitly
refreshes one side. The left/right Move buttons serialize only their named
side, session ID, state sequence, point, and canonical tip frame. The other
browser field is neither read nor sent. A deliberate **Move both** control uses
the paired action and displays both canonical targets for confirmation.

All move controls are disabled unless state is current, verified, DISARMED or
ARMED_IDLE as appropriate, motion is explicitly enabled, the scene is current,
`collision.checked=true`, no busy/stop/fault latch exists, and this session owns
the lease. Do not show a green Cartesian “safe range.” Planning rejection
leaves current fields/state unchanged and displays the authoritative result.

The fixed control order is:

1. **Auto Calibrate — simulation verification only** (DISARMED, idle);
2. **Enable virtual motion** with confirmation;
3. left/right/paired move controls; and
4. **Request stop (not a safety E-stop)**, always visible and visually dominant.

The stop handler disables move/enable/reset locally before starting any normal
request, POSTs `/api/stop`, and never clears on refresh/reconnect. Reset is
separate, confirmation-gated, ends DISARMED, and never implies Enable.
Decorative progress is capped below 100% until the matching measured COMPLETED
event; browser timers and RViz pixels are never completion authority.

Send fixed CSP/no-store/nosniff/frame-denial/referrer/permissions headers, no
inline code, external asset, service worker, eval, permissive CORS, or remote
font. Bound request line to 1 KiB, aggregate headers to 16 KiB, command body to
8 KiB, reject chunked mutations/CL+TE/duplicate Host or CL, and close after a
mutation if needed to make smuggling behavior simple.

## Stage D — Bash launch and clean shutdown

Extend `scripts/launch_rviz.sh`; keep the current lock, XWayland/GLX/renderer
environment, direct RViz `setsid`, and `WM_DELETE_WINDOW` helper. For portal
mode it retains ROS process-group PID, direct RViz PID/start ticks, and portal
process-group PID. Start ROS with `rviz:=false`, wait for one healthy transient
PortalState, start RViz, capture its start identity, start portal, and print the
canonical URL only after one valid complete frame and HTTP health are ready.
Do not auto-open a browser.

`wait -n` watches all three children. Unexpected exit of any child fails the
whole virtual session. Shutdown is idempotent and ordered:

1. TERM portal; it rejects mutations/revokes lease, requests disable through
   ROS, waits a bounded time for a fresh coherent DISARMED state, closes HTTP,
   joins ROS and capture threads, and releases X resources;
2. if portal does not exit, TERM then KILL its process group (cleanup only, not
   a safety success);
3. close RViz with `close_rviz_window PID --timeout 3`, then bounded TERM/KILL;
4. INT the ROS launch group, then bounded TERM/KILL;
5. wait/reap all children and release the flock descriptor.

If disable acknowledgement is absent, log
`STOP STATE UNCONFIRMED — VIRTUAL SESSION ABORTED`; never report successful
shutdown based only on killing a process. Physical hardware is unreachable, so
the stronger physical-E-stop warning is not used to imply a physical backend.

The existing `rviz:=false` headless path may remain an `exec ros2 launch` path.
“Compiled” applies to the controller and portal products, not to ROS 2's
existing Python launch infrastructure; no Python web server or command client
is introduced.

## Build/package order

1. Generate and review the calibrated scene/proxy artifact; make geometry
   digest drift fatal.
2. Add scene ABI, production manifest, TCP ABI, jerk plant, collision planner,
   and native tests; install `OpenArm::Control`.
3. Add ROS messages/actions/services and generated type support.
4. Add/test `VirtualControlSession`, then replace the old node and legacy shim.
5. Add portal capture/JPEG and HTTP/security modules in parallel, then link the
   ROS client and static UI.
6. Install `openarm_portal`, static data compiled in, launch/config/URDF scene,
   and exact package dependencies (rclcpp_action, rosidl runtime, TF2,
   Boost/Threads, X11/XComposite, JPEG).
7. Extend the launcher only after installed executable paths are stable.
8. Run clean native, headless ROS/HTTP, and finally logged-in-host RViz gates.

ROS and portal ELF dependency gates must show no CAN, transport, commission,
Python web, VNC, xpra, websockify, or libXtst dependency. Transport remains a
separately invoked query-only product and is not a transitive dependency.

## Mandatory tests

### Native ABI, dynamics, and collision

1. Strict installed C11/C++17 consumers cover every new symbol/record/prefix;
   frozen original V1 consumers and all existing sizes/offsets/symbols remain
   unchanged. Null/bad ABI/undersize/oversize/reserved/allocation failures leave
   output sentinels and registries unchanged.
2. Manifest output is deterministic, I/O-free, identity-mapped, uniquely
   named/identified, limit-correct, and shared by ROS/tests. Adversarial mapping
   fixtures stay separate.
3. Scene generation verifies source and generated digests, pole calibration
   fields, proxy containment of every mesh triangle, fixed-stroke gripper
   coverage, TCP keep-out, exclusion-list exactness, 25 mm minimum margin, and
   deterministic output. Missing/unapproved/stale pole data fails closed.
4. Collision unit tests cover every included/excluded link pair, touching,
   margin-1 epsilon/margin/margin+1 epsilon, GJK non-convergence, NaN/Inf,
   degenerate hull, pole endcaps/sides, body/pole, hand/finger/TCP, arm-arm,
   same-arm nonadjacent pairs, and mirrored sides.
5. Continuous-path adversaries collide only between knots or between coarse
   time samples. The recursive sweep certificate must reject them. Random
   paths are cross-checked against a much finer independent mesh oracle; any
   disagreement fails closed. Subdivision limit/overflow returns collision,
   never unchecked success.
6. Planning binds scene/model/manifest/epoch/both sequences/both start q.
   Changing scene after planning, moving either arm, reset/reverify, or another
   controller rejects stale/identity. Every successful plan report says checked
   with finite clearance; reject-all and unchecked policies never reach ROS.
7. Single-TCP tests cover both sides/axes, every validation boundary, all 17
   knots, inactive q bit-identical in the plan, only selected-chain IK, active
   measured completion, inactive advancing feedback and hold tolerance.
8. Reference and plant tests cover every joint, signs/offsets, short/long/reverse
   moves and variable legal dt. Ordinary motion respects reference
   `0.5/1/5` and plant `1/2/10`; no snap branch bypasses jerk. Stop exceptions
   occur only under explicit stopping/ESTOP lifecycle.

### ROS/controller truth and priority

9. Startup failure injection at every scene/manifest/create/verify/FK/collision
   step advertises no ready motion service and leaks nothing. Success is
   verified, checked, coherent, DISARMED, and cannot move before Enable.
10. Barrier-race single, paired, legacy, enable/reset/verify requests. Exactly
    one reservation wins; all others explicitly busy; no queue/mixed targets/
    double terminal. Stop bypasses them all.
11. Inject stop before reservation, during TF, every IK knot/collision
    subdivision, after plan/before execute, executing, settling, cancel, reset,
    and shutdown while capture/JPEG/HTTP are stalled. Ingress latches
    immediately, no post-latch plan executes, both arms disable, terminal result
    occurs once, repeated requests add no event, and request-to-core <=100 ms
    on the release host or faults visibly.
12. Reset/enable/disable/cancel truth tables prove reset never enables, clear
    input and fresh nonce are required, cancel/disable end DISARMED, and no
    reconnect/restart/browser retry clears a latch.
13. Freeze/drop/fault every joint; test timeout/skew boundaries, heartbeat,
    event overflow, clock jumps, stale scene, and second authority. Invalid
    feedback publishes no newly fresh state or success.
14. JointState has exactly 14 measured names/q/dq/tau and conservative oldest
    stamp; state TCP pose equals FK for the exact sequences. Exactly one
    JointState, TF, and static-TF publisher exists; no fingers or commanded
    targets are presented as measured.
15. Auto Calibrate UI/service performs zero advance beyond normal idle ticks,
    zero planning/execution/commission/transport calls, changes no mapping, and
    returns the exact simulation-only truth fields/text. Physical capability is
    absent, not a goal which pretends to run and returns unavailable.

### HTTP, UI, stream, and shutdown

16. Exact Host/Origin/session/CSRF/lease/nonce/replay matrices; request
    smuggling, duplicate fields, NaN/Infinity/huge numbers, chunked mutation,
    traversal, CSP, framed/cross-site attempts, stale epochs/state/scene, and
    concurrent clients all fail with the specified status/code and no ROS call.
17. UI tests prove first-state field seeding, dirty-field preservation, per-side
    serialization, deliberate paired confirmation, permanent disclosures,
    fixed speed display, collision-gated buttons, measured-only progress, and
    dominant lease-independent stop behavior.
18. Fake-X/JPEG/MJPEG tests cover PID reuse, multiple/no window, native child
    state, resize/depth/map/unmap/destroy, visual masks/byte order/stride,
    libjpeg fatal recovery, one shared frame, one write/client, slow clients,
    viewer caps, reconnect, and idempotent cleanup under ASan/UBSan/TSan.
19. Logged-in-host live acceptance under default software renderer and
    integrated renderer proves the full actual RViz top level including moving
    Ogre content and Qt panels, PID/start binding, occlusion correctness,
    minimize overlay, resize/HiDPI stability, <=250 ms normal stream latency,
    four-client bounded-resource soak, and no root/covering-window pixels.
20. SIGINT/SIGTERM and unexpected ROS/RViz/portal exit at every startup,
    command, capture, write, resize, and shutdown phase close the listener,
    preserve RViz WM close, reap all groups, release locks, and allow immediate
    relaunch. Store commit/build/library/renderer IDs, scene digest/revision,
    collision certificates, live frame evidence, latency/resource traces, and
    final process/socket listings.

## Final blockers and claim boundary

The design is complete, but two empirical inputs cannot be supplied by source
inspection:

1. the large support pole needs an approved pose/dimension/uncertainty record
   tied to `openarm_body_link0`; the current combined body STL is not sufficient
   evidence, so motion must fail closed until that artifact exists; and
2. XComposite capture must prove on the logged-in XWayland/GLX host that the
   stock RViz top-level pixmap contains the live native Ogre child.

Even after those gates, the release claim is limited to collision checking
against the versioned robot, body, gripper/TCP envelope, and calibrated support
pole in a virtual simulation. Orientation remains free, unmodelled environment
objects remain outside the scene, the web stop is not safety rated, and no
physical motion/calibration authority exists.
