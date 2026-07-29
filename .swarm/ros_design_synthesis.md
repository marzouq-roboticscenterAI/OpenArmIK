# Production ROS/control simulation design synthesis

Date: 2026-07-29 (America/Los_Angeles)  
Tree inspected: `main` at `19a92de`  
Status: **DESIGN COMPLETE — IMPLEMENT IN STAGES**

This is a read-only design synthesis. No executable was run against ROS, CAN,
the network, or a GUI. The only write is this requested report.

## Decision summary

1. **Replace the implementation of the existing `openarm_ik_ros_node` in
   place. Do not add a second control/state node.** Keep the executable, node,
   package, and `openarm_ik_rviz.launch.py` names for launch compatibility, but
   replace `PairedTransactionProcessor` with a C++17 `VirtualControlSession`
   over the installed `openarm_control` C ABI. There must be only one process
   capable of publishing `/joint_states` in the launch.
2. **Keep `robot_state_publisher` as the only TF authority.** The replacement
   node publishes no TF and no static TF. It publishes exactly the 14 measured
   arm joints. It does not fabricate the two finger actuator positions.
3. **Use the control simulator's decoded feedback as the only state source.**
   A command target, IK result, motion-plan report, or last requested posture is
   never copied into `JointState`.
4. **Add two custom ROS 2 actions as the production command API:** one named
   joint target and one transaction with explicitly named `left_tcp_m` and
   `right_tcp_m` fields. A shared reject-new arbiter allows exactly one reserved,
   planning, executing, settling, or cancelling goal across both actions.
5. **Retain `/openarm_ik/paired_xyz` for one compatibility cycle only as a
   deprecated shim into that same arbiter.** Its documented index 0/1 mapping is
   left/right. It gets no separate processor, queue, state, or publisher. New
   clients use the named paired action. Remove the shim at the next advertised
   major ROS API boundary.
6. **Replace the Python command helper with a compiled C++ action client.** One
   installed `openarm_control_cli` executable has `joint` and `paired-tcp`
   subcommands, waits for the matching action UUID and terminal result, and
   returns nonzero on rejection, abort, cancellation, timeout, or shutdown.
   Python is not part of the controller, node, or CLI runtime path.
7. **Fix the public header collision without changing binary layouts or symbol
   names.** Introduce module-prefixed status types/constants in the model and
   control headers and make every public declaration use them. Preserve the old
   generic names as guarded, deprecated single-header source aliases. Frozen V1
   binary consumers continue to link.
8. **Add one top-level native build/install graph.** It builds and exports CAN,
   model, transport, commission, and control, but does not combine their
   authorities. ROS links only `OpenArm::Control` and its model dependency.
   Transport remains a separately invoked query-only product; physical control
   remains unsupported.

These choices are the smallest safe product change. A new general-purpose
runtime ABI, ROS lifecycle conversion, physical coordinator, gripper simulator,
calibration integration, collision engine, and manifest persistence are useful
future work but are not prerequisites for the virtual measured-state adapter.

## Why the present adapter must be replaced

`openarm_ik_ros_node.cpp` currently owns `PairedTransactionProcessor`, calls the
model IK directly, immediately replaces its two stored joint arrays with the IK
solution, and republishes that command result forever with `now()` as its stamp.
It never creates, verifies, arms, advances, snapshots, heartbeats, stops, or
polls an `oa_controller`. Its `JointState` also appends two constant finger
positions that do not exist in `oa_snapshot`.

In contrast, `libopenarm_control` already provides the required simulation
semantics:

- a bounded-acceleration independent plant;
- DaMiao-format quantization and decode before measured state changes;
- complete-generation masks, sequences, timestamps, fault masks, and cross-bus
  coherence;
- measured-seed joint and paired-TCP planning;
- plan/controller/verification/manifest/model/scene/sequence/start-pose binding;
- producer and control-cycle watchdogs;
- measured q/dq and measured-FK completion with a three-cycle dwell; and
- a fail-closed physical backend (`open_and_verify` returns
  `OA_EUNSUPPORTED`).

The adapter should expose those facts rather than recreate them.

## Stage 0 — collision-free public ABI and unified native build

This stage is a release prerequisite for ROS because a compiled integration
consumer must be able to include all installed public headers.

### Header correction with binary compatibility

Change the current canonical declarations as follows:

- model: `oa_model_status`, `OA_MODEL_OK`, `OA_MODEL_EINVAL`,
  `OA_MODEL_ENONFINITE`, and the remaining `OA_MODEL_*` statuses;
- control: `oa_control_status`, `OA_CONTROL_OK`, `OA_CONTROL_EINVAL`,
  `OA_CONTROL_ESTATE`, and the remaining `OA_CONTROL_*` statuses;
- use `oa_model_status` in `oa_ik_diagnostics` and all model prototypes;
- use `oa_control_status` in `oa_event` and all control prototypes.

The underlying types stay exactly `int32_t` for model and `uint32_t` for
control. No record size, alignment, offset, calling convention, exported symbol,
or numeric value changes. Each header may define its old `oa_status` and
`OA_OK`/`OA_EINVAL` spellings only inside a common guarded legacy-alias block.
Thus a consumer of only one old header remains source-compatible; a consumer of
both headers compiles in either order and is required to use the unambiguous
module-prefixed names. Also provide a documented
`OPENARM_DISABLE_LEGACY_GENERIC_STATUS` opt-out so an umbrella/all-header
consumer receives no generic aliases. The frozen original control header
remains a binary consumer test. Do not silently grow any V1 structure.

This is preferable to a new wrapper ABI: it corrects the collision at its
source, keeps the already-hardened controller seam, and avoids a second handle
and lifecycle implementation.

### One native build/install graph

Add a root CMake project that adds the five native components and installs a
consistent component package (`OpenArm::Can`, `OpenArm::Model`,
`OpenArm::Transport`, `OpenArm::Commission`, and `OpenArm::Control`) while
retaining existing package aliases for compatibility.

Required build cleanup:

- give CAN install/export rules;
- give control build/install include interfaces, an exported target, config and
  version files, with `find_dependency(openarm_model)` or the equivalent unified
  model component;
- make transport link the CAN codec target rather than compiling a second copy
  of `can/src/openarm_can.c`;
- compile allocation-failure/test registry hooks only into test objects, not
  installed release archives;
- keep sanitizer options component-specific but make a root test option drive
  all ordinary tests consistently;
- do not run the vcan smoke test in the hardware-free default profile; and
  keep the query-only transport as a separate target, never a transitive ROS
  dependency.

`scripts/build.sh` should configure/build/install this native graph into
`ros2_ws/install`, then invoke colcon. The ROS package uses
`find_package(openarm_control CONFIG REQUIRED)` (or the stable unified alias)
and links the exported target. It must not add monorepo-relative model/control
sources and must not create a second model target.

Add one strict installed C11 and one installed C++17 consumer that include all
five public headers in both forward and reverse orders, use only module-prefixed
statuses, create the standard virtual controller, take a snapshot, and destroy
it. That is the compiled non-ROS ABI usage proof.

### Standard virtual manifest

Move the exact 2x7 OpenArm v1.0 virtual manifest builder now embedded in
`control/tests/test_control.cpp::valid_config` into production control code and
exercise that same builder from the tests. Expose one narrow function for the
fixed standard virtual manifest—specifically
`oa_manifest_create_openarm_v10_virtual(oa_manifest **out)` returning
`oa_control_status`—rather than copying the test record into ROS. It fixes
canonical names, model/manifest revisions, URDF limits, motor families (J1/J2
DM8009, J3/J4 DM4340, J5-J7 DM4310), mappings, unique virtual identities, and
watchdog data. It never probes or opens `vcan*`; bus labels must make their
in-process nature explicit. `oa_manifest_load` remains unsupported.

## Stage 1 — reusable virtual control session

Add a small C++17 RAII library in the ROS package, unit-testable without an ROS
executor. It includes only the collision-free control header and owns, in
destruction order, the standard manifest, controller, and any transient plan.
It is the sole caller of the controller C API.

Startup is fixed and not parameter-selectable:

1. create the standard manifest;
2. create a controller with `OA_BACKEND_VIRTUAL`, explicit
   `OA_COLLISION_VIRTUAL_UNCHECKED`, scene revision 1, a 20 ms maximum control
   cycle, 100 ms feedback timeout, and 1 ms maximum cross-arm skew;
3. open and verify, require verified mask `0x3` and zero failure mask;
4. obtain the arm challenge and arm it; and
5. take and validate the initial coherent measured snapshot.

No ROS parameter selects `OA_BACKEND_PHYSICAL`, CAN interface, transport,
commissioning, calibration, simulator state injection, or collision policy.
The unchecked virtual collision opt-in is explicit in code and every success
remains diagnostic WARN with `collision_checked=false`.

### Clock and control loop

A dedicated owner thread serializes all C API calls. ROS callbacks only validate
the cheap envelope, reserve the arbiter, and enqueue immutable work. The worker
uses a 5 ms `steady_clock` cadence and passes elapsed nanoseconds since session
startup—not UNIX time and not a synthetic fixed increment—to
`oa_controller_advance`. The first advance is therefore near 5 ms from the
controller's initial zero. A late cycle over 20 ms is intentionally allowed to
fault in the core.

After every successful advance the worker:

- renews an executing command heartbeat with an overflow-checked controller
  deadline 100 ms ahead;
- drains `oa_controller_poll_event` with deadline zero;
- takes one snapshot;
- requires both `fresh_mask == expected_mask == 0x7f`, zero fault masks,
  coherent timestamps within 1 ms, a new feedback sequence, and a permitted
  lifecycle; and
- publishes/copies state only after all checks pass.

Any advance, heartbeat, integrity, snapshot, publication-clock, or unexpected
event failure attempts `oa_controller_stop(..., OA_STOP_DISABLE)`, latches the
adapter fault, terminates the active action exactly once, rejects all later
commands, emits ERROR diagnostics, and stops publishing apparently live joint
state. It never automatically resets, reverifies, or rearms after a fault.

`/use_sim_time=true` is rejected in this first standalone simulator release.
Controller progress is wall/steady driven; accepting a paused or jumping ROS
clock would make ingress freshness ambiguous. A sampled ROS/system-to-steady
offset is monitored for backward or excessive jumps and faults closed. This
restriction can be removed later only with an explicit clock contract.

### Planning and execution

For either command, the worker obtains one fresh snapshot and uses its exact
feedback sequence(s). It creates a controller-domain plan expiry with checked
addition, plans, reads the immutable report, and rejects reports whose duration
cannot fit the configured maximum command horizon plus settle margin. It then
creates checked absolute `start`, execution-expiry, and producer deadlines,
executes with `OA_STOP_DISABLE`, destroys the plan immediately, and retains only
the command ID and immutable report fields.

Fixed initial motion policy is preferable to a large unsafe parameter surface:
velocity/acceleration/jerk scales 0.5, joint position tolerance 5e-4 rad,
joint velocity tolerance 2e-2 rad/s, TCP tolerance 1e-3 m, maximum paired branch
step 2.0 rad, and the core's documented singularity policy. Bounds and maximum
request age/horizon may be node parameters only after finite, range, duration,
and integer-overflow validation at construction.

## Stage 2 — production ROS contract

Generate two actions in `openarm_ik_ros`; generated support is build-time ROS
infrastructure, while the node and client remain compiled C++ executables.
The servers are `/openarm_ik/move_joint` and
`/openarm_ik/move_paired_tcp`, both reliable with ROS action correlation and
terminal semantics.

The package build adds `rosidl_default_generators`, `rclcpp_action`,
`builtin_interfaces`, `std_msgs`, `unique_identifier_msgs`, `tf2_ros`, and
`tf2_geometry_msgs`; exports `rosidl_default_runtime`; and retains `rclcpp`,
`sensor_msgs`, `geometry_msgs`, `diagnostic_msgs`, `openarm_description`, and
the X11 close helper. Remove the direct `openarm_model` dependency and old
transaction target. `package.xml` records the ROS dependencies and documents
the locally installed non-ament `openarm_control` prerequisite. The node,
session tests, and CLI link the generated type support and exported control
target, never private control/model sources.

### `MoveJoint`

Goal fields:

- nonzero request stamp;
- exact canonical `joint_name` from
  `openarm_{left,right}_joint1..7`; and
- finite `target_rad`.

The adapter maps the name to zero-based side/joint only after confirming it
matches the standard manifest. No numeric side/index is accepted over ROS.

### `MovePairedTcp`

Goal fields:

- one nonzero common header stamp and source frame;
- named `geometry_msgs/Point left_tcp_m`; and
- named `geometry_msgs/Point right_tcp_m`.

The names, not serialization order, define sides. All six values must be
finite. Both points are transformed transactionally at the common request stamp
into `openarm_body_link0`; missing, stale, extrapolated, or inconsistent TF
rejects the whole goal without planning. Orientation is absent rather than
silently ignored. The control ABI's paired coordinates must be documented as
body-frame metres.

Both actions return a stable outcome enum, control status, action goal UUID,
controller command ID (zero if execution never started), seed and terminal
feedback sequences, final lifecycle/event/cause, collision flags, and a short
machine-readable reason. Feedback carries controller event/lifecycle, command
ID, current feedback sequences, and measured progress. Action results—not
diagnostics—are the authoritative acknowledgement.

### Arbitration and cancellation

One mutex-protected reservation spans both action servers and the legacy topic.
The policy is **reject new; no queue; no preemption**. Reservation occurs in the
goal callback before returning acceptance, so simultaneous joint and paired
requests have one deterministic winner. It remains held through validation that
needs TF, planning, execution, settling, and terminal result publication.

Action cancellation is accepted for the active owner only and is serialized on
the worker. It calls `OA_STOP_DISABLE`, reports CANCELED exactly once, leaves the
controller disarmed, and latches a `stopped_requires_restart` adapter state.
Subsequent motion is rejected until process restart. Shutdown uses the same
disable behavior. This deliberately avoids an implicit re-arm policy.

### Backward compatibility

Keep the old executable, ROS node, package, launch filename, `/joint_states`,
`/openarm_ik/diagnostics`, and `/openarm_ik/paired_xyz` names for one migration
cycle. The old topic validates exactly two poses and treats index 0 as left and
index 1 as right, but then creates the same named internal paired command and
uses the same arbiter/session. It may accept `world` only through an actual
timestamped TF lookup; it may not reinterpret world coordinates as body
coordinates because the current URDF transform happens to be identity.

Compatibility diagnostics echo the exact request stamp and do not set
`committed=true` until the matching measured COMPLETED event. They are WARN on
unchecked success. Concurrent or duplicate legacy producers are unsupported
and rejected. The old model-only `PairedTransactionProcessor`, its target, and
its unit test are removed; there is no switch that restores instantaneous
command-as-state behavior.

The existing Python helper is replaced by the compiled action CLI. A temporary
shell wrapper may print a migration message and exec the compiled paired client,
but it must not implement ROS behavior in Python.

## State, TF, diagnostics, and authority

### Joint state

Publish the validated initial decoded snapshot once, then publish reliable,
volatile, depth-10 `/joint_states` only for a new coherent snapshot after a
successful advance. Each message contains exactly:

- 14 canonical arm names;
- `position = oa_snapshot.arm[side].q`;
- `velocity = oa_snapshot.arm[side].dq`; and
- `effort = oa_snapshot.arm[side].tau`.

There are no finger entries. The controller has no gripper plant or gripper
feedback, so zero finger positions cannot be described as measured. Missing
finger dynamic TF in this release is preferable to fabricated authority;
`robot_state_publisher` remains responsible for mimic behavior if a valid
actuated finger state is added later.

A single `JointState` stamp conservatively represents the oldest member of the
coherent pair:

`stamp = ros_now - (controller_now - min(left.t_ns, right.t_ns))`.

The conversion is made from paired steady/ROS samples with checked arithmetic;
zero, negative, future, or jumped results fault/suppress publication. Per-arm
controller times, sequences, calculated ages, and skew remain in diagnostics.
Publishing again never refreshes a measurement timestamp.

The canonical launch contains exactly one replacement adapter and one
`robot_state_publisher`; it contains no `joint_state_publisher`, ros2_control,
static-transform publisher, or second adapter. The adapter never broadcasts
TF. `robot_state_publisher` alone publishes `/tf` and `/tf_static`, including
the URDF's fixed `world -> openarm_body_link0` and dynamic arm transforms driven
by measured joint state.

Before advertising `/joint_states`, the adapter takes a nonblocking host lock
keyed by UID and `ROS_DOMAIN_ID`. A second local authority exits nonzero. The
existing RViz launcher lock remains useful for GUI ownership, but is not the
state-authority lock. DDS cannot make this exclusive across hosts, so deployment
must isolate domains/namespaces and monitor the graph.

### Diagnostics

Publish periodic health and event-driven `DiagnosticArray` messages. Stable
key/value fields include:

- `backend=virtual`, capability bits, `virtual_execution_enabled=true`,
  `physical_motion_authorized=false`, `collision_policy=virtual_unchecked`,
  `collision_checked=false`, and `orientation_constrained=false`;
- `state_source=oa_snapshot_encoder_feedback`;
- controller lifecycle, adapter state, active action UUID/command ID, last
  event and control cause;
- manifest/model/scene/verification revisions;
- per-arm expected/fresh/fault masks, feedback sequence/time/age, and pair skew;
- request stamp, plan seed sequences, duration, and terminal measured sequence;
  and
- shutdown/fault/cancel reason.

Unchecked successful planning/execution is always WARN, malformed/busy
rejection is WARN, and controller, freshness, coherence, watchdog, or shutdown
failure is ERROR. Do not serialize unavailable achieved poses as zeros.

## Graceful shutdown

The node owns an explicit idempotent `close()` and RAII guards around ROS and
the C handles. On SIGINT, SIGTERM, ROS context shutdown, constructor failure, or
callback exception:

1. atomically enter closing and reject new goals;
2. stop/reset ROS ingress and timers so no new work can retain handles;
3. tell the worker to terminalize pending/active goals;
4. on the owner thread call disable-stop when armed/executing, then disarm if
   the lifecycle permits;
5. drain nonblocking events needed for the terminal result, request worker exit,
   and join it;
6. destroy the plan if one exists, controller, then manifest exactly once;
7. release the authority lock and destroy the node; and
8. call `rclcpp::shutdown()` from both normal and exception paths.

No executor callback blocks in `oa_controller_poll_event`; deadline zero is
used. Planning and worker exit have tested finite upper bounds. Shutdown does
not depend on destroying a controller to wake ordinary work.

## Physical and query-only boundary

This work does not create a physical product:

- ROS has no backend selector, CAN/interface parameter, discovery endpoint,
  calibration endpoint, physical action server, or manifest write endpoint;
- `OA_BACKEND_PHYSICAL` continues to fail verification with
  `OA_EUNSUPPORTED` and zero physical-motion capability;
- the ROS ELF does not link CAN, transport, or commission;
- public transport remains query-only and in a separately invoked process or
  library; interface observation never claims side/joint identity; and
- simulator or caller-written commissioning evidence never authorizes physical
  calibration or motion.

The unified build unifies compilation, packaging, and header consumption only.
It must not be misrepresented as a unified physical runtime.

## Dependency and merge order

1. **ABI names and tests:** prefix model/control statuses and add coexistence,
   layout, and frozen-header tests.
2. **Native packaging:** root build, CAN/control exports, transport-to-CAN target
   cleanup, release test-hook gating, installed multi-header consumers.
3. **Control production fixture:** standard virtual manifest builder and its
   exact config/URDF drift tests.
4. **Session library:** RAII startup, monotonic worker, snapshot validation,
   planning/execution, heartbeat, event mapping, fault latch, and shutdown unit
   tests.
5. **ROS interfaces and compiled CLI:** action definitions, shared arbiter,
   TF input transform, results/feedback, CLI exit semantics.
6. **Replace the node:** measured `JointState`, diagnostics, authority lock,
   compatibility topic, and upgraded launch.
7. **End-to-end/fault/shutdown gates:** only after all lower-level suites pass;
   then update README and remove the Python runtime helper.

Stages 1–7 must not be merged around a failing Stage 0. The old ROS adapter
must not be partly wired beside the controller during migration.

## Exact acceptance tests

All tests are hardware-free and make no network/CAN access unless a separately
selected query-only test profile says otherwise.

1. **Header coexistence and ABI freeze.** Compile C11 and C++17 installed
   consumers with model/control/CAN/transport/commission headers in forward and
   reverse order under `-Wall -Wextra -Wpedantic -Werror`. Use prefixed status
   names. Assert every public V1 `sizeof`, alignment, and field offset. Run the
   frozen original model/control consumers against the new archives.
2. **Unified build/install.** A clean root configure, build, ctest, and install
   succeeds; a consumer outside the source tree finds and links every exported
   component. `nm` shows no installed test-injection symbols and transport has
   one CAN codec implementation.
3. **No physical capability.** For every ROS parameter combination, no physical
   server appears. Direct `OA_BACKEND_PHYSICAL` open/verify returns
   `OA_EUNSUPPORTED`, verified mask zero/failure mask `0x3`, and no plan can be
   executed.
4. **Standard manifest drift.** The production builder produces exactly two
   arms and 14 unique canonical names, expected motor families, IDs/serials,
   affine mappings, model/manifest revisions, and limits equal to the generated
   URDF/model. Tests and ROS consume this builder, not a copy.
5. **Session startup/RAII.** Cover failure at every create/verify/challenge/arm/
   snapshot step, correct reverse destruction, double `close()`, constructor
   exception, and no leaked handles. Initial state is a coherent decoded
   snapshot, not hand-written zeros.
6. **Joint plan mapping and limits.** For every one of 14 names, target exact
   lower/upper limits and boundary +/- epsilon. The correct zero-based side and
   joint reaches the core; all other joints hold their measured start. Reject
   unknown/duplicate names, nonfinite targets, stale stamps, overflowed
   deadlines, and durations beyond the horizon without plant change.
7. **Paired ingress atomicity.** Test asymmetric named left/right points,
   swapped serialized field order, NaN/Inf on each axis, one unreachable side,
   stale/future/zero stamps, and invalid/missing/stale/extrapolated TF. Failure
   starts no command and leaves measured q, revisions, and active-command state
   unchanged; normal idle feedback sequences may continue to advance.
8. **Measured joint motion.** From the initial snapshot command, for example,
   `openarm_left_joint4` to an in-range nonzero target. Observe multiple new
   feedback sequences and at least two intermediate decoded q values before the
   target; the first accepted/started state is not the report's target q. Only
   that joint moves materially. COMPLETED arrives only after measured q/dq are
   in tolerance for three full cycles.
9. **Measured paired motion and TF.** Send reachable asymmetric named targets.
   Observe both arms move through intermediate measured states and both TCP TFs
   converge together. Terminal result occurs only after the core's measured
   q/dq/FK dwell. At every sample TF is explainable by that sample's
   `JointState`, never by `oa_motion_plan_report.target_q`.
10. **Feedback provenance delay.** Apply the same bounded virtual-only
    `feedback_delay_ns` below timeout to both arms. Assert state stamps are older
    than publication time by that age within scheduling tolerance, sequences
    advance, and plant-lag intermediates remain visible. This injection
    backdates timestamps; it does not queue historical values, and the test must
    not claim otherwise.
11. **Freshness/coherence/fault.** Independently freeze/drop/fault each joint,
    set delay at timeout and timeout+1, and skew arms at 1 ms and 1 ms+1 ns.
    Partial/frozen feedback never completes a goal. On invalid generation the
    adapter disable-stops, emits the exact core cause, publishes no newer
    `JointState`, rejects later goals, and never resets itself.
12. **Arbitration race.** Barrier-race N joint, paired, and legacy commands under
    single- and multi-threaded executors. Exactly one reserves; every other
    request is explicitly busy. There is no queue, preemption, mixed pair,
    double result, or result attributed to another UUID/stamp.
13. **Cancellation and stop.** Cancel in reserved, planning, queued, executing,
    and settling states. When core execution exists, `OA_STOP_DISABLE` is
    observed. Exactly one CANCELED/ABORTED terminal appears, measured velocity
    goes to zero/disabled as the core defines, and later commands receive
    `stopped_requires_restart`.
14. **Diagnostics truth table.** Assert stable fields and severity for startup,
    idle, accepted, started, settling, completed, rejected, busy, cancelled,
    faulted, and closing. Unchecked success is WARN; stale/fault is ERROR; action
    UUID, command ID, request stamp, sequences, masks, ages, revisions, source,
    and causes are correct. Unavailable achieved values are absent.
15. **CLI correlation and exits.** Preload unrelated diagnostics/results, race
    another client, and run both CLI subcommands. The client uses only its action
    UUID, waits for terminal measured completion, and returns 0 only for
    COMPLETED. Discovery failure, goal rejection, core abort, cancellation,
    timeout, and shutdown return distinct nonzero codes. `file` identifies an
    ELF executable and `ldd` shows no Python runtime.
16. **Authority/TF launch contract.** Headless launch reports one
    `/joint_states` publisher, one `/tf` publisher, one `/tf_static` publisher,
    no joint-state/static-transform publisher process, and no TF broadcaster in
    the adapter. A second local adapter fails before advertising because of the
    authority lock. The 14 state names are exact and contain no fingers.
17. **No CAN/control-transport path.** `ldd`/`nm` show the ROS node and CLI do
    not depend on CAN/transport/commission. `strace -f` across launch, both
    commands, fault, and shutdown shows no `AF_CAN`/`PF_CAN` or netlink socket
    and no shell tool that changes an interface. DDS sockets are expected ROS
    middleware and are not mistaken for a hardware transport violation.
18. **Clock boundary.** Reject `/use_sim_time`, zero stamps, stale stamps,
    unsupported future stamps, arithmetic overflow, and configured ROS clock
    jumps. Exact expiry and +/-1 ns cases are deterministic. Controller
    deadlines are elapsed steady time and never ROS epoch nanoseconds.
19. **Graceful shutdown under load.** Send SIGINT, SIGTERM, and ROS context
    shutdown during idle, TF validation, planning, execution, event drain,
    diagnostics publication, cancellation, and concurrent goal arrival. Within
    a fixed two-second budget the node rejects new work, terminalizes the owner
    where possible, disable-stops, joins the worker, destroys each handle once,
    and exits without deadlock, use-after-free, or post-destruction callback.
20. **Compatibility.** The old launch/executable and a valid legacy PoseArray
    still work through the measured controller path. Its request stamp matches
    the terminal compatibility diagnostic and `committed=true` is never emitted
    at plan acceptance. Diagnostics WARN that the topic is deprecated and uses
    positional left/right mapping; canonical named action semantics are
    unaffected. No test can select the old instantaneous processor.
21. **Regression and soak.** Preserve all strict control/model/native tests,
    generated-URDF/mesh validation, invalid parameter tests, and no-CAN tests.
    Run ASan/UBSan, TSan session/arbiter tests, and a deterministic long virtual
    campaign with randomized commands, cancellation, stale feedback, skew,
    faults, clock boundaries, and shutdown. Memory/event queues stay bounded,
    no commanded state is stamped as measured, and no false completion occurs.

## Merge gate

Do not call the integration production-ready until tests 1–21 pass from a clean
install under ROS 2 Lyrical. A partial change that merely links control while
retaining `PairedTransactionProcessor`, `PoseArray` as the only production
command, target-as-state publication, fabricated finger values, diagnostic
acknowledgements, or a backend selector must be rejected.
