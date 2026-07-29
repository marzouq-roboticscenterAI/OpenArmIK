# Design critique 1: OpenArm v1.0 C control and IK proposal

Reviewed against the pinned evidence in `research_control.md`: `openarm_can` c32ecd31, `openarm_description` 6c7b720f, `openarm_ros2` 4e837e1d, and `openarm_teleop` eb2d4933.

## Verdict

Reject the proposal as a production physical-control design in its current form. Accept it as a hardware-free model/visualization milestone after tightening the kinematics and IK contracts.

The good choices are C11, a virtual backend, an exact MIT codec, explicit lifecycle/no auto-arm, and keeping ROS 2 in a separate C++ process. The blockers are narrower but fundamental:

1. “Fixed left/right model” does not identify which canonical generated coordinate system it freezes. Upstream single-arm and bimanual xacro paths differ, particularly J2 fixed RPY, J1/J2 effective limits, base transforms, and left J7 axis.
2. “Joint origins” is mathematically ambiguous: a URDF joint origin is the pre-motion frame, while consumers usually need the post-joint child-link frame. `hand_tcp` in the current v1.0 example is fixed at zero offset from link7, not necessarily the gripper center or installed tool TCP.
3. Position-only IK is a 3-constraint, 7-variable problem with four-dimensional redundancy in a regular pose. A posture term does not merely “bias” the answer; it largely defines it. A naïve DLS-plus-clamp implementation will give misleading convergence and unreachable classifications at active joint bounds.
4. A dual-arm facade does not provide bimanual planning, collision safety, or atomic cross-bus execution. Independent xyz targets can create self/inter-arm collisions and inconsistent half-updates.
5. MIT encode/decode alone is not a safe CAN layer. Production requires feedback status decoding, freshness, exact expected-node probing, watchdog behavior, bus error handling, command expiry, and commissioned motor-to-joint maps.
6. “SocketCAN enumeration” overstates what is feasible. Linux interfaces can be enumerated; motors cannot be generally enumerated. Upstream actively scans candidate ESC IDs/bitrates and assumes receive IDs `send+0x10` or zero. It cannot identify physical joint, side, type, direction, zero, duplicate IDs, or arbitrary Master IDs.
7. “Simulation never opens physical CAN” is a policy statement until backend construction, linkage, defaults, and syscall tests enforce it.

## API usability and model correctness

### Fixed generated models

The implementation must name the frozen model precisely. The safest useful pair is **the left and right chains extracted from the canonical bimanual v1.0 URDF**, expressed from `openarm_body_link0` to `openarm_{side}_hand_tcp`, at `openarm_description` commit 6c7b720f. This preserves the documented base transforms and bimanual effective limits. Calling them simply “left/right v1.0” invites callers to mix them with the single-arm frame convention.

Every generated C file should contain:

- upstream repository, commit, source xacro/YAML paths, generation command/tool version, and Apache-2.0 attribution;
- model ID/version and an integrity hash;
- parent, root, link, joint, and tip names;
- base fixed transform, each fixed joint-origin transform, axis, effective limit, and hand-TCP fixed transform;
- a generated test fixture containing the equivalent flattened URDF values.

The current proposal should not hand-transcribe constants. A small generator should parse the flattened canonical URDF and emit checked-in C constants. Runtime remains fixed and dependency-free; regeneration is an explicit developer action.

### FK output contract

“Return all joint origins” must be replaced with an explicit result type. At minimum:

```c
typedef struct {
    oa_transform base_in_parent;
    oa_transform joint_pre[7];   /* parent/model base -> fixed joint frame, before q[i] */
    oa_transform link_post[7];   /* parent/model base -> child link frame, after q[i] */
    oa_transform hand_tcp;       /* exact named URDF frame */
} oa_fk_result;
```

Define transform direction, quaternion order or matrix layout, multiplication convention, radians/metres, and whether outputs are relative to the arm base or body frame. Reject non-finite joint input. Do not call `hand_tcp` an end-effector/tool pose without exposing its source frame name: upstream's v1.0 fixed `hand_tcp` is coincident with link7 in the generated example, while gripper geometry extends beyond it. Installed tool TCP must be a caller-supplied fixed transform or separately commissioned model.

### Joint coordinates versus motor coordinates

The model API accepts model joint coordinates only. The physical layer must not assume those equal raw motor coordinates merely because upstream ROS and teleop currently use identity conversion. Bimanual xacro applies side-specific origins/limits, the calibration script has an unimplemented precise-home step, and firmware RIDs `dir` and `Gr` are installation facts.

Keep a mandatory per-joint physical manifest:

`q_model = sign * scale * q_motor + offset`

with corresponding velocity and power-consistent torque transforms. The fixed model may ship without a physical manifest and work in simulation. A physical arm cannot transition to ARMED until a commissioned manifest matches side, IDs, serial/firmware/config registers, and legal current pose.

## IK critique and smallest mathematical correction

### What position-only IK means

For target position `p*`, residual `e = p* - p(q)` and translational Jacobian `Jp` is 3x7. At a regular pose the null space is four-dimensional, not the one-dimensional redundancy associated with full 6D pose IK. Orientation is completely unconstrained and may change drastically between nearby solutions if the seed/posture policy changes. This can twist a gripper, cables, or payload even when xyz error is small.

The API must therefore be called `oa_ik_position`, not generic `oa_ik`, and must require:

- target frame/model ID;
- seed `q_seed[7]`;
- posture reference and per-joint weights;
- position tolerance, max iterations, max joint step, damping policy, and time/iteration budget;
- explicit effective limits/safety margin;
- result status plus achieved xyz error, final orientation, iterations, active-limit mask, minimum singular value/conditioning indicator, and whether it stopped by convergence, bounds, stagnation, or budget.

The ROS xyz service/action must communicate that orientation is free and return it. It must not directly command physical motors.

### DLS details that cannot be omitted

A defensible weighted task step is of the form

`dq_task = W^-1 Jp^T (Jp W^-1 Jp^T + lambda^2 I)^-1 e`.

Posture/limit motion belongs in a null-space term based on the same generalized inverse. Damping must rise near rank loss; a fixed damping value trades accuracy for stability unpredictably. Cap joint and Cartesian step, use a merit-decreasing line search/trust region, and terminate on both residual and step/stagnation tests.

Do not compute an unconstrained step and merely clamp final q. Clamping invalidates the null-space/projector calculation and often causes limit cycling or false success. The smallest adequate bound treatment is an active-set loop: hold joints that hit a bound, recompute the reduced Jacobian/step for free joints, and only accept a merit-decreasing feasible step. A smooth joint-limit/posture cost is secondary; it does not replace hard bounds.

“Unreachable” cannot be inferred solely from exhausting iterations. Return distinct `NO_CONVERGENCE`, `STAGNATED_AT_BOUNDS`, `SINGULAR`, and `BUDGET_EXHAUSTED`; reserve `UNREACHABLE` for a bounded multi-start/search policy that failed under a documented completeness approximation. Never treat the best approximate xyz as success.

### Production motion implication

Position-only IK can be production-quality as a mathematical utility, but not as a general end-effector motion planner. Before its output reaches hardware, a higher layer must validate a time-parameterized joint path for velocity/acceleration/jerk, model and commissioned limits, self/body/other-arm/environment collision, and acceptable final/intermediate orientation. The smallest current correction is to keep ROS xyz targets simulation/visualization-only. Add 6D pose IK later if physical Cartesian control is required.

## Dual-arm facade critique

Two independent arm handles are useful, but “dual-arm” must not imply coordinated robotics semantics. Standard v1.0 arms reuse CAN IDs 1–8 and require separate physical buses. CAN offers no atomic cross-bus transmit.

The smallest honest facade should:

- own two independent arm/model/backend handles with explicit left/right roles;
- accept one versioned paired target/command object rather than two unrelated mutable callbacks;
- snapshot both states with monotonic timestamps and expose measured skew;
- define fault coupling (`either fault => request stop on both` by default);
- reject stale/partial paired updates;
- make no collision-free, synchronization, or bimanual-planning claim.

For simulation, solve both targets against one consistent state version and publish both results together. For eventual hardware, a coordinator may schedule shared command epochs, but must measure and bound cross-bus skew. Inter-arm collision checking remains a required higher-level gate.

## CAN and lifecycle critique

### Enumeration/probe terminology

Split the concepts:

- `oa_socketcan_list_interfaces()` uses read-only OS inspection/rtnetlink to list host CAN interfaces and their state/configuration.
- `oa_arm_probe_expected(bus, manifest, report)` actively transmits register queries only to expected ESC IDs while torque is disabled.
- An optional commissioning `scan_candidates` may alter host bitrate and try candidate IDs, but it is not part of production startup and must not be called “enumeration.”

A read-only motor probe means “no register/flash/motion write,” not “no transmit” or “no side effects.” It requires a configured matching bitrate and known/assumed Master IDs, can contend with another bus owner, and may affect firmware communication-timeout bookkeeping. It cannot prove physical identity without a commissioned manifest.

### Exact codec scope

Accept an MIT-only control mode for the first release, but “exact MIT codec” must include:

- motor-specific P/V/T ranges (DM8009, DM4340, DM4310);
- exact 16/12-bit packing and feedback unpacking;
- status/error nibble and embedded motor-ID decoding;
- enable, disable, refresh, and register-query frames needed by the lifecycle/probe;
- CAN SFF/EFF/RTR/error flag and DLC validation;
- deterministic non-finite/out-of-range policy, with saturation reported rather than silent;
- byte-order-independent implementation and golden vectors.

The raw protocol range is not a joint safety limit. Apply commissioned model limits and dynamic command bounds before encoding.

### Minimum physical lifecycle

No auto-arm is necessary but insufficient. A production physical backend also requires:

- initial torque-disabled state; explicit expected-node probe and fresh valid state for all seven motors;
- verification of status nibble, IDs, motor type/config/firmware and commissioned mapping;
- monotonic feedback freshness and command-expiry deadlines;
- configured/verified motor TIMEOUT behavior, plus host watchdog;
- exact SocketCAN receive/error filters, timestamps, short-write/overflow/interface-down/bus-off handling;
- latched FAULT and ESTOP states; no automatic clear/re-arm;
- repeated best-effort disable on fault, while acknowledging that CAN disable is not a physical E-stop;
- physical power E-stop as an external commissioning prerequisite;
- no zeroing, mode/config/flash writes in the runtime API.

The transition should be `CLOSED -> OPEN -> PROBED -> DISARMED -> ARMED`, with stale state/command, motor error, bus error, invalid command, or limit violation entering latched `FAULT`. Simulation may implement the same transitions, but enabling a virtual motor must never make a physical backend reachable implicitly.

### Proving simulation isolation

Use distinct constructors taking typed configurations, not a permissive interface-name string. The ROS node defaults to and explicitly constructs `oa_virtual_backend`; it should not link the SocketCAN backend in the simulation-only executable if practical. A separate physical executable/build option can link it later. Reject unknown backend types and never fall back from virtual to physical.

Tests must intercept `socket`, `ioctl`, `bind`, netlink, and `open` or use `strace`/seccomp to prove the simulation executable performs no `PF_CAN`/network-interface operations. Also test malicious configuration such as backend=`socketcan`, `can0`, absent config, and environment overrides. “Never opens CAN” is then an enforced property.

## ROS 2 node critique

Publishing `JointState` is sufficient for `robot_state_publisher` to publish TF. The proposed node should not independently publish the same link TF tree or it will create competing authorities. Publish:

- one coherent JointState message containing all 14 named arm joints with one timestamp;
- diagnostics including model ID, IK outcome/error, backend=`virtual`, and target/result sequence;
- optionally target/result markers, not duplicate robot TF.

Accept paired xyz targets through a service/action or a single stamped message with explicit reference frame, rather than independent left/right parameters/topics. Validate finite values, frame name, sequence/age, and both solve results before committing either simulated state. If one side fails, retain both previous states unless an explicit partial-update option is requested. RViz interaction is not motion authorization.

Lyrical support must be demonstrated by a clean Ubuntu 26.04/ROS 2 Lyrical source build. Upstream does not currently publish OpenArm into Lyrical rosdistro, so repository source compatibility cannot be assumed.

## Upstream reuse: legal and technical assessment

`openarm_can`, `openarm_description`, `openarm_ros2`, and `openarm_teleop` at the reviewed commits are Apache-2.0. Their relevant code/configuration may be reused or modified provided redistribution complies with Apache-2.0: ship the license, preserve applicable copyright/license notices, mark modified files, and propagate any NOTICE file if one is present. No NOTICE file was found in the reviewed roots, but release packaging should check again. Generated C model data derived from Apache-licensed xacro/YAML should retain provenance and license attribution. This is an engineering assessment, not legal advice.

Do not accidentally import CAD/hardware assets under the separate CERN hardware license when only software/model data is needed. Do not copy vendor-manual artwork or substantial text; implement protocol facts and retain documentation citations.

Technically, directly reusing current `openarm_can` defeats “pure C11”: it is C++, exposes classes/vectors/maps/exceptions, allocates dynamically, writes warnings to streams, silently clamps, and lacks required status/freshness behavior. A C ABI wrapper would hide, not remove, those properties. The smallest clean approach is:

- generate C kinematic constants from the canonical Apache-licensed flattened URDF;
- implement the small codec independently in C from the documented frame tables and motor ranges;
- use pinned upstream outputs as golden-test oracles, not runtime dependencies;
- reuse ROS package concepts/configuration selectively in the C++ adapter with attribution.

## Smallest corrected deliverable

Ship the first milestone as explicitly **simulation and diagnostics only**:

1. Generated, provenance-stamped bimanual left/right fixed models from body link to named hand-TCP.
2. Unambiguous FK result with pre-joint and post-link frames.
3. `oa_ik_position` with seed, weighted posture, adaptive DLS, active bounds, line search, explicit residual and failure reason.
4. Dual simulation bundle with paired stamped targets and all-or-nothing state update; no bimanual/collision-free claim.
5. Exact MIT plus feedback/status/enable-disable/query codecs, exhaustively tested but not connected by the ROS simulation executable.
6. Virtual backend and lifecycle; simulation binary cannot link/open SocketCAN.
7. Lyrical C++ node publishes JointState and lets `robot_state_publisher` own TF.
8. SocketCAN interface inspection and expected-manifest probe may exist behind a separate diagnostics-only executable, incapable of enable/control/config writes.

Defer physical arming/command transmission until the full feedback/watchdog lifecycle and a per-arm commissioned mapping exist. This is smaller and more production-quality than exposing a nominal physical backend whose coordinate and stop semantics remain unresolved.

## Required test gates

### Gate A: provenance/model

- Generator is deterministic; regenerated C is byte-identical in CI.
- Flattened canonical bimanual URDF is validated and archived with commit/hash.
- Every base transform, origin, axis, effective limit, joint/link name, and TCP transform matches the flattened URDF.
- C FK matches independent KDL/URDF FK at zero, each one-joint pose, limit endpoints, and thousands of random in-limit vectors.
- Translational and full geometric Jacobians match central finite differences, including left J7 reflection.

### Gate B: IK

- `FK(IK_position(FK(q).position, seed))` meets declared tolerance or returns non-success; approximate results never masquerade as success.
- Random, boundary, near-singular, rank-deficient, unreachable, and NaN/Inf cases; deterministic iteration cap and result reason.
- Active-limit tests prove no bound violations or clamp oscillation.
- Seed/posture sensitivity and path-continuity tests quantify orientation changes and joint jumps.
- Independent reference comparison against KDL/MoveIt numerical solutions where applicable; equality of q is not required, position residual and constraints are.

### Gate C: codec

- Golden bytes at min, zero, max, arbitrary interior values for all three v1.0 motor types.
- Round-trip error within half quantization step; endianness tests; malformed ID/flags/DLC/status tests.
- Every fault nibble 8–E and disabled/enabled states decoded and surfaced.
- Fuzz/sanitizer/static-analysis runs; no UB for all 2^8 byte values/DLC combinations sampled or exhaustively structured.

### Gate D: simulation isolation and ROS

- Simulation process cannot select or fall back to SocketCAN and performs zero PF_CAN/netlink/interface syscalls.
- Paired-target all-or-nothing semantics, timestamp/frame validation, one coherent 14-joint publication, no duplicate TF authority.
- Clean Ubuntu 26.04/Lyrical build and launch; complete TF tree and RViz RobotModel meshes for both arms.
- Simulation lifecycle/fault injection tests require explicit arm and never cross into a physical backend.

### Gate E: diagnostics-only SocketCAN

- `vcan` motor simulator tests expected-manifest probe with absent, duplicate, late, wrong-ID, wrong-status and extra frames.
- Exact receive/error filters, timestamps, interface-down, queue-overflow and bus-off reporting.
- Diagnostics executable has no enable, control, zero, mode-write, ID-write, bitrate-write, or flash code path.

### Gate F: before any future physical control release

- Commissioned sign/scale/zero/torque mapping and serial/config manifest for every joint/side.
- Measured motor TIMEOUT semantics and host watchdog/freshness fault tests.
- Supervised tests of command loss, process death, bus-off, motor fault, power E-stop, restart interlock, load drop, limits and thermal/current envelope.
- Measured bus utilization/latency margin and dual-bus skew.
- Path-level dynamic-limit and self/body/inter-arm/environment collision validation.

Until Gate F passes, the production claim must be limited to kinematics, simulation, codec correctness, and read-only diagnostics—not physical motion control.

