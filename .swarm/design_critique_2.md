# Independent architecture critique

The proposal has the right fail-closed direction, but it combines a useful model/visualization MVP with a premature physical-control surface. The largest defect is not C11 or DLS; it is that a bare Cartesian `xyz` target is radically under-specified while the dual-arm and hardware APIs make it look motion-ready.

## Findings ranked by importance

### 1. Position-only IK is too under-constrained for the advertised control surface

A 3x7 translational Jacobian leaves four local null-space dimensions. Joint-limit and posture bias choose one solution, but do not define end-effector orientation, wrist roll, elbow branch, continuity, collision behavior, or a shared-object relation between arms. Small target/seed changes can cause large posture changes; independent per-arm solves can collide with the body, the other arm, or each other. Upstream's KDL configuration is evidence for numerical IK, not evidence that translation-only solutions are safe commands.

Mandatory correction:

- Name the function and ROS command explicitly **position IK**, never generic IK.
- Require target frame, selected tip, seed/measured posture, desired posture policy, position tolerance, step/velocity bounds, iteration cap and deadline.
- Return residual, iteration count, active-limit mask, singularity metric, convergence reason and achieved joint vector. “Best effort” is not success.
- Enforce box limits with a projected/trust-region or bounded least-squares method. A null-space limit gradient alone does not guarantee feasibility.
- Use task-priority projection or a constrained objective so posture bias cannot trade away the primary position tolerance silently.
- Add an optional orientation task/mask now, or hold the seed orientation as an explicit secondary objective. A later full-pose API must not require an ABI break.
- Keep collision checking outside basic IK, but make the result type state clearly that it is **not collision checked**.

For dual-arm manipulation, two independent `xyz` targets are not a bimanual constraint. Shared-object pose, relative-hand transform and inter-arm collision belong to a later coordinator/planner.

### 2. “FK reports every joint origin” is an insufficient and ambiguous model API

Positions alone cannot reconstruct orientations, transform joint axes, form a geometric Jacobian, or publish a complete TF tree. “Joint origin” is also ambiguous between the fixed URDF joint frame before motion and the moving child/link frame after applying `q`.

Return named rigid transforms for base, each joint frame, each link and selected tool, plus the world/base-frame joint axes used by the Jacobian. Define multiplication order, matrix storage, quaternion order, SI units and frame convention in the public header. Prefer a frame-query API and a fixed-size snapshot over an undocumented positional array.

The current-main audit reports `link7 -> hand` 0.1025 m and `hand -> hand_tcp` 0.0835 m, but cross-pin equality was not established. Therefore `hand_tcp` must be extracted from the exact pinned generated URDF and covered by a provenance/golden test; it must not be hand-coded from the audit. There is no upstream `claw` or `tool0` frame.

### 3. The pin strategy is not yet coherent

The reports identify stable release commits and newer main commits, but no vendor lockfile proves that independently released description/CAN/ROS tags are one compatible bundle. The exact axes and effective limits were independently checked at `openarm_description@6c7b720...`, whereas the source report recommends release `5db5232...` for reproducibility.

Choose one immutable manifest and record:

- repository commit;
- xacro arguments and selected side/configuration;
- xacro/tool versions;
- generated URDF hash;
- extracted model-data hash;
- effective joint names, origins, axes, limits and tip transforms.

Do not claim current-main geometry for a release-generated library until a golden comparison proves equality. Archived v1 paths are stale, and the generic single-arm xacro is not frame-equivalent to selecting one side of the bimanual model.

### 4. A first-class dual-arm API is larger than necessary

Both arms are instances of one seven-joint model with side-specific transforms/axes/limits. A large dual-arm object risks duplicating solver/control logic and conflating independent CAN buses with coordinated planning.

Use a small `oa_arm_model`/`oa_arm` object parameterized by a generated side manifest. Compose two instances in an optional `oa_bimanual_scene` that owns only body/world transforms, snapshot timing and coordination policy. One bus worker remains the sole owner of one physical interface; standard-configured arms reuse IDs 1–8 and must not share a bus.

### 5. “Exact DaMiao codec” must not mean copying current upstream behavior

Current `openarm_can` is useful protocol evidence, but the reports show that it discards feedback byte 0 status/fault, has no freshness state, and clamps MIT values only at broad protocol bounds. An exact production codec must preserve all bits and reject malformed or non-finite input; it must not reproduce those omissions.

Separate:

- pure frame encode/decode;
- commissioned motor ranges and joint mapping;
- mechanical/operational safety limits;
- SocketCAN transport and timing.

Encoding outside protocol range should return explicit saturation/error policy. Decoding must validate SFF/EFF/RTR/error flags, arbitration ID, DLC, embedded motor ID, status nibble, temperatures and plausible numeric range. Mechanical limits remain far inside protocol ranges.

### 6. A read-only probe cannot discover or prove the arm

It can verify known IDs/registers while torque remains disabled. It cannot establish physical joint assignment, arbitrary Master IDs, duplicate nodes, side, polarity, zero, gear/output convention, TCP, or safe limits. Call it `verify_manifest`, not discovery or commissioning.

Commissioning should be a separate executable/build and persistent manifest process, not a boolean method on the runtime library. It must record serial, model/variant, hardware/firmware/sub-version, send/master IDs, direction, gear, P/V/T ranges, mode, bitrate, TIMEOUT, zero, scale and side association. Runtime rejects missing, extra, duplicate or mismatched feedback.

The J3 physical BOM is DM-J4340P while runtime uses the generic `DM4340` enum; this is acceptable only after commissioning confirms the protocol parameters. The runtime enum is not a procurement identity.

### 7. `commission + arm` is necessary but not a safety architecture

Explicit arming does not replace:

1. an independent physical power E-stop/contactor and assessed payload-drop behavior;
2. a configured and measured motor TIMEOUT;
3. a host monotonic watchdog for stale command/state, deadline miss, invalid telemetry, bus faults, temperature/electrical faults and limit violations;
4. a latched lifecycle with explicit reset and no automatic return-to-zero;
5. commissioned speed, torque, acceleration and jerk bounds;
6. measured bus utilization, latency, loss and dual-bus skew.

The physical backend should be excluded at build time by default, require an explicit backend selection plus an immutable commissioned manifest at runtime, and still begin torque-disabled. Never let construction, ROS lifecycle activation or process startup enable motors. Parameter writes, zeroing, hard-stop calibration, bitrate/ID changes and firmware flashing do not belong in the runtime API.

### 8. Virtual simulation needs two distinct backends

A kinematic virtual arm is useful for FK/IK/RViz but cannot test CAN safety. Add a `vcan` motor simulator implementing IDs/registers, enable/disable, command-triggered feedback, TIMEOUT and every feedback fault. Otherwise the physical-control path remains untested even though “simulation default” appears reassuring.

Neither virtual backend proves dynamics, payload, collision clearance or timing on the real USB-CAN-FD adapter. Make backend identity unmissable in diagnostics and refuse to open a real interface unless the physical build/runtime gates are satisfied.

### 9. The ROS target contract is unsafe and underspecified

A subscriber that “accepts per-arm xyz targets” omits frame, timestamp, expiry, tip, seed/posture policy, tolerances, dry-run/execute intent and result. ROS may replay or retain stale data; a callback must never command hardware directly.

Use a service/action or custom stamped command containing:

- `Header`/frame ID and monotonic expiry;
- arm/side and explicit tip frame;
- position plus optional orientation/task mask;
- posture seed/reference and constraints;
- plan-only versus execute authority;
- structured result with residual/flags.

The C core remains authoritative for lifecycle, freshness and limits. ROS only transforms/validates requests and enqueues expiring commands outside the bus thread. Publish `JointState` only from fresh mapped snapshots with exact generated names and timestamps; never publish initialized zeros as measured state. Publish diagnostics and arm/backend state. `robot_state_publisher` receives the exact pinned URDF.

OpenArm upstream recommends Humble and has no evidenced Lyrical support guarantee. Lyrical must be a source-build/CI target, not a claim of upstream compatibility. Visualization via JointState -> robot_state_publisher -> RViz is the lowest-risk first deliverable; MoveIt and ros2_control come later.

## Smaller, better staged design

### Stage A: model-only deliverable

- A deterministic generator emits immutable **data**, not bespoke FK source: side/configuration manifest, frame graph, transforms, axes, effective limits and provenance hash.
- Pure C11 `oa_model` provides named-frame FK, geometric Jacobian and validation.
- `oa_ik_position` provides bounded numerical position IK with explicit orientation/posture policy and rich diagnostics; a masked SE(3) solver may share the implementation.
- Two model instances plus a thin bimanual scene provide visualization transforms only.
- Lyrical adapter publishes JointState/TF and plan-only IK results; RViz is the default UI.

This meets the useful IK/RViz goal without any motor dependency.

### Stage B: protocol laboratory

- `oa_can_codec` is pure and fuzzable.
- `oa_bus` owns SocketCAN deterministically, one worker per interface.
- `oa_vcan_motor` simulates registers, feedback, timeout and faults.
- `oa_device` verifies an explicit manifest while torque-disabled.
- No physical backend or enable API ships yet.

### Stage C: safety-controlled arm

- Add `oa_arm` lifecycle, mapping, limits, watchdog and explicit arm token only after the commissioning manifest and physical E-stop procedure exist.
- Compose two arms with a thin coordinator that timestamps snapshots, measures skew and defines both-arm stop policy.
- Add the ROS hardware adapter last; do not reuse upstream auto-enable/return-to-zero semantics.

## Mandatory tests before merge

### Provenance/model

- Deterministic generation and hash test for pinned repo, xacro arguments and output.
- Assert bimanual joint/link names, body/base transforms, all origins/axes, effective left/right J1/J2 limits, mirrored J7 and selected `hand_tcp` chain.
- Reject wrong side, single-vs-bimanual mix and unknown frame/tip.
- Compare every named transform and the 6x7 Jacobian against KDL/another URDF implementation at zero, axis-only and randomized legal poses.
- Finite-difference every translational and rotational Jacobian column.

### Position IK

- Random legal `q -> FK position -> IK -> FK` property tests for both sides.
- Nearby/remote seeds, all limit faces, asymmetric J1/J2 limits, singular/near-singular poses, unreachable points, wrap-adjacent states and deterministic iteration/deadline exhaustion.
- Verify no returned success violates limits or tolerance; verify posture bias cannot degrade the primary task unnoticed.
- Cartesian path continuity tests bound per-step joint change and detect branch flips.
- Mirror-consistency tests for corresponding left/right targets.
- Explicit test showing IK success does not imply collision-free status.

### API/ABI

- C and C++ consumers, fixed-width layout, `struct_size`/`abi_version`, null/size/alignment errors, SI/frame/matrix conventions and no hidden allocation in step functions.
- Two arm instances must not share mutable model state; a dual scene must not imply atomic dual-bus execution.
- Every solver/control failure has a closed status; no library printing or silent saturation.

### Codec/virtual bus

- Independent golden vectors for enable/disable/zero/query/write/refresh/MIT and feedback, including endpoints, midpoint, quantization and endianness.
- NaN/Inf, overflow, wrong ID/DLC/flags, malformed embedded ID and all status/fault nibbles.
- Fuzz all decoders under ASan/UBSan and strict warnings.
- `vcan` tests for missing/late/duplicate/reordered frames, duplicate IDs, unknown motors, queue overflow, interface down, bus-off, timeout and fault recovery.
- Deterministic lifecycle tests prove no auto-arm, stale command/state latch fault, repeated best-effort disable, explicit reset and both-arm stop policy.

### ROS 2 Lyrical

- Clean pinned Lyrical source build; do not infer compatibility from Humble.
- Launch hardware-free and assert robot description, exact JointState names, complete TF trees, unique side names, mesh resolution and backend diagnostics.
- Reject unknown/stale/expired targets, invalid frame/tip, unreachable IK and transform lookup failure.
- Verify ROS callback/executor never owns the bus and cannot bypass C lifecycle/limits.
- Test plan-only default and require separate execute authority.

### Future physical acceptance

No physical test is part of ordinary CI. Separately authorized acceptance must progress isolated unloaded motor -> constrained joint -> reduced-limit single arm -> dual arm, with fixture, exclusion zone and independent E-stop observer. It must measure mapping, endpoints, watchdog loss, bus-off/process kill, E-stop interruption, payload drop, thermal/current behavior and restart interlocks.

## Verdict

Approve the pinned-model, C11 FK/Jacobian, bounded position-IK and hardware-free Lyrical/RViz portion after the corrections above. Defer the dual-arm physical API and real SocketCAN backend until a manifest verifier, fault-complete codec, `vcan` simulator, lifecycle/watchdogs and external E-stop gates exist. The smallest credible first release is a model/IK library plus plan-only RViz node—not a Cartesian robot controller.
