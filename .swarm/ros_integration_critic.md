# Independent ROS/control architecture and safety critique

Status: **DONE_WITH_CONCERNS**

Scope: read-only review of the current control C ABI, model/CAN/transport/commissioning boundaries, and `openarm_ik_ros`. No GUI, CAN interface, network, or hardware path was opened. The only workspace write is this report.

## Executive conclusion

The existing Stage-A control core is a much safer basis than the current ROS adapter: it has a deterministic virtual plant, coherent feedback generations, lifecycle gates, measured completion, watchdogs, plan/controller/epoch/scene binding, and a fail-closed physical backend. It should become the sole motion/state authority used by ROS and external programs.

The current ROS node must not be promoted into that role. It bypasses `libopenarm_control`, runs model IK directly, commits an instantaneous commanded pose, republishes it forever with a new timestamp, has no controller lifecycle or measured feedback, accepts implicit left/right ordering, and reports a successful unchecked solve as diagnostic `OK`. This is acceptable only as a clearly isolated visualization demo.

Before a unified bridge can be accepted, there are two release-blocking defects:

1. `openarm_model.h` and `openarm_control.h` cannot be included in the same C or C++ translation unit. The former declares signed `int32_t oa_status`; the latter declares unsigned `uint32_t oa_status`, and both define unscoped `OA_OK`/`OA_EINVAL`. I reproduced the compile failure in both include orders with `-std=c11 -Wall -Wextra -Werror`. A public umbrella/runtime ABI must use module-prefixed types and constants, or the existing headers must be corrected in a deliberately versioned ABI break.
2. No physical action may be exposed. `OA_BACKEND_PHYSICAL` correctly returns `OA_EUNSUPPORTED` during verification today, and public transport is query-only. Preserve that behavior. The commissioning library's “hardware qualified” fields are caller assertions rather than an authority chain; a ROS client must never be allowed to turn its next-action output into motor commands. Physical calibration, arming, and motion remain unavailable until independently commissioned identities, authorization, interlocks, watchdogs, and a reviewed collision implementation exist.

## Evidence and adversarial findings

### Lifecycle and authority

- The ROS node owns only `PairedTransactionProcessor`; it never constructs a manifest/controller or calls verify, arm, plan, execute, advance, stop, disarm, snapshot, event, or commissioning APIs (`openarm_ik_ros_node.cpp:26-177`). Thus ROS “committed” does not mean controller-accepted, executing, or measured complete.
- The processor updates both joint arrays atomically only after both IK calls succeed (`paired_transaction.cpp:66-90`), which is good for the narrow visualization transaction, but it jumps immediately to the terminal posture. It has no trajectory, velocity/acceleration/jerk enforcement, execution deadline, cancel, watchdog, settling, or measured convergence.
- `Controller` is the appropriate OOP authority. It serializes operations behind opaque C handles and binds plans to controller instance, verify epoch, manifest/model/scene revisions, both feedback sequences, and measured start pose (`control_core.cpp:759-800`). Preserve that single-authority pattern.
- `deadman_active_` currently defaults to `true` (`control_core.hpp:215`). The physical backend is hard-gated, so this is not an immediate hardware hazard, but it is an unsafe default for any future physical implementation. Physical-capable code must default all enabling interlocks false and require a fresh external interlock sample.
- ROS node lifecycle and controller lifecycle are distinct. A ROS node being `active` must not imply an armed controller. Controller lifecycle must be published explicitly and remain authoritative.

### Units, timestamps, and frames

- Model coordinates are metres/radians and target XYZ is explicitly in `openarm_body_link0` (`openarm_model.h:20-22,53-55`). The ROS adapter accepts `world` and passes the numbers directly into body-frame IK (`paired_transaction.cpp:40-43,68-73`). This works only because the generated URDF currently fixes `world -> openarm_body_link0` at identity. A future non-identity mount silently makes the result wrong.
- `oa_paired_tcp_move.left_tcp_m/right_tcp_m` is unit-labelled but not frame-labelled. Document body frame in V1 immediately and add an explicit frame enum/ID to the next ABI. ROS should transform an input at its source stamp into the canonical body frame, then reject missing, extrapolated, or stale TF; never reinterpret `world` coordinates as body coordinates.
- The control ABI mixes two clock domains without a public clock contract: controller `advance`, command expiry, challenge expiry, and feedback timestamps use caller-driven controller monotonic time, while `poll_event` deadlines are host `std::chrono::steady_clock` absolute time. Names like `expiry_ns` do not identify the domain. Add explicit clock IDs and clock-specific field names in the next ABI.
- ROS request validation uses ROS time and permits a request up to one second in the future, but executes it immediately (`paired_transaction.cpp:52-60`). Either future stamps are execution times and must be scheduled, or future stamps must be rejected. The current middle ground is unsafe.
- `/joint_states` is stamped with publish time, not feedback time, and the 10 Hz timer continually republishes the last commanded posture with a fresh stamp (`openarm_ik_ros_node.cpp:116-130`). This makes stale or disconnected state appear fresh. Publish only measured snapshots, use measurement time after an explicit monotonic-to-ROS time mapping, and publish age/fresh masks separately. Never refresh a measurement timestamp merely by republishing it.
- Handle `/use_sim_time`, time jumps, overflow, zero time, and clock reset explicitly. ROS command freshness belongs at ingress; controller deadlines stay monotonic and are derived from a sampled, bounded conversion—not by treating ROS epoch nanoseconds as controller monotonic nanoseconds.

### Joint identity, side mapping, and gripper semantics

- The model and manifest use canonical `openarm_{left,right}_joint1..7`; manifest validation also fixes side/index/name and rejects duplicate bus IDs and serials (`control_core.cpp:83-145`). This strong mapping should generate all ROS names and descriptors; do not maintain a second hand-written map.
- `PoseArray` encodes left/right only by index 0/1. Reversed order is syntactically valid and silently swaps arms. Replace it with a named paired command containing explicit `left` and `right` fields, plus a transaction/goal ID.
- The current `JointState` appends two finger joints at `0.0` (`openarm_ik_ros_node.cpp:128-129`). These joints are prismatic in metres with `[0, 0.044]` limits and mimic counterparts; the control snapshot contains only 7 arm joints. The zero values are fabricated visualization state, not measured gripper state.
- Add explicit gripper descriptors and state: aperture/actuator position in metres, validity/source (`unknown`, `virtual`, `measured`), sequence, and timestamp. Publish the single actuated `finger_joint1` per side only when a value is semantically available. If RViz needs a virtual default, expose a configured virtual-gripper value and mark it `virtual`; never blend it into measured arm feedback without a source flag. Mimic joint2 remains robot-state-publisher's responsibility.

### Stale feedback, concurrency, and plan invalidation

- The control core correctly requires complete fresh generations and cross-bus coherence, but snapshot freshness ages only when controller time advances. The bridge must have one monotonic control loop that advances independently of ROS traffic. A stalled loop must fault; a state publisher must not substitute its own wall/ROS time.
- C calls on one controller are mutex-serialized, but semantic command concurrency is still first-lock-wins. Two clients can plan from the same seed; the winner executes and the loser gets `OA_ESTATE`/`OA_ESTALE` nondeterministically. Introduce a bridge-level command arbiter with exactly one active goal and an explicit reject-or-preempt policy. Planning does not reserve execution.
- The present processor is not internally synchronized. It is incidentally safe under the current single-threaded `rclcpp::spin`, but a multi-threaded executor or reentrant callback group would race `sequence_`, `left_q_`, and `right_q_`. Do not depend on executor choice for correctness.
- Every accepted action result must include goal/command ID, plan seed sequences, revisions, lifecycle, final status/cause, and measured terminal sequence. Diagnostics are not acknowledgements. The helper currently accepts the first matching diagnostic without correlating it to its request, so a queued old diagnostic can be mistaken for success.
- Configuration replacement, re-verification/reset, collision-scene change, feedback-sequence change, start-pose drift, calibration commit, or backend replacement must invalidate all outstanding plans. Existing control checks cover most execution-time cases; the bridge must also terminate/reject affected ROS goals and publish the invalidation reason.
- Manual calibration requires an exclusive controller/configuration lease and a disarmed controller. A calibration session created independently through `openarm_commission` must not race motion or configuration. Simulation evidence must remain permanently distinguishable from hardware-qualified evidence and must never promote a physical manifest.

### Collision and diagnostics policy

- The default control policy rejects all planning; only explicit virtual unchecked mode permits plans, and reports retain `collision_checked == 0`. This is the right fail-closed baseline.
- A collision scene revision does not mean collision checking happened. The current virtual unchecked plan binds an arbitrary nonzero revision while setting `collision_checked=false`. Diagnostics and action results must keep those concepts separate.
- Successful unchecked IK currently produces diagnostic level `OK` (`openarm_ik_ros_node.cpp:136-157`). It should be `WARN` at minimum, with stable machine-readable fields for `backend=virtual`, `motion_authorized=false`, `collision_policy=unchecked`, `collision_checked=false`, `orientation_constrained=false`, and capability bits.
- For a safe simulator, offer two modes: `preview` may solve/publish an unchecked ghost/plan but is never represented as authorized execution; `virtual_execute` may advance the deterministic plant only after an explicit opt-in to unchecked collision. Any physical mode without a collision checker stays `REJECT_ALL`.
- Publish periodic health, not command-only diagnostics: controller lifecycle, backend, capabilities, manifest/model/scene revisions, each arm's expected/fresh/fault masks, feedback sequence/time/age, cross-arm skew, active command ID, last event/cause, calibration state/evidence kind, and collision state. Keep human text secondary to enums/fields.

### Executor and shutdown

- Current `main` calls `rclcpp::shutdown()` only after normal `spin` return. Constructor or callback exceptions bypass it. Use RAII/context guards and catch/report exceptions.
- The bridge should own a bounded control-loop thread and a bounded event-poll thread, or a single worker that performs both. Shutdown order must be: reject new work; cancel ROS goals/timers/subscriptions; request stop/disarm for virtual controller; stop and join workers; destroy plans/sessions/controller/manifest; then shut down the ROS context. No callback may retain a destroyed handle.
- Never block an executor callback indefinitely in `oa_controller_poll_event`; use finite monotonic deadlines and a stop token. Controller destruction does wake a polling call after marking the slot closing, which is useful, but shutdown should not rely on destruction as ordinary cancellation.
- Use mutually exclusive callback groups or post all mutations to the single owner worker. Publishers may consume immutable copied snapshots. Define cancellation/preemption at one serialization point.

### C ABI issues

- **Header collision (blocking):** public `oa_status`, `OA_OK`, and `OA_EINVAL` collide between model and control. Use `oa_model_status`/`OA_MODEL_*` and `oa_control_status`/`OA_CONTROL_*` in a new version, or expose a new collision-free umbrella runtime header that consumers need not combine with legacy headers.
- Prefix compatibility exists only for three selected V1 records. Other V1 inputs, including nested manifest records, require current `sizeof`, so appending fields there would break old binaries. Freeze V1 layouts; add V2 function/record names instead of assuming all `struct_size` records are append-compatible.
- Freeze and test `sizeof`, `_Alignof`, and `offsetof` on supported architectures. Fixed-width scalars help, but C padding/layout and compiler/architecture variation are still ABI facts.
- Integer-derived fake pointer tokens are never dereferenced and are registry-validated, but integer-to-pointer conversion is implementation-defined and is unsuitable for capability-pointer architectures. A next ABI should use allocated opaque objects with generation-protected registry membership, or integer handle types explicitly declared as such.
- Add a runtime ABI/capability query. `backend` alone is insufficient to distinguish discovery-only, simulated calibration, virtual motion, collision checking, and physical authorization.
- Specify output-on-error behavior for every function. Today some outputs remain untouched, while `oa_controller_open_and_verify` writes a report even on error. Both patterns can be valid, but callers need a uniform contract.

## Recommended modular design

Keep the C++ implementations private and make one collision-free C ABI the integration seam:

```text
ROS actions/services/topics       external C / C++ programs
             \                         /
              openarm_runtime C ABI (versioned, capability queried)
                              |
        RuntimeCoordinator (single authority / command arbiter)
        |          |             |              |
  ConfigStore  Controller    Calibration     Discovery
  immutable    + Planner     Coordinator     Coordinator
                  |             |              |
          IArmBackend      commission C ABI  query-only CAN/transport
          |        |
     VirtualPlant  PhysicalBackendStub (no arm/calibration/motion capability)
```

Suggested private C++ interfaces are `IClock`, `IArmBackend`, `ICollisionChecker`, `IIdentityVerifier`, and `IEventSink`. Concrete Stage-A implementations are `ManualMonotonicClock`, `VirtualPlantBackend`, `RejectAllCollisionChecker`, and `QueryOnlyDiscovery`. Keep `PhysicalBackendStub` capability-empty and fail closed. The `RuntimeCoordinator` owns manifests, controller, calibration sessions, active plan/goal, and all state transitions; lower layers cannot grant authority.

The runtime C ABI should expose opaque `oa_runtime`, `oa_runtime_plan`, and `oa_runtime_calibration` handles plus records with module-prefixed status/enums. Minimum operations:

- ABI/capabilities and backend identity;
- enumerate virtual arms and optionally query-only host CAN interfaces, explicitly reporting “interface observed” rather than “arm identified”;
- get immutable configuration/joint/gripper/frame descriptors and revisions;
- get measured snapshot/kinematics with clock ID, sequence, time, freshness, source, and units fixed by the record type;
- get lifecycle/diagnostic/calibration reports;
- begin/step/abort simulation-only manual calibration, with no hardware action executor;
- plan/execute/cancel virtual joint and paired-TCP actions, with immutable reports and explicit goal ownership;
- advance virtual time or run a runtime-owned loop, but never both concurrently.

Build a small public C++ RAII wrapper *on top of* this C ABI for external C++ callers. ROS should use that same wrapper/ABI rather than linking private controller/model classes, ensuring ROS and compiled clients see identical lifecycle and safety behavior.

Recommended ROS surface:

- a lifecycle node, inactive by default, whose activation starts only the virtual loop;
- `GetCapabilities`, `ListArms`, and `GetConfiguration` services;
- periodic custom `RuntimeState`, `ArmState`, `CalibrationState`, and standard `DiagnosticArray` topics;
- measured `/joint_states`, with one authoritative configurable publisher and measurement stamps;
- `MoveJoint` and `MovePairedTcp` actions (not raw command topics), named left/right fields, explicit source frame, goal UUID/correlation, deadlines, feedback, cancel, and terminal measured status;
- a simulation-only calibration action/service surface. Do not register physical calibration or physical motion servers at all when capabilities are absent.

Avoid `PoseArray` for position-only input; a custom message with two `PointStamped`-equivalent named targets expresses the actual contract and avoids invalid/ignored quaternions. Transform both targets transactionally at one timestamp into `openarm_body_link0` before planning.

## Exact acceptance tests

All tests below are release gates; hardware-free CI must perform no network/CAN access.

1. **C/C++ header coexistence:** compile C11 and C++17 consumers including runtime, model, control, commission, CAN, and transport public headers in every include order with `-Wall -Wextra -Wpedantic -Werror`; no typedef/macro/symbol collision. Also link a consumer against installed packages only.
2. **ABI freeze:** assert `sizeof`, alignment, and every public field offset against checked-in x86_64 and aarch64 manifests; compile/run old frozen consumers against the new shared libraries; V1 records never grow except where existing prefix behavior is already guaranteed.
3. **Output contract/fuzz:** for every C function, test null, wrong type handle, stale/double-destroyed handle, wrong ABI, every truncated size, oversized size, NaN/Inf, boundary integer, concurrent destroy, and allocation failure. Verify documented output unchanged/diagnostic-write behavior under ASan/UBSan and registry races under TSan.
4. **No physical capability:** with default build and every ROS parameter combination, capability bits for physical arm, calibration action, and motion are zero; physical servers are absent; `OA_BACKEND_PHYSICAL` verification returns unsupported; binaries have no hidden write-authority symbol/path.
5. **No CAN linkage in simulation:** inspect the ROS node's dynamic dependencies/symbols and syscall trace a complete launch/command/shutdown. It must not open PF_CAN/netlink/network sockets, execute shell tools, or change interfaces. Query-only discovery is a separate explicitly enabled process/test profile.
6. **Query-only transport:** exhaustive frame-class tests prove public transport sends only correlated register/status queries and rejects motion, enable, disable, write, zero, clear, save, malformed, and unknown frames. Fuzz all 8-byte payloads and assert no forbidden class reaches the backend.
7. **Identity claims:** discovery of a CAN interface or feedback ID must report `UNCOMMISSIONED/UNIDENTIFIED`, never side/joint/serial identity. Supplying only caller-written serial/revision strings cannot enable physical calibration or motion.
8. **Lifecycle table:** exhaustively test every runtime/controller state × operation. Closed/unconfigured cannot snapshot/plan; disarmed cannot execute; preview cannot authorize; executing rejects a second goal; fault/estop cannot arm; reset returns closed and forces re-verification; ROS active never changes controller arming by itself.
9. **Interlock default:** immediately after creation/reverification, deadman is false for any physical-capable build. Arming without a fresh explicit interlock sample fails. Expiry, E-stop, and deadman release cancel the active goal and produce the documented disabled fallback.
10. **Clock-domain boundaries:** test zero, exact expiry, expiry+1 ns, maximum horizon, overflow, future stamps, `/use_sim_time`, backward/forward ROS jumps, paused ROS time, and monotonic reset attempts. A future ROS stamp is either rejected or scheduled—never executed early. `poll_event` deadlines remain host-monotonic and are clearly distinct.
11. **Frame transform:** use a non-identity and time-varying `world -> openarm_body_link0`; command stamped targets in `world`; verify the controller receives the correctly transformed body-frame metres at the request timestamp. Missing/stale/extrapolated TF rejects the entire pair without state change.
12. **Side/name mapping:** command asymmetric reachable targets and prove named left affects only all seven `openarm_left_*` joints and named right only right. Swap serialized field order and prove semantics do not change. Reject any manifest/name/index mismatch.
13. **Gripper semantics:** without gripper state, no value is labelled measured. With virtual aperture values at 0, 0.044, and boundaries±epsilon, verify metres, limits, side mapping, source/validity, mimic TF behavior, and rejection outside limits. Arm feedback sequence must not imply gripper freshness.
14. **Measured-state timestamp:** freeze/drop virtual feedback while publishers continue. `/joint_states` measurement stamp must not advance; state/diagnostics age must increase; freshness becomes false; plans/actions reject/fault. Republishing alone cannot restore freshness.
15. **Feedback coherence:** independently freeze/drop each of 14 joints, inject arm timestamp skew at threshold and threshold+1 ns, duplicate/out-of-order sequences, and partial paired command cycles. No partial generation is published as fresh and both-arm motion faults to the documented safe state.
16. **Ingress atomicity:** malformed left, malformed right, one unreachable side, nonfinite XYZ, stale stamp, invalid frame, and failed TF must leave both arm states, active plan, and revisions unchanged.
17. **Command concurrency:** race N clients submitting joint and paired goals at barriers under single- and multi-threaded executors. Exactly one policy outcome is deterministic: one accepted and all others explicitly busy, or documented preemption with the old goal terminal before the new starts. No data race, mixed pair, lost terminal result, or ambiguous diagnostic acknowledgement.
18. **Correlation:** preload stale diagnostics/results, then send a goal. Client accepts only matching UUID/command ID. Sequence wrap policy is tested. Every accepted goal gets exactly one terminal result across success, reject, cancel, fault, shutdown, and preemption.
19. **Plan invalidation matrix:** after planning, independently change feedback sequence, start q by 1e-3 boundary and ±epsilon, manifest/model/scene revision, verification epoch, configuration, calibration revision, controller instance, and deadline. Execute must return the exact documented stale/identity code and never alter plant state.
20. **Collision gate:** default `REJECT_ALL` rejects joint and paired execution. Virtual unchecked requires explicit opt-in, returns `collision_checked=false`, diagnostic WARN, and `motion_authorized=false` for preview. Scene revision changes invalidate plans but never turn unchecked into checked. A physical backend without `ICollisionChecker` cannot plan.
21. **Limits and trajectory:** for every joint and both sides test lower/upper exact and ±epsilon, raw PMAX, velocity/acceleration/jerk scales, branch-step and singular-value boundaries. Sample each executed trajectory densely and verify q/dq/acceleration/jerk limits plus synchronized paired timing and measured three-cycle settling.
22. **Calibration isolation:** calibration creation fails unless disarmed and exclusively leased. Motion/configuration calls are busy during a session. Abort from every state is terminal. Simulation evidence cannot be committed as hardware qualified, and arbitrary ROS-provided qualification strings/revisions cannot unlock a physical action.
23. **Diagnostics truth table:** for every lifecycle/fault/collision/freshness/calibration state, assert enum, severity, capability bits, revisions, masks, age/skew, active command, source, and cause. Successful unchecked IK is WARN, stale feedback is at least WARN/ERROR, and no absent achieved pose is serialized as zeros.
24. **Shutdown under load:** send SIGINT/SIGTERM and ROS context shutdown during planning, execution, blocking event poll, diagnostics publication, calibration step, and concurrent goal arrival. Within a fixed timeout the node rejects new work, returns terminal goal results where possible, stops/disarms virtual state, joins all threads, destroys each handle once, and exits with no use-after-free or stuck executor.
25. **Executor equivalence:** run the full bridge suite with single-threaded and multi-threaded executors and randomized callback scheduling. Observable state transitions/results must be identical under the declared arbitration policy.
26. **Long soak/fault campaign:** deterministic virtual execution for at least 10^6 cycles with randomized commands, cancellations, stale feedback, skew, faults, clock boundaries, and plan invalidations; assert bounded memory/event queues, no stale state stamped fresh, no false completion, and reproducible results from a seed.

## Acceptance disposition

Safe simulation integration is feasible and should proceed through a collision-free runtime C ABI backed by the existing controller core, with ROS as a thin lifecycle/action/state adapter. Do not merge a “unified” ROS control interface that still calls model IK directly, uses `PoseArray` as an action surrogate, republishes commands as fresh measurements, or exposes any physical calibration/motion endpoint. The header collision, clock/frame contract, measured-state publication, command arbitration, and shutdown gates above should be resolved first.
