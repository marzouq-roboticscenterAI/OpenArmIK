# Independent C-ABI safety critique

Date: 2026-07-29  
Scope: static, adversarial, read-only review of the separate CAN, transport,
model, commissioning, and control C ABIs. No CAN interface, network, GUI, or
hardware was accessed. Apart from this report, no file was changed.

## Bottom line

The current ABIs cannot honestly be presented as one API that auto-detects,
configures, calibrates, queries, solves, and moves a real OpenArm.

They can honestly be presented as five deliberately incomplete building blocks:

1. a C model that performs FK and independent position-only IK for either arm;
2. a CAN codec that can construct both diagnostic and dangerous frames but has
   no physical transport;
3. a public SocketCAN transport that permits only register/status queries;
4. caller-driven manual and supervised calibration calculations that neither
   acquire identity-qualified samples nor persist a manifest; and
5. a virtual controller with joint and paired-XYZ motion, while its physical
   backend fails closed with `OA_CONTROL_EUNSUPPORTED`.

That separation is currently a safety virtue. It is not an end-to-end hardware
capability. In particular, successful return from `oa_can_probe_expected` is not
motor identity, successful commissioning is not authenticated qualification,
and `collision_scene_revision != 0` is not collision checking.

Physical configuration, calibration motion, and ordinary motion must remain
unavailable. The present tree cannot pass a production physical-motion
acceptance because it has no physical controller backend and no collision
engine.

## Honest capability matrix

| Requested capability | What exists | Honest result |
|---|---|---|
| Interface auto-detection | `oa_can_linux_list_interfaces` lists Linux CAN link attributes without opening CAN. | Partial enumeration only. It does not establish adapter identity, arm side, topology, or motor identity. |
| Motor auto-detection | `oa_can_probe_expected` sends refresh queries to caller-supplied expected IDs. | No. It confirms fresh-looking disabled feedback at expected arbitration/embedded IDs only. It starts from a manifest and cannot discover physical joint assignment, serial, motor family, sign, zero, gearing, firmware, or safe mapping. |
| Configuration | Typed register query/write codecs and special-frame builders exist. Public SocketCAN sends only queries. | No end-to-end configuration. The Linux public path correctly refuses writes, set-zero, save, enable, disable, and motion. There is no configure/read-back/rollback/persist transaction. |
| Manual encoder calibration | `oa_commission_manual_*` computes `q_model = a*q_output + b` from caller-supplied disabled samples. | Algorithmic building block only. Samples and fixture angles are not bound to a live interface, CAN identity, serial, side, joint, clock, or signed fixture record; the patch is not applied or persisted. |
| Automatic encoder calibration | `oa_commission_recipe_*` is a supervised hard-stop state machine that emits bounded next actions. | Not automatic hardware calibration. It emits requests, not commands. Hardware qualification, interlocks, posture, samples, and evidence revisions are caller assertions. No public component executes the actions. |
| Joint/TCP coordinates | `oa_fk`; virtual `oa_controller_snapshot` and `oa_controller_get_kinematics`. | Yes for supplied model coordinates and coherent virtual feedback. No live-hardware query. The model frame is `openarm_body_link0`; the control request/result records do not carry an explicit frame identifier. |
| Individual XYZ IK | Call `oa_ik_position_v2` on the left or right immutable model. | Yes, position only, in body coordinates, with free orientation and no collision checking. |
| Paired XYZ IK | A caller may solve both models; the virtual controller solves both during `oa_controller_plan_paired_tcp`. | Partial. The model ABI has no atomic paired request/result. The controller provides atomic paired planning only for the virtual backend and only with the explicitly unchecked virtual collision policy. |
| Individual XYZ motion | No single-arm TCP planning function exists. `oa_controller_plan_joint` moves one joint, not one TCP. | No. |
| Paired XYZ motion | Virtual `oa_controller_plan_paired_tcp` plus execute/advance. | Virtual-only, position-only, and collision-unchecked. Physical open/verify returns unsupported. |
| Gripper coordinates/calibration/motion | Commissioning reserves joint index 7 and supports a gripper recipe. Model/control cover seven arm joints. | Calibration-patch calculation only. No gripper state, kinematic mapping, endpoint/force model, planning, or motion exists in control. |

## Properties that should be preserved

- Model inputs/results are explicitly metres/radians and model-joint
  coordinates. IK reports that collision was not checked and returns achieved
  orientation rather than pretending it was constrained.
- The public transport has no authority-issuance entry point. Linux SocketCAN
  categorically refuses private authority issuance, and caller-opened handles
  reject motion and all state-changing frame classes.
- `oa_can_probe_expected` requires disabled feedback and rejects observed
  duplicate expected replies, enabled replies, faults, stale timestamps, and an
  exhausted receive budget.
- Control manifest validation rejects many internally inconsistent records,
  including duplicate buses, duplicate per-bus send/receive IDs, duplicate
  serial strings, wrong pinned motor family, non-unit software mapping,
  out-of-model limits, and wrong declared codec/gear spans.
- The physical control backend fails before verification or motion. The default
  collision policy rejects every plan. Unchecked planning is virtual-only.
- Controller plans bind measured feedback sequence, controller instance,
  verification epoch, manifest/model revisions, and collision-scene revision.
- Commissioning sessions check size/version metadata, sample freshness,
  monotonic sequence/timestamps, disabled state for manual collection, bounded
  motion evidence for recipes, repeatability, and explicit review.

These are useful local invariants. None proves that caller-provided identity,
configuration, samples, or qualification evidence corresponds to the physical
machine.

## Unsafe gaps and adversarial counterexamples

### 1. Interface and motor identity

- Interface name and ifindex are runtime routing identifiers, not stable adapter
  identity. The discovery record has no USB/PCI path, adapter serial, physical
  channel, topology fingerprint, or signed left/right assignment. Renaming or
  swapping adapters can exchange arms while preserving valid-looking bus names.
- The expected-ID probe is not discovery: it requires send ID, receive ID,
  embedded ID, decode motor type, and joint metadata in advance.
- `commissioned_serial` is local metadata and may be empty in the CAN manifest.
  The probe never queries or compares it. The control manifest requires a
  nonempty unique string but accepts any caller-invented value.
- Register responses can be correlated to target and response IDs, but two
  physical drives configured with the same IDs cannot always be distinguished.
  Identical replies may look like one node; divergent replies may cause CAN
  errors. Observed duplicate frames are evidence of ambiguity, but absence of a
  duplicate is not proof of one physical responder.
- `oa_can_probe_expected` counts unexpected and invalid frames but can still
  return `OA_CAN_OK` when those counters are nonzero. That may be reasonable on
  a shared diagnostic bus, but it is not an acceptance-grade exclusivity test.

### 2. ID, bitrate, and mode collisions

- CAN/control validation rejects duplicate send IDs and duplicate receive IDs
  within one arm, but does not establish the complete arbitration-ID collision
  matrix across command IDs, response IDs, broadcast/query IDs, and every
  enabled control mode. A send ID equal to another drive's receive ID is not
  explicitly rejected.
- Control requires distinct bus-name strings, but does not bind those names to
  discovered stable adapters. It accepts any nonzero per-motor bitrate and does
  not require all motors on an arm to declare one common bitrate or compare the
  declaration to the live link bitrate.
- Link enumeration can report absent/unknown bitrate data. There is no
  fail-closed facade rule connecting live link MTU/bitrate/FD state to every
  motor record.

### 3. Gear ratio, encoder zero, and sign

- Control validates declared family-specific gear ratios against hardcoded
  values, not live readback. CAN exposes a gear-ratio register, but no unified
  verifier correlates it with serial, firmware, output-position semantics, and
  the immutable manifest.
- Commissioning assumes the supplied sample already is output-shaft radians and
  deliberately applies no second gearbox ratio. Nothing in that ABI proves the
  caller decoded the output-position rather than motor-position register or
  applied the correct units and wrap/multiturn convention.
- Manual sign/offset evidence is not linked to a fixture revision. One-point
  calibration with caller-asserted `known_sign` can produce a valid patch from
  the wrong joint or a false reference angle.
- The recipe's qualification/evidence/fixture revisions are untrusted integers
  and strings supplied by the same caller. Equality on subsequent steps proves
  consistency of an assertion, not authenticity or qualification.
- Neither commission API binds a sample to interface identity, receive ID,
  embedded ID, serial-number readback, side, joint, or capture session nonce.
  A fresh sample from motor A can be committed as motor B's patch.
- There is no checked patch-application API, no comparison with current live
  readback, and no proof that `direction`, software `q_scale`, and drive
  direction-register semantics form one qualified mapping.

### 4. Clock and freshness composition

- CAN probing requires a monotonic receive timestamp that represents arrival
  after the corresponding request.
- Public transport exposes a monotonic **dequeue** timestamp and a kernel
  **realtime** arrival timestamp. It does not expose a kernel monotonic arrival
  timestamp or an explicit realtime-to-monotonic correlation.
- Therefore, a straightforward adapter that assigns
  `received_monotonic_ns = dequeue_monotonic_ns` can accept feedback already
  queued before the query as fresh. The separate public APIs do not currently
  compose into the freshness guarantee claimed by the CAN probe.
- Controller time and commissioning time are caller-supplied integers, while
  event polling constructs a system steady-clock absolute deadline. There is no
  facade clock-domain ID or required clock source spanning transport, probe,
  controller, calibration samples, expiries, and events.

### 5. Units, frames, TCP, and orientation

- Names such as `_rad`, `_rad_s`, `_nm`, and `_m` help, but the facade cannot
  reject degrees passed as radians. Discovery/register evidence does not carry
  a unit/convention tag.
- Model FK/IK is body-frame and the exact named URDF `hand_tcp`. The controller
  TCP move record contains XYZ values but no frame ID, model digest, TCP/tool
  revision, transform convention, or orientation policy field.
- There is no world-to-body registration, independently calibrated base pose,
  user tool transform, payload, or replaceable TCP. A caller can accidentally
  send world coordinates to a body-coordinate function.
- IK and controller TCP motion constrain position only. Orientation is free.
  Returning a 4x4 achieved transform is useful, but it does not make the target
  a six-degree-of-freedom pose or guarantee tool orientation along the path.

### 6. Gripper ambiguity

- The CAN and commissioning schemas permit an eighth device/joint index 7, but
  the model and controller have exactly seven arm DOFs. There is no unified
  mapping from gripper motor output angle to finger aperture/pose or force.
- A simulation-only gripper patch is labelled as such, but no facade prevents a
  consumer from copying its numeric `a,b` into another manifest. Simulation
  evidence must be structurally ineligible for real capability, not merely a
  field the caller is expected to inspect.

### 7. Persistence and provenance

- `oa_manifest_load` is reserved and returns unsupported. Runtime manifest
  creation copies caller records into memory; commissioning only returns a
  patch record.
- There is no canonical serialization, schema migration, atomic file replace,
  checksum/signature verification, rollback, monotonic revision store, audit
  log, or power-cycle readback.
- `manifest_revision`, `model_revision`, evidence revisions, and scene revision
  are nonzero caller-selected counters. The control manifest is not bound to
  the model's exposed source/data/URDF hashes. A revision number is not a digest.

### 8. Collision scene and motion qualification

- Model IK always reports `collision_checked = 0` and checks no self, body,
  inter-arm, gripper, payload, or environment geometry.
- Control's `collision_scene_revision` is only a caller integer. No scene is
  supplied or evaluated. The sole permissive policy is explicitly
  `OA_COLLISION_VIRTUAL_UNCHECKED`.
- The virtual planner checks sampled Cartesian waypoints for IK continuity,
  bounds, protocol span, branch jump, and singularity policy, but those are not
  continuous collision checks or proof that the smooth joint interpolation is
  collision-free between samples.
- Consequently, the existing code cannot satisfy a physical joint-motion or
  XYZ-motion acceptance gate, even if a transport backend were added.

### 9. Activation, authority, E-stop, and lifecycle

- CAN can construct enable, motion, set-zero, clear-error, register-write, and
  save frames. Its README correctly says the codec library is not an
  incapable-of-control safety boundary. A caller-provided transport callback
  can transmit them outside the query-only public SocketCAN path.
- The controller challenge is a lifecycle anti-staleness token, not operator
  authentication or a safety authorization. It is virtual-only today and must
  not be reused as physical authority.
- `deadman_active_` defaults true in the controller. That is tolerable only
  because physical verification is unsupported; a physical design must default
  every external permission false/unknown and require fresh positive evidence.
- `oa_controller_set_interlock` receives caller booleans. There is no independent
  E-stop channel, safety relay feedback, drive-safe-torque-off feedback, or
  authenticated source.
- Commissioning next actions are advisory. Destroying or aborting a session
  sends no disable command, and the library cannot verify that a caller obeyed
  `ABORT_DISABLE` or action bounds.
- Public transport handles are ordinary allocated pointers rather than the
  monotonic registries used by control/commission. Its documented close/destroy
  sequencing is required; arbitrary stale or double-destroyed handles are not a
  safe lifecycle API for a unified facade.
- Process death, scheduler stalls, host power loss, cable loss, CAN bus-off,
  partial paired transmission, and drive-local watchdog behavior have not been
  validated on real drives. Virtual stop materialization is not evidence of
  physical stopping.

### 10. ABI and status-domain collisions

- Model and control deliberately retain incompatible generic legacy `oa_status`
  and `OA_*` names. Their compile-time guard prevents silent coexistence unless
  legacy names are disabled, but this is still unsuitable as the primary
  unified include experience.
- CAN, transport, commission, control, and model status numbers overlap with
  different meanings. Casting or directly returning one module's code from an
  adapter can silently misreport errors. For example, CAN and transport timeout
  codes are not numerically interchangeable.
- ABI macro naming and version conventions differ. Some APIs accept append-only
  prefixes, some require full current records, and output-write-on-error policy
  differs. A facade needs one record and error policy rather than asking every
  consumer to remember five policies.

## Minimal Stage-A unified facade

Stage A should unify truthful hardware-free functionality, not join all lower
symbols into a larger apparent robot API. Its public header should expose no raw
CAN frame sender, no custom transport callback, no register write, no set-zero,
no save, no enable, and no physical controller constructor.

### Required common contract

- One ABI macro and one `{struct_size, abi_version}` header convention.
- One namespaced facade status type whose values do not alias lower-module
  numbers. Preserve `{facility, lower_code}` in an error-detail record.
- Registry-validated monotonic opaque handles for every object, including
  discovery/transport handles; destroy is idempotent and stale tokens fail.
- One facade-owned monotonic clock. Every timestamped input carries the clock
  domain ID; foreign domains are rejected.
- Immutable backend kind and capabilities returned by the handle, not accepted
  as caller claims. Stage A reports `VIRTUAL` and `REAL_READ_ONLY` distinctly.
- Every coordinate request/result explicitly carries frame ID, units version,
  model-data digest, TCP revision, orientation policy, and collision policy.
- Atomic multi-arm semantics: a paired request returns per-arm diagnostics but
  commits neither arm if either side fails.

### Small useful surface

1. `create_virtual` / `destroy` and `get_capabilities`.
2. `now` from the single facade clock.
3. `get_model_identity` returning exact model/data/URDF hashes, body frame, tip
   frame, DOF, and immutable units/conventions.
4. `fk` for an arm mask and supplied model-joint vectors.
5. `solve_xyz` with arm mask `{LEFT, RIGHT, BOTH}`, explicit body frame,
   position-only orientation policy, seeds/options, and atomic paired result.
6. Virtual `snapshot` / `get_coordinates`, with feedback sequence and complete
   transforms.
7. Virtual `plan_joint` and `plan_xyz` with arm mask, followed by
   `plan_report`, `execute`, `heartbeat`, `stop`, `disarm`, and event polling.
   `plan_xyz(LEFT|RIGHT)` is new work; it must not be advertised until its
   single-arm implementation and tests exist.
8. Read-only discovery-session creation and evidence export as the only Stage-B
   extension point.

Capability bits should distinguish at least model FK, single XYZ IK, atomic
paired XYZ IK, virtual coordinate query, virtual joint motion, virtual single
XYZ motion, virtual paired XYZ motion, real discovery/query, real configuration,
real calibration action, and real motion. In the current implementation the
last three must always be false. `collision_checked` is a result fact, never a
caller-set capability.

Stage A may expose unchecked virtual motion only with an explicit per-session
opt-in whose reports remain `collision_checked=false`. The default remains
reject-all.

## Fail-closed Stage-B discovery and commissioning workflow

Stage B remains read-only with respect to drive state. “Read-only” permits
passive receive and documented status/register queries; it does not permit
disable, clear-error, set-zero, register write, flash save, enable, or motion.
Commissioning produces a candidate artifact and never executes its next action.

1. **Create an evidence session.** Bind software build digest, facade ABI,
   model/data/URDF hashes, approved register schema, one monotonic clock-domain
   ID, operator/test-plan identity, and a random session nonce.
2. **Enumerate without opening CAN.** Record name, ifindex, link flags, classic
   CAN MTU, bitrate, FD state, and a new stable adapter/channel identity. Reject
   virtual interfaces and any missing or changing attribute. Require explicit
   operator selection; never assign left/right from enumeration order.
3. **Prove a controlled bus context.** The acceptance procedure requires an
   isolated arm bus or a documented exclusive topology. Observe a bounded
   passive interval. Any CAN error, overflow, link transition, unexpected node,
   or unapproved traffic makes the session inconclusive.
4. **Open query-only with exact filters.** Subscribe before evidence capture,
   drain/quarantine pre-session frames, and timestamp frame arrival in the same
   monotonic domain as query send. A dequeue timestamp is insufficient.
5. **Inventory candidates conservatively.** Query only an approved ID range and
   rate. Correlate every response by interface/channel, receive ID, echoed target
   ID, opcode, RID, session interval, and monotonic arrival. Repeatedly read
   serial, hardware/software/sub-version, send/master IDs, control mode, bitrate,
   timeout, direction, gear ratio, PMAX/VMAX/TMAX, and motor/output position.
   Any inconsistency, duplicate, error, overflow, invalid/unexpected response,
   or receive-budget exhaustion fails the session. Same-ID physical duplicates
   remain impossible to exclude by protocol alone and require isolation.
6. **Bind physical identity explicitly.** An operator-approved fixture/label
   step binds stable adapter channel plus repeatedly read serial to side and
   joint. IDs are routing data, not identity. The tool must never infer a joint
   from motor family or arbitration ID.
7. **Collect disabled manual evidence.** With drives demonstrably disabled and
   an independent restraint/E-stop, collect fresh output-position samples at at
   least two qualified fixture angles per arm joint. Bind every sample to the
   session nonce and complete live identity tuple. Compute sign/offset, then
   repeat the fixture run independently. Gripper evidence is a separate schema
   and cannot be inserted into the seven-joint arm model.
8. **Dry-run supervised recipes only.** Replay trace data through the recipe
   state machine and verify every bound/action/abort transition. Before real
   hardware acceptance, no recipe action may be transmitted. Simulation-only
   evidence is structurally marked non-promotable.
9. **Construct a candidate manifest.** Include stable bus identities, complete
   register readbacks/raw bytes, unit and encoder conventions, per-joint
   mapping evidence, thresholds and their qualification source, model hashes,
   exclusions (notably gripper and collision), and an evidence digest. Validate
   all generated arbitration IDs and a single bitrate/mode per bus.
10. **Persist outside runtime atomically.** Canonically serialize, sign or
    authenticate, fsync a temporary file, atomically replace, retain prior
    revision, and verify by reload. Stage B never writes drive flash. Runtime
    accepts only a digest-matching, monotonic, explicitly accepted artifact.
11. **Replay hardware-free.** Feed captured raw frames through codecs, query FK,
    solve single and paired XYZ, exercise stale/error/duplicate/overflow/link
    faults, and prove that no real-write capability appears.
12. **End closed.** Close transport, invalidate nonce and evidence handles, and
    report `CANDIDATE_NOT_ACCEPTED`. No call in this workflow arms or moves a
    drive.

## Exact acceptance gates

All gates are conjunctive. “Unknown,” “not measured,” “caller asserted,” or
“not applicable” is failure unless the capability is explicitly excluded from
the accepted scope.

### A. Public-API no-command gate

- Facade capability report has real configuration, real calibration action, and
  real motion bits clear.
- The Stage-A/Stage-B public symbol surface has no raw send, custom transmitter,
  write, set-zero, save, clear-error, enable, disable, or motion entry point.
- Tests prove every dangerous/unknown/motion frame class is rejected by every
  public Linux transport path, including malformed aliases.
- Discovery and commissioning session destruction cannot leave a write
  authority alive. No authority is ever issued on SocketCAN.

Current lower-level CAN frame builders may remain a separately documented
codec, but they must not be re-exported by the facade or treated as a safety
boundary.

### B. ABI and composition gate

- All facade headers compile as strict C11 and C++ in every include order with
  no legacy generic names.
- Every record size/version boundary, smaller prefix, larger tail, invalid
  pointer token, double close/destroy, concurrent close, and status mapping is
  tested.
- Every lower error maps injectively to facade `{facility, code}`; no numeric
  cast is used.
- One documented monotonic source and domain ID is used for send, kernel arrival,
  sample capture, expiry, controller time, and polling. A pre-query queued frame
  fails freshness.

### C. Discovery-integrity gate

- Exactly the accepted stable adapter/channel identities are present and bound
  to explicit sides; classic-CAN MTU, link-up state, non-FD mode, and approved
  bitrate all match the signed artifact.
- No link transition, CAN error, RX overflow, invalid frame, unexpected frame,
  duplicate expected response, stale response, or receive-budget exhaustion
  occurs in any qualification capture.
- The bus is physically isolated for collision exclusion, or each candidate is
  qualified one-at-a-time. CAN observations alone are not accepted as proof
  that a same-ID duplicate drive is absent.

### D. Identity/configuration gate

- The exact expected topology is declared. For the present controller scope it
  is seven arm joints per side; grippers are explicitly excluded.
- Every included motor has a nonempty globally unique, repeatedly stable serial
  and an allowlisted hardware/software/sub-version tuple.
- Live readback of send/master IDs, embedded ID semantics, mode, common bus
  bitrate, watchdog timeout, direction, gear ratio, PMAX, VMAX, and TMAX exactly
  matches the candidate artifact and protocol-qualified ranges.
- The full arbitration-ID collision matrix for all used/query response IDs is
  collision-free. Every response is correlated to its live serial and session.
- Motor/output-position register meaning, radians convention, wrap/multiturn
  behavior, and output-shaft gearing are established by hardware evidence, not
  inferred from a family constant.

### E. Mapping/calibration gate

- Every accepted sample has a strictly increasing sequence and capture time,
  age within the qualified bound, complete identity/session binding, finite SI
  values, disabled/fault-free drive state for manual calibration, and no
  transport integrity event.
- Each arm joint has at least two distinct qualified fixture references whose
  separation, dwell, position spread, velocity, and scale error satisfy signed
  hardware-qualified thresholds. `a` is exactly `-1` or `+1`; both references
  agree on `b`; an independent repeated run agrees within its signed tolerance.
- Mapped URDF limits, fixture poses, raw protocol span, sign, zero, and live
  direction/gear evidence are mutually consistent. FK of all fixture poses
  matches independently measured geometry within a signed tolerance.
- Power-cycle/reconnect readback reproduces identity, configuration, encoder
  convention, and software mapping. No drive zero or flash write is assumed.
- Manual-fixture, hardware-qualified recipe, and simulation-only evidence are
  distinct non-convertible types. A simulation patch can never satisfy a real
  gate.

Numeric motion/contact/thermal thresholds must come from a separately approved
hardware qualification artifact. This repository contains no evidence from
which safe values can be invented; absence of any required value fails the
gate.

### F. Persistence/provenance gate

- Canonical accepted manifest binds raw evidence digest, stable bus identities,
  every motor serial/configuration, calibration results, thresholds and their
  sources, exact model/data/URDF hashes, TCP/tool revision, ABI/build digest,
  operator/reviewer approvals, and exclusions.
- Revision strictly increases in a protected store; signature/MAC and digest
  verify before use; reload is byte-semantically identical; interrupted update
  retains the prior accepted artifact.
- Runtime compares live readback with the accepted artifact before exposing any
  real capability. A mismatch revokes acceptance and returns to read-only.

### G. Independent hardware-safety gate

- An independent E-stop/STO path, deadman, restraints, guarded workspace, and
  hardware watchdog are installed and tested. Permission defaults false/unknown
  on boot, reconnect, process restart, and source loss.
- A controlled qualification harness, separate from the ordinary public ABI,
  measures worst-case stop behavior for E-stop, deadman release, producer
  timeout, process death, scheduler overrun, cable loss, link down, bus-off,
  stale/partial feedback, partial paired transmission, motor fault, and thermal
  fault under approved loads.
- Drive disable/STO feedback is independently observed; sending a disable frame
  is not proof of a disabled actuator.

Passing A-F may authorize only a specifically reviewed laboratory
qualification procedure. It does not authorize general motion. The ordinary
public build remains command-incapable until the resulting real-hardware safety
evidence is accepted.

### H. Motion/geometry gate before any production physical motion

- A physical backend exists and is separately reviewed; command authority is
  unpredictable, instance/identity/artifact-bound, one-shot, short-lived,
  least-privilege, and revoked on every lifecycle/integrity event.
- Real measured feedback, not a simulator plant, drives state, completion, and
  stop confirmation. Timing/jitter/skew and paired atomicity meet signed
  hardware-qualified limits.
- Joint, velocity, acceleration, jerk, torque/current, temperature, power, and
  workspace limits are validated end-to-end against live units and loads.
- A real collision engine checks self, body, inter-arm, gripper, payload/tool,
  environment, and continuous paths against immutable scene geometry. The plan
  and execute transaction bind its digest, not just an integer revision.
- World/body/base registration and exact TCP/tool transform are independently
  calibrated. Position-only/free-orientation behavior is explicitly accepted
  for the task, or orientation-constrained IK/planning is implemented.
- Single-arm XYZ and paired XYZ trajectories, branch continuity, singularity
  policy, unreachable targets, limit approaches, completion, abort, and stop
  behavior pass real-hardware tests.
- Gripper state/motion remains absent unless aperture/force mapping, limits,
  collision geometry, object interaction, and stop behavior pass a separate
  acceptance.

The current repository necessarily fails H: physical verification is
unsupported and collision checking is always false. Therefore production
physical commands must remain unavailable.

## What cannot be tested without real hardware

- Stable adapter-to-arm wiring, actual bus topology, termination, bitrate
  margin, electrical errors, and same-ID physical collisions.
- Whether a response serial belongs to the physically labelled side/joint and
  whether IDs persist or change after power cycles.
- Firmware/register semantics, actual motor family, gear ratio, output- versus
  motor-position meaning, radians scaling, sign, zero, wrap/multiturn behavior,
  torque/current scaling, and flash persistence.
- Fixture truth, encoder repeatability/backlash, hard-stop repeatability,
  structural compliance, gravity/backdrive/brake behavior, gripper aperture and
  force mapping, and TCP/base/tool geometry.
- Safe velocity, acceleration, jerk, torque, temperature, energy, payload, and
  workspace thresholds.
- Independent E-stop/STO and deadman behavior; drive-local watchdog; disable
  acknowledgement; stop distance/time under load.
- Kernel/USB/CAN latency and jitter under contention; bus-off recovery; overflow,
  cable loss, process death, scheduler stall, partial command, and cross-bus
  skew on the actual interfaces and drives.
- Physical self/body/inter-arm/gripper/tool/environment collision, continuous
  path clearance, singularity behavior under load, and the consequences of free
  TCP orientation.
- Real command encoding/decoding, individual/paired execution atomicity,
  measured completion, fault containment, and lifecycle recovery.

Simulation, vcan, synthetic frames, ABI tests, sanitizers, fuzzing, replay, and
fault injection should be required before hardware work, but none can substitute
for these tests or qualify a real arm.

## Final assessment

The existing documentation is mostly candid and the strongest safety property
is that no public Linux path can currently move hardware. Preserve that.

The next safe deliverable is not a physical controller. It is a small unified,
capability-reporting, hardware-free facade plus an evidence-producing read-only
discovery/commissioning session that fixes clock composition, identity binding,
status domains, persistence, and provenance. Only after the explicit gates
above are backed by real-hardware evidence should a separately versioned,
least-authority physical command component even become available.
