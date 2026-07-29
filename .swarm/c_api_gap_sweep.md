# Unified modular C API requirements gap sweep

Date: 2026-07-29 (America/Los_Angeles)  
Tree inspected: `main` at `21ab251ab673618dc46201dc633c5b94aac53379`  
Disposition: **DONE_WITH_CONCERNS**

This was a source/header/test/report inspection only. No executable was run, no
GUI or network was opened, no CAN interface or SocketCAN socket was inspected,
and no hardware operation was attempted. The only workspace write is this
report. Existing unrelated untracked `.swarm` reports were read where relevant
and otherwise left untouched.

## Bottom line

The repository now has five installable, versioned C APIs and a verified common
build/install path, but it still does **not** provide the requested unified
object-oriented C product interface.

- `openarm_can` has codecs, typed register queries/writes, expected-ID probing,
  fake diagnostics, and read-only Linux CAN-interface enumeration.
- `openarm_transport` opens real classic SocketCAN but its public authority is
  permanently query-only: only register/status query frames can be sent.
- `openarm_commission` has well-tested manual and supervised calibration state
  machines, but they consume caller-provided samples and return data/action
  records; they own no transport or controller and apply nothing.
- `openarm_model` provides immutable left/right FK, all joint origins/axes,
  Jacobians, and bounded XYZ-only position IK.
- `openarm_control` provides an encoder-derived two-arm virtual controller with
  snapshots, joint coordinates, TCP coordinates, one-joint plans, paired-TCP
  plans, execution, lifecycle, watchdogs, and events. The physical backend is
  intentionally unsupported before verification and sends no traffic.

The modules compile and install together, but their authorities and objects do
not connect. There is no discovery-to-manifest workflow, no register inventory
or identity matcher, no persisted manifest loader/writer, no calibration-patch
application, no controller/commissioning lease, no CAN/transport-backed
controller, and no unified event/status stream. The current ROS node is still a
separate model-only visualization adapter; the production ROS/control reports
describe future work, not installed behavior.

This separation is safety-positive. It must not be “fixed” by granting public
SocketCAN control authority or by allowing `OA_BACKEND_PHYSICAL` to progress.
The smallest safe completion is a new module-prefixed orchestration facade over
the existing libraries, with virtual motion only and physical inventory/query
only.

## Evidence and installed-SDK qualification

The source build graph in `scripts/build_native.sh` builds and installs, in
dependency order, CAN, model, commission, transport, and control. The installed
CMake targets are:

| Package | Installed CMake target | Public header |
|---|---|---|
| `OpenArmCan` | `OpenArm::Can` | `openarm_can.h` |
| `openarm_model` | `OpenArm::Model` (plus legacy package target) | `openarm_model.h` |
| `openarm_commission` | `OpenArm::Commission` | `openarm_commission.h` |
| `OpenArmTransport` | `OpenArm::Transport` (plus legacy target) | `openarm_transport.h` |
| `openarm_control` | `OpenArm::Control` (plus `openarm_control::openarm_control`) | `openarm_control.h` |

The final clean unified-build review at `25b2735` records a fresh Release build
with **14/14 native CTests passed**: CAN 1, model 4, commission 2, transport 3,
and control 4. It also records successful strict installed C11 and C++17
all-header consumers linked against all five production archives, clean
cross-prefix reuse, no test-only symbols in installed control/commission
archives, and no duplicate CAN implementation embedded in transport. Current
`main` adds only review documentation after that result.

There is an important local-state distinction: the existing checked-in-worktree
output at `ros2_ws/install` is stale and currently contains only
`openarm_model.h`/`libopenarm_model.a` plus the ROS packages. It is not a current
five-library SDK prefix. A clean run of the present `scripts/build.sh` would
replace it with the five native installs before building ROS, but this audit did
not run that command. Therefore this report treats the source install/export
contracts and the independently reviewed fresh prefix as authoritative, not the
stale default output directory.

All-header consumers must define
`OPENARM_DISABLE_LEGACY_GENERIC_STATUS` before including model and control.
Canonical declarations now use `oa_model_status`/`OA_MODEL_*` and
`oa_control_status`/`OA_CONTROL_*`; the generic `oa_status`/`OA_*` spellings are
single-header compatibility aliases and deliberately fail when the two headers
are combined ambiguously.

## Classification used below

- **Implemented + tested**: public symbol and exercised contract exist in its
  intended non-hardware scope.
- **Implemented but disconnected**: useful public component exists, but no
  shipped owner connects it to the requested workflow.
- **Virtual-only**: controller behavior works only in the deterministic
  in-process simulator.
- **Physical intentionally blocked**: real inspection/query exists, but state
  change, calibration motion, arming, or motion is denied by design.
- **Missing**: no public operation provides the capability.

## Requested-capability map

| Requested capability | Exact current public surface | Classification | Actual contract and gap |
|---|---|---|---|
| Auto-detect CAN interfaces | `oa_can_linux_list_interfaces` -> `oa_can_linux_interface[]` | **Implemented + parser-tested; live enumeration deliberately untested** | Linux rtnetlink `RTM_GETLINK` only. Reports name, ifindex, flags, MTU, bitrate/data bitrate, link-up and FD flag. It never changes link state/bitrate/MTU and never opens CAN. It does not say that an interface belongs to an OpenArm. |
| Auto-detect motors/arms | `oa_can_probe_expected` with caller `oa_can_arm_manifest`, `oa_can_transport`, and options | **Implemented but disconnected and expected-only** | Sends refresh-status queries only to already supplied send IDs. Verifies fresh disabled feedback at expected arbitration and embedded IDs, with duplicate/fault/enabled/stale accounting. It does **not** enumerate IDs or verify serial, motor family, firmware, joint, side, mapping, registers, or an “arm.” There is no adapter from public `oa_transport *` to the callback-style `oa_can_transport`, and no inventory object. |
| Open real CAN for inspection | `oa_transport_open`, `oa_transport_send`, `oa_transport_receive`, `oa_transport_close/destroy` | **Physical intentionally query-only** | Opens an existing up classic-CAN interface without configuring it. Public sends accept strict typed register/status query frames only; unknown, motion, enable, disable, register-write, set-zero, clear-error, and save frames fail. Receives expose CAN errors, link transitions, kernel timestamp metadata, and overflow. SocketCAN/vcan/xcan/slcan cannot issue private authority. |
| Configure exact motor identities/protocol/limits in memory | `oa_manifest_create(const oa_manifest_config*, ...)`; `oa_motor_config`, `oa_arm_config` | **Implemented + tested, but compiled-record only** | Copies immutable 2x7 configuration and rejects missing/duplicate bus names, IDs and serials; wrong side/index/name; wrong J1-2/J3-4/J5-7 motor families; non-MIT mode; missing bitrate/watchdog/version; non-unit sign mapping; invalid URDF limits/dynamics; wrong hard-coded P/V/T spans or 9/40/10 gear profiles. It does not obtain or read back any fact. It accepts caller assertions after local validation. |
| Construct the standard virtual OpenArm configuration | No production symbol; the complete valid fixture is assembled in `control/tests/test_control.cpp` | **Missing** | The ROS design proposes `oa_manifest_create_openarm_v10_virtual`, but it is not implemented. Every external virtual consumer must currently reproduce the large exact 2x7 configuration, identities, profiles, limits and revisions. |
| Query exact physical identity/configuration | CAN typed RID functions (`oa_can_make_register_query_typed`, `oa_can_decode_register_response`, `oa_can_mit_profile_from_registers`) plus query-only transport | **Implemented but disconnected/partial** | Exact typed query/response correlation and P/V/T profile construction exist. No orchestrator queries the complete serial, hardware/software/sub-version, IDs, bitrate, mode, timeout, gear, direction, and spans, compares them with a manifest, reports omissions/ambiguity, or binds the result to controller verification. Some desired “identity” facts are not safely discoverable as side/joint/assembly identity from motor registers at all. |
| Persist/load/preview configuration | `oa_manifest_load(path, sha256_path, ...)` | **Missing (reserved symbol)** | Non-null arguments always return `OA_CONTROL_EUNSUPPORTED`. There is no schema, digest verification, serializer, atomic writer, revisioned draft, diff preview, armable-vs-draft state, or accessor/export from an `oa_manifest`. Runtime never edits a manifest. |
| Manual calibration from encoder feedback | `oa_commission_manual_create/sample/begin_review/commit/get_report/abort/destroy` | **Implemented + tested, but disconnected** | Consumes fresh, stable, fault-free, torque-disabled output-shaft samples. One reference requires a known sign; two separated fixture references can establish sign. Produces `q_model = a*q_output + b`, with `a` exactly +/-1. It does not source snapshots, own an exclusive controller lease, write motor zero, or apply/save its `oa_commission_mapping_patch`. |
| Supervised automatic calibration using encoder feedback | `oa_commission_recipe_create/step/commit/get_report/abort/destroy` and `oa_commission_next_action` | **Implemented + tested as an abstract state machine; disconnected** | Enforces feedback freshness/sequence, E-stop, deadman, drive state/fault, posture, travel, phase deadlines, speed, torque, temperature, contact energy, two contact dwells, retreat/reapproach repeatability, explicit review, and evidence revisions. Output is a bounded next-action request, not motor authority. Arm-joint recipes require nonzero hardware-qualification and fixture evidence supplied by the caller; simulation-only evidence is allowed only for grippers. No controller executes an action and no physical recipe is available through transport. |
| Apply calibration | `oa_commission_mapping_patch` output only | **Missing** | Nothing validates a patch against a live manifest/detection fingerprint, applies it transactionally, increments a persisted revision, previews affected raw/model limits, reloads/reprobes, or rolls back on failure. Firmware `0xFE` set-zero exists only as a frame builder and remains untransmitted. |
| Expose joint coordinates and raw encoder state | `oa_controller_snapshot` -> `oa_snapshot.arm[side]`; model metadata functions | **Virtual-only in controller; implemented + tested there** | Exposes coherent feedback sequence/time, expected/fresh/fault masks, mapped `q/dq/tau`, raw output `q/dq/tau`, status and temperatures for both 7-DOF arms. State is generated from quantized/decoded simulator feedback, not commands. No physical transport feeds it. |
| Expose joint origins/axes and TCP coordinates | `oa_controller_get_kinematics`; `oa_fk`; `oa_geometric_jacobian` | **Implemented + tested; controller path virtual-only** | `oa_arm_kinematics` binds to an exact feedback sequence and returns model `q`, seven joint XYZ/axes, TCP row-major transform, and TCP XYZ. Model FK is usable independently for any valid q. Coordinates are relative to `openarm_body_link0`; metres/radians. |
| Individual joint motion | `oa_controller_plan_joint`, report, execute, advance, heartbeat, stop | **Virtual-only, implemented + tested** | Requires armed-idle, fresh healthy measured state and exact feedback sequence. Targets one side/index, holds the other six measured start coordinates, validates model/raw bounds and scale/tolerances, and binds controller/verify/manifest/model/scene/start. Default collision policy rejects; success requires explicit `OA_COLLISION_VIRTUAL_UNCHECKED`, so reports are not collision-authorized. |
| Paired target XYZ IK and motion | `oa_controller_plan_paired_tcp`; model `oa_ik_position_v2` | **Virtual-only, implemented + tested with major qualification** | Accepts explicitly named left/right XYZ and both measured sequences. Builds 17 predecessor-seeded Cartesian position waypoints and rejects IK residual, bounds/profile span, branch jump, singularity threshold, or scene-revision errors. IK is position-only; orientation is free. Targets are body-frame metres, though the control field names do not encode the frame. There is no collision engine: the only successful policy is virtual-unchecked and `collision_checked==0`. |
| Status and events | Control snapshot/lifecycle, `oa_controller_poll_event`; transport events; CAN probe report; commission reports/actions | **Implemented in each module, disconnected globally** | Control has verified/armed/started/completed/stopped/faulted/disarmed/queued/settling/aborted/E-stop events and fail-closed event overflow. Transport has frame/CAN-error/link events. Commission has state/abort reports. There is no single correlated source/status/event type, capability report, detection/calibration event, or stable translation between module status spaces. |
| External-program consumption | Five C headers, opaque handles, installed CMake packages, C11/C++17 consumers | **Implemented + tested** | External C can consume every component. Records use size/version validation and opaque handles where ownership is needed. The consumer must currently orchestrate all conversions, clocks, ownership, statuses and persistence itself. |
| ROS consumption | Current `openarm_ik_ros`; future design in `.swarm/ros_design_synthesis.md` | **Current model-only demo; production adapter missing** | Current node calls model IK directly, instantly replaces stored q, publishes those commanded values (plus fabricated zero fingers) repeatedly with fresh ROS timestamps, and never creates/advances/snapshots an `oa_controller`. The proposed `VirtualControlSession`, measured snapshots, joint/paired actions, arbitration and compiled CLI are design only. |

## Exact safety and semantic constraints that must remain visible

### Detection is not identity

Interface presence, a CAN reply, an arbitration ID, or even a motor serial does
not establish left/right side, joint index, spline assembly, kinematic zero,
sign, fixture, safe limit, or collision geometry. `oa_can_probe_expected`
correctly states this limitation and succeeds only for a caller-supplied
expected layout. Broad automatic baud/ID scanning is not currently implemented
and would not be passive: upstream discovery changes host link bitrate and sends
queries. Any future inventory operation must report candidates and unresolved
facts, never silently manufacture an arm mapping.

Factory-default duplicate IDs are especially dangerous for configuration. A
register write addressed to an ID can affect multiple motors. Configuration
writes and ID changes must remain outside the normal runtime and require an
isolated single-motor commissioning tool. This facade proposal therefore does
not add them.

### Physical authority is deliberately absent

`oa_transport_send` passes no public authority token. It permits only register
and status queries. The internal exact-frame, one-shot authority mechanism is
available only to injected test/simulator backends; every Linux SocketCAN
backend refuses issuance. The CAN codec can construct enable, motion,
register-write, set-zero, clear-error and save frames, but construction is not
authorization and no CAN library transport sends them.

`OA_BACKEND_PHYSICAL` may be selected when creating a controller, but
`oa_controller_open_and_verify` resets to closed, reports failure mask `0x3`,
and returns `OA_CONTROL_EUNSUPPORTED` before motion. The unchecked collision
policy is rejected for a physical backend. These are required product states,
not gaps to bypass.

### Calibration cannot discover home by itself

DaMiao output feedback is single-turn absolute output-shaft position when the
drive is correctly configured. It does not reveal the assembled link's semantic
zero. Set-zero (`0xFE`) saves the current encoder pose as zero; it does not find
home. Manual fixture/reference evidence or a qualified supervised external
reference is required. The commission library appropriately commits only a
software mapping patch and never sends set-zero/save.

### Limits have different meanings

PMAX/VMAX/TMAX are protocol mapping spans, not proven structural, continuous,
peak, contact or collision-safe limits. The control manifest separately carries
model lower/upper and virtual velocity/acceleration/jerk limits. It currently
pins P/V/T and gear values by family. Exact physical readback and independently
qualified operational/thermal/torque limits remain missing.

The present simulator uses max jerk only to size the seventh-order command
reference duration. Its lag plant changes acceleration by a sample step and has
two direct position/velocity snap-to-target branches. The untracked
`motion_profile_design.md` correctly concludes that this cannot support a
no-sudden-jerk plant claim. Virtual motion exists, but a production portal or
motion facade should not advertise jerk-continuous executed plant truth until
that design is implemented and tested.

`OA_STOP_CONTROLLED` currently materializes an immediate encoder-visible
zero-velocity enabled hold; it is not a jerk-limited controlled deceleration.
Cancellation and safety failures should use the explicitly named disable path
until a collision-checked braking trajectory exists. Also, the controller's
internal deadman state currently defaults true. That is contained by the
physical-backend gate, but a facade must initialize interlocks fail-safe and
require an explicit fresh deadman assertion before any virtual arm transition;
future physical work must never inherit the current default.

### Collision is a hard boundary

Model IK always reports `collision_checked=0`. Control defaults to
`OA_COLLISION_REJECT_ALL`; the only successful policy is explicitly
`OA_COLLISION_VIRTUAL_UNCHECKED`. The coordinate verification report exercised
17-waypoint virtual examples successfully but could not prove clearance from
the other arm or the central pole. A scene revision number is not geometry and
does not make an unchecked plan checked. Physical motion and any production
portal motion remain blocked until continuous swept-path validation exists.

## Current clock, unit and frame contracts

| Domain | Current contract | Concern for a unified caller |
|---|---|---|
| CAN probe | `oa_can_transport.now/send/receive` and received timestamps are caller-supplied monotonic nanoseconds. Probe computes one monotonic deadline. | Callback adapter and deadline policy are caller work. |
| SocketCAN transport | Send/receive deadlines and dequeue time are absolute host monotonic nanoseconds. Kernel software timestamps are explicitly tagged `OA_TRANSPORT_CLOCK_REALTIME`; overflow and link state are separate fields. | Realtime kernel stamps must never be used as controller deadlines without a sampled conversion. |
| Controller timeline | The caller drives `oa_controller_advance(monotonic_ns)`; plan/start/expiry/producer deadlines, feedback `t_ns` and control event `t_ns` are in that controller timeline. | The API does not publish a clock ID or `now()` tied to that timeline. |
| Control event wait | `oa_controller_poll_event(deadline_ns, ...)` interprets nonzero deadline as absolute host `std::chrono::steady_clock`, unlike the controller's caller-driven timeline. | Two meanings share the name `deadline_ns`; a facade must hide this mismatch. |
| Model/control coordinates | Joint q is model-joint radians. Raw q is motor output radians. Mapping is `q=a*qout+b`, `dq=a*dqout`, `tau=tauout/a`; no second gearbox division. TCP/joint XYZ are metres in `openarm_body_link0`; transforms are row-major parent-to-child with column vectors. | Control TCP request field names say metres but not frame. Orientation is not requested and is free. |
| Temperatures/effort | Temperature bytes are degrees Celsius; torque is protocol-reported Nm estimate, not electrical current. | Facade names and metadata must not relabel torque as current or protocol spans as safety limits. |

## Smallest stable unified facade

Add one new library/header, `libopenarm_runtime` / `openarm_runtime.h`, rather
than changing the five existing ABIs. It is an orchestrator and policy owner,
not a sixth implementation of CAN, model, calibration, or motion.

### Object model

Use only module-prefixed fixed-width names and the existing size/version record
pattern:

```c
#define OA_RUNTIME_ABI_V1 UINT32_C(1)
typedef uint32_t oa_runtime_status;
typedef struct oa_runtime oa_runtime;
typedef struct oa_runtime_inventory oa_runtime_inventory; /* immutable */
typedef struct oa_runtime_manifest oa_runtime_manifest;   /* immutable draft/armable */
typedef struct oa_runtime_calibration oa_runtime_calibration; /* exclusive session */
typedef struct oa_runtime_plan oa_runtime_plan;           /* immutable */
```

One `oa_runtime` owns the selected backend, one immutable manifest, at most one
controller, a bounded event queue, the clock conversion policy, and an
exclusive operation state. It must serialize operations and use monotonic,
never-reused registry tokens as the hardened control/commission APIs do.
Virtual creation should use one production canonical-manifest builder owned by
the runtime/control package; no adapter may copy the test fixture record.

The minimal ownership/lifecycle is:

```text
runtime
  -> inventory report (zero or more immutable snapshots)
  -> manifest (draft -> locally valid -> inventory matched -> armable)
  -> either calibration lease OR controller lease
  -> zero or one immutable plan
  -> one bounded correlated event stream
```

The facade should wrap, not expose, `oa_transport`, `oa_commission_*`,
`oa_manifest`, `oa_controller`, and `oa_motion_plan` handles. Existing expert
callers can still use component libraries directly, but a runtime caller cannot
mix epochs or inject an unrelated patch/plan/handle.

### Backend and capability contract

Creation options select only:

- `OA_RUNTIME_BACKEND_VIRTUAL`: deterministic inventory and virtual controller;
- `OA_RUNTIME_BACKEND_SOCKETCAN_QUERY`: interface/inventory queries and disabled
  encoder sampling only; and
- optionally `OA_RUNTIME_BACKEND_OFFLINE`: manifest/model/preview without I/O.

Do **not** add “physical control” as an accepted runtime backend until the
existing physical gate is independently implemented. Every runtime exposes a
capability bitset such as interface-enumeration, register-query,
disabled-feedback, manual-calibration-observation, virtual-joint-motion,
virtual-paired-motion, and collision-validated-motion. The SocketCAN runtime
must never report a motion or supervised-calibration-actuation capability.

The implementation must call only public query operations on real SocketCAN.
It must not gain access to transport internals, authority issuance, or raw
dangerous send. A test-time fake backend can prove every emitted real-mode
frame class is register/status query.

### Detection/inventory contract

Provide two phases rather than claiming magical arm discovery:

1. `oa_runtime_list_interfaces(...)` returns the read-only Linux report or a
   deterministic virtual interface report.
2. `oa_runtime_inventory_query(runtime, query_options, &inventory)` performs a
   bounded query scan over an explicit interface and explicit candidate-ID
   allowlist, or verifies an expected manifest. It never changes bitrate/link.

The immutable inventory report needs per-interface and per-response records
with arbitration/embedded IDs, fresh/duplicate/fault/enabled state, every
successfully correlated register value and presence bit, raw timestamps,
protocol profile, and a stable query fingerprint/digest. It also needs explicit
`unknown_mask`, `ambiguous_mask`, `conflict_mask`, and `unresolved_assignment`
flags. “Arm detected” is true only when a separately commissioned manifest
exactly matches a complete inventory; discovery alone never assigns side/joint.

For exact configuration matching, query and compare at least serial, hardware,
software, firmware sub-version, ESC/send ID, master/receive ID, control mode,
CAN bitrate setting, timeout, PMAX/VMAX/TMAX, gear ratio and direction. Missing
or unsupported registers stay explicit, never default silently. Enabled,
faulted, duplicate-ID, duplicate-serial, incomplete, stale, changing, or
ambiguous inventories are not armable.

### Manifest persistence and preview

Implement strict local persistence in the new runtime/configuration module;
leave `oa_manifest_load` unchanged for ABI compatibility until it can delegate
to the same parser.

Use a versioned UTF-8 JSON schema plus detached SHA-256, with:

- duplicate-key rejection, exact known-field policy, bounded strings/arrays,
  finite numeric parsing, canonical side/joint names and fixed 2x7 cardinality;
- manifest/model/schema/calibration/inventory revisions and model/provenance
  hashes;
- exact motor identities/register expectations, affine mapping, model limits,
  separately named protocol spans and operational limits, evidence records and
  qualification kind;
- explicit state: `DRAFT`, `LOCALLY_VALID`, `INVENTORY_MATCHED`, `ARMABLE`;
- load verifies digest before object publication;
- save writes a same-directory temporary, `fsync`s file, atomically renames,
  and `fsync`s the directory; failure leaves the old pair intact; and
- runtime control only accepts an immutable `ARMABLE` object and never edits it.

Preview functions return a structured diff and complete validation report
without writing or changing the active runtime. `oa_runtime_manifest_apply_patch`
must require exact base revision, side/joint, serial, evidence kind and inventory
fingerprint; recompute raw endpoint/span feasibility; increment revision; and
return a new draft plus preview. Saving, reloading, re-querying and exact match
are separate explicit steps. Simulation evidence can never make a physical
manifest armable.

### Calibration integration

`oa_runtime_calibration_begin` acquires an exclusive lease only while motion is
absent and the runtime is disarmed/query-only. Manual sessions should pull an
exact coherent encoder sample from the runtime (or accept a sample carrying the
exact inventory/feedback epoch) and internally feed the existing commission
session. Their commit still returns only a proposed patch.

Supervised sessions expose the existing bounded next-action semantics and
reports. In virtual mode, an adapter may execute actions only after a dedicated
virtual calibration plant exists and is tested. In SocketCAN-query mode every
action requiring enable or movement returns `OA_RUNTIME_EUNSUPPORTED` while the
action remains available for preview/logging. There is no path from caller
“hardware qualified” text to physical authority.

Abort, stale feedback, interlock loss, identity drift, inventory revision
change, runtime destruction, or event overflow terminates the lease and leaves
no patch applied. Calibration and normal planning are mutually exclusive.

### State, motion and events

The facade can initially delegate virtual state/motion directly to
`openarm_control`, with these narrow calls:

- `oa_runtime_snapshot` and `oa_runtime_get_kinematics`;
- `oa_runtime_plan_joint` and `oa_runtime_plan_paired_tcp_body`;
- `oa_runtime_plan_get_report`, `oa_runtime_execute`, `heartbeat`, `stop`, and
  `poll_event`.

Use `*_body` in the paired symbol (or an explicit frame enum fixed to
`OA_RUNTIME_FRAME_OPENARM_BODY_LINK0`) so the frame is contractual. Keep
left/right as named fields, never positional ambiguity. Do not accept
orientation fields while orientation is free.

The runtime event record should contain one runtime event kind, source module,
source status and normalized runtime status, runtime monotonic timestamp,
measurement timestamp/age, manifest/inventory/calibration/model/scene epochs,
feedback sequences, command ID, lifecycle, capability bits, and collision
checked/authorized flags. Preserve the source code; lossy status translation
alone is insufficient.

Success for a motion means matching measured `OA_EVENT_COMPLETED`, not plan
creation, IK success, reference completion, elapsed duration, or ROS
publication. Until a validated collision policy exists, the facade should
either expose unchecked virtual motion only under an explicitly test/demo
option with `motion_authorized=0`, or reject it for production mode.

### Clock convention

Expose `oa_runtime_now_monotonic_ns(runtime, &now)` and define every runtime
deadline, expiry, event time, controller advance and measurement age in that
one epoch. Suffix fields with `_runtime_monotonic_ns`. Zero is “nonblocking” only
where explicitly documented; otherwise use a separate option, not an overloaded
timestamp.

The runtime owns the controller owner thread/cadence in normal use, so external
programs do not need to call `oa_controller_advance` correctly. It also hides
the current `oa_controller_poll_event` host-steady deadline mismatch. Realtime,
ROS, browser and kernel timestamps remain tagged metadata with an explicit
sampled conversion and uncertainty; they never become control deadlines.

### Unit/frame convention

Freeze these in the runtime V1 header and schema:

- model joints: radians, radians/second, Newton-metres estimate;
- raw actuator feedback: output-shaft radians/radians per second/Nm estimate;
- mapping: `q_model=a*q_output+b`, `dq_model=a*dq_output`,
  `tau_model=tau_output/a`; gear ratio is metadata and not applied again;
- XYZ: metres in `openarm_body_link0`;
- transforms: row-major parent-to-child 4x4, column-vector convention;
- temperatures: degrees Celsius;
- target IK: position only, orientation free;
- joint and side identity: canonical names plus explicit numeric index/side;
- protocol spans, operational motion limits, and collision margins are distinct
  fields and never interchangeable.

## Minimum acceptance-test matrix for the facade

1. **Installed ABI:** strict C11/C++17 consumers include all six headers in both
   orders, use only module-prefixed names, link all six installed targets, and
   exercise create/query/destroy. Freeze V1 sizes/offsets and prior component ABI
   tests.
2. **Inventory:** fake interface/register/status streams cover no interfaces,
   capacity query, duplicate IDs/serials, unexpected IDs, stale/duplicate/
   enabled/fault frames, missing registers, mismatched P/V/T/type/version/mode/
   timeout/gear/direction, response reordering, deadline/frame-budget edges, and
   changing inventory. No response may infer side/joint.
3. **Real-mode authority proof:** record every attempted frame from the runtime;
   only strict register/status queries are possible. Motion, enable, disable,
   write, zero, clear and save attempts return permission/unsupported before a
   backend send. Linux SocketCAN still refuses authority issuance. Default CI
   uses only fakes; vcan remains opt-in and sends no control.
4. **Manifest parser/persistence:** schema corpus plus fuzzing; duplicate keys,
   unknown fields, nonfinite/overflow/locale numbers, path/symlink/permission
   issues, digest mismatch, truncation, power-loss fault injection at each
   write/fsync/rename, canonical round-trip, revision monotonicity, and unchanged
   active manifest on every failure.
5. **Preview/patch:** exact-base and stale-base patch, wrong serial/side/joint/
   inventory/evidence, sign/offset endpoint inversion, raw-span/model-limit
   violations, simulation-to-physical promotion rejection, deterministic diff,
   and no file/runtime mutation before explicit save+reload+match.
6. **Calibration integration:** manual one/two-reference sessions consume exact
   runtime feedback epochs; instability/staleness/enable/fault/identity drift
   abort. Recipe action bounds and all existing commission failures propagate.
   Calibration excludes planning, planning excludes calibration, destroy/abort
   releases the lease, and no failure applies a patch.
7. **Virtual end to end:** deterministic virtual inventory -> manifest match ->
   verify -> arm -> joint plan/execute -> measured completion -> paired body-TCP
   plan/execute -> measured completion. Check raw/model mapping, coherent masks,
   feedback sequence binding, FK/TCP results, plan revisions, heartbeat, stop,
   faults and correlated events.
8. **Physical gate end to end:** exact query inventory may succeed, manual
   disabled observation may produce a draft patch, but controller verification,
   supervised actuation, arm, plan and execute all remain unsupported and send no
   state-changing frame.
9. **Clock boundaries:** one runtime epoch; zero, equality, +1 ns, overflow near
   `UINT64_MAX`, backward/equal advance, late cycle, conversion uncertainty,
   realtime/ROS jumps and event-wait cancellation. Republishing never refreshes
   measurement time.
10. **Coordinate/frame:** both sides/all joints, canonical names, mapping signs,
    exact limit endpoints, body-frame TCP/FK consistency, row-major transforms,
    nonidentity world-to-body test, swapped left/right serialization, and
    orientation-absent contract.
11. **Motion truth:** frozen/dropped/delayed feedback never completes; measured
    dwell does. Implement the motion-profile report's analytic-plant correction
    before claiming jerk-continuous execution. Test quantization error floors,
    no target snap, absolute-time evaluation and stop exceptions.
12. **Collision:** production mode rejects every motion until an immutable
    geometry-bearing scene continuously certifies self/body/inter-arm/pole swept
    clearance. An unchecked plan always reports checked/authorized false; a
    scene revision number alone cannot pass.
13. **Concurrency/lifetime:** invalid/stale/cross-type tokens, allocation and
    exception containment, destroy overlap, blocking event cancellation,
    bounded queues/registries, event overflow fail-closed, one controller owner,
    and long deterministic fault/command campaigns.
14. **ROS adapter later:** production ROS links only `OpenArm::Runtime`, publishes
    measured snapshots with measurement-derived timestamps, provides correlated
    joint/paired actions and single-goal arbitration, and never restores the
    current command-as-state processor. Physical parameters/endpoints remain
    absent.

## Recommended implementation order

1. Add `openarm_runtime.h`, installed package/target, unified status/capability/
   clock/event records, and an offline/virtual owner wrapping current control.
2. Add strict manifest parse/preview/persist/apply-to-new-draft and inventory
   fingerprinting; do not change active manifests in place.
3. Add fake-backed bounded inventory orchestration and a thin public-transport
   query adapter. Preserve real SocketCAN's query-only enforcement.
4. Bind manual calibration samples and patch preview under an exclusive runtime
   lease. Keep supervised physical action unsupported.
5. Add end-to-end virtual/runtime tests and installed consumers.
6. Correct the virtual plant's executed jerk semantics and add continuous
   collision validation before exposing production motion.
7. Replace the ROS model-only state owner with the reviewed runtime adapter.

Physical enabled motion is intentionally not in this sequence. It requires a
separate qualified controller/transport authority implementation, hardware
evidence, external E-stop/deadman/watchdogs, operational limits and continuous
collision validation. Until then, the correct unified result is: rich virtual
motion, rich offline/configuration/calibration preview, and query-only physical
inspection.

## Disposition

**DONE_WITH_CONCERNS.** The requested capability can be represented cleanly by a
small new runtime facade, but it is not present today. The largest functional
gaps are motor/arm inventory orchestration, exact readback matching, manifest
persistence/preview, calibration-patch application and cross-module ownership.
The largest safety blockers are the absence of collision validation, the
current virtual plant's unproven executed jerk behavior, and the intentionally
unsupported physical backend. None should be bypassed to make the API appear
complete.
