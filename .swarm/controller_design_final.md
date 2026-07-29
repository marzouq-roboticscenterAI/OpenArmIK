# OpenArm controller: adjudicated implementation plan

Status: **APPROVED FOR STAGED IMPLEMENTATION ONLY.** This replaces the prior
"corrections required" verdict by making every listed correction a mandatory
gate. It does **not** approve physical motion before the Stage-C hardware gates.

## Non-negotiable design decisions

- Build C++17 `libopenarm_control` behind a versioned ISO-C ABI; reuse the
  existing C `openarm_can` codec and `openarm_model`. No Python, ROS, shell, or
  privilege escalation in the runtime.
- One `oa_controller` is the public owner. Internally it owns `ArmRuntime[2]`,
  `BusWorker[2]`, planner, immutable manifest, fixed queues, and event ring.
  Each worker is the sole reader/writer of one CAN FD.
- State truth is only an immutable, timestamped, complete fresh encoder snapshot.
  Commands, elapsed time, IK output, and prior plans are never state.
- Normal startup only matches a complete commissioned serial/configuration set;
  it never discovers an arm, cycles bitrates, rewrites IDs/registers, saves flash,
  or homes. Ambiguous/missing/mismatched identity is torque-disabled.
- J1--J7 feedback is output-shaft joint coordinate: never apply 9:1/10:1/40:1
  gearing again. Use one affine mapping `q=a*q_out+b`, `dq=a*dq_out`,
  `tau=tau_out/a`; require `abs(a)==1` for arm joints unless mechanical evidence
  approves otherwise. Torque is an estimate, never measured electrical current.
- Query and exactly match serial, hardware/software/sub-version, IDs, `Gr`,
  direction, control mode, bitrate, timeout and PMAX/VMAX/TMAX. Stage A supports
  only known codec profiles: any range mismatch refuses arming/decoding.
- A collision validator is required for physical motion and defaults to
  `RejectAll`. Position-only IK is not collision approval and leaves orientation
  free. TCP means named `openarm_{left,right}_hand_tcp` in `openarm_body_link0`.

## Stage A — fail-closed runtime and virtual execution

Deliver `libopenarm_control`, strict manifest/schema loader, fake transport and
motor simulator, deterministic lifecycle/trajectory tests, and `vcan` transport.
Physical SocketCAN may open only to verify disabled motors; physical planning and
execution return a gate status until Stage C. Keep destructive operations in a
separate `libopenarm_commission`/compiled CLI, never in normal runtime.

### Exact minimal Stage-A C ABI

All public records begin with `uint32_t struct_size, abi_version`; ABI version is
`OA_CONTROL_ABI_V1`. Inputs/outputs are caller-owned and fully validated before
write; handles are opaque; all units are SI; wrappers catch all C++ exceptions.

```c
typedef struct oa_manifest oa_manifest;
typedef struct oa_controller oa_controller;
typedef struct oa_motion_plan oa_motion_plan;
typedef uint32_t oa_status;
typedef uint32_t oa_side;
enum { OA_LEFT=0, OA_RIGHT=1 };
enum { OA_OK=0, OA_EINVAL=1, OA_EABI=2, OA_ESTATE=3, OA_ESTALE=4,
       OA_ETIMEOUT=5, OA_ECAN=6, OA_EFAULT=7, OA_EESTOP=8, OA_ELIMIT=9,
       OA_EIDENTITY=10, OA_EUNREACHABLE=11, OA_ECOLLISION=12,
       OA_EBUSY=13, OA_EIO=14, OA_ENOMEM=15, OA_EUNSUPPORTED=16 };
typedef struct { uint32_t struct_size, abi_version; uint64_t feedback_seq, t_ns;
  uint32_t expected_mask, fresh_mask, fault_mask; double q[7], dq[7], tau[7];
  double raw_q[7], raw_dq[7], raw_tau[7]; uint8_t status[7], mos_c[7], coil_c[7];
} oa_arm_snapshot;
typedef struct { uint32_t struct_size, abi_version; oa_arm_snapshot arm[2];
  uint64_t manifest_revision, model_revision, max_cross_bus_skew_ns;
} oa_snapshot;
typedef struct { uint32_t struct_size, abi_version; uint64_t expiry_ns,
  required_feedback_seq; oa_side side; uint32_t joint; double target_rad,
  velocity_scale, acceleration_scale, jerk_scale, position_tol_rad,
  velocity_tol_rad_s; } oa_joint_move;
typedef struct { uint32_t struct_size, abi_version; uint64_t expiry_ns,
  required_feedback_seq[2]; double left_tcp_m[3], right_tcp_m[3];
  double velocity_scale, acceleration_scale, jerk_scale, tcp_tol_m;
  uint64_t collision_scene_revision; } oa_paired_tcp_move;
typedef struct { uint32_t struct_size, abi_version; uint64_t start_ns, expiry_ns,
  producer_deadline_ns; uint32_t stop_kind; } oa_execute_request;
typedef struct { uint32_t struct_size, abi_version; uint32_t kind, lifecycle;
  uint64_t t_ns, command_id, feedback_seq; oa_status cause; } oa_event;
typedef struct { uint32_t struct_size, abi_version, backend; uint64_t cycle_ns; } oa_controller_options;
typedef struct { uint32_t struct_size, abi_version; uint64_t verify_epoch;
  uint32_t verified_mask, failure_mask; } oa_verify_report;
typedef struct { uint32_t struct_size, abi_version; uint64_t verify_epoch, nonce, expiry_ns; } oa_arm_challenge;
typedef struct { uint32_t struct_size, abi_version; uint64_t verify_epoch, nonce; } oa_reset_request;

oa_status oa_manifest_load(const char*, const char*, oa_manifest**);
void oa_manifest_destroy(oa_manifest*);
oa_status oa_controller_create(const oa_manifest*, const oa_controller_options*, oa_controller**);
oa_status oa_controller_open_and_verify(oa_controller*, oa_verify_report*);
oa_status oa_controller_snapshot(oa_controller*, oa_snapshot*);
oa_status oa_controller_arm(oa_controller*, const oa_arm_challenge*);
oa_status oa_controller_plan_joint(oa_controller*, const oa_joint_move*, oa_motion_plan**);
oa_status oa_controller_plan_paired_tcp(oa_controller*, const oa_paired_tcp_move*, oa_motion_plan**);
oa_status oa_controller_execute(oa_controller*, const oa_motion_plan*, const oa_execute_request*, uint64_t*);
oa_status oa_controller_stop(oa_controller*, uint32_t stop_kind);
oa_status oa_controller_disarm(oa_controller*, uint64_t deadline_ns);
oa_status oa_controller_reset_fault(oa_controller*, const oa_reset_request*);
oa_status oa_controller_poll_event(oa_controller*, uint64_t deadline_ns, oa_event*);
void oa_motion_plan_destroy(oa_motion_plan*); void oa_controller_destroy(oa_controller*);
```

Plans bind
manifest/model/limits/collision revisions, start snapshot sequence, target/path,
and expiry. Execute rejects any changed or stale dependency.

### Stage-A state, command and stop gates

Lifecycle: `CLOSED -> VERIFYING -> DISARMED -> ARMING -> ARMED_IDLE -> EXECUTING`;
`COMMISSIONING` is exclusive from `DISARMED`; normal stop is `STOPPING`; any
violation latches `FAULT`, and physical interlock latches `ESTOP`. Reset performs
full reverify and returns only to `DISARMED`.

Arming requires exact verified identity/configuration, fresh legal disabled
feedback for all motors, healthy buses, known motor timeout behavior, live
producer/watchdog, clear external E-stop/deadman, explicit expiring operator
challenge, operational limits, and an available collision validator. Any stale
feedback/producer/plan, CAN error/bus-off/link-down/overflow, deadline miss,
fault nibble, discontinuity, nonfinite data, limit/following/thermal breach,
identity drift, interlock loss, partial send, or excess bus skew latches fault;
default policy stops both arms. Repeated CAN disable is bounded best effort, not
an E-stop; installation hazard analysis selects hold/decelerate/disable/power-cut.

Every plan starts and immediately pre-execute rechecks a coherent fresh measured
snapshot. Single-joint motion commands all seven motors and holds six at measured
start pose. TCP planning uses measured seed then predecessor waypoints, rejects
residual/bounds/branch/singularity/collision failures, samples a bounded
joint trajectory, and verifies measured joint+dq dwell; TCP additionally needs
measured FK residual. Paired acceptance is all-or-none, but CAN execution is not
atomic: schedule a shared epoch, measure skew, and stop both on violation.

## Stage B — commissioning and calibration (separate compiled product)

Implement read-only link/identity verification, isolated-motor ID configuration,
and strict register readback; host link changes use netlink only after explicit
CLI confirmation. No normal-runtime write path. Manifest is strict JSON schema,
duplicate-key rejecting, atomic+`fsync`/rename, detached SHA-256; draft manifests
are never armable and runtime never edits one.

Manual calibration is torque-disabled, fixture/operator named-joint reference,
stable fresh sample dwell, staged software offset preview, atomic manifest patch,
reload+reprobe. One reference cannot establish sign: require a commissioned sign
or two distinct references. Firmware `FE` zero/save is rare, separately armed,
disabled, serial-challenged, previewed, confirmed, power-cycle verified and
documented; never boot-time and never an automatic fallback.

Hard-stop calibration is unavailable except qualified gripper-close until every
joint recipe is hardware-approved. Recipe binds serial/side/joint, fixture and
other-joint posture, direction/start corridor, speed, current/torque estimate,
travel/time/contact-energy/temperature ceilings, contact dwell, retreat,
repeatability, stop coordinate, deadman/E-stop. State machine:
`PRECHECK -> WAIT_INTERLOCK -> APPROACH -> CONTACT_DWELL -> RETREAT ->
REAPPROACH -> REPEATABILITY -> CANDIDATE -> REVIEW -> SOFTWARE_COMMIT`;
any failure goes `ABORTING -> DISABLED`, with no zero write/manifest commit.
Contact needs low measured velocity **and** torque evidence plus prior travel;
calibration cannot infer side, joint, TCP, safe stop, or collision geometry.

## Stage C — hardware qualification and enabled motion

Enable physical commands only after recorded acceptance, per installed firmware:
motor package/serial/version/FD capability/PVT ranges; timeout units and disable
latency; encoder persistence/wrap; side/sign/offset/effort sign; E-stop/deadman/
contactors and payload-drop stop policy; limits/gains/thermal behavior; actual
TCP/gripper curve/payload/cables; and a revisioned self/body/inter-arm/environment
collision scene with continuous-clearance argument.

Acceptance sequence is mandatory: (1) adapter + isolated unloaded motor, (2) one
mechanically constrained joint at reduced limits, (3) unloaded single arm in
fixture/exclusion zone, (4) single arm with payload, (5) both arms reduced
envelope, (6) full approved envelope. At each step test direction/zero/endpoints,
watchdog loss, bus-off, process kill, encoder loss, E-stop, temperatures/effort,
restart interlock, tracking/skew/utilization and logged pass criteria. Otherwise
remain `DISARMED`; paired TCP execution remains rejected.

## Source-unit dependency DAG and isolated-agent work

```text
openarm_can + openarm_model
  -> control/manifest + schema ----------------------> c_api
  -> control/register_codec --------------------------> bus_worker
  -> control/transport_iface -> fake_transport/sim | socketcan_transport(vcan)
  -> control/snapshot_mapping + limits + trajectory + kinematics_adapter
  -> collision_iface(default RejectAll) + motion_planner
  -> bus_worker + motion_planner + lifecycle/controller -> c_api -> CLI/adapters
commission/ (depends on manifest, register_codec, lifecycle; never control write API)
```

Parallel ownership after headers/interfaces freeze: Agent 1 `manifest/register_codec`;
Agent 2 `fake_transport/sim` and tests; Agent 3 `snapshot_mapping/limits/trajectory/
kinematics`; integrator `bus_worker/lifecycle/planner/c_api`. SocketCAN starts only
after `transport_iface`; commissioning starts only after manifest/register codec.
Each agent modifies disjoint source units and supplies unit tests; integrator alone
resolves API changes and adds cross-component tests.

## Test acceptance

- C11 ABI consumer; short/wrong records, invalid handles, canaries, exception and
  allocation containment, close/event overflow; no C++ exceptions/callbacks cross ABI.
- Parser fuzzing and manifest ambiguity/missing/duplicate IDs/serials/buses,
  mismatch PVT/mode/versions, enabled response and stale/duplicate feedback all
  refuse arm. Identical IDs on two virtual buses prove bus isolation.
- Codec golden vectors/status/RID validation/nonfinite/range rejection; `vcan`
  filters/timestamps/error frames/overflow/bus-off/link loss/shutdown/soak.
- Frozen encoder with advancing commands never completes; measured convergence
  does. Prove mapping derivative/power consistency, no double gearing, and wrap
  discontinuity faults.
- Exhaustive lifecycle/fault/producer expiry/partial-send/skew/stop-both tests;
  no constructor/open/probe/calibration creation emits enable or motion frames.
- Calibration tests cover unstable/unknown-sign/noisy/no-contact/overtravel/time/
  thermal/deadman/abort/repeatability and prove all failures emit no zero/save or
  manifest commit. Trajectories prove bounds, measured start drift/completion,
  collision reject, IK branch/singularity/revision change and paired failure.

Physical motion is accepted only by the Stage-C signed hardware records, never by
CI, simulation, a successful probe, or a plan endpoint.
