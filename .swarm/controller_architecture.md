# OpenArm 1.0 controller architecture

Date: 2026-07-29  
Scope: design only; no controller code or hardware state was changed.

## Outcome

The smallest credible controller is a stable, versioned C ABI implemented by
modular C++17 objects. It reuses the existing C11 `openarm_can` codec and
`openarm_model` FK/Jacobian/position-IK libraries. It has no Python runtime,
does not enable motors during construction or discovery, and treats fresh
DaMiao encoder feedback as the only source of actual joint position.

The design deliberately separates three products:

1. `libopenarm_control`: normal runtime, state, trajectories, safety lifecycle,
   FK/IK, virtual and physical transports. It cannot write motor configuration,
   flash zeros, or change host link settings.
2. `libopenarm_commission`: optional, separately built commissioning API for
   manual reference capture and supervised hard-stop calibration. Destructive
   motor-register persistence is a separately confirmed operation.
3. `openarmctl`: a foreground C++ CLI over the same C ABI. It never invokes
   `sudo` or a shell. It can emit a reviewable Bash script for privileged host
   CAN-link configuration.

The runtime can be implemented and exhaustively tested without arms. Physical
arming remains unavailable until the two arms, two CAN interfaces, physical
E-stop, and commissioned manifest are present and accepted.

## Evidence and constraints

The design is based on the repository's pinned sources:

- `openarm_can@c32ecd31da267967f0c913c2118c843177d88b91`
- `openarm_ros2@4e837e1d0dae692ff67b560b69d8d281d7a8d4ed`
- `openarm_description@6c7b720f1ba48e8bafa3a3dc752c45f397b42221`
- `openarm_teleop@eb2d49338bf70ace95282ea724903849397b7811`

Important facts:

- DaMiao state feedback contains position, velocity, torque estimate, two
  temperatures, and a status/fault nibble. OpenArm's ROS bridge consumes the
  reported position directly as joint position.
- The reported position is output-shaft encoder position. The integrated gear
  ratio must be verified but must not be multiplied into joint angle again.
- Commanded position and elapsed time are never measurements. State, trajectory
  progress, and target completion are calculated only from fresh feedback.
- Encoders do not identify the physical joint, arm side, URDF zero, sign, safe
  limits, or TCP. CAN IDs also do not prove those facts.
- J1/J2 use the DM8009 protocol family, J3/J4 use DM4340, and J5-J7/gripper use
  DM4310 in current upstream defaults. J3's physical `DM-J4340P` variant must be
  recorded separately from the generic protocol enum.
- PMAX, VMAX, and TMAX are motor registers. Hard-coded family defaults cannot be
  trusted after a register mismatch because they affect encoder decoding.
- Both standard arms reuse send IDs `0x01..0x08` and receive IDs
  `0x11..0x18`; they therefore require separate CAN buses.
- Upstream automatic calibration detects contact using encoder velocity and
  torque, but its precise-home function is unimplemented and it writes zeros
  without executing the calculated home move. It must not be ported verbatim.
- Current local IK is exact bounded position IK for `hand_tcp`; orientation is
  free and collision checking is explicitly absent.

## Proposed source layout

```text
control/
  include/
    openarm_control.h           stable normal-runtime C ABI
    openarm_commission.h        separately built commissioning C ABI
  src/
    c_api.cpp                   exception containment and handle validation
    controller.{hpp,cpp}        bimanual lifecycle and paired policy
    arm_controller.{hpp,cpp}    one arm, mappings, limits, motion state
    motor.{hpp,cpp}             immutable identity plus measured state
    bus_worker.{hpp,cpp}        sole owner of one transport
    transport.hpp               C++ ITransport interface
    socketcan_transport.cpp     Linux production transport
    fake_transport.cpp          deterministic unit-test transport
    vcan_motor_sim.cpp          protocol-level simulator
    trajectory.{hpp,cpp}        synchronized bounded interpolation
    kinematics_adapter.cpp      calls existing openarm_model C API
    manifest.{hpp,cpp}          strict immutable manifest loader
    calibration.{hpp,cpp}       session state machines
  cli/
    openarmctl.cpp              no Python, no shell execution
  schema/
    commissioning-v1.schema.json
    commissioning-v1.example.json
  tests/
    c_abi_test.c                compiled as C, not C++
    *_test.cpp
```

Dependency direction is one way:

```text
extern "C" ABI
     |
BimanualController
  +-- ArmController[2]
  |    +-- Motor[7] + optional Gripper
  |    +-- TrajectoryGenerator
  |    +-- KinematicsAdapter -> libopenarm_model
  +-- BusWorker[2]
       +-- ITransport -> SocketCAN / vcan / deterministic fake
       +-- DamiaoCodec -> libopenarm_can plus strict register codecs
```

ROS and RViz are adapters above this boundary. Neither belongs in the bus
worker or safety state machine.

## C ABI rules

The public headers are ISO C-compatible even though the implementation is C++:

- opaque handles only (`oa_controller`, `oa_motion_plan`,
  `oa_calibration_session`, `oa_manifest`);
- fixed-width integer status/type fields, not C/C++ enums or `bool`;
- every public record starts with `uint32_t struct_size` and
  `uint32_t abi_version`;
- fixed-size arrays for two arms and seven joints; SI units only;
- no STL type, exception, ownership-bearing pointer, or compiler-dependent
  layout crosses the ABI;
- C wrappers catch all exceptions and map them to a closed status code;
- no library printing, process exit, signal-handler installation, environment
  mutation, `sudo`, `system()`, or shell interpolation;
- output metadata is validated before the first write, following the existing
  CAN/model ABI convention;
- caller buffers are copied synchronously unless a function explicitly creates
  an opaque owned object;
- opaque objects have explicit `destroy`; destruction attempts best-effort
  disable but is not represented as a safety guarantee;
- state/events are polled or deadline-waited. User callbacks are not executed
  from the servo thread.

Suggested status family:

```c
typedef uint32_t oa_control_status;
#define OA_CONTROL_OK                    UINT32_C(0)
#define OA_CONTROL_EINVAL                UINT32_C(1)
#define OA_CONTROL_EABI                  UINT32_C(2)
#define OA_CONTROL_ESTATE                UINT32_C(3)
#define OA_CONTROL_ESTALE                UINT32_C(4)
#define OA_CONTROL_ETIMEOUT              UINT32_C(5)
#define OA_CONTROL_ECAN                  UINT32_C(6)
#define OA_CONTROL_EFAULT                UINT32_C(7)
#define OA_CONTROL_EESTOP                UINT32_C(8)
#define OA_CONTROL_ELIMIT                UINT32_C(9)
#define OA_CONTROL_EFOLLOWING            UINT32_C(10)
#define OA_CONTROL_EIDENTITY             UINT32_C(11)
#define OA_CONTROL_EUNREACHABLE          UINT32_C(12)
#define OA_CONTROL_ECOLLISION_UNCHECKED  UINT32_C(13)
#define OA_CONTROL_EBUSY                 UINT32_C(14)
#define OA_CONTROL_EIO                   UINT32_C(15)
#define OA_CONTROL_ENOMEM                UINT32_C(16)
#define OA_CONTROL_EUNSUPPORTED          UINT32_C(17)
```

Do not reuse the existing model and CAN numeric statuses implicitly. Translate
them at the control boundary and preserve the subordinate status in diagnostics.

## Principal C types

Names below define the intended surface, not final source code.

```c
typedef struct oa_manifest oa_manifest;
typedef struct oa_controller oa_controller;
typedef struct oa_motion_plan oa_motion_plan;
typedef struct oa_calibration_session oa_calibration_session;

typedef uint32_t oa_arm_side;
#define OA_ARM_LEFT  UINT32_C(0)
#define OA_ARM_RIGHT UINT32_C(1)

typedef uint32_t oa_controller_lifecycle;
#define OA_LIFECYCLE_CREATED   UINT32_C(0)
#define OA_LIFECYCLE_OPEN      UINT32_C(1)
#define OA_LIFECYCLE_PROBING   UINT32_C(2)
#define OA_LIFECYCLE_DISARMED  UINT32_C(3)
#define OA_LIFECYCLE_ARMING    UINT32_C(4)
#define OA_LIFECYCLE_ARMED     UINT32_C(5)
#define OA_LIFECYCLE_EXECUTING UINT32_C(6)
#define OA_LIFECYCLE_STOPPING  UINT32_C(7)
#define OA_LIFECYCLE_FAULT     UINT32_C(8)
#define OA_LIFECYCLE_ESTOP     UINT32_C(9)
#define OA_LIFECYCLE_CLOSED    UINT32_C(10)

typedef struct oa_joint_state {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t feedback_monotonic_ns;
    uint64_t feedback_sequence;
    double raw_motor_position_rad;
    double raw_motor_velocity_rad_s;
    double raw_motor_torque_nm;
    double joint_position_rad;
    double joint_velocity_rad_s;
    double joint_effort_nm;
    uint8_t status_nibble;
    uint8_t mos_temperature_c;
    uint8_t rotor_temperature_c;
    uint8_t fresh;
} oa_joint_state;

typedef struct oa_arm_state {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t cycle_sequence;
    uint64_t published_monotonic_ns;
    uint32_t expected_mask;
    uint32_t fresh_mask;
    uint32_t fault_mask;
    oa_joint_state joints[7];
    oa_joint_state gripper;
} oa_arm_state;
```

`raw_motor_*`, mapped `joint_*`, requested command, and current trajectory
reference are separate fields. A caller can therefore never mistake a setpoint
for encoder state.

A kinematic snapshot is derived from one immutable encoder snapshot:

```c
typedef struct oa_arm_kinematic_state {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t feedback_sequence;
    char body_frame[64];
    char tip_frame[64];
    double q_rad[7];
    oa_transform joint_pre[7];
    oa_transform link_post[7];
    double joint_origin_body_m[7][3];
    double joint_axis_body[7][3];
    oa_transform hand_tcp;
    double hand_tcp_body_m[3];
} oa_arm_kinematic_state;
```

This is how joint XYZ and claw/TCP XYZ are provided: exact FK from measured,
mapped encoder angles. Joint XYZ is not assigned or guessed.

## Transport injection and ownership

Each `BusWorker` is the only object allowed to touch one CAN file descriptor.
No API caller and no ROS executor performs socket I/O directly. The production
worker should use `poll`/`ppoll`, kernel receive timestamps, strict CAN filters,
CAN error frames, queue-overflow counters, and an interruptible shutdown fd.

The C ABI supports a custom transport binding for deterministic tests and
embedding. The vtable must include:

- monotonic `now`;
- deadline-bounded `send` returning the actual send timestamp;
- deadline-bounded `receive_event` returning either a CAN frame, CAN error,
  link-down, queue-overflow, or wake/stop event;
- health snapshot;
- interrupt/close;
- optional context destroy.

The transport context is called by exactly one bus worker, never concurrently.
The controller owns it only if the binding explicitly transfers ownership.
No transport callback may throw across the ABI.

There are three implementations:

- `SocketCanTransport`: real Linux transport, never selected implicitly from a
  name that did not exactly match the manifest;
- `VcanTransport`: same SocketCAN code against `vcan` for integration tests;
- `FakeTransport`: deterministic clock and event queue for unit tests.

The controller defaults to a virtual backend in developer tools. Opening a real
SocketCAN link requires an immutable physical manifest and explicit backend
selection.

## Commissioning manifest

Use strict JSON with a published JSON Schema. JSON is parsed by a pinned C++
library; no Python is involved. Unknown fields, duplicate keys, non-finite
numbers, missing required values, and values outside schema ranges are rejected.
The CLI writes files atomically (`fsync`, rename) and writes a detached SHA-256
over the exact bytes. The checksum detects accidental edits; it is not a digital
signature. Production permissions must prevent untrusted modification.

The parsed manifest becomes an immutable C++ object. The runtime never edits it.
Calibration produces a staged patch and a new manifest generation.

Minimum schema:

```json
{
  "schema": "org.openarm.commissioning/v1",
  "robot_id": "site-assigned-id",
  "generation": 1,
  "model": {
    "name": "openarm-v1.0-bimanual",
    "source_commit": "6c7b720f1ba48e8bafa3a3dc752c45f397b42221",
    "flattened_urdf_sha256": "...",
    "model_data_sha256": "...",
    "left_tip": "openarm_left_hand_tcp",
    "right_tip": "openarm_right_hand_tcp"
  },
  "safety": {
    "cycle_period_ns": 2000000,
    "feedback_timeout_ns": 10000000,
    "producer_timeout_ns": 100000000,
    "max_cross_bus_skew_ns": 2000000,
    "stop_both_on_one_arm_fault": true,
    "external_estop_required": true
  },
  "arms": [
    {
      "side": "left",
      "bus": {
        "interface": "can1",
        "adapter_serial": "operator-recorded-stable-id",
        "driver": "expected-driver",
        "fd": true,
        "bitrate": 1000000,
        "data_bitrate": 5000000
      },
      "joints": [
        {
          "index": 0,
          "urdf_name": "openarm_left_joint1",
          "physical_motor": "DM-J8009P-2EC",
          "protocol_motor": "DM8009",
          "serial": 0,
          "send_id": 1,
          "receive_id": 17,
          "registers": {
            "hw_version": 0,
            "sw_version": 0,
            "sub_version": 0,
            "gear_ratio": 9.0,
            "direction": 0.0,
            "pmax_rad": 12.5,
            "vmax_rad_s": 45.0,
            "tmax_nm": 54.0,
            "control_mode": 1,
            "can_bitrate_code": 0,
            "timeout_raw": 0,
            "timeout_unit_verified": false
          },
          "mapping": {
            "position_scale": 1.0,
            "position_offset_rad": 0.0,
            "velocity_scale": 1.0,
            "effort_scale": 1.0,
            "output_encoder_verified": false,
            "wrap_policy": "reject-discontinuity"
          },
          "operational_limits": {
            "position_min_rad": 0.0,
            "position_max_rad": 0.0,
            "velocity_max_rad_s": 0.0,
            "acceleration_max_rad_s2": 0.0,
            "jerk_max_rad_s3": 0.0,
            "effort_max_nm": 0.0,
            "following_error_max_rad": 0.0,
            "mos_temperature_max_c": 0,
            "rotor_temperature_max_c": 0
          },
          "control": {
            "kp": 0.0,
            "kd": 0.0
          },
          "calibration": {
            "method": "manual-known-pose",
            "raw_encoder_position_rad": 0.0,
            "reference_joint_position_rad": 0.0,
            "captured_monotonic_samples": 0,
            "operator_record": "",
            "acceptance_record_sha256": ""
          }
        }
      ]
    }
  ]
}
```

The example's zero values are placeholders that fail physical validation; the
schema/template must never make a draft manifest armable. A final manifest must
contain both arms, seven unique joints per arm, optional grippers, nonzero safe
limits, exact serial/register expectations, calibration evidence, and distinct
buses.

Runtime validation enforces:

- exact model/provenance hashes;
- exactly one left and one right arm;
- distinct CAN interfaces/adapters;
- unique IDs within a bus and standard 11-bit IDs;
- exact joint order/names matching `oa_model`;
- serial, hardware/software/sub-version, gear ratio, direction, PMAX/VMAX/TMAX,
  mode, bitrate, and timeout agreement;
- finite affine mappings and invertible nonzero position scale;
- operational limits contained in both URDF and commissioned mechanical limits;
- protocol limits containing every possible mapped command;
- no physical arming while `timeout_unit_verified`, output encoder, E-stop, or
  acceptance fields remain false/empty.

## Detection and configuration policy

### Normal startup

Normal startup does not perform broad discovery. It:

1. enumerates links read-only;
2. matches the exact manifest interface and stable adapter identity;
3. verifies link up/FD/bitrate/data bitrate;
4. opens with exclusive controller ownership;
5. sends refresh and register queries while torque remains disabled;
6. requires fresh disabled feedback from every expected motor;
7. compares all commissioned identity/configuration registers;
8. enters `DISARMED`, never `ARMED`.

Zero, finite initialized memory is never presented as motor state. State is
invalid until the first correctly identified, timestamped encoder frame.

### Commissioning discovery

Broad ID/baud discovery is an explicit commissioning operation. It may query
registers but never enables torque or writes parameters. It returns candidates,
not arm assignments. Ambiguous or duplicate responses fail closed.

Discovery can identify a previously commissioned motor by serial and registers.
It cannot determine a new motor's joint, side, sign, zero, or TCP. Assignment of
a new motor requires an isolated connection plus operator/fixture evidence.

### Host CAN configuration

`openarmctl interface render-script` validates an interface name and emits an
idempotent Bash script using literal, shell-quoted values. The user reviews and
runs it with `sudo`. The library never mutates link settings or elevates itself.
Afterward, `openarmctl probe` verifies settings read-only.

### Motor ID/register configuration

Motor register writes are available only in the separately built commissioning
library. The target motor must be physically isolated, disabled, identified by a
fresh serial query, and the operation must show a before/after diff. Persistence
requires a short-lived confirmation challenge containing target serial, register,
old/new value, and manifest generation. Flash writes are never performed in a
loop. No generic arbitrary-register write is exported by the normal runtime.

## Encoder-based mapping

For joint `i`:

```text
q_joint = position_scale * q_motor_encoder + position_offset_rad
dq_joint = velocity_scale * dq_motor_encoder
tau_joint = effort_scale * tau_motor_feedback
```

The inverse command mapping is:

```text
q_motor_request = (q_joint_request - position_offset_rad) / position_scale
dq_motor_request = dq_joint_request / velocity_scale
```

The ordinary state-feedback position is an output-shaft value. Integrated gear
ratio is verified as identity/configuration metadata; it is not applied again.
Effort sign/scale is independently commissioned because feedback torque is an
estimate and not measured electrical current.

Unknown multi-turn/wrap behavior must not be guessed. Until verified on the
installed firmware, the runtime rejects encoder discontinuities larger than the
maximum physically possible one-cycle motion plus quantization margin. It does
not silently unwrap by adding `2*pi`.

Each state carries timestamp, sequence, status, temperature, raw q/dq/tau, and
mapped q/dq/tau. FK, IK seed, limit checks, following error, and completion all
consume the same immutable state sequence.

## Controller state machines

Top-level lifecycle:

```text
CREATED -> OPEN -> PROBING -> DISARMED -> ARMING -> ARMED -> EXECUTING
                         ^         |          |         |        |
                         |         +----------+---------+--------+
                         |                    STOPPING
                         |
any state -- fault --------------------------> FAULT (latched)
any state -- physical/software E-stop -------> ESTOP (separately latched)
```

Rules:

- construction, opening, probing, ROS activation, and calibration object
  creation never enable motors;
- `ARMING` requires a fresh preflight generation/challenge, exact manifest hash,
  fresh disabled state for all expected motors, healthy buses, clear E-stop,
  live command producer, verified motor timeout, and explicit operator action;
- the arming challenge expires and is invalidated by any state/config change;
- `FAULT` and `ESTOP` are latched. Reset only returns to `DISARMED` after fresh
  disabled feedback and full re-verification; it never re-arms;
- one-arm fault stops both arms by default;
- shutdown and Ctrl+C request stop, send repeated disable frames on both buses,
  wait only to a bounded deadline, publish the result, then close transports;
- a destructor/signal handler is best-effort cleanup, not an E-stop;
- loss of torque may drop a payload, so the physical stop policy and external
  power interruption must be accepted for the actual application.

Fault triggers include stale state, stale command producer, command expiry,
wrong/missing/duplicate ID, status fault nibble, CAN error/bus-off/link down,
queue overflow, deadline miss, non-finite value, encoder discontinuity, identity
drift, q/dq/acceleration/jerk/effort/temperature violation, excessive following
error, invalid E-stop input, and cross-bus skew beyond its limit.

## Motion API

Representative public operations:

```c
oa_control_status oa_manifest_load(const char *path,
                                   const char *sha256_path,
                                   oa_manifest **out);
void oa_manifest_destroy(oa_manifest *manifest);

oa_control_status oa_controller_create(const oa_manifest *manifest,
                                       const oa_controller_options *options,
                                       oa_controller **out);
oa_control_status oa_controller_create_injected(
    const oa_manifest *manifest,
    const oa_transport_binding transports[2],
    const oa_controller_options *options,
    oa_controller **out);
oa_control_status oa_controller_open(oa_controller *controller);
oa_control_status oa_controller_probe(oa_controller *controller,
                                      oa_preflight_report *out);
oa_control_status oa_controller_get_arm_challenge(
    oa_controller *controller, oa_arm_challenge *out);
oa_control_status oa_controller_arm(oa_controller *controller,
                                    const oa_arm_challenge *challenge);
oa_control_status oa_controller_disarm(oa_controller *controller,
                                       uint64_t deadline_monotonic_ns);
oa_control_status oa_controller_reset_fault(oa_controller *controller,
                                            const oa_reset_request *request);
oa_control_status oa_controller_get_state(oa_controller *controller,
                                          oa_bimanual_state *out);
oa_control_status oa_controller_get_kinematics(
    oa_controller *controller, oa_arm_side side,
    uint64_t required_feedback_sequence,
    oa_arm_kinematic_state *out);
oa_control_status oa_controller_poll_event(oa_controller *controller,
                                           uint64_t deadline_monotonic_ns,
                                           oa_control_event *out);
void oa_controller_destroy(oa_controller *controller);
```

Motion is plan then execute:

```c
oa_control_status oa_controller_plan_joint_position(
    oa_controller *, const oa_joint_position_request *, oa_motion_plan **);
oa_control_status oa_controller_plan_arm_joints(
    oa_controller *, const oa_arm_joint_request *, oa_motion_plan **);
oa_control_status oa_controller_plan_tcp_position(
    oa_controller *, const oa_tcp_position_request *, oa_motion_plan **);
oa_control_status oa_controller_plan_paired_tcp_position(
    oa_controller *, const oa_paired_tcp_position_request *, oa_motion_plan **);
oa_control_status oa_motion_plan_get_report(const oa_motion_plan *,
                                            oa_motion_plan_report *);
oa_control_status oa_controller_execute_plan(
    oa_controller *, const oa_motion_plan *, const oa_execute_authorization *,
    uint64_t *out_command_id);
oa_control_status oa_controller_cancel(oa_controller *, uint64_t command_id,
                                       uint32_t stop_kind);
oa_control_status oa_controller_wait_command(
    oa_controller *, uint64_t command_id, uint64_t deadline_monotonic_ns,
    oa_command_result *out);
void oa_motion_plan_destroy(oa_motion_plan *);
```

Every request contains:

- request ID, issue time, absolute monotonic expiry, and required manifest/arm
  epoch;
- side(s), exact frame, and exact tip name;
- target(s) in SI units;
- speed/acceleration/jerk scaling no greater than manifest maxima;
- position and velocity completion tolerances;
- collision-policy requirement;
- maximum execution duration.

Only `world`, the exact body frame, and manifest-declared fixed frames are
accepted by the C core. Arbitrary ROS TF lookup stays in a ROS adapter.

An individual-joint move is still a complete seven-joint command: the selected
joint moves, while the other six are actively held at the fresh measured start
pose. It cannot omit motor frames or accidentally shift command indexing.

## XYZ/IK path

For each Cartesian request:

```text
fresh timestamped raw encoder feedback
  -> commissioned motor-to-joint affine mapping
  -> exact q_joint snapshot used as IK seed and posture reference
  -> exact pinned URDF bounded position IK
  -> reject failure, residual, bounds, or singularity policy violation
  -> sampled/approved collision policy
  -> synchronized bounded joint trajectory
  -> inverse mapping to motor position/velocity requests
  -> expiring MIT setpoints with manifest gains and zero torque feedforward
  -> verify every cycle from fresh feedback
  -> complete from measured joint tolerance and measured TCP FK tolerance
```

The API is named `tcp_position`, not generic pose IK. Current IK leaves
orientation free and returns achieved orientation. Its report includes both
underlying IK statuses, residuals, iterations, singularity metrics, active-limit
masks, achieved transforms, `collision_checked`, and the exact feedback sequence
used as seed.

Paired planning is all-or-nothing. Both arms use snapshots within the permitted
skew, both IK solves must succeed, and one immutable plan contains both
trajectories and a coordinated future start time. Physical CAN cannot be atomic;
the controller measures dispatch skew and stops both if it exceeds policy.

The existing model has no collision checker. A physical XYZ plan with
`collision_checked=0` is not executable and returns
`OA_CONTROL_ECOLLISION_UNCHECKED`. Virtual RViz motion can execute such a plan
while reporting that fact. A future planner/MoveIt adapter may return an approved
sampled plan, but it cannot bypass controller limits or freshness.

## Trajectory and feedback rules

- Start from the latest fresh measured encoder pose, never the last setpoint.
- Use a synchronized, fixed-capacity seventh-order time scaling with zero
  endpoint velocity/acceleration/jerk. Select duration analytically so every
  joint remains below commissioned q/dq/acceleration/jerk limits. A conventional
  jerk-limited S-curve is also acceptable if it has fewer moving parts and the
  same provable bounds.
- Sample and validate the entire trajectory before execution; no protocol-range
  saturation is allowed on a physical command.
- At each fixed-period cycle, generate q/dq reference, inverse-map it, encode
  with range-reject policy, send one complete set of motor commands, and consume
  identified fresh feedback.
- Normal position moves use commissioned MIT kp/kd and zero torque feedforward.
  The ordinary motion API does not expose arbitrary torque or gain overrides.
- Track commanded q, measured q, following error, command age, feedback age,
  temperature/status, send time, receive time, and bus skew separately.
- Excess following error for the configured consecutive-cycle threshold latches
  fault; it is not hidden by replanning from the lagging pose.
- Joint completion requires measured q and dq inside tolerance for N consecutive
  fresh cycles. Cartesian completion additionally requires measured FK TCP
  position within tolerance. Reaching the final setpoint time is not completion.
- Preemption is explicit. The new plan must start from a fresh measured state and
  pass all checks; otherwise the prior motion is stopped.
- Hardware command deadlines are monotonic and absolute. Wall-clock changes do
  not affect control.

The effective limit for each joint is the intersection of protocol capability,
URDF limit, commissioned mechanical/cable limit, and application operational
limit, minus margins. The tightest applicable velocity, acceleration, jerk,
effort, and temperature limit wins.

## Calibration APIs and state machines

Calibration is a session with exclusive controller ownership. General motion is
unavailable while it exists.

Representative C API:

```c
oa_control_status oa_calibration_begin_manual(
    oa_controller *, const oa_manual_calibration_request *,
    oa_calibration_session **);
oa_control_status oa_calibration_begin_hard_stop(
    oa_controller *, const oa_hard_stop_recipe *,
    const oa_commission_authorization *, oa_calibration_session **);
oa_control_status oa_calibration_poll(
    oa_calibration_session *, uint64_t deadline_monotonic_ns,
    oa_calibration_report *);
oa_control_status oa_calibration_capture_reference(
    oa_calibration_session *, const oa_reference_confirmation *);
oa_control_status oa_calibration_abort(oa_calibration_session *);
oa_control_status oa_calibration_export_patch(
    const oa_calibration_session *, oa_manifest_patch *);
void oa_calibration_destroy(oa_calibration_session *);
```

### Manual known-pose calibration

1. Require the selected motor to be disabled, correctly identified, fresh, and
   fault-free.
2. The operator/fixture places the named joint at a declared URDF coordinate.
3. Capture N consecutive encoder samples with bounded dq, spread, and timestamp
   age; reject motion, wrap discontinuity, or other-joint instability.
4. Compute `offset = q_joint_reference - scale*q_motor_mean`.
5. Preview raw samples, mean/spread, old/new mapping, resulting FK, and limit
   margins.
6. Store a staged software mapping with operator/fixture record. Do not send the
   DaMiao `set-zero` command.
7. A new process reloads and probes the staged manifest before acceptance.

The operator-supplied physical reference is indispensable. Encoder precision
does not create a known URDF zero by itself.

### Supervised automatic hard-stop calibration

There is no universal auto-calibration recipe. Each side/joint needs an approved,
hardware-validated recipe containing seek direction, maximum travel, velocity,
effort/torque-feedback ceiling, contact thresholds, minimum motion before
contact, consecutive confirmation cycles, expected stop-angle window, retreat
distance, stop-to-URDF-reference offset, time limit, hold policy for other
joints, and required fixture/payload state.

State machine:

```text
CREATED -> PRECHECK -> AWAIT_OPERATOR -> ARM_FOR_SEEK -> SEEK
  -> CONTACT_CONFIRM -> RETREAT -> MOVE_TO_REFERENCE -> VERIFY
  -> STAGED -> DISARMED

any failure/Ctrl+C/E-stop -> ABORTING -> repeated disable -> FAILED
```

Contact requires both low measured encoder velocity and sufficiently high
absolute torque estimate for consecutive fresh cycles, plus minimum prior
travel. Either signal alone is insufficient. Every state enforces travel, time,
temperature, feedback freshness, torque, and following-error bounds.

Crucially, after contact the controller retreats, moves to the calculated URDF
reference, and verifies it from encoder feedback before a mapping can be staged.
This corrects the incomplete upstream sequence. Failed/aborted calibration can
never emit a zero-write or accepted manifest patch.

Default persistence is a software offset. Writing the current motor position as
firmware zero is a different command in the optional commissioning library. It
requires disabled state, an exact serial challenge, preview, explicit
confirmation, a verified reference pose, then power-cycle/re-probe validation.

### What calibration cannot infer

- A safe hard-stop direction or contact energy.
- Which motor is physically attached to which joint/side.
- A CAD/URDF reference without a fixture, known stop offset, or external
  measurement.
- Installed TCP correction.
- Accurate gripper jaw displacement/force from motor angle alone.
- Collision clearance, payload behavior, or safe loss-of-torque behavior.

## CLI design

One C++ binary, no Python:

```text
openarmctl interfaces list
openarmctl interfaces inspect --interface can0
openarmctl interfaces render-script --interface can0 --fd \
    --bitrate 1000000 --data-bitrate 5000000 --output configure-can0.sh

openarmctl commission init --model v1.0-bimanual --output draft.json
openarmctl commission discover --interface can0 --disabled-only
openarmctl commission assign --manifest draft.json --side left --joint 1 \
    --isolated-motor --send-id 1 --receive-id 17
openarmctl commission verify-registers --manifest draft.json
openarmctl calibrate manual --manifest draft.json --side left --joint 1 \
    --known-joint-rad 0.0
openarmctl calibrate hard-stop --manifest draft.json --recipe approved.json \
    --side left --joint 1
openarmctl commission persist-zero --manifest staged.json --side left --joint 1

openarmctl probe --manifest commissioned.json
openarmctl status --manifest commissioned.json --watch
openarmctl simulate --manifest simulated.json
openarmctl control --manifest commissioned.json
```

`control` is a foreground interactive owner. Commands include `preflight`,
`arm`, `disarm`, `stop`, `move-joint`, `move-joints`, `plan-tcp`, `move-tcp`,
`plan-paired-tcp`, `move-paired-tcp`, `state`, and `kinematics`. Ctrl+C requests a
controlled stop and disable; a second Ctrl+C accelerates shutdown but cannot be
advertised as a physical safety function. Process exit does not leave an
unowned daemon controlling motors.

Normal application integration uses the C ABI directly. A daemon/network/ROS
bridge is a later adapter with authenticated authorization and producer expiry,
not part of the first safety core.

## Hardware-free implementation boundary

The following can be implemented and verified now without physical arms:

- complete versioned C headers and C++ opaque-handle implementation;
- strict manifest schema/loader, immutable validation, hashes, templates, and
  draft-vs-armable distinction;
- C++ OOP state machines, limit engine, watchdogs, events, and exception-safe C
  wrappers;
- extended strict register query/response/write codecs and dynamic
  PMAX/VMAX/TMAX handling;
- deterministic fake clock/transport and DaMiao motor simulator with real wire
  quantization, encoder state, registers, motor timeout, faults, and hard stops;
- Linux SocketCAN transport tested against `vcan` without actuators;
- manual and automatic calibration algorithms exercised against the simulator;
- encoder mapping, FK/joint XYZ/TCP XYZ, position IK seeded from simulated
  measured q, trajectories, individual joints, complete arm joints, and paired
  targets;
- virtual/RViz adapter using measured simulator state;
- no-Python unit, property, sanitizer, fuzz, ABI, and integration tests.

The following require hardware and cannot honestly be completed now:

- actual adapter identity and CAN/FD timing qualification;
- motor serial/model/firmware/register discovery and the J3/J4 physical-model
  reconciliation;
- actual joint/side/sign/zero assignment and encoder wrap behavior;
- manual reference capture or hard-stop recipe validation;
- motor TIMEOUT units and measured disable latency;
- physical E-stop integration and payload-drop behavior;
- operational q/dq/acceleration/jerk/effort/temperature limits;
- true gripper jaw mapping and installed TCP calibration;
- collision envelope, self/inter-arm/environment clearance;
- physical calibration, single-joint motion, arm motion, or bimanual motion.

## Test matrix

All new runtime and tests are C/C++. Existing Python model-generation tooling can
remain a provenance/build-time utility; no new controller function depends on
it.

### ABI and object lifetime

- Compile/link a consumer as strict C11 and a separate consumer as C++17.
- Every null, short `struct_size`, wrong version, invalid handle, invalid enum
  value, mis-sized output, and destroy-order case.
- Canary tests prove invalid outputs are not partially written.
- Exceptions, allocation failures, and transport errors never cross the C ABI.
- Concurrent state readers plus one command producer; reject unsupported
  multi-producer access deterministically.
- Repeated create/open/close/destroy and shutdown while every lifecycle state is
  active; LeakSanitizer clean.

### Manifest and detection

- Valid left/right manifests and exact model hashes.
- Missing/extra/duplicate keys, duplicate IDs/serials/joints, same bus for both
  arms, wrong names/order, hash mismatch, non-finite values, zero scales,
  overlong strings, path abuse, and malformed JSON.
- Property/fuzz tests for parser and validator.
- Draft manifests are never armable.
- Interface ambiguity, renamed interface, stable-adapter mismatch, wrong
  bitrate/FD/MTU/link state, extra/missing motors, duplicate responder,
  unexpected enabled motor, wrong serial/register, and stale reply.
- Broad discovery remains disabled-only and never emits enable/config frames.

### Codec and transport

- Preserve all existing MIT/feedback golden vectors and quantization properties.
- Add every typed register query/response/write vector, set-zero/save/clear-error
  vector, all status nibbles, every motor family range, and dynamic-range
  mismatch behavior.
- Reject NaN/Inf, wrong ID/DLC/flags/embedded ID/RID/type, truncation,
  endianness error, and physical-command saturation.
- Fuzz all frame and register decoders under ASan/UBSan.
- `vcan`: filters, kernel timestamps, receive overflow, reordered/late/duplicate
  frames, unknown IDs, error frames, interface-down, bus-off, socket close, and
  interruptible shutdown.
- Two vcan buses with identical motor IDs prove strict left/right isolation.

### Encoder state and mapping

- Raw-to-joint and joint-to-raw round trips for both signs and offsets.
- Output-shaft position is not gear-multiplied; gear mismatch blocks arming.
- Wire quantization bounds, timestamp/freshness, out-of-order sequence rejection,
  discontinuity rejection, invalid initial state, and immutable snapshot reads.
- Explicit test where last commanded q differs from encoder q: FK, IK seed,
  trajectory start, and completion must all use encoder q.
- Torque feedback remains labeled torque estimate, never electrical current.

### Lifecycle and safety

- Exhaustive legal/illegal transition table.
- No constructor/open/probe/ROS activation/calibration creation can emit enable
  or MIT frames.
- Arming requires every precondition and rejects expired/replayed challenges.
- Command producer expiry, feedback stale, wrong state, every status fault,
  thermal/effort/limit/following error, queue overflow, bus-off, identity drift,
  deadline miss, E-stop, and one-arm fault all latch and stop both.
- Repeated disable attempts, bounded shutdown, no automatic fault reset, no
  automatic re-arm, and restart begins disarmed.
- Inject process/Ctrl+C shutdown around every worker wait and every command
  phase; no deadlock or orphan thread.

### Calibration

- Manual capture with stable noisy encoder samples computes the exact affine
  offset; moving, stale, faulted, wrong-serial, high-spread, or discontinuous
  samples fail without a patch.
- Simulated hard-stop contacts for every direction with noise, backlash, delayed
  feedback, low-speed false positives, torque spikes, excessive contact torque,
  no contact, early contact, travel/time ceiling, and E-stop.
- Require consecutive velocity+torque contact evidence and expected stop window.
- Prove retreat, precise reference move, measured verification, then staging in
  that order.
- Every abort/failure path proves no set-zero/register-save frame was sent.
- Persistent zero requires isolated motor, disabled state, exact serial,
  nonreplayable challenge, and post-write power-cycle verification.

### Trajectory, joints, and XYZ

- Analytic/sample proofs of q/dq/acceleration/jerk bounds at endpoints and dense
  interior points for positive/negative/zero-distance moves.
- Multi-joint synchronization, preemption, command expiry, no range saturation,
  following-error fault, and measured-N-cycle completion.
- Single-joint move changes only its target; other six hold their measured start
  pose and all seven receive correctly indexed commands.
- FK/joint/TCP transforms match the existing model for mapped encoder q.
- IK seed equals the recorded fresh feedback sequence; stale seed invalidates
  the plan before execute.
- Reachable/unreachable, bounds, singular/near-singular, target expiry, invalid
  frame/tip, free-orientation reporting, and collision-unchecked rejection.
- Paired all-or-nothing solve, synchronized start, maximum bus skew, one-side
  failure, cancel, and stop-both behavior.
- Target completion is checked by measured FK TCP residual, not planned q or
  elapsed duration.

### Performance and soak

- Fixed-cycle load with allocation instrumentation proving no heap allocation in
  the servo step after initialization.
- Worst-case send/receive/compute latency and jitter at intended cycle rates;
  queue and bus-utilization margin, not average-only results.
- Multi-hour fake/vcan soak with delayed/dropped frames, event-ring wrap, command
  churn, and repeated arm/disarm; no leak, deadlock, stale success, or sequence
  regression.

### Future physical acceptance

Physical acceptance is separately authorized and never ordinary CI:

1. adapter and isolated unloaded motor;
2. one mechanically constrained joint at reduced limits;
3. one unloaded arm with fixture/exclusion zone;
4. one arm with intended payload;
5. both arms at reduced envelope;
6. only then full application envelope.

At each stage verify direction, zero, endpoints, encoder repeatability, watchdog
by command loss, bus-off, process kill, external E-stop, temperature/effort,
payload drop, restart interlock, and logged acceptance criteria. An independent
operator must hold the physical E-stop.

## Ranked risks

1. **Critical — wrong mapping:** an incorrect side/joint/sign/zero can command an
   immediate hard-stop collision. CAN discovery cannot solve it.
2. **Critical — no present hardware safety case:** no arms, adapters, accepted
   E-stop, timeout behavior, or payload-drop policy are available for testing.
3. **Critical — unsafe calibration reuse:** upstream's automatic script omits its
   calculated precise-home move before writing zero.
4. **High — decode range drift:** PMAX/VMAX/TMAX register differences change the
   meaning of encoder/velocity/torque bits.
5. **High — collision and orientation:** position-only XYZ IK leaves orientation
   free and currently checks no self, body, inter-arm, or environment collision.
6. **High — firmware uncertainty:** TIMEOUT units, FD support, bitrate codes,
   direction writes, wrap behavior, and fault recovery vary or are unresolved.
7. **High — duplicate IDs:** factory-default duplicate motors cannot be safely
   identified or configured while connected together.
8. **High — trajectory/stop energy:** encoder accuracy does not make hard-stop
   contact, gains, torque, speed, or payload behavior safe.
9. **Medium — gripper/TCP:** gripper motor angle is not a calibrated jaw
   displacement/force measurement, and the installed TCP is unknown.
10. **Medium — feedback semantics:** reported torque is an estimate, not a raw
    measured current; safety thresholds require physical qualification.

## Recommended implementation order

1. Extend pure codecs only for typed register operations and dynamic ranges.
2. Add manifest types/schema/strict validation and the C ABI skeleton.
3. Build fake clock/transport/motors and exhaustive lifecycle tests.
4. Implement encoder snapshots, mappings, limits, FK/IK adapter, and trajectories.
5. Add individual/full-arm/paired virtual motion and RViz state adapter.
6. Add manual and recipe-driven calibration state machines against simulation.
7. Add SocketCAN transport and vcan integration/soak tests.
8. Add the optional commissioning CLI and generated host-config script.
9. Stop at `DISARMED` until the physical commissioning inputs and E-stop are
   supplied; then perform staged hardware acceptance.

This sequence exposes useful, testable virtual C control early without making
unsupported claims about physical calibration or motion.
