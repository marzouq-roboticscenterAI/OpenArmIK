# Physical controller Phase-1 reconnaissance

Date: 2026-07-29  
Scope: read-only analysis only. No CAN socket was opened, no interface was reconfigured, and no motor command was transmitted.

## Bottom line

The requested controller can expose a stable C ABI while using modular C++ objects internally. The existing `can/` library is a good pure-C codec and read-only diagnostics foundation, and `model/` already supplies the exact generated OpenArm v1.0 FK/Jacobian/position-IK implementation. A new physical runtime must add a real SocketCAN transport, register verification, per-motor encoder state, commissioned motor-to-joint mappings, lifecycle/watchdogs, trajectory generation, and calibration sessions.

The DaMiao position feedback is a real encoder-derived output-shaft angle; it must be used as the controller state and IK seed. It is not acceptable to estimate joint angles from elapsed time, command values, or graphical pose. However, the encoders cannot by themselves tell software which physical joint a motor is attached to, the arm side, the CAD/URDF zero, the sign convention, or the installed TCP. Those facts require a commissioning manifest and a known physical reference.

Upstream's automatic hard-stop calibration script is not safe to port verbatim. It detects stops from measured velocity and torque, but its `move_to_precise_home()` is unimplemented and the computed absolute home is never executed before all motor zeros are written (`upstream/openarm_can/setup/openarm-can-zero-position-calibration:135-147,377-403`). Any automatic calibration must be a separately armed, operator-supervised commissioning state machine with bounded travel/time/torque and a physical E-stop. Normal startup should use the retained absolute encoder state and verify the commissioned configuration; it should not re-home against hard stops.

## Audited source baseline

The pinned sources and commits are recorded in `UPSTREAM_SOURCES.md:1-25`:

- `enactic/openarm_can@c32ecd31da267967f0c913c2118c843177d88b91`
- `enactic/openarm_ros2@4e837e1d0dae692ff67b560b69d8d281d7a8d4ed`
- `enactic/openarm_description@6c7b720f1ba48e8bafa3a3dc752c45f397b42221`
- `enactic/openarm@990fda921c82ae9d12b00f23e449793a9a313afd`

The local C codec is explicitly based on that pinned CAN evidence (`can/include/openarm_can.h:1-6`). The existing run ledger correctly limits it to read-only physical discovery and an in-memory fake (`.swarm/ledger.md:8-14`).

## Installed actuator map and encoder facts

The OpenArm v1.0 bimanual BOM distinguishes the actual packages as follows (`upstream/openarm/website/src/components/ActuatorsTable.tsx:26-50`):

| Joint | Physical motor | Protocol enum used upstream | Integrated reduction |
|---|---|---|---:|
| J1, J2 | DM-J8009P-2EC | `DM8009` | 9:1 |
| J3 | DM-J4340P-2EC | `DM4340` | 40:1 family |
| J4 | DM-J4340-2EC | `DM4340` | 40:1 |
| J5, J6, J7 | DM-J4310-2EC V1.1 | `DM4310` | 10:1 |
| J8 gripper | DM-J4310-2EC V1.1 | `DM4310` | 10:1 |

The protocol library intentionally does not distinguish the `P` mechanical variant (`upstream/openarm_can/include/openarm/damiao_motor/dm_motor_constants.hpp:22-37`). The motor specification gives reductions 10:1, 40:1, and 9:1 and identifies two 14-bit single-turn magnetic encoders per actuator (`upstream/openarm/website/versioned_docs/version-1.0/hardware/specifications/motor.mdx:20-40`). The locally archived DM4310, DM4340, and DM8009 manuals likewise say the output shaft has single-turn absolute position that is retained across power loss, and that CAN reports position, velocity, torque, and temperatures (the PDFs under `upstream/openarm/website/static/file/hardware/specification/motor/`, pp. 2, 6, and final specification page).

All official OpenArm runtime code consumes the reported output position directly as the arm joint angle. The ROS hardware bridge copies motor q/dq/tau into joint q/dq/effort without another gearbox multiplication (`upstream/openarm_ros2/openarm_hardware/src/openarm_simple_hardware.cpp:253-265`). Therefore the controller must not multiply encoder angles by 9/10/40 again. It must query and record the motor `Gr` register and verify the physical motor family during commissioning.

The encoder facts do **not** establish the URDF zero. They establish a precise angle relative to the motor's stored zero/firmware mapping. The gripper is the one documented simple reference: closed is zero and the rotor traverses 60 degrees to fully open (`upstream/openarm/website/versioned_docs/version-1.0/hardware/specifications/gripper.mdx:12-33`). Even there, the ROS prismatic linkage conversion is explicitly approximate (`upstream/openarm_ros2/openarm_hardware/src/openarm_simple_hardware.cpp:267-279`).

## Exact DaMiao CAN protocol found in the pinned sources

All OpenArm IDs fit 11-bit standard CAN. Standard v1.0 motor assignment is J1..J7 send IDs `0x01..0x07`, receive/master IDs `0x11..0x17`, and gripper `0x08/0x18` (`upstream/openarm/website/versioned_docs/version-1.0/software/setup/1-motor-id.mdx:10-23`).

### Commands

| Operation | Arbitration ID | DLC | Bytes / units | Evidence |
|---|---:|---:|---|---|
| Enable | ESC/send ID | 8 | `FF FF FF FF FF FF FF FC` | `upstream/openarm_can/src/openarm/damiao_motor/dm_motor_control.cpp:25-28,214-216` |
| Disable | ESC/send ID | 8 | `FF FF FF FF FF FF FF FD` | same file `:30-32,214-216` |
| Set current position as motor zero | ESC/send ID | 8 | `FF FF FF FF FF FF FF FE` | same file `:34-36,214-216` |
| Clear error | ESC/send ID | 8 | `FF FF FF FF FF FF FF FB` | `upstream/openarm_can/setup/cli/commands/clear_error_commands.cpp:45-56` |
| MIT impedance/torque | ESC/send ID | 8 | packed q16, dq12, kp12, kd12, tau12 | `upstream/openarm_can/src/openarm/damiao_motor/dm_motor_control.cpp:133-152` |
| Position + velocity | ESC ID + `0x100` | 8 | little-endian float32 q rad, then float32 dq rad/s | same file `:43-47,155-165` |
| Velocity | ESC ID + `0x200` | 4 | little-endian float32 dq rad/s | same file `:50-56,167-174` |
| Position + force/current limit | ESC ID + `0x300` | 8 | float32 q rad; LE u16 speed rad/s x100; LE u16 current per-unit x10000 | same file `:59-64,176-200` |
| Query register | `0x7FF` | 8 | `[ESC_L, ESC_H, 33, RID, 00, 00, 00, 00]` | same file `:66-68,203-212` |
| Write register | `0x7FF` | 8 | `[ESC_L, ESC_H, 55, RID, value LE32]` | `upstream/openarm_can/include/openarm/damiao_motor/dm_motor_control.hpp:111-147` |
| Refresh state | `0x7FF` | 8 | `[ESC_L, ESC_H, CC, 00, 00, 00, 00, 00]` | `upstream/openarm_can/src/openarm/damiao_motor/dm_motor_control.cpp:75-85` |
| Save RAM parameters to flash | `0x7FF` | 8 | `[ESC_L, ESC_H, AA, 00, 00, 00, 00, 00]` after disable | `upstream/openarm_can/setup/cli/commands/write_motor_param_commands.cpp:86-111` |

MIT layout is exactly:

```text
D0=q[15:8] D1=q[7:0]
D2=dq[11:4]
D3=dq[3:0]<<4 | kp[11:8]
D4=kp[7:0]
D5=kd[11:4]
D6=kd[3:0]<<4 | tau[11:8]
D7=tau[7:0]
```

The current local C encoder implements and golden-tests that layout (`can/src/openarm_can.c:131-185`, `can/tests/test_openarm_can.c:58-99,101-132`). It rejects non-finite values and can reject rather than silently clamp protocol ranges (`can/src/openarm_can.c:87-99,148-168`).

### State feedback

State feedback uses the configured Master/receive arbitration ID and eight bytes:

```text
D0 = (status_or_fault << 4) | (ESC_ID low nibble)
D1:D2 = position unsigned 16-bit, big-endian
D3 + high nibble D4 = velocity unsigned 12-bit
low nibble D4 + D5 = torque unsigned 12-bit
D6 = MOS temperature in deg C
D7 = rotor/coil temperature in deg C
```

This is decoded locally with arbitration-ID, DLC, embedded-ID, and status validation (`can/src/openarm_can.c:193-229`). The pinned upstream decoder confirms the bit packing and physical conversion (`upstream/openarm_can/src/openarm/damiao_motor/dm_motor_control.cpp:88-110`). The archived motor manuals give the same table on feedback page 6.

Status/fault nibble meanings are defined in `can/include/openarm_can.h:47-56`:

| Nibble | Meaning |
|---:|---|
| `0x0` | disabled |
| `0x1` | enabled |
| `0x8` | over-voltage |
| `0x9` | under-voltage |
| `0xA` | over-current |
| `0xB` | MOS over-temperature |
| `0xC` | rotor/coil over-temperature |
| `0xD` | lost communications |
| `0xE` | overload |

The physical manuals say the thermal, voltage, current, and communication-loss protections exit enabled mode (final specification page of each archived DM4310/4340/8009 PDF). Software must still latch and surface the original fault; a subsequent disabled state is not evidence the prior fault never occurred.

The feedback `T` field is a torque estimate/mapped torque, not a raw measured bus-current field. MIT torque becomes a current-loop request (`upstream/openarm/website/static/file/hardware/specification/motor/dm4310.pdf`, control discussion before p. 6), while position-force mode separately expresses an explicit per-unit current limit (`upstream/openarm_can/include/openarm/damiao_motor/dm_motor_control.hpp:68-72`). Regular feedback exposes no amperes measurement. Do not label `T` as measured electrical current.

### Ranges and quantization

The pinned default protocol ranges are in `upstream/openarm_can/include/openarm/damiao_motor/dm_motor_constants.hpp:90-112` and the local checked codec at `can/src/openarm_can.c:101-128`:

| Motor | q | dq | tau | q wire step | dq wire step | tau wire step |
|---|---:|---:|---:|---:|---:|---:|
| DM8009 | +/-12.5 rad | +/-45 rad/s | +/-54 Nm | 25/65535 rad | 90/4095 rad/s | 108/4095 Nm |
| DM4340 | +/-12.5 rad | +/-10 rad/s | +/-28 Nm | 25/65535 rad | 20/4095 rad/s | 56/4095 Nm |
| DM4310 | +/-12.5 rad | +/-30 rad/s | +/-10 Nm | 25/65535 rad | 60/4095 rad/s | 20/4095 Nm |

`kp` maps `[0,500]` over 12 bits and `kd` maps `[0,5]` over 12 bits (`upstream/openarm_can/src/openarm/damiao_motor/dm_motor_control.cpp:133-152`). The q wire field is 16 bits, but the hardware source is documented as a 14-bit encoder; wire width must not be advertised as 16-bit physical accuracy.

PMAX, VMAX, and TMAX are writable motor registers, so a family enum alone is not sufficient to decode arbitrary installed firmware configuration. The runtime must query these values and either require exact agreement with the commissioned manifest or use validated queried values in the codec. It must never silently decode with default ranges after a mismatch.

### Relevant configuration registers

The complete pinned enum is `upstream/openarm_can/include/openarm/damiao_motor/dm_motor_constants.hpp:41-88`. Commissioning-relevant entries are:

- 0 under-voltage threshold, 1 torque constant, 2 over-temperature threshold, 3 over-current threshold.
- 4 acceleration, 5 deceleration, 6 maximum speed.
- 7 Master/receive ID, 8 ESC/send ID, 9 CAN command timeout, 10 control mode.
- 13 hardware version, 14 software version, 15 serial number, 16 pole-pair count.
- 20 gear ratio, 21 PMAX, 22 VMAX, 23 TMAX.
- 29 over-voltage threshold, 30 gear torque efficiency.
- 35 CAN bitrate, 36 firmware sub-version.
- 54 mechanical offset, 55 direction, 80 motor position, 81 output-shaft position.

Control mode values are MIT=1, POS_VEL=2, VEL=3, POS_FORCE=4 (`upstream/openarm_can/include/openarm/damiao_motor/dm_motor_constants.hpp:39`). Parameter response data uses byte 2 `0x33` or `0x55`, byte 3 RID, and LE32 integer or IEEE float data in bytes 4..7 (`upstream/openarm_can/src/openarm/damiao_motor/dm_motor_control.cpp:113-130,242-258`).

Three upstream inconsistencies must be resolved against the installed firmware before writes:

1. The CLI describes `dir` as read-only float but its generic writer treats it as writable integer (`upstream/openarm_can/setup/cli/commands/motor_read_param_commands.cpp:79-86`; `write_motor_param_commands.cpp:49-63`).
2. CAN bitrate metadata says `[0,9]`, while CLI maps codes 10 and 11 as 8 and 10 Mbit/s (`motor_read_param_commands.cpp:77,96-99`).
3. The archived motor PDFs describe fixed classic CAN at 1 Mbit/s, while current OpenArm tools use configurable CAN-FD/BRS. Installed hardware/firmware must be queried and bench-qualified; arm version is not proof of FD capability.

The local pinned sources establish that RID 9 is a uint32 CAN timeout and that loss of commands disables the motor, but they do not establish the timeout unit or whether zero disables protection (`motor_read_param_commands.cpp:49-52`; archived motor manuals, final specification page). The earlier research report mentions a 50 microsecond unit from an external vendor guide, but that unit is not independently present in the pinned local evidence. Treat it as unverified until checked against the exact installed hardware/software version.

## What detection and configuration can safely do

### Read-only auto-detection

The existing local C API can enumerate Linux CAN interfaces and report link state, MTU, nominal/data bitrate, and FD flag through read-only rtnetlink (`can/include/openarm_can.h:199-210,239-244`; `can/src/openarm_can_linux.c:232-315`). It can probe a **known manifest** by sending only refresh requests and requiring fresh disabled feedback from each expected ID (`can/src/openarm_can.c:298-387`). This is the correct normal-startup model.

Upstream's broader discovery utility is only a commissioning aid. It mutates the host interface through multiple bitrates, scans ESC IDs 1..N, tries only receive IDs `send+0x10` and `0x00`, and detects a response by querying Master ID (`upstream/openarm_can/setup/cli/commands/discover_motor_commands.cpp:38-48,80-95,97-164`). It cannot discover:

- an arbitrary receive ID outside those two candidates;
- two physical motors sharing the same arbitration IDs;
- motor type/variant with safety-grade certainty;
- physical joint, arm side, direction, CAD zero, or TCP;
- a quiet motor passively.

Discovery must run torque-disabled with exclusive bus ownership. It must not be part of ordinary controller startup and must not silently change motor registers.

### ID and bitrate configuration

The standard ID layout may be offered as a commissioning template, not inferred truth. Upstream changes ESC ID first through register 8, then addresses the new ESC ID to change Master ID through register 7; optional persistence disables the motor and sends save command `0xAA` (`upstream/openarm_can/setup/cli/commands/change_motor_id_commands.cpp:34-67,69-112`).

If multiple factory-default motors share an ID, they cannot be individually configured on the same live bus. Each motor must be isolated or connected one at a time. Duplicate responders can make discovery ambiguous or corrupt frames. Flash writes are limited to roughly 10,000 cycles and must never occur in a control loop (`change_motor_id_commands.cpp:69-78`; motor configuration guide `upstream/openarm/website/versioned_docs/version-1.0/software/setup/4-motor-config.mdx:24-44`).

Host interface configuration requires privilege. Keep it out of the control library: provide a separate C++ commissioning CLI or a generated Bash script for the operator to run with sudo, then have the runtime verify the resulting interface read-only. Do not copy upstream's `system("sudo ip ...")` design or interpolate unsanitized interface names (`upstream/openarm_can/setup/cli/commands/can_configure_commands.cpp:24-80`).

## Calibration capability matrix

| Requested capability | What can be implemented | What the encoders cannot supply |
|---|---|---|
| Encoder state | Exact reported motor output q/dq/tau/temp/status with timestamp and freshness | Ground-truth metrology accuracy beyond encoder/gear/backlash limits |
| Manual zero | Operator/fixture places a named joint at a known URDF pose; controller verifies stable disabled feedback, records software offset, optionally sends explicit `FE` after confirmation | The known physical reference itself |
| Startup validation | Match bus, IDs, serial, versions, gear, direction, limits, mode, bitrate, timeout, and plausible q against manifest | Physical joint identity if the original manifest was wrong |
| Automatic hard-stop zero | One supervised joint at a time, measured q/dq/tau, bounded low-energy motion, explicit stop threshold, travel/time ceiling, retreat, move to calculated reference, verify, then optional zero | Whether collision/contact is safe, whether stop geometry is undamaged, CAD/TCP metrology |
| Gripper zero | Supervised low-current close to the documented closed stop, then verify 0 and calibrated opening curve | Accurate jaw separation/force without linkage calibration |
| TCP calibration | Can consume externally measured points and solve/store TCP correction | Cannot be derived from motor encoders alone |
| Auto arm/joint assignment | Can compare serials to a previously commissioned manifest | Cannot infer side/joint safely from CAN ID or motion without operator/fixture evidence |

The archived setup guide explicitly requires the operator to place the arm roughly in the shown zero pose and warns that calibration moves automatically, one arm at a time, with an E-stop (`upstream/openarm/website/versioned_docs/version-1.0/software/setup/4-motor-config.mdx:62-95`).

Preferred policy:

1. Store an affine mapping per joint: `q_joint = scale*q_motor + offset`, `dq_joint = scale*dq_motor`, with separately verified effort sign.
2. Use software offsets by default to avoid flash wear and make rollback/audit possible.
3. Offer motor `set zero` only as an explicit persistent commissioning action after a preview and confirmation.
4. Once commissioned, rely on retained absolute encoder feedback and do not bump hard stops on every boot.

## Current local APIs and gaps

### Useful foundations already present

- `oa_can_encode_mit`, strict feedback decode, enable/disable/refresh/register-query codecs, versioned records, and explicit motor mappings: `can/include/openarm_can.h:65-237`.
- Read-only CAN-interface enumeration and a fake transport that rejects control: `can/include/openarm_can.h:239-256`.
- Exact bimanual FK, all pre-joint/post-link transforms, body-frame joint axes, hand TCP, Jacobian, URDF limits, and bounded position IK: `model/include/openarm_model.h:20-109`.
- IK explicitly reports achieved TCP/residual and explicitly does not collision-check: `model/include/openarm_model.h:71-83`; `model/README.md:13-17`.

### Required additions before physical motion

- Physical SocketCAN transport with kernel timestamps, filters, CAN error frames, queue-overflow detection, and one owner thread per interface.
- Register-write/save/set-zero/clear-error and parameter-response codecs with strict target/RID/type correlation.
- Dynamic or manifest-verified PMAX/VMAX/TMAX decode ranges.
- Immutable timestamped motor snapshots with raw status, freshness, serial/config, raw motor q and mapped joint q.
- Motor/arm/bimanual lifecycle, explicit arming, command expiry, host watchdog, verified motor timeout, repeated disable-on-fault, and latched reset.
- Mechanical joint/rate/acceleration/jerk/temperature/torque limits below all callers.
- Trajectory interpolation and measured-feedback completion tests; never send a large IK result as one instantaneous setpoint.
- Calibration session objects and a durable, checksummed commissioning manifest.
- Collision-aware planning or, at minimum, refusal to represent position-only IK as collision safe.

The pinned upstream C++ stack is a useful class-layout reference (`OpenArm -> ArmComponent/GripperComponent -> DMDeviceCollection -> DMCANDevice -> CANSocket`, documented at `upstream/openarm/website/versioned_docs/version-1.0/software/can.mdx:27-60`), but it is not a safety boundary. Specific gaps include:

- Motor state starts as finite zero with no timestamp/valid bit (`upstream/openarm_can/src/openarm/damiao_motor/dm_motor.cpp:23-32`).
- `is_enabled()` is never updated anywhere after construction (`dm_motor.cpp:34-35`; repository search has no caller).
- Upstream state decode discards byte-0 status/fault (`dm_motor_control.cpp:88-110`).
- `mit_control_all()` does not require one command per registered motor (`dm_motor_device_collection.cpp:116-131`).
- Default destructor only closes the socket; it does not prove disable (`include/openarm/can/socket/openarm.hpp:26-30`).
- ROS activation enables every motor and automatically returns to zero (`upstream/openarm_ros2/openarm_hardware/src/openarm_simple_hardware.cpp:223-235`), which must not be copied.
- ROS returns success on every read/write without freshness/fault enforcement (`openarm_simple_hardware.cpp:253-303`).

## Recommended modular OOP implementation behind a C ABI

Pure C has no language-level classes. Satisfy both requirements by keeping the public API C-compatible and implementing its opaque handles with C++17 classes. No Python is needed for the runtime or tests.

```text
extern "C" versioned C ABI
  oa_controller_handle / oa_arm_handle / oa_calibration_handle
            |
Controller (owns lifecycle, watchdog, paired policy)
  +-- ArmController left/right
  |     +-- Motor[7] + optional Gripper
  |     +-- CalibrationSession
  |     +-- TrajectoryGenerator
  |     +-- KinematicsAdapter -> existing oa_model C library
  +-- BusWorker per SocketCAN interface
        +-- ITransport -> SocketCanTransport / FakeTransport / VcanTransport
        +-- DamiaoCodec -> existing/extensible pure codec
```

Suggested class responsibilities:

- `DamiaoCodec`: stateless frame/register encode/decode only.
- `ITransport`: injected monotonic send/receive interface; real and deterministic fake implementations.
- `BusWorker`: sole file-descriptor owner; timestamping, filters, error frames, expected/fresh masks.
- `Motor`: immutable commissioned identity/config plus atomically published measured state.
- `ArmController`: motor-to-joint transforms, limits, enable state, joint commands, synchronized step.
- `TrajectoryGenerator`: velocity/acceleration/jerk-limited joint interpolation.
- `KinematicsAdapter`: calls `oa_fk`/`oa_ik_position_v2`; exposes every joint XYZ and named `hand_tcp` XYZ.
- `CalibrationSession`: explicit manual-known-pose and supervised end-stop workflows; no general controller commands while active.
- `BimanualController`: two independent bus workers, paired target transaction, maximum cross-bus skew and stop-both policy.

All C functions should be exception-safe wrappers: catch every exception, never let C++ unwind across the ABI, return fixed-width status codes, and expose error text through caller buffers. Every public record should retain the repository's current `struct_size`/`abi_version` convention. No library function should print, invoke sudo, call a shell, flash firmware, or auto-enable.

Important public operations:

- enumerate/verify interfaces;
- create controller from a checksummed manifest;
- read-only discover/probe and read parameter snapshot;
- begin/step/commit/abort manual or automatic calibration;
- arm/disarm/reset-fault with explicit precondition reports;
- get raw encoder state, mapped joint state, joint transforms/XYZ, and TCP transform/XYZ;
- move one named joint to a bounded target;
- submit a full joint target/trajectory;
- submit left/right TCP XYZ targets atomically, returning IK diagnostics before execution;
- poll immutable state/events and wait with a deadline.

For an XYZ command, the correct closed-loop path is:

```text
fresh measured encoder q_motor
 -> commissioned q_joint mapping
 -> use q_joint as IK seed
 -> exact URDF bounded IK
 -> reject residual/bounds/singularity/collision-policy failures
 -> time-parameterize joint path
 -> inverse mapping q_joint target to q_motor command
 -> issue expiring MIT setpoints
 -> verify every cycle against fresh measured q/dq/tau/status
 -> complete only from measured TCP FK and tolerance
```

This directly answers the concern about guessed joint angles: commanded positions are requests; measured encoder positions are state and determine progress/completion.

## Safety and watchdog constraints

The motor's communication-loss protection is necessary but insufficient. The physical safety guide requires secured mounting, an exclusion zone, PPE, and an immediately accessible E-stop, and warns that de-energizing can drop the load (`upstream/openarm/website/versioned_docs/version-1.0/getting-started/safety-guide.mdx:21-75`).

Required controller state machine:

```text
CLOSED -> OPEN -> PROBED -> DISARMED -> ARMING -> ARMED -> STOPPING
              \-----------------------> FAULT (latched)
physical E-stop -----------------------> ESTOP (separately latched)
```

Arming must require all expected serial/config registers, fresh disabled feedback, legal measured pose, healthy bus, live command producer, verified timeout, clear E-stop, and explicit operator action. Any stale feedback/command, fault nibble, bus-off/error, deadline miss, non-finite value, limit/rate violation, excessive temperature/effort, or identity/config drift must latch fault and attempt disable. Ctrl+C/destructors are best-effort cleanup, never the safety mechanism.

Normal motion testing cannot be claimed until hardware is connected and the commissioning gates are complete. The upstream docs themselves mark the physical ROS bridge as unstable, especially gripper logic (`upstream/openarm/website/versioned_docs/version-1.0/software/ros2/control.mdx:50-58`).

## No-Python verification plan

The new runtime can be tested entirely in C/C++:

1. Pure codec golden vectors for every command, register type, motor range, status, malformed ID/DLC/flag, and quantization boundary.
2. C++ fake DaMiao motor objects with deterministic encoder dynamics, register memory, status faults, dropped/delayed/duplicate frames, timeout disable, and hard stops.
3. C ABI tests compiled as C, proving opaque-handle lifetime, ABI versions, buffer bounds, and exception containment.
4. `vcan` integration for SocketCAN filters, timestamps, shutdown, queue overflow/error paths, and two independent buses; interface setup may be a separate operator-run Bash script.
5. Joint mapping/manual calibration/auto-calibration state-machine tests, including abort/timeout/travel/torque ceilings and proof that failed calibration cannot write zero.
6. FK/IK/trajectory tests seeded from simulated encoder feedback; completion must be based on measured FK, not the target command.
7. Process kill, Ctrl+C, producer expiry, feedback loss, bus-off, thermal/electrical fault, and one-arm failure tests proving the paired stop policy.
8. Only after all above: supervised isolated-motor, single-joint, single-arm, then bimanual hardware acceptance with a physical E-stop observer.

## Ranked risks

1. **Critical:** wrong joint/side/sign/zero mapping can drive immediately into a stop. CAN discovery cannot solve this.
2. **Critical:** current local code has no physical transport/lifecycle/watchdog; upstream ROS auto-enables and is not reusable unchanged.
3. **Critical:** copying the upstream hard-stop calibration would write zeros without moving to its computed precise home.
4. **High:** configured PMAX/VMAX/TMAX may differ from hardcoded family ranges, corrupting decoded encoder/torque units.
5. **High:** motor timeout units/zero behavior and FD capability are firmware-specific and unverified in pinned local evidence.
6. **High:** position-only IK has free orientation and no collision checking; a numerically valid target is not automatically a safe path.
7. **High:** duplicate/default IDs cannot be safely auto-configured with multiple motors connected.
8. **Medium:** gripper motor-angle to jaw-distance/force and installed TCP remain calibration problems, not encoder-only facts.

