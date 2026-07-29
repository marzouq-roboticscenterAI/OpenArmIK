# OpenArm 1.0 production C control/kinematics research

Research date: 2026-07-28. This is a read-only design investigation; no CAN interfaces or hardware were touched. Canonical repositories were inspected at these commits:

- `openarm_can` [`c32ecd31da267967f0c913c2118c843177d88b91`](https://github.com/enactic/openarm_can/tree/c32ecd31da267967f0c913c2118c843177d88b91)
- `openarm_description` [`6c7b720f1ba48e8bafa3a3dc752c45f397b42221`](https://github.com/enactic/openarm_description/tree/6c7b720f1ba48e8bafa3a3dc752c45f397b42221)
- `openarm_ros2` [`4e837e1d0dae692ff67b560b69d8d281d7a8d4ed`](https://github.com/enactic/openarm_ros2/tree/4e837e1d0dae692ff67b560b69d8d281d7a8d4ed)
- `openarm_teleop` [`eb2d49338bf70ace95282ea724903849397b7811`](https://github.com/enactic/openarm_teleop/tree/eb2d49338bf70ace95282ea724903849397b7811)

## Executive result

A production-quality C layer is feasible, but it must treat current upstream code as protocol/model evidence rather than as a safety-certified implementation. The safest first scope is Linux/SocketCAN, one physical CAN interface per arm, MIT mode only, explicit configuration, deterministic single-owner I/O, a numerical 7-DOF IK solver, and an optional ROS 2 adapter outside the real-time core.

Do not ship motion until commissioning resolves the raw motor-to-joint mapping for both sides and validates every firmware watchdog. Current upstream assumes identity mapping for arm joints, while its bimanual URDF introduces side-specific axes, origins, and joint-limit offsets. Current `openarm_can` also discards the feedback status/error nibble, has no frame freshness/sequence state, silently clamps commands at protocol bounds, and does not configure or verify the motor communication timeout. The current ROS hardware plugin initializes state to finite zero, does not prove that motors replied, returns to zero automatically on activation, and uses an explicitly approximate gripper mapping. Those are unacceptable as production safety boundaries.

## Exact CAN protocol and upstream API

The canonical low-level implementation is [the encoder/decoder](https://github.com/enactic/openarm_can/blob/c32ecd31da267967f0c913c2118c843177d88b91/src/openarm/damiao_motor/dm_motor_control.cpp), with constants in [`dm_motor_constants.hpp`](https://github.com/enactic/openarm_can/blob/c32ecd31da267967f0c913c2118c843177d88b91/include/openarm/damiao_motor/dm_motor_constants.hpp). The DaMiao manual linked by OpenArm independently defines the feedback layout and protection behavior: [DM-J4310-2EC V1.1 PDF](https://docs.openarm.dev/assets/files/dm4310-5d089e0b013d8e2fc203c5b670008a66.pdf). All IDs are 11-bit standard CAN IDs.

### Frames

| Operation | Arbitration ID | DLC/data |
|---|---:|---|
| Enable | motor ESC/send ID | `FF FF FF FF FF FF FF FC` |
| Disable | motor ESC/send ID | `FF FF FF FF FF FF FF FD` |
| Flash current position as zero | motor ESC/send ID | `FF FF FF FF FF FF FF FE` |
| MIT command | motor ESC/send ID | 8-byte packed `q16, dq12, kp12, kd12, tau12` |
| Position/velocity command | ESC ID + `0x100` | little-endian IEEE-754 float position, then float velocity |
| Velocity command | ESC ID + `0x200` | little-endian IEEE-754 float velocity, DLC 4 |
| Position/force command | ESC ID + `0x300` | float position; LE uint16 speed×100; LE uint16 per-unit current×10000 |
| Query register | `0x7FF` | `[ESC_ID_L, ESC_ID_H, 33, RID, 00, 00, 00, 00]` |
| Write register | `0x7FF` | `[ESC_ID_L, ESC_ID_H, 55, RID, LE value bytes 0..3]` |
| Refresh status | `0x7FF` | `[ESC_ID_L, ESC_ID_H, CC, 00, 00, 00, 00, 00]` |
| Feedback/state | configured Master/receive ID | byte 0 = `(status_or_fault << 4) | motor_id_low_nibble`; bytes 1–2 q16; bytes 3–4 dq12; bytes 4–5 tau12; byte 6 MOS °C; byte 7 rotor °C |

MIT maps linearly. `kp` is `[0,500]`, `kd` `[0,5]`; position, velocity, and torque use each declared motor type's symmetric protocol range. For v1.0 the canonical configuration is J1/J2 DM8009, J3/J4 DM4340, J5/J6/J7 DM4310, and gripper DM4310. Relevant codec ranges are:

| Joint/motor | protocol position | protocol velocity | protocol torque |
|---|---:|---:|---:|
| J1/J2 DM8009 | ±12.5 rad | ±45 rad/s | ±54 Nm |
| J3/J4 DM4340 | ±12.5 rad | ±10 rad/s | ±28 Nm |
| J5–J7, gripper DM4310 | ±12.5 rad | ±30 rad/s | ±10 Nm |

The encoder currently clamps out-of-range values. A production API should reject non-finite inputs, report saturation explicitly, and clamp again to the much smaller mechanical/commissioned limits. Quantization is unavoidable: one q LSB is `25/65535` rad, while dq/tau use 12 bits across their motor-specific spans.

The standard IDs are canonical and documented by OpenArm: J1..J7 use send IDs `0x01..0x07`, receive IDs `0x11..0x17`, and the gripper is `0x08/0x18`; see [Motor ID Configuration](https://docs.openarm.dev/api-reference/setup/motor-id/) and the [v1.0 ROS hardware defaults](https://github.com/enactic/openarm_ros2/blob/4e837e1d0dae692ff67b560b69d8d281d7a8d4ed/openarm_hardware/include/openarm_hardware/openarm_simple_hardware.hpp).

### Control modes and registers

`CTRL_MODE` is RID 10: MIT=1, POS_VEL=2, VEL=3, POS_FORCE=4. Other operationally important RIDs include Master ID 7, ESC ID 8, TIMEOUT 9, gear ratio 20, P/V/T maxima 21/22/23, CAN bitrate 35, and direction 55. Runtime mode changes are register writes, not ordinary control frames. Start with MIT mode only: it is the best evidenced path, supports smooth impedance/torque composition, and avoids mode-dependent arbitration IDs until every firmware variant is characterized.

The upstream high-level API is `OpenArm -> ArmComponent/GripperComponent -> DMDeviceCollection -> CANSocket`; see [official CAN documentation](https://docs.openarm.dev/1.0/software/can/) and [`OpenArm`](https://github.com/enactic/openarm_can/blob/c32ecd31da267967f0c913c2118c843177d88b91/src/openarm/can/socket/openarm.cpp). `recv_all()` waits for the first frame (default 500 µs) and then drains immediately available frames. Official guidance gives roughly 100 µs minimum, 500 µs normal control, and 1000–2000 µs for slow enable/query operations, and warns that an eight-motor loop above 1 kHz may be unstable.

### Feedback faults that must be implemented

The motor status nibble is 0 disabled, 1 enabled, 8 over-voltage, 9 under-voltage, A over-current, B MOS over-temperature, C rotor over-temperature, D lost communications, E overload. The vendor manual says these protections exit enabled mode. Current `openarm_can` ignores byte 0 completely when parsing state. It therefore cannot distinguish enabled, disabled, or faulted feedback. It also initializes every numeric state to zero and stores no receive timestamp or validity flag, so zero can mean either a real value or never received. Production decoding must validate arbitration ID, SFF/EFF/RTR/error flags, DLC, embedded motor ID, status nibble, finite/range plausibility, and arrival deadline, and expose a timestamped immutable snapshot plus per-motor freshness/error state.

## Bus setup and discovery feasibility

Current OpenArm guidance recommends CAN FD with nominal 1 Mbit/s, data 5 Mbit/s, BRS, sample points 0.75/0.75, DSJW 2, and bus auto-restart 100 ms; classic CAN is 1 Mbit/s. See [CAN setup](https://docs.openarm.dev/setup/openarm-setup/can-setup/) and [CLI reference](https://docs.openarm.dev/api-reference/can/cli/). The v1.0 BOM specifies a USB-to-CAN-FD converter ([electronics BOM](https://docs.openarm.dev/hardware/bill-of-materials/electrical/)). Nevertheless, the linked older motor manuals describe fixed classic CAN at 1 Mbit/s, while current tools support programmable FD data rates. Therefore probe and record the actual firmware and RID 35 configuration per installed motor; never infer FD capability from the arm version alone.

The C library should validate an already configured interface, not invoke privileged `ip link` changes from a control process. Validate link-up, CAN vs CAN-FD mode, nominal/data bitrate, restart policy, adapter MTU, termination/wiring by commissioning procedure, and bus-off/error counters. Install exact `CAN_RAW_FILTER`s and CAN error filters. Keep one owner thread/socket per bus, enable kernel timestamps if available, and treat short writes, queue overflow, error frames, `ENETDOWN`, `ENOBUFS`, and bus-off as faults.

Upstream now has an active `discover` CLI, implemented by scanning ESC IDs and baud settings and querying a register over ID `0x7FF`: [source](https://github.com/enactic/openarm_can/blob/c32ecd31da267967f0c913c2118c843177d88b91/setup/cli/commands/discover_motor_commands.cpp). It is useful only as a commissioning aid:

- It repeatedly reconfigures the host interface and tests send IDs, with receive candidates only `send+0x10` and `0x00`.
- It cannot discover an arbitrary Master ID, motor type, physical joint assignment, polarity, zero, gear setting, duplicate physical nodes with the same IDs, or whether a response came from the intended joint.
- Duplicate IDs can collide or be indistinguishable. Passive listening cannot enumerate quiet motors.
- Scanning should never occur while an arm is armed or another controller owns the bus. It changes host bus configuration and can disrupt all nodes, though the query itself is read-only.

Production startup should load an explicit signed/hashed commissioning manifest, probe exactly those IDs with torque disabled, read identity/configuration registers, reject missing/extra/conflicting feedback, and require an operator-confirmed arm/side serial association. Discovery must not silently rewrite IDs, baud, mode, direction, zero, or flash.

## Coordinate conventions, chain, and limits

Use the generated canonical v1.0 URDF for the exact selected configuration, not a hand-written DH approximation. Source files are [`openarm_arm.xacro`](https://github.com/enactic/openarm_description/blob/6c7b720f1ba48e8bafa3a3dc752c45f397b42221/assets/robot/openarm_v1.0/urdf/arm/openarm_arm.xacro), [kinematics](https://github.com/enactic/openarm_description/blob/6c7b720f1ba48e8bafa3a3dc752c45f397b42221/assets/robot/openarm_v1.0/config/arm/kinematics.yaml), [offsets](https://github.com/enactic/openarm_description/blob/6c7b720f1ba48e8bafa3a3dc752c45f397b42221/assets/robot/openarm_v1.0/config/arm/kinematics_offset.yaml), and [limits](https://github.com/enactic/openarm_description/blob/6c7b720f1ba48e8bafa3a3dc752c45f397b42221/assets/robot/openarm_v1.0/config/arm/joint_limits.yaml).

URDF convention is SI units and a child transform `T_parent_joint = Trans(xyz) * R_fixed(rpy)`, followed by rotation about the joint's axis. The v1.0 serial chain from link0 to link7 is:

| Joint | origin xyz m | bimanual fixed rpy | right axis | left axis |
|---|---|---|---|---|
| J1 | `(0,0,0.0625)` | `(0,0,0)` | `(0,0,1)` | `(0,0,1)` |
| J2 | `(-0.0301,0,0.0600)` | right `(+π/2,0,0)`, left `(-π/2,0,0)` | `(-1,0,0)` | `(-1,0,0)` |
| J3 | `(0.0301,0,0.06625)` | zero | `(0,0,1)` | same |
| J4 | `(0,0.0315,0.15375)` | zero | `(0,1,0)` | same |
| J5 | `(0,-0.0315,0.0955)` | zero | `(0,0,1)` | same |
| J6 | `(0.0375,0,0.1205)` | zero | `(1,0,0)` | same |
| J7 | `(-0.0375,0,0)` | zero | `(0,1,0)` | `(0,-1,0)` |

In the canonical bimanual body, the right arm base is `(0,-0.031,0.698)`, RPY `(+π/2,0,0)` and the left is `(0,+0.031,0.698)`, RPY `(-π/2,0,0)` relative to `openarm_body_link0`; see the [description docs](https://docs.openarm.dev/api-reference/description/). Single-arm and bimanual xacro paths do not generate identical intermediate frames, so the model API must identify the configuration and base/tip names explicitly. Do not mix a single-arm FK model with bimanual joint coordinates.

Base mechanical limits in the YAML are J1 `[-1.396263,3.490659]`, J2 `[-1.745329,1.745329]`, J3 `[-1.570796,1.570796]`, J4 `[0,2.443461]`, J5 `[-1.570796,1.570796]`, J6 `[-0.785398,0.785398]`, J7 `[-1.570796,1.570796]` rad. Velocities are 16.754666 for J1/J2, 5.445426 for J3/J4, and 20.943946 rad/s for J5–J7; efforts are 40, 40, 27, 27, 7, 7, 7 Nm. Those limits are far inside protocol ranges and still should receive commissioned safety margins and acceleration/jerk limits. The bimanual xacro reflects/offsets J1/J2 limits; generated values are left J1 `[-3.490659,1.396263]`, left J2 `[-3.316125,0.174533]`, right J1 `[-1.396263,3.490659]`, right J2 `[-0.174533,3.316125]`, with the other limits as above. This distinction must be frozen by golden generated-URDF tests.

The parallel gripper is modeled as a 0..0.044 m prismatic joint with a mimicked second finger. Hardware motion is approximately 0 rad closed to -1.0472 rad open; current ROS code applies a linear map and labels it approximate. OpenArm's gripper documentation says the rotor travels 60° and zero is fully closed: [gripper specification](https://docs.openarm.dev/hardware/specifications/gripper/). Calibrate the actual linkage curve if finger separation accuracy/force matters.

### Motor gearing, signs, and zero

Integrated motor reductions are DM4310 10:1, DM4340 40:1, and DM8009 9:1 according to the [OpenArm motor specification](https://docs.openarm.dev/hardware/openarm-2.0/motor/). OpenArm bolts joints directly to motor outputs, and all canonical arm software uses the reported motor output angle directly as the joint angle; the firmware's `Gr` register is the likely internal reduction conversion. Do not multiply by these ratios again unless commissioning proves the firmware is reporting rotor rather than output angle.

The runtime arm mapping in both [ROS hardware](https://github.com/enactic/openarm_ros2/blob/4e837e1d0dae692ff67b560b69d8d281d7a8d4ed/openarm_hardware/src/openarm_simple_hardware.cpp) and [teleop converter](https://github.com/enactic/openarm_teleop/blob/eb2d49338bf70ace95282ea724903849397b7811/src/joint_state_converter.hpp) is identity for position, velocity, and torque. The zero-calibration tool uses J1/J2 sign `-1` only while calculating motion toward stops and ultimately flashes motor zeros; it does not define a general runtime sign transform. Critically, its `move_to_precise_home()` is still `TODO`, yet the script later writes zero: [calibration source](https://github.com/enactic/openarm_can/blob/c32ecd31da267967f0c913c2118c843177d88b91/setup/openarm-can-zero-position-calibration). Thus the only defensible production representation is a per-joint affine manifest:

`q_joint = sign * scale * q_motor + offset`, `dq_joint = sign * scale * dq_motor`, `tau_joint = sign * tau_motor / scale` (with a separately validated torque convention).

Default `scale=1`, `offset=0` may mirror upstream but must not arm hardware until fixtures/measurements verify direction, endpoints, CAD zero, and torque sign for each joint on each side. Read and record RIDs `dir`, `Gr`, P/V/T maxima, IDs, mode, bitrate, timeout, firmware/hardware version, and serial number. Never use motion into a hard stop as routine startup discovery.

## Dual-arm and cycle concerns

Canonical bimanual defaults are right `can0`, left `can1`, because both arms reuse IDs 1..8. OpenArm teleoperation documentation explicitly states one arm per CAN port: [v1.0 setup](https://docs.openarm.dev/1.0/teleop/leader-follower/setup-guide/). Do not put two standard-configured arms on one bus.

Use two independent bus workers and timestamp snapshots with the same monotonic clock. A coordinator may issue paired commands with a shared execute epoch, but CAN does not provide atomic cross-bus delivery; specify and measure allowable skew. If either arm faults, the coordinated policy must define whether both stop. Keep collision checking and inter-arm trajectory coordination above the per-arm servo layer, while each bus independently enforces limits and watchdogs.

Bus load must be measured, not guessed. The current ROS plugin refreshes all motors in `read()` and then sends MIT commands in `write()`, while each operation elicits feedback. That can approach 32 frames per eight-motor arm per controller-manager update. Its configured 750 Hz rate is therefore not evidence of margin. In an active MIT loop, use command-triggered feedback as the state acquisition path; do not also refresh every motor unless required. Track expected/received masks, latency distribution, queue depth, dropped frames, and bus utilization, then cap the rate with at least a documented fault-time margin.

## Emergency stop, watchdog, and lifecycle semantics

The physical v1.0 system documentation describes E-stop buttons that remove power to paired arms: [Power & CAN](https://docs.openarm.dev/1.0/hardware/wiring-and-casing-guide/power-can/). This is the only evidenced independent stop path. A CAN `FD` disable command is useful but is not an emergency stop: it depends on the PC, adapter, bus, motor firmware, and power stage all functioning.

DaMiao feedback fault D means communications lost. Vendor documentation says TIMEOUT RID 9 is in 50 µs units and that loss of CAN commands exits enabled mode; see the upstream-linked [Seeed DaMiao parameter guide](https://wiki.seeedstudio.com/damiao_series/). Current OpenArm code exposes RID 9 but does not configure, verify, or service it intentionally. Commissioning must determine the actual stored timeout, whether zero disables protection, persistence, what frame classes reset it, fault-clear behavior, and worst-case disable latency on every firmware revision.

Recommended state machine: `CLOSED -> OPEN/PROBED -> DISARMED -> ARMING -> ARMED -> STOPPING -> DISARMED`, with any violation entering latched `FAULT`; physical E-stop enters separately latched `ESTOP`. Startup is torque-disabled and requires fresh valid state from every expected motor, matching manifest registers, healthy bus, legal pose, a live command producer, and explicit operator arm. Never auto-enable or auto-return-to-zero merely because a process starts. Reset requires the fault cause to disappear plus explicit acknowledgement; E-stop reset also requires the physical safety procedure.

Use three layers:

1. Independent physical power E-stop/contactors, with a documented gravity/load consequence and safety assessment.
2. Motor TIMEOUT configured and verified to disable on command loss.
3. Host monotonic watchdog: stale producer command, stale feedback, bus error, deadline miss, invalid telemetry, temperature/voltage/current/fault status, limit violation, or internal inconsistency latches fault and attempts repeated disable frames on all affected buses.

On healthy communications, a controlled deceleration/hold may be safer than immediate de-energization; on a hard fault or E-stop the system may lose holding torque and fall. That choice is application/payload dependent and cannot be inferred from upstream. Signal handlers/destructors are best-effort cleanup only, not safety mechanisms.

## Recommended C API architecture

Keep the core C11 ABI independent of ROS, C++, allocation policy, and logging. Suggested modules:

- `oa_can_codec`: pure frame encode/decode and register types; no file descriptors.
- `oa_bus`: SocketCAN ownership, filters/timestamps/error frames, one deterministic worker per interface.
- `oa_device`: motor manifest/probe, status/freshness, mode and enable state.
- `oa_arm`: seven joints plus optional gripper, mapping, limits, lifecycle, watchdog, synchronized `step`.
- `oa_model`: immutable generated v1.0 kinematic parameters, FK and Jacobian.
- `oa_ik`: bounded numerical solver using `oa_model`.
- `oa_ros2`: optional adapter process/library; never a dependency of the safety/control core.

Use opaque handles and versioned structs beginning with `struct_size` and `abi_version`. Return a closed status enum (`OA_OK`, `OA_EINVAL`, `OA_ESTATE`, `OA_ETIMEOUT`, `OA_ECAN`, `OA_ESTALE`, `OA_EFAULT`, `OA_ESTOP`, `OA_EUNREACHABLE`, ...); never print from the library. Every state snapshot should include monotonic timestamp, cycle number, expected/fresh masks, raw status nibble, temperatures, raw and mapped q/dq/tau, and bus diagnostics. Commands should include a monotonic expiry/deadline and mode. SI units only; define quaternion order, matrix storage, base/tip frames, and effort sign in the public header.

Illustrative surface:

```c
oa_status oa_bus_open(const oa_bus_config *, oa_bus **);
oa_status oa_arm_create(oa_bus *, const oa_arm_manifest *, oa_arm **);
oa_status oa_arm_probe(oa_arm *, oa_probe_report *);       /* torque remains disabled */
oa_status oa_arm_arm(oa_arm *, const oa_arm_token *);      /* explicit interlocked action */
oa_status oa_arm_step(oa_arm *, const oa_joint_command[7], oa_arm_state *);
oa_status oa_arm_stop(oa_arm *, oa_stop_kind);             /* controlled or immediate */
oa_status oa_arm_disarm(oa_arm *);
oa_status oa_model_fk(const oa_model *, const double q[7], oa_pose *);
oa_status oa_model_jacobian(const oa_model *, const double q[7], double J[42]);
oa_status oa_model_ik(const oa_model *, const oa_pose *, const oa_ik_options *, oa_ik_result *);
```

Allocate and lock required memory during initialization; perform no heap allocation, blocking name lookup, parameter writes, discovery, or logging in the servo path. Use fixed-size arrays and SPSC latest-command/latest-state buffers. The bus worker owns all socket operations. Separate read-only probing from configuration writes; flash writes and zeroing should be absent from the runtime API or require a distinct commissioning library/build and physical authorization.

## IK: analytic versus numerical

FK and the geometric Jacobian are straightforward products of the canonical URDF transforms. IK is intrinsically non-unique: seven revolute joints control a six-dimensional tool pose, leaving at least one redundancy degree away from singularities. A unique analytic API would have to expose or choose an elbow/swivel parameter and enumerate branches, then enforce asymmetric limits and configuration-specific mirrored frames. The small non-collinear offsets and lack of a validated motor/model transform make a rushed closed form especially risky.

Use numerical constrained IK initially. Upstream MoveIt independently chooses `kdl_kinematics_plugin/KDLKinematicsPlugin`, 0.005 search resolution and 5 ms timeout for each arm: [kinematics configuration](https://github.com/enactic/openarm_ros2/blob/4e837e1d0dae692ff67b560b69d8d281d7a8d4ed/openarm_bimanual_moveit_config/config/openarm_v1.0/kinematics.yaml). Implement damped least squares or trust-region SQP with a six-dimensional SE(3) residual, box limits, velocity-step bounds, adaptive damping near singularities, seed continuity, and a null-space objective for distance from limits/reference posture. Return achieved residual, iterations, singularity/limit flags, and never return a merely approximate solution as success. Multi-start may enumerate distinct branches for planning; the real-time servo should warm-start from the measured pose. Collision constraints belong in the planner, not the basic IK function.

An analytic solver is a later optimization only after symbolic/numerical cross-validation shows a genuine latency need. It must accept a redundancy parameter or posture policy, enumerate all branches, and be tested against the same FK; it is not a prerequisite for production-quality control.

## ROS 2 Lyrical and RViz integration

ROS 2 Lyrical Luth was released May 2026, is LTS to May 2031, and targets Ubuntu 26.04 Resolute on amd64/arm64: [release documentation](https://docs.ros.org/en/kilted/Releases/Release-Lyrical-Luth.html) and [Lyrical installation](https://docs.ros.org/en/lyrical/Installation/Alternatives/Ubuntu-Install-Binary.html). OpenArm's install/readme targets Ubuntu 22.04/24.04, and `openarm` is absent from the current Lyrical `rosdistro` distribution file. Therefore there is no evidenced upstream Lyrical binary-support promise; source-build and test all OpenArm repositories in a pinned Lyrical/Resolute environment.

Three integration levels are feasible:

1. **Visualization first (recommended):** build `openarm_description`, generate the exact v1.0 URDF, publish mapped `sensor_msgs/msg/JointState`, run `robot_state_publisher`, and display RViz2 `RobotModel`. This is independent of `ros2_control` and MoveIt and exercises names/TF/meshes safely. ROS documents this standard JointState -> robot_state_publisher -> RViz flow [here](https://docs.ros.org/en/ros2_documentation/lyrical/Tutorials/Intermediate/URDF/Building-a-Movable-Robot-Model-with-URDF.html).
2. **ros2_control adapter:** wrap the C arm layer in a Lyrical `hardware_interface::SystemInterface`. Port toward framework-managed state/command interfaces and implement lifecycle error/shutdown hooks, command-mode switching, hardware status, and stale/fault propagation. The current plugin still overrides deprecated ownership-transfer `export_*_interfaces`, exposes position/velocity/effort simultaneously without real mode switching, auto-enables/returns to zero on activation, and ignores read/write failures. It may compile with warnings in current Lyrical APIs, but that is not a compatibility or behavior test.
3. **MoveIt/RViz planning:** source-build the upstream bimanual MoveIt config and use its KDL solver only after xacro/SRDF/controller configs pass Lyrical tests. Keep MoveIt trajectories above the C safety layer; MoveIt/URDF limits do not protect hardware by themselves. Validate both fake-hardware and hardware-disabled paths before connecting a bus.

Do not run a ROS executor in the hard real-time bus thread. ROS nodes may publish diagnostics/state and enqueue expiring commands; the C core remains authoritative for freshness, limits, lifecycle, and stop policy.

## Unresolved hardware facts: mandatory commissioning gates

1. Actual motor model, hardware/firmware/sub-version, serial number, FD support, RID 35 bitrate, configured protocol P/V/T ranges, `Gr`, and `dir` at every joint.
2. Exact output-angle, velocity, and torque sign/scale for all seven joints on both sides; correspondence between raw motor zero and every generated URDF coordinate, especially bimanual J1/J2 offsets and mirrored J7.
3. Zero-calibration fixture/procedure, tolerance, persistence, power-cycle behavior, and correction for the calibration tool's unimplemented precise-home step.
4. Actual TIMEOUT value and units on each firmware, which frames refresh it, behavior at zero, persistence, disable latency, and recovery/clear-error semantics.
5. Whether command frames always trigger one feedback frame, timing/order under load, and behavior when two nodes share IDs.
6. Motor protection thresholds and safe continuous/peak torque, speed, temperature, current, acceleration and jerk for the assembled arm, wiring, supply, payload, duty cycle, and ambient conditions.
7. Physical E-stop/contactors' schematic, safety rating, measured interruption time, restart interlock, and arm behavior when torque disappears under payload. Current documentation is descriptive, not a safety validation.
8. Encoder absolute-position behavior across power cycles and multi-turn/wrap behavior relative to the ±12.5 rad protocol field.
9. True gripper motor-to-finger displacement/force mapping and tool-center-point for the installed end effector.
10. Assembly/cable hard stops and safe margins versus URDF limits; collision-mesh accuracy and inter-arm/self-collision envelope.
11. USB-CAN-FD adapter driver, oscillator/bit timing, hardware timestamps, queue capacity, termination, cable length, grounding, EMI, and bus-off recovery under the final installation.
12. Required pose/repeatability accuracy. OpenArm's FAQ currently says accuracy/repeatability documentation is still being prepared: [FAQ](https://docs.openarm.dev/faq/).

Any one of items 1–7 blocks unattended or human-adjacent production motion.

## Test and verification strategy

### Hardware-free CI

- Codec golden vectors for every command and feedback status, derived independently from the vendor tables and pinned upstream code. Test endpoints, midpoint, endianness, exact IDs/DLC, quantization error, NaN/Inf rejection, overflow, malformed flags/DLC, and explicit saturation.
- Property tests `decode(encode(x))` within half an LSB; fuzz every decoder and manifest/URDF input. Run with ASan/UBSan, static analysis, strict warnings, 32/64-bit and little-endian assumptions made explicit.
- `vcan` integration with an eight-motor simulator implementing registers, command-triggered feedback, TIMEOUT, enable/disable and every status fault. Inject missing, late, duplicate, reordered, unknown-ID, wrong embedded-ID, short, RTR/EFF/error frames, queue overflow, interface-down and bus-off events.
- Deterministic lifecycle/watchdog tests using a fake monotonic clock: no auto-arm, stale command and stale state latch faults, repeat-disable behavior, no automatic reset, both-arm stop policy, and restart/power-cycle state.
- Dual-arm tests on two independent virtual buses with identical node IDs; verify isolation, timestamp skew accounting, partial-arm failure, and no cross-dispatch.
- Load tests at increasing cycle rates with recorded worst-case latency, expected/fresh masks, CPU scheduling interference and bounded memory. Model actual frame counts; prove margin rather than relying on average Hz.

### Kinematics/IK/model CI

- Generate and archive canonical single/right/left/bimanual URDFs from the pinned xacro. Run xacro and URDF validators, assert joint order/names, base transforms, axes and all effective limits.
- Golden FK at zero and selected axis-only poses; independently compare the C FK/Jacobian against KDL or another URDF implementation. Finite-difference every Jacobian column over random in-limit poses.
- Mapping round trips and endpoint tests for each commissioned side: motor -> joint -> motor for q/dq/tau, with explicit torque power-consistency checks.
- IK property tests: sample legal q, compute FK, solve from nearby and remote seeds, verify FK(solution) residual and limits. Include singular/near-singular poses, all limit faces, unreachable position/orientation, wrap-adjacent states, redundant solutions, deterministic iteration/time caps, and continuity along Cartesian paths.
- Collision/planning tests separately cover self, body, other arm and environment; basic IK success never implies a collision-free command.

### ROS 2 Lyrical CI

- Pin a clean Ubuntu 26.04/Lyrical container or VM; source-build `openarm_can`, description, adapter, MoveIt config and RViz dependencies. Do not mix binaries from Humble/Jazzy/Kilted.
- Launch fake-hardware visualization and assert `/joint_states`, `/robot_description`, complete TF trees, unique dual-arm names and mesh resolution. Exercise RViz headlessly where practical.
- Run `ros2_control` hardware/component tests with the virtual backend, lifecycle transitions, controller switching, command expiry, diagnostics, and fault propagation. Ensure RViz/MoveIt cannot bypass the C safety envelope.

### Staged hardware acceptance (future, supervised)

Proceed only after the facts above are documented: isolated motor with no load; one joint mechanically constrained; one arm at reduced torque/speed; then dual arm. At each stage verify direction, zero, endpoints, torque sign, watchdog by deliberate command loss, bus-off, process kill and power E-stop, thermal/current behavior, payload drop behavior, and restart interlocks. Use a fixture, exclusion zone, independent E-stop observer and logged acceptance results. No autonomous hard-stop calibration is an acceptable substitute for this commissioning record.

