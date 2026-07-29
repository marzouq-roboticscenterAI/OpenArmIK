# Independent hardware/protocol verification

Scope: read-only verification of the pinned `upstream/openarm_can`, `upstream/openarm_ros2`, and `upstream/openarm` trees plus DaMiao's official motor manuals/protocol manual. No CAN frames were transmitted. Pinned commits inspected:

- `openarm_can` `c32ecd31da267967f0c913c2118c843177d88b91`
- `openarm_ros2` `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed`
- `openarm` `990fda921c82ae9d12b00f23e449793a9a313afd`

The external primary reference was DaMiao's official `dmBots/damiao-document` protocol manual, commit `5e5a6932b8f013636415174c7433b9685ba24aa2`, file `调试助手使用说明书（达妙驱动控制协议）V1.4.pdf`. Page references below are to its printed page numbers. The motor-specific PDFs are pinned under `upstream/openarm/website/static/file/hardware/specification/motor/`.

## Bottom line

The protocol facts needed for a controller are mostly recoverable, but several tempting assumptions are false:

1. Feedback is not IEEE float. Position is unsigned 16-bit and velocity/torque are unsigned 12-bit fields linearly mapped through the motor's configured `PMAX/VMAX/TMAX`; the physical units are rad, rad/s, and Nm. Temperatures are direct one-byte °C readings. The configured mapping spans must match the decoder. The pinned library instead hard-codes spans by enum motor type.
2. The hard-coded `tMax` values are encoding spans, not safe actuator/joint torque limits. They exceed the OpenArm-listed peak torque for DM4310 and DM8009.
3. Feedback byte 0 contains motor ID plus enable/fault status, but the pinned library discards it. `is_enabled()` is never updated. ROS therefore has neither fault state nor trustworthy enable state, feedback freshness, live temperature, or current.
4. The geared actuators have dual encoders and a **single-turn absolute output-shaft** measurement that survives power loss. That does not reveal the assembled robot's kinematic zero or distinguish revolutions. The official first-time OpenArm calibration obtains a reference by deliberately contacting mechanical stops and is explicitly warned to move automatically; no safe, contact-free, reference-free home is observable.
5. Arm motor feedback/commands are passed 1:1 between DaMiao output coordinates and ROS joint coordinates. No gear division or per-joint sign conversion exists in the ROS hardware layer. This is consistent with output-shaft feedback, but correct signs remain a motor configuration/assembly/calibration precondition, not something the controller discovers.
6. `0xFE` is the DaMiao **save position zero** command: it makes the current output position zero. It does not find home. The exact flash endurance/atomicity and explicit across-power guarantee for the saved offset are not stated in the pinned material, although DaMiao calls the operation “save” and separately guarantees power-loss-resistant output absolute position.
7. ID/baud discovery is not universally passive: it takes the host interface down/up at multiple bitrates and sends addressed parameter queries. It only tries Master IDs `ESC_ID+0x10` and `0`. ID reassignment is dangerous when multiple factory-default motors share the current ID because all matching motors receive the same write.

## 1. Exact feedback encoding and units

### Confirmed wire layout

For every control mode the state feedback frame has the same eight-byte layout:

| Field | Wire bits | Decode |
|---|---|---|
| motor/status | `D[0]` | low nibble motor ID, high nibble `ERR/status` for the IDs OpenArm uses |
| position | `D[1]:D[2]` | `u16` big-endian, linearly mapped from `[0,65535]` to `[-PMAX,+PMAX]` rad |
| velocity | `D[3]` and high nibble of `D[4]` | `u12`, mapped to `[-VMAX,+VMAX]` rad/s |
| torque | low nibble of `D[4]` and `D[5]` | `u12`, mapped to `[-TMAX,+TMAX]` Nm |
| MOS temperature | `D[6]` | direct byte, °C |
| rotor/coil temperature | `D[7]` | direct byte, °C |

Evidence:

- The pinned decoder extracts exactly those bit fields at `upstream/openarm_can/src/openarm/damiao_motor/dm_motor_control.cpp:96-108` and implements `x = u/(2^bits-1) * (max-min) + min` at `:236-240`.
- The DaMiao protocol manual defines the same layout, status meanings, °C temperatures, field widths, and a worked 12-bit `VMAX` example on pp. 32-33. The pinned DM4310 manual repeats it on pp. 6-7 (`upstream/openarm/website/static/file/hardware/specification/motor/dm4310.pdf`); DM4340 and DM8009 do likewise.
- The C++ API documents the mapping units as rad, rad/s, Nm at `upstream/openarm_can/include/openarm/damiao_motor/dm_motor_constants.hpp:90-95`.
- The official protocol says `PMAX/VMAX/TMAX` are the maximum feedback mapping values in rad, rad/s, and Nm (protocol manual p. 40). The pinned CLI also exposes them as writable mapping values at `upstream/openarm_can/setup/cli/commands/motor_read_param_commands.cpp:63-65`.

The precise decode formula used by the pinned library is:

```text
q   = q_u   / 65535 * (2*PMAX) - PMAX
dq  = dq_u  / 4095  * (2*VMAX) - VMAX
tau = tau_u / 4095  * (2*TMAX) - TMAX
```

This is an offset-binary mapping, not two's-complement despite the manuals' loose wording “signed fixed point.” Exact encoded midpoint is quantized: zero does not have a perfectly symmetric integer representation.

### Critical scaling precondition

The motor's configured mapping spans are authoritative. DaMiao explicitly says that `PMAX/VMAX/TMAX` configure MIT command scaling and feedback scaling (official protocol manual p. 10 and p. 40). The pinned library never reads those registers before decoding; it selects hard-coded values by `MotorType` at `upstream/openarm_can/include/openarm/damiao_motor/dm_motor_constants.hpp:97-112` and uses them at `upstream/openarm_can/src/openarm/damiao_motor/dm_motor_control.cpp:104-108`.

Therefore decoded q/dq/tau are physically correct **only if each drive is configured with the same spans as `MOTOR_LIMIT_PARAMS` and the correct motor enum was registered**. A controller must verify those three registers during commissioning (or decode from queried values), not merely infer them from the motor label.

The CLI `monitor` violates that precondition for a full OpenArm: it registers every requested ID as `DM4310` (`upstream/openarm_can/setup/cli/commands/monitor_motor_status_commands.cpp:79-86`), while the physical OpenArm uses DM8009 on J1/J2, DM4340 on J3/J4, and DM4310 on J5-J7 (`upstream/openarm_ros2/openarm_hardware/include/openarm_hardware/openarm_simple_hardware.hpp:82-96`). Thus its J1-J4 q/dq/tau display can be incorrectly scaled even though the dashboard labels them rad/rad/s/Nm at `monitor_motor_status_commands.cpp:119-133`.

### Mapping span is not a safety limit

`MOTOR_LIMIT_PARAMS` calls `tMax` a torque limit, but its values are 10 Nm for DM4310 and 54 Nm for DM8009 (`dm_motor_constants.hpp:99-111`). OpenArm's motor table lists peak torque 7 Nm for DM4310 and 40 Nm for DM8009 (`upstream/openarm/website/docs/hardware/openarm-2.0/motor.mdx:23-34`). Therefore these constants cannot be treated as continuous, peak, structural, or collision-safe joint limits. They are protocol encoding/clipping spans. The same caution applies to `pMax` and `vMax`.

No live phase/bus current is present in the eight-byte feedback. `T` is torque feedback. Over-current threshold is a configurable percentage (`motor_read_param_commands.cpp:44-45`; official protocol manual p. 10), and POS_FORCE's `i` is a per-unit current limit in `[0,1]`, not amperes (`upstream/openarm_can/include/openarm/damiao_motor/dm_motor_control.hpp:68-72`, packing at `dm_motor_control.cpp:180-200`). The Python example contradicts the implementation by labeling it amperes at `upstream/openarm_can/python/examples/test_posforce.py:32-35`.

## 2. MIT versus POS_VEL

### MIT mode

`ControlMode::MIT` is register value 1 (`upstream/openarm_can/include/openarm/damiao_motor/dm_motor_constants.hpp:39`). Its standard-ID control frame is sent at `ESC_ID`, with:

- 16-bit mapped `q`/`p_des` in rad;
- 12-bit mapped signed `dq`/`v_des` in rad/s;
- 12-bit `kp` in `[0,500]`;
- 12-bit `kd` in `[0,5]`;
- 12-bit mapped `tau`/`t_ff` in Nm.

Packing is exact at `upstream/openarm_can/src/openarm/damiao_motor/dm_motor_control.cpp:133-152`. Values are clipped to the library's hard-coded spans before quantization at `:219-227`. DaMiao says MIT can act as velocity control (`kp=0`, nonzero `kd`) or torque control (`kp=kd=0`, `t_ff`), and warns that position control with `kd=0` can oscillate or lose control (official protocol manual pp. 30-31; pinned DM4310 PDF p. 5).

### POS_VEL mode

`ControlMode::POS_VEL` is register value 2. The command arbitration ID is `ESC_ID + 0x100`, and payload is two native little-endian IEEE-754 `float32`s: desired position in rad followed by **maximum absolute travel speed** in rad/s (`upstream/openarm_can/src/openarm/damiao_motor/dm_motor_control.cpp:43-47,155-165`; official protocol manual pp. 30, 35). `v_des` is not a signed simultaneous velocity target as in MIT; it bounds the trapezoidal profile's cruise speed. The drive's ACC/DEC and position/velocity loops govern the trajectory.

### Mode-selection trap

Changing `CTRL_MODE` is a parameter write to system ID `0x7FF` (`dm_motor_control.cpp:70-73`). The library immediately changes only its local `DMCANDevice::control_mode_` and then sends the write, without receiving or verifying an acknowledgment (`upstream/openarm_can/src/openarm/damiao_motor/dm_motor_device_collection.cpp:92-103`). DaMiao's commissioning procedure instead uses “write parameters,” which automatically resets the drive, and then confirms mode from boot output or by reading the parameter back (official protocol manual pp. 11-12). It explicitly says a separate power cycle is no longer needed after that full write/reset operation. The lightweight library call does not perform that documented confirmation.

The ROS hardware does not set/query physical mode at all; it initializes without a `control_modes` vector (`upstream/openarm_ros2/openarm_hardware/src/openarm_simple_hardware.cpp:153-155`) and always emits MIT commands (`:285-300`). Its C++ device object defaults locally to MIT (`upstream/openarm_can/include/openarm/damiao_motor/dm_motor_device.hpp:41-51`). Consequently “the API accepted the MIT call” does not prove that the physical drive is in MIT mode. Physical mode, firmware, and mapping registers must be commissioned and read back before activation.

## 3. Encoder, gear ratio, joint coordinates, and sign

### Confirmed physical sensing

OpenArm lists two 14-bit magnetic, **single-turn** encoders per actuator and the integrated gear ratios 10:1 (DM4310), 40:1 (DM4340), and 9:1 (DM8009P) at `upstream/openarm/website/docs/hardware/openarm-2.0/motor.mdx:23-42`. Each pinned DaMiao motor manual says the dual-encoder actuator reports the **output shaft's single-turn absolute position** and does not lose that output absolute position on power loss (DM4310 p. 2, DM4340 p. 2, DM8009 p. 2).

DaMiao's drive parameter `Gr` is the gearbox ratio and affects output speed/position and indirectly torque feedback (official protocol manual p. 10; the pinned CLI identifies separate motor-position `p_m` and output-position `xout` registers at `motor_read_param_commands.cpp:83-86`). This supports treating ordinary `POS/VEL/T` feedback as geared-output quantities when the drive is correctly calibrated/configured. A host-side 9/10/40 division would double-apply the reduction and is not present upstream.

### What the ROS layer actually does

For arm joints, ROS copies motor q/dq/tau directly into joint position/velocity/effort (`upstream/openarm_ros2/openarm_hardware/src/openarm_simple_hardware.cpp:259-265`) and directly copies joint q/dq/tau commands into MIT q/dq/tau (`:285-293`). There is no arm gear ratio, transmission, offset, or sign transform. Only the gripper has an approximate special mapping (`:267-279,375-392`).

Thus 1 motor output rad == 1 joint rad is the upstream contract. However **the positive sign for every physical left/right joint is not established by the protocol**. The calibration utility contains special `-1` factors for J1/J2 and `+1` elsewhere (`upstream/openarm_can/setup/openarm-can-zero-position-calibration:71-75`) and side-dependent stop directions (`:152-224,227-309`), while normal ROS applies no such factors. The drive also exposes a motor-direction register (`dm_motor_constants.hpp:79-87`; CLI description at `motor_read_param_commands.cpp:83-86`). The inspected sources do not provide a definitive commissioned `dir` value for every joint/side. Per-joint command/feedback sign relative to the URDF axis remains a commissioning unknown and must be verified at low energy.

Also, “absolute” is only single-turn. No inspected source promises multi-turn absolute recovery. The arm's mechanical ranges are within one output turn, but software must not generalize this sensor to unlimited multi-turn mechanisms.

## 4. Set-zero and homing/calibration observability

### Exact set-zero semantics

The wire command is eight bytes `FF FF FF FF FF FF FF FE` at the motor's command ID (`upstream/openarm_can/src/openarm/damiao_motor/dm_motor_control.cpp:34-36,214-216`). DaMiao calls this command **“save position zero”** and documents the same frame (official protocol manual p. 37). The OpenArm CLI describes it as “current position = 0 rad” and uses Disable -> Set Zero -> Disable at `upstream/openarm_can/setup/cli/commands/zero_position_commands.cpp:27-30,60-92`.

This operation only assigns zero to the present output-shaft pose; it does not measure the robot's intended zero. It should be disabled before the command. The pinned direct library method does not enforce disable itself (`upstream/openarm_can/src/openarm/damiao_motor/dm_motor_device_collection.cpp:44-55`).

Persistence is not perfectly specified in the inspected pinned sources. The official name “save position zero” and power-loss-resistant absolute output sensing strongly suggest retained behavior, but the manual does not explicitly state the zero offset's flash endurance, atomicity, or across-power guarantee. OpenArm warns that motor parameter flash has about 10,000 writes (`upstream/openarm/website/docs/setup/openarm-setup/4-motor-config.mdx:23-27`), but does not explicitly say whether that endurance applies to `0xFE`. A product controller should treat set-zero as commissioning-only and verify it after power cycle, not issue it at boot.

### No safe reference-free first-time home

The absolute output encoder provides actuator output angle relative to its stored encoder/zero configuration, not the assembled link's semantic home. Without a previously valid saved zero, mechanical stop, switch, indexed external encoder, fixture, vision, or human pose confirmation, the inspected signals do not make robot home observable. This is a physical observability result, not merely an API omission: identical encoder readings can correspond to differently assembled output splines/links or differently saved offsets.

The official OpenArm procedure confirms this limitation:

- It tells the user to physically place the arm roughly in the shown zero pose first (`upstream/openarm/website/docs/setup/openarm-setup/4-motor-config.mdx:45-50`).
- It warns that calibration moves automatically and requires PPE, a clear workspace, and readiness to emergency-stop (`:57-61`).
- The script steps each actuator into a mechanical stop and declares contact only when velocity is small and torque exceeds a threshold (`upstream/openarm_can/setup/openarm-can-zero-position-calibration:97-132`). The stops are the external references.
- It then disables and writes zero (`:398-403`). The computed `arm_goal_abs` is not applied because `move_to_precise_home` is a TODO/pass (`:145-147,393-401`); final accuracy relies on the executed stop-and-offset motion sequence, not that computed refinement.

Therefore an unattended “safe automatic calibration” without external references is not supported or observable. Stop-bumping is an available automatic reference method, but it is contact-based, can load the structure, has no timeout/travel bound in `while True`, and is explicitly not safe to run blindly.

The ROS hardware's `return_to_zero()` is not homing. On activation it enables motors and immediately interpolates from reported q to stored motor zero (`upstream/openarm_ros2/openarm_hardware/src/openarm_simple_hardware.cpp:223-235,305-352`). It assumes zero was already valid.

## 5. CAN IDs and discovery/configuration safety

OpenArm's commissioned mapping is ESC/send IDs `0x01..0x08` and Master/feedback IDs `0x11..0x18` (`upstream/openarm/website/docs/setup/openarm-setup/1-motor-id.mdx:10-23`; identical ROS defaults at `openarm_simple_hardware.hpp:93-101`). `ESC_ID` is the ID the drive receives; `MST_ID` is the feedback arbitration ID (`dm_motor_constants.hpp:47-52`; official protocol manual p. 10).

Important safety findings:

- The official OpenArm setup warns to configure IDs before running arm code and says the existing ID is usually `0x01` (`1-motor-id.mdx:27-33,88-97`). Multiple factory-default motors on one bus can therefore share an ID.
- `change_id` sends a `0x7FF` parameter write addressed only by the current ESC ID, changes ESC ID, then Master ID, optionally flash-saves (`upstream/openarm_can/setup/cli/commands/change_motor_id_commands.cpp:26-67,69-112`). If two connected motors have the same current ID, both consume the write; there is no serial-number addressing or collision check. Configure/flash one uncommissioned motor at a time.
- `discover` is query-only with respect to motor parameters, but not passive to the system. It repeatedly takes the host interface down/up, changes nominal/data bitrate, and sends `MST_ID` queries for every candidate ESC ID (`upstream/openarm_can/setup/cli/commands/discover_motor_commands.cpp:80-95,97-163`). It can disrupt any active controllers and should only run on an isolated, torque-disabled bus.
- Discovery is incomplete: for each ESC ID it only registers feedback candidates `ESC_ID+0x10` and `0x00` (`discover_motor_commands.cpp:128-159`). A motor with another Master ID is not discoverable by this implementation even if its ESC ID/baud are correct.
- Two motors sharing a Master ID cannot be distinguished by the library because devices are keyed only by receive arbitration ID (`upstream/openarm_can/src/openarm/canbus/can_device_collection.cpp:25-31,44-58`). Simultaneous different payloads at one CAN ID may also produce bus errors. Unique Master IDs are a hard precondition.
- Parameter/system frames use standard classic CAN ID `0x7FF`; control can be classic CAN or CAN-FD depending configuration. The configuration CLIs intentionally open classic CAN (`change_motor_id_commands.cpp:29-38`, `zero_position_commands.cpp:55-58`). Do not infer a motor's physical bitrate/FD setting merely from its logical IDs.

## 6. Faults, temperatures, current, watchdog, and stale feedback

DaMiao defines `D[0]` high-nibble states: 0 disabled, 1 enabled, 8 overvoltage, 9 undervoltage, A overcurrent, B MOS overtemperature, C winding overtemperature, D communication loss, E overload (official protocol manual pp. 32-33; pinned DM4310 PDF pp. 6-7). `D[6]` and `D[7]` are MOS and coil temperatures in °C.

The pinned decoder begins at `D[1]` and never parses or validates `D[0]` (`upstream/openarm_can/src/openarm/damiao_motor/dm_motor_control.cpp:88-110`). Consequences:

- no enable/disable/fault status reaches `StateResult` (`upstream/openarm_can/include/openarm/damiao_motor/dm_motor_control.hpp:37-44`);
- motor-ID-in-payload is not cross-checked;
- `Motor::enabled_` starts false and `set_enabled()` has no caller, so `is_enabled()` remains false regardless of hardware (`upstream/openarm_can/src/openarm/damiao_motor/dm_motor.cpp:23-35`; only definition at `:34-35`);
- temperature is available through the low-level getters, but ROS exports only position/velocity/effort and never checks temperature (`openarm_simple_hardware.cpp:190-220,253-282`);
- clear-error `0xFB` exists (`upstream/openarm_can/setup/cli/commands/clear_error_commands.cpp:45-57`), but clearing a flag is not evidence that the causal overtemperature/current/voltage/load condition is safe.

The official protocol defines `TIMEOUT` as an unsigned count of 50 µs periods; if no CAN command is detected by the count, the drive enters motor protection (official protocol manual p. 10 and p. 39). The pinned library exposes register 9 (`dm_motor_constants.hpp:49-53`) but ROS neither queries nor configures it. The effect of value zero and whether refresh/query frames reset the watchdog are not specified in the inspected pinned sources. A controller must read back and deliberately commission a nonzero value appropriate to its proven command period rather than assume a watchdog is active.

There is no host-side watchdog or feedback freshness in the ROS implementation. Motor state initializes to finite zeros (`dm_motor.cpp:23-32`); a frame is considered valid solely if it has at least eight bytes (`dm_motor_control.cpp:89-110`); `recv_all()` gives no count/timestamp/status (`upstream/openarm_can/src/openarm/can/socket/openarm.cpp:90-118`); and ROS `read()` always returns OK (`openarm_simple_hardware.cpp:253-283`). On activation it enables, receives once, and proceeds to motion without proving all eight motors responded (`:223-235`). A missing motor can therefore look like a valid zero state, and stale last values persist indefinitely.

## Contradictions / assumptions disproved

| Assumption | Result |
|---|---|
| Feedback torque is current, or a current estimate is available | False. Feedback field is torque mapped in Nm; no live current field exists. |
| `MOTOR_LIMIT_PARAMS` are safe physical joint limits | False. They are mapping/clipping spans and some exceed documented peak torque. |
| Selecting a `MotorType` guarantees correct decode | False unless the drive's actual PMAX/VMAX/TMAX match the table. |
| The standard monitor safely reports every OpenArm joint in SI units | False for J1-J4: it registers every ID as DM4310. |
| The API reports enabled/fault state | False. Feedback status byte is discarded and `is_enabled()` never changes. |
| Absolute encoder means robot home is known | False. It is single-turn output-shaft absolute sensing; robot zero is a separate saved/calibrated relationship. |
| `set_zero` homes or calibrates | False. It saves the current pose as zero. |
| Automatic first-time home can be safe and reference-free | False. Upstream uses deliberate mechanical-stop contact and explicit human safety precautions. |
| ROS compensates gearbox and joint signs | False for arm joints. It passes output q/dq/tau straight through; signs are a commissioning precondition. |
| Calling POS_VEL with `dq` means track that signed velocity | False. It is the profile's maximum absolute speed. |
| Setting local control mode proves physical mode changed | False. Library gating is local and no readback/activation confirmation is performed. |
| CAN discovery finds arbitrary valid ID configurations | False. It only tries two Master-ID patterns and alters interface bitrate while scanning. |
| ROS stops on missing/stale/fault feedback | False. It has no such state or checks. |

## Remaining unknowns requiring bench/manufacturer confirmation

1. Exact commissioned `PMAX/VMAX/TMAX`, `Gr`, `GREF`, `dir`, `TIMEOUT`, firmware, and control mode values shipped on each OpenArm joint/side. The source contains desired assumptions, not a per-serial readback record.
2. Exact sign correspondence between each left/right motor's positive output torque/angle and each URDF joint axis.
3. Explicit persistence, flash endurance, and failure/atomicity behavior of the `0xFE` saved-zero operation; also whether it can wear the same ~10k-write parameter flash.
4. Value-zero semantics for `TIMEOUT`, which classes of CAN frame reset it, and the drive's precise torque-off/braking behavior on timeout/fault.
5. Accuracy and sign of torque feedback after gear efficiency, friction, temperature, and `KT_OUT/Gr/GREF` configuration. It is protocol-labeled Nm, but no uncertainty/calibration specification was found.
6. Whether newer CAN-FD firmware changes feedback layout, fault codes, mode-change activation timing, or set-zero behavior. The pinned motor PDFs describe classic 1 Mbps CAN; OpenArm defaults to CAN-FD.
7. Safe operational temperature/current/torque thresholds for the assembled arm. Motor protection thresholds are configurable and are not equivalent to robot structural/contact safety limits.

Until those are read back or bench-verified, a physical controller should start torque-disabled, require explicit commissioned identity/configuration, validate fresh feedback including `D[0]`, avoid automatic zero writes, and require a human-confirmed or externally referenced home before any position motion.
