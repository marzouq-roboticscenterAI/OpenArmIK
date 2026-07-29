# OpenArmIK research synthesis

Synthesized 2026-07-28 from exactly three inputs: **S** = `research_sources.md`, **A** = `research_audit.md`, **C** = `research_control.md`. An evidence suffix such as **[A+C]** means two reports independently support the conclusion. Anything lacking two-report agreement is explicitly labeled **UNCERTAIN**. No hardware operation is authorized by this document.

## Reconciled baseline

The project is Enactic's **OpenArm 1.0/v1.0/OpenArm 01**: seven serial revolute arm joints plus a separate J8 gripper actuator. Canonical authority is `github.com/enactic`, with `openarm.dev` and `docs.openarm.dev`; similarly named arms and `openarm.io` are not model authorities. **[S+A]**

The implementation baseline should be the immutable snapshots independently inspected by A and C, not moving branch names:

| Repository | Immutable baseline | Role | Evidence |
|---|---|---|---|
| `enactic/openarm_description` | `6c7b720f1ba48e8bafa3a3dc752c45f397b42221` | v1 xacro, meshes, limits, model truth | A+C |
| `enactic/openarm_can` | `c32ecd31da267967f0c913c2118c843177d88b91` | protocol/codec and commissioning-tool evidence | A+C |
| `enactic/openarm_ros2` | `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed` | ROS 2/RViz/MoveIt and hardware-plugin reference | A+C |
| `enactic/openarm_teleop` | `eb2d49338bf70ace95282ea724903849397b7811` | optional mapping/dual-arm behavioral reference | S+C |
| `enactic/openarm` | `990fda921c82ae9d12b00f23e449793a9a313afd` | docs/site/repository index only | S+A |
| `enactic/openarm_hardware` | `12c07510c09b2c10b7dfe48010dae5c05cbe887f` | hardware metadata and external CAD pointers | S+A |

S found release pins `openarm_description@5db5232...` (1.0.4), `openarm_can@7549401...` (1.2.9), and `openarm_ros2@73ef898...` (0.9.2). Those are attractive stable-release alternatives, but no first-party lockfile proves the three tags form one compatible bundle, and A/C evaluated later main snapshots. **UNCERTAIN:** do not silently mix release and main pins. Choose one manifest, archive generated outputs, and require the full hardware-free test suite before changing it.

### What “all software” reasonably means

For this repository, “all software required” means only the dependency closure of the requested feature, not every official OpenArm repository. Model/IK requires `openarm_description`; C motor control additionally requires protocol evidence from `openarm_can`; ROS/RViz additionally requires `openarm_ros2`. `openarm` and `openarm_hardware` are documentation/artifact authorities, not runtime dependencies. `openarm_teleop` is optional reference material. **[S+A, S+C]**

S also enumerates `openarm_mujoco`, `openarm_isaac_lab`, `openarm_dataset`, and `dora-openarm`. **UNCERTAIN:** only S established that full optional ecosystem inventory and its exact pins. Treat MuJoCo as an optional independent simulator, and exclude Isaac, dataset, Dora, CAD, and firmware unless a separately scoped feature consumes them. Vendor motor firmware and mutable Drive CAD are artifacts, not normal source clones. **[S+A for external CAD; firmware scope A plus licensing uncertainty S]**

## Authoritative clone manifest

Use detached commits and preserve every upstream license. The minimal manifests are:

```text
# Tier 1: model, FK, Jacobian, IK, RViz RobotModel
https://github.com/enactic/openarm_description.git @ 6c7b720f1ba48e8bafa3a3dc752c45f397b42221

# Tier 2: add C CAN/control implementation evidence
https://github.com/enactic/openarm_can.git         @ c32ecd31da267967f0c913c2118c843177d88b91

# Tier 3: add ROS 2 bringup/MoveIt reference and adapter tests
https://github.com/enactic/openarm_ros2.git        @ 4e837e1d0dae692ff67b560b69d8d281d7a8d4ed

# Optional behavioral reference only
https://github.com/enactic/openarm_teleop.git      @ eb2d49338bf70ace95282ea724903849397b7811
```

The v1 entry is `assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro`. Generate the exact selected configuration and commit its SHA-256 plus a machine-readable extraction of joint names, origins, axes, limits, and base/tip frames. Archived v1 docs showing `urdf/robot/v10.urdf.xacro` are stale. **[S+A]**

Do not clone `openarm_hardware` expecting CAD: both S and A found that current Git points v1 CAD to a mutable Google Drive folder. Acquire a release artifact only when geometry validation needs it and record its checksum/license. **[S+A]**

## Reconciled model facts

The following bimanual chain is agreed by A and C; axes are expressed in each URDF joint frame and origins are parent-to-joint:

| Joint | xyz m | fixed rpy | right axis | left axis | effective right limits rad | effective left limits rad |
|---|---|---|---|---|---|---|
| J1 | `0 0 .0625` | `0 0 0` | `0 0 1` | `0 0 1` | `[-1.396263,3.490659]` | `[-3.490659,1.396263]` |
| J2 | `-.0301 0 .0600` | right `+pi/2 0 0`; left `-pi/2 0 0` | `-1 0 0` | `-1 0 0` | `[-.174533,3.316125]` | `[-3.316125,.174533]` |
| J3 | `.0301 0 .06625` | zero | `0 0 1` | same | `[-1.570796,1.570796]` | same |
| J4 | `0 .0315 .15375` | zero | `0 1 0` | same | `[0,2.443461]` | same |
| J5 | `0 -.0315 .0955` | zero | `0 0 1` | same | `[-1.570796,1.570796]` | same |
| J6 | `.0375 0 .1205` | zero | `1 0 0` | same | `[-.785398,.785398]` | same |
| J7 | `-.0375 0 0` | zero | `0 1 0` | `0 -1 0` | `[-1.570796,1.570796]` | same |

Right base xyz/rpy is `0 -.031 .698 / +pi/2 0 0`; left is `0 +.031 .698 / -pi/2 0 0`. Bimanual names are `openarm_{right|left}_joint1..7` and links `openarm_{right|left}_link0..7`. Single-arm generation is not frame-equivalent to selecting one bimanual side, so API manifests must name configuration, base, and tip explicitly. **[A+C]**

The base YAML limits are not always the effective bimanual limits: xacro mirrors/offsets J1/J2. Golden generated-URDF tests—not hand-copied YAML—must freeze effective limits. **[A+C]**

The physical/runtime motor mapping reconciles as follows:

| Joint | Physical BOM distinction | Runtime codec enum | Resolution |
|---|---|---|---|
| J1–J2 | DM-J8009P-2EC | `DM8009` | family mapping agreed S+A+C; runtime intentionally lacks `P` variant |
| J3 | DM-J4340P-2EC | `DM4340` | family mapping agreed; **UNCERTAIN** whether `P` has any protocol parameter difference because only S preserved this BOM suffix |
| J4 | DM-J4340-2EC | `DM4340` | agreed S+A+C |
| J5–J7 | DM-J4310-2EC | `DM4310` | agreed S+A+C |
| J8 gripper | DM-J4310-2EC | `DM4310` | agreed S+A+C |

Thus J3 and J4 share the upstream protocol type but are not necessarily the same purchased mechanical package. Never use the simplified runtime table as a procurement BOM. Motor reductions are handled upstream as output-angle reports; do not multiply joint angles by gear ratios without commissioning evidence. **[S+C]**

The physical J8 rotates about 60 degrees, while URDF models one 0–0.044 m prismatic finger plus a mimicked finger and ROS labels its conversion approximate. Treat the gripper as a planning approximation until calibrated. **[A+C]**

A reports current-main frames `link7 -> hand` at 0.1025 m and `hand -> hand_tcp` at 0.0835 m, with no canonical `claw`/`tool0`. **UNCERTAIN across pins:** C agrees that installed TCP and gripper mapping require commissioning, but did not independently verify those exact offsets. The public model API must require an explicit tip frame and must not invent “claw.”

## Implementation dependency DAG

```text
D0  Freeze upstream pins + licenses + generated-v1 configuration
 |\
 | +--> D1  Generate/validate URDF; extract immutable oa_model manifest
 |       |\
 |       | +--> D3  C FK + Jacobian --> D5 constrained numerical IK
 |       | +--> D4  JointState/TF publisher --> D7 RViz visualization
 |       | +--> D6  SRDF/controller-name validation --> D9 MoveIt planning
 |
 +----> D2  Pure CAN codec + fault/status decoder + golden/fuzz tests
          |
          +--> D8  SocketCAN bus/device layer + vcan motor simulator
                    |
                    +--> D10 commissioned motor<->joint manifest
                    |      |
                    |      +--> D11 arm lifecycle/limits/watchdogs
                    |              |
                    |              +--> D12 dual-bus coordinator
                    |              +--> D13 ROS 2 Lyrical adapter
                    |
                    +--> D14 bus-load/fault-injection qualification

D3 + D5 + D11 + D14 + all safety gates --> D15 supervised staged hardware acceptance
D7 is hardware-free and can ship before D8–D15.
D9 may plan, but cannot command hardware except through D11/D13.
```

This ordering separates model/RViz work from motion authority, keeps ROS outside the bus thread, and makes collision planning distinct from IK. **[S+C]**

## Explicit safety gates

No gate may be replaced by “the upstream demo worked.” These gates are derived from A/C's source audit and S/C's hardware-free-first recommendation.

1. **Scope gate:** build/test defaults contain no motor enable, zeroing, hard-stop calibration, baud/ID/register writes, firmware flash, privileged `ip link`, or physical CAN discovery. CI uses generated models, fake hardware, MuJoCo if selected, and `vcan`. **[S+A+C]**
2. **Model gate:** pinned xacro generation passes validators and golden assertions for configuration, names, base transforms, axes, effective limits, and chosen tip. FK/Jacobian agree with an independent URDF implementation. **[A+C]**
3. **Protocol gate:** codec rejects non-finite values, validates CAN flags/DLC/arbitration and embedded ID, decodes the status/fault nibble, timestamps freshness, reports saturation, and passes golden/fuzz/fault-injection tests. **[A+C]**
4. **Manifest gate:** torque-disabled probing matches expected bus, joint, send/master IDs, motor family/variant, serial, hardware/firmware/sub-version, direction, gear, P/V/T ranges, control mode, bitrate, timeout, zero and scale. Missing, extra, duplicated, or mismatched devices latch fault. **[S+C]**
5. **Mapping gate:** fixtures verify `q_joint = sign*scale*q_motor + offset`, velocity and torque sign, both side-specific zeros, endpoints, J1/J2 offsets, mirrored J7, and gripper curve/TCP. Upstream identity mapping is only a starting hypothesis. **[A+C]**
6. **Stop gate:** an independent physical power E-stop/contactor is installed, rated, measured, restart-interlocked, and assessed for payload drop. CAN disable is never called an E-stop. **[A+C]**
7. **Watchdog gate:** motor TIMEOUT is read/configured/verified per firmware and a host monotonic watchdog latches stale command/state, invalid telemetry, deadline miss, bus error/off, thermal/electrical fault, or limit violation. Fault reset requires cause removal plus explicit acknowledgement. **[A+C]**
8. **Lifecycle gate:** startup remains torque-disabled; arming requires fresh state from all expected motors, legal pose, healthy bus, live expiring command producer, and explicit operator action. No process lifecycle callback may auto-enable or auto-return-to-zero. **[A+C]**
9. **Dynamics/bus gate:** commissioned speed/torque/temperature/current plus conservative acceleration/jerk limits are enforced below ROS; bus utilization, latency, dropped frames, queue depth, dual-bus skew, and fault-disable latency have measured margin. **[A+C]**
10. **Acceptance gate:** isolated motor/no load -> constrained single joint -> reduced-limit single arm -> dual arm, supervised with exclusion zone, fixture, independent E-stop observer, injected command loss/bus-off/process kill, and logged acceptance. Items 1–7 in C's unresolved list block unattended or human-adjacent motion. **[S+C]**

Firmware is not a routine prerequisite. A says v1 docs do not require it; S says distributed vendor V3 binaries have unclear repository licensing; C says actual hardware/firmware/FD capability must be probed. Flash only under a separately authorized commissioning procedure after exact hardware-version matching. **[S+A+C]**

## Ranked implementation risks

| Rank | Area | Risk | Rating | Required mitigation | Evidence |
|---:|---|---|---|---|---|
| 1 | C API / hardware | Wrong sign, zero, side, or effective limit can immediately drive into a stop; upstream assumes identity despite mirrored URDF coordinates | Critical / likely until commissioned | Per-joint affine manifest, torque-disabled probe, fixture validation, hard safety margins | A+C |
| 2 | C API / hardware | Upstream ignores feedback status/fault and freshness and does not verify motor TIMEOUT | Critical / confirmed gap | Full decoder, timestamped masks, motor+host watchdogs, latched fault state | A+C |
| 3 | ROS 2 hardware | Upstream plugin can enable and return to zero on activation; approximate gripper and weak error propagation are not a production boundary | Critical / confirmed behavior | New adapter over the C lifecycle; never reuse activation semantics unchanged | A+C |
| 4 | ROS 2 Lyrical | No evidenced upstream Lyrical support; v1 recommends Humble and current plugin APIs/behavior need porting | High / likely | Pinned clean Lyrical environment, source build, API port, lifecycle/controller/fake-backend tests | S+C |
| 5 | Dual-arm CAN | Separate buses reuse IDs; frame load, feedback strategy and cross-bus skew are unqualified | High / likely | One owner worker per bus, shared monotonic clock, measured load/skew, explicit paired-stop policy | A+C |
| 6 | IK/model | 7-DOF IK is redundant/non-unique; singularities, asymmetric limits and seed discontinuities can produce unsafe jumps | High when connected / inherent | Constrained numerical IK, warm start, adaptive damping, null-space posture objective, residual/limit/singularity result flags | S+C |
| 7 | Model pinning | Old docs, release pins and current main differ; single and bimanual frames are not interchangeable | High / confirmed drift | One immutable manifest, generated golden artifacts, explicit side/config/base/tip in ABI | S+A and A+C |
| 8 | RViz/MoveIt | RViz visualization is safe and feasible, but users may mistake display/planning success for collision-free, limit-enforced hardware safety | Medium alone; High if commanding | Ship visualization first; route every command through C safety envelope; keep collision checking above basic IK | A+C |
| 9 | Gripper/TCP | URDF prismatic abstraction and approximate ROS mapping do not establish installed jaw displacement, force, or TCP | Medium / confirmed | Calibrate linkage/force/TCP; expose explicit tip; keep gripper optional | A+C |
| 10 | Adapter/firmware | CAN-FD timing, termination, firmware version and vendor-binary compatibility vary by installed unit | High / uncertain installation | Commission adapter and every RID; checksum artifacts; never auto-flash | S+A+C |

## Recommended delivery sequence

Ship in four independently testable increments: (1) pinned model extraction, C FK/Jacobian/constrained IK and golden tests; (2) ROS 2 Lyrical JointState/TF/RViz visualization with no hardware dependency; (3) pure C codec/bus/lifecycle using `vcan` and a faulting motor simulator; (4) only after every safety gate, supervised physical acceptance and then the ROS hardware adapter. **[S+C]**

The present evidence supports production-quality model and hardware-free visualization work now. It does **not** support unattended or human-adjacent motion without commissioning, watchdog, E-stop, mapping, bus-load and lifecycle gates. **[S+A+C]**
