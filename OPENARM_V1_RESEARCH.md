# OpenArm v1.0 research and integration record

Audited 2026-08-06 against the official Enactic documentation and repositories.
This document separates OpenArm v1.0 facts from newer OpenArm 2.0 material and
from the unrelated M1/Ranger configuration originally collected with DREAD.

## Primary sources

- [Official OpenArm repository hub](https://github.com/enactic/openarm)
- [OpenArm v1.0 robot-description documentation](https://docs.openarm.dev/api-reference/description/)
- [OpenArm v1.0 CAN library documentation](https://docs.openarm.dev/1.0/software/can/)
- [OpenArm CAN CLI documentation](https://docs.openarm.dev/api-reference/can/cli/)
- [OpenArm v1.0 gripper specification](https://docs.openarm.dev/hardware/specifications/gripper/)
- [Official ROS 2 integration](https://github.com/enactic/openarm_ros2)
- [Pinned source/commit and license manifest](UPSTREAM_SOURCES.md)

All ten repositories linked by the official hub are full clones under
`upstream/`: hardware, description, CAN, ROS 2, teleoperation, Isaac Lab,
MuJoCo, dataset, Dora, and the hub itself. Their remote histories and tags were
refreshed on 2026-08-06 without changing the audited detached worktrees. The
latest description update removes only redundant top-level generated URDFs;
the v1.0 xacro/config/mesh subtree is unchanged from the pinned commit used by
this project.

## Hardware and kinematic model

OpenArm v1.0 is a seven-revolute-joint arm plus an eighth DaMiao motor driving a
linkage-based parallel gripper. A bimanual unit therefore has 14 arm joints and
two gripper motors. The current description entry point is:

```text
upstream/openarm_description/assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro
```

The v1.0 gripper has an 88 mm maximum jaw-to-jaw opening. The motor rotates 60
degrees from closed to open. In the URDF this is represented by two prismatic
finger joints, each moving from 0 to 0.044 m; `finger_joint2` mimics
`finger_joint1`. The upstream ROS hardware plugin uses the approximate mapping
0..0.044 m to 0..-1.0472 motor rad. This repository's live calibration replaces
that nominal scale with the encoder-measured closed/open sweep.

The exact arm joint limits used by the C model and DREAD map are taken from the
pinned v1.0 xacro/config, not estimated from RViz. The generated installed
bimanual URDF is:

```text
ros2_ws/install/openarm_ik_ros/share/openarm_ik_ros/urdf/openarm_v10_bimanual.urdf
```

Visual and collision STL files are included in Git under
`upstream/openarm_description/assets/robot/openarm_v1.0/mesh/`. The web viewer's
audited subset is installed under
`ros2_ws/install/openarm_ik_ros/share/openarm_ik_ros/viewer/mesh/`.

## Motors and CAN

The official v1.0 ROS hardware implementation defines this motor order per arm:

| CAN ID | Joint | Motor family | Reply ID |
|---:|---|---|---:|
| 1 | J1 | DM8009 | 0x11 |
| 2 | J2 | DM8009 | 0x12 |
| 3 | J3 | DM4340 | 0x13 |
| 4 | J4 | DM4340 | 0x14 |
| 5 | J5 | DM4310 | 0x15 |
| 6 | J6 | DM4310 | 0x16 |
| 7 | J7 | DM4310 | 0x17 |
| 8 | parallel gripper | DM4310 | 0x18 |

The official `openarm_can` stack is C++ over Linux SocketCAN. Its high-level
`OpenArm` object owns an `ArmComponent` and `GripperComponent`; below those are
DaMiao motor/device objects and the CAN socket layer. MIT commands carry
position, velocity, `kp`, `kd`, and torque. State feedback supplies encoder
position, velocity, torque, motor status/fault information, and temperatures.

The official current CAN-FD default is 1 Mbit/s arbitration and 5 Mbit/s data.
This machine's `can0` and `can1` are configured that way with the `gs_usb`
driver and a dual-channel DM-USB2FDCAN. A status refresh is opcode `0xCC` sent
to arbitration ID `0x7FF`; it requests feedback without enabling or commanding
motion.

The upstream xacro defaults to robot-left=`can1` and robot-right=`can0`. That is
a default, not an identity protocol, but it also matches this pair's
operator-confirmed motion mapping. Local launchers export it explicitly because
both channels share one USB identity. Both buses reuse motor IDs 1..8, so CAN
IDs alone cannot distinguish the physical sides.

## Software selected for this repository

- `openarm_description`: authoritative v1.0 xacro, limits, transforms, meshes,
  inertials, and ros2_control description.
- `openarm_can`: protocol and SocketCAN reference. This repository also exposes
  a strict C11 frame codec/diagnostic API under `can/`.
- `openarm_ros2`: reference hardware plugin, bringup, and MoveIt integration.
  This repository's portal/runtime is separate and exposes public C interfaces.
- `openarm_hardware`: manufacturing CAD, wiring, and assembly source; it is not
  used as executable control code.
- `openarm_teleop`, `openarm_isaac_lab`, `openarm_mujoco`, `openarm_dataset`, and
  `dora-openarm`: retained as complete upstream sources for future integration;
  none is required to run the current RViz/portal/C controller.

The native implementation keeps coordinates, kinematics, calibration
transforms, trajectory generation, and collision distances as IEEE-754
binary64 (`double`). Public coordinate APIs take an explicit unit (`m`, `cm`, or
`in`) and normalize once to metres. Joint angles come from DaMiao encoder
feedback through a fitted transform, never from a visual estimate.

## Calibration behavior

The adapted DREAD wizard uses the current URDF names and limits plus
`dread/config/motor_map.openarm_v10.yaml`. Before energizing anything it requires
complete status feedback from all eight motors on every selected bus. During
manual back-driven calibration, each safe two-stop encoder sweep is fitted to
the complete URDF range in binary64. J4 uses only its safe straight/open stop
and retains its prior commissioned scale. The gripper maps measured closed/open
encoder values to 0..0.044 m.

The old M1/Ranger maps remain reference-only. In particular, their approximately
plus/minus 0.7854 rad rotary gripper domain is incompatible with this v1.0
prismatic gripper and cannot be preserved by the adapted wizard.

## Physical verification status

The most recent read-only scan on 2026-08-06 received clean replies from all
eight motors on `can1` and from IDs 1 and 3..8 on `can0`. `can0` ID 2 / reply
`0x12` did not respond across four requests. Both SocketCAN interfaces remained
error-active with zero bus errors, warning, passive, bus-off, RX error, or TX
error counters. No motor was enabled and no movement command was sent.

Physical calibration and green-LED enable verification remain blocked until
`can0` motor 2 responds and the complete 16/16 read-only preflight passes.
