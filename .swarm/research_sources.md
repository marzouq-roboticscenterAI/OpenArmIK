# OpenArm 1.0 authoritative-source audit

Audited 2026-07-28 (America/Los_Angeles). This report is read-only research: no robot, CAN, motor, firmware, or external-state commands were issued.

## Identity and naming

**Fact.** The project in scope is Enactic, Inc.'s **OpenArm**, a human-scale, 7-DOF humanoid arm (8th actuator for the parallel gripper), whose canonical hub is <https://github.com/enactic/openarm>, site is <https://openarm.dev>, and documentation is <https://docs.openarm.dev>. The hub's official repository table is the controlling index: <https://github.com/enactic/openarm#-repositories>.

**Fact.** The v1 product is called both **OpenArm 1.0**, **OpenArm v1.0**, and **OpenArm 01** in first-party material. The main-project release tagged `1.1` is titled “OpenArm 01: Release No.2”: <https://github.com/enactic/openarm/releases/tag/1.1>. Versioned documentation lives at <https://docs.openarm.dev/1.0/> and explicitly says the 1.0 line is no longer actively maintained.

**Exclusions.** This is not TheRobotStudio/Hugging Face “Standard Open Arm” SO-100/SO-101, not the Hanson Robotics “Open Arms” platform, and not other similarly named projects. Do not mix their URDFs, CAD, firmware, or control software.

**Repository migration.** Older URLs under `reazon-research` (for example `reazon-research/openarm`, `openarm_ros2`, `openarm_simulation`, and `openarm_mjcf`) redirect to Enactic repositories. Canonical new references should use `https://github.com/enactic/...`.

## Canonical repositories and observed revisions

Commit IDs and refs below were observed with `git ls-remote` on the audit date. A branch HEAD is moving state; use the peeled release commit where supplied for reproducibility.

| Repository | Purpose / v1 relevance | License | Default branch and observed HEAD | Releases/tags relevant to pinning |
|---|---|---|---|---|
| <https://github.com/enactic/openarm> | Project hub, web/docs source, release history; not the model asset repo | Apache-2.0 | `main` `990fda921c82ae9d12b00f23e449793a9a313afd` | v1-era umbrella tag `1.1` `5fc656ab096b49115528e64dd19a0c08f43e9bb3`; earlier `0.3`, `0.2`, `0.1` |
| <https://github.com/enactic/openarm_hardware> | Hardware release pointers, CAD/STEP/STL/wiring assets | CERN-OHL-S-2.0 (strongly reciprocal), official license: <https://github.com/enactic/openarm_hardware/blob/main/LICENSE.txt> | `main` `12c07510c09b2c10b7dfe48010dae5c05cbe887f` | v1 release `1.0.1`, peeled commit `0403835afae64949ede85e440e86a283b747b9dd`; tags also `1.0.0`, `1.1.0` |
| <https://github.com/enactic/openarm_description> | Canonical ROS description, xacro/URDF, meshes, kinematics, inertials, limits; **primary OpenArmIK input** | Apache-2.0, <https://github.com/enactic/openarm_description/blob/main/LICENSE.txt> | `main` `6c7b720f1ba48e8bafa3a3dc752c45f397b42221` | latest observed release `1.0.4`, peeled commit `5db5232d4bbf7396222437a568c625176bac1139` |
| <https://github.com/enactic/openarm_can> | Linux SocketCAN/CAN-FD C++ library and setup/diagnostic CLI for Damiao motors | Apache-2.0, <https://github.com/enactic/openarm_can/blob/main/LICENSE.txt> | `main` `c32ecd31da267967f0c913c2118c843177d88b91` | latest observed `1.2.9`, peeled commit `7549401e65366d89cfed793fa6038490eec94efb` |
| <https://github.com/enactic/openarm_ros2> | ros2_control hardware plugin, bringup, controller and MoveIt 2 packages | Apache-2.0, <https://github.com/enactic/openarm_ros2/blob/main/LICENSE> | `main` `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed` | latest observed `0.9.2`, peeled commit `73ef89838763496b94da30ede38fc92218bea18e` |
| <https://github.com/enactic/openarm_teleop> | 1:1 leader/follower unilateral, bilateral force-feedback, gravity-compensation controls | Apache-2.0, <https://github.com/enactic/openarm_teleop/blob/main/LICENSE.txt> | `main` `eb2d49338bf70ace95282ea724903849397b7811` | no tag observed |
| <https://github.com/enactic/openarm_mujoco> | MJCF simulation assets; explicit `v1/` tree plus v0.3/v2 | Apache-2.0, <https://github.com/enactic/openarm_mujoco/blob/master/LICENSE> | **`master`** `8955afb54e4adfb59a236e2b4d15192b7a02865c` | `2.0.1`, peeled commit equal to observed HEAD; tag number is package release, while v1 assets remain under `v1/` |
| <https://github.com/enactic/openarm_isaac_lab> | Isaac Sim/Lab models and RL environments; optional simulation, not a URDF authority | Apache-2.0, <https://github.com/enactic/openarm_isaac_lab/blob/main/LICENSE.txt> | `main` `bad82e23716e6941c2de78ccb978f57c78b37734` | no tag observed |
| <https://github.com/enactic/openarm_dataset> | Dataset format, recorder/API; not needed for IK/model import | Apache-2.0, <https://github.com/enactic/openarm_dataset/blob/main/LICENSE.txt> | `main` `2da1062f524a5b240e61e1031f637d765076569d` | `0.4.0`, peeled commit equal to observed HEAD |
| <https://github.com/enactic/dora-openarm> | Dora dataflow nodes for collection, inference, and teleop; current application stack, not needed for IK | Apache-2.0, <https://github.com/enactic/dora-openarm/blob/main/LICENSE> | `main` `d988dd24537d670f07b6d6e85e0cdd25f0b05b82` | `1.0.0`, peeled commit `0ccb704cee632154e5b4181c5c1b362a1cf2c9e8` |

The purposes and licenses in this table are facts from the official hub README. Revisions are direct Git observations. A release tag's numeric version is not necessarily the physical arm generation (notably `openarm_mujoco` and the ROS packages).

## Canonical v1.0 robot model

**Current canonical xacro entry point:**

`assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro`

Pinned source: <https://github.com/enactic/openarm_description/blob/5db5232d4bbf7396222437a568c625176bac1139/assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro>

Current documentation and generation examples: <https://docs.openarm.dev/api-reference/description/>.

At the pinned/current layout, supporting assets are:

- `assets/robot/openarm_v1.0/config/arm/{kinematics,kinematics_link,kinematics_offset,joint_limits,inertials,control_gains}.yaml`
- `assets/robot/openarm_v1.0/config/body/...`
- `assets/robot/openarm_v1.0/mesh/arm/{visual,collision}/...`
- `assets/robot/openarm_v1.0/urdf/{arm,body,ee,robot,ros2_control}/...`
- v1 parallel gripper assets under `assets/end_effector/parallel_link/`
- a generated example at `assets/robot/openarm_v1.0/urdf/example/v1.urdf`

**Stale-path warning.** The versioned v1 docs at <https://docs.openarm.dev/1.0/software/description/> still show the older checkout path `urdf/robot/v10.urdf.xacro`. The repository was reorganized; consumers pinned to current `1.0.4` must use the `assets/robot/openarm_v1.0/...` path above. This is a documented-source discrepancy, not two different robot authorities.

The xacro supports single/bimanual generation, optional end effector, and optional ros2_control/fake hardware. For pure IK, keep `ros2_control:=false`; no CAN or hardware library is needed.

## ROS packages and supported host stack

`openarm_description` is its own ament package (`package.xml` version 1.0.0). The `openarm_ros2` repository contains these packages:

- `openarm` — metapackage, package version 1.0.0
- `openarm_bringup` — controllers/launch/RViz, package version 1.0.0
- `openarm_hardware` — ros2_control hardware plugin backed by `openarm_can`, package version 0.3.0
- `openarm_bimanual_moveit_config` — v1.0 and v2.0 SRDF, kinematics, limits, controller and MoveIt configuration, package version 0.3.0

Tree/source: <https://github.com/enactic/openarm_ros2/tree/73ef89838763496b94da30ede38fc92218bea18e>. The repository's `openarm.repos` imports only `openarm_can` from `main`; nevertheless the metapackage also has a runtime dependency on separately distributed `openarm_description`. For a reproducible source workspace, pin both explicitly rather than importing moving `main`.

**Supported/recommended facts from v1 docs:**

- Setup requires Linux with SocketCAN; official v1 setup names Ubuntu **22.04 or 24.04** (or other Linux with SocketCAN): <https://docs.openarm.dev/1.0/software/setup/>.
- The v1 Ubuntu guide recommends **Ubuntu 22.04 LTS**: <https://docs.openarm.dev/1.0/software/ubuntu/>.
- ROS 2 **Humble is recommended**. The official install page documents Humble and Jazzy, but explicitly says Jazzy support was still being worked on and may be unstable: <https://docs.openarm.dev/1.0/software/ros2/install/>.
- Current `openarm_can` packages explicitly support Ubuntu 22.04 and 24.04 and also document EPEL-family Linux installation: <https://github.com/enactic/openarm_can#quick-start>.
- Current Isaac Lab repository reports a tested matrix of Ubuntu 22.04, Isaac Sim 5.1.0, Isaac Lab 2.3.0, and Python 3.11: <https://github.com/enactic/openarm_isaac_lab#overview>. This is its current simulator matrix, not evidence that all v1 physical-control paths are certified on that stack.

**Inference/recommendation.** For v1 ROS work, Ubuntu 22.04 + ROS 2 Humble is the lowest-uncertainty pairing. Jazzy and 24.04 may work in parts but should not be presented as equally proven for the complete v1 hardware bridge.

## Motor models, firmware, and control tooling (safety-critical)

The official v1 BOM maps each arm's actuators as follows (bimanual quantities are twice these):

| Joint(s), per arm | Motor |
|---|---|
| J1, J2 | DM-J8009P-2EC |
| J3 | DM-J4340P-2EC |
| J4 | DM-J4340-2EC |
| J5, J6, J7, J8/gripper | DM-J4310-2EC V1.1 |

Source: <https://docs.openarm.dev/1.0/hardware/bill-of-materials/procuring-components/>. The motor specifications page confirms the DAMIAO 43-series/8009P family, 24 V operation, CAN interface, and warns that its linked 8009 datasheet is used as a near-equivalent reference for the actual 8009P: <https://docs.openarm.dev/1.0/hardware/specifications/motor/>.

Control/setup layers:

1. `openarm_can`: SocketCAN and DAMIAO MIT-mode protocol, library plus CLI/configuration/diagnostic tools. Official API docs: <https://docs.openarm.dev/1.0/software/can/>.
2. `openarm_ros2/openarm_hardware`: ros2_control hardware plugin; `openarm_bringup` loads controllers; `openarm_bimanual_moveit_config` supplies MoveIt 2 planning configuration. Docs: <https://docs.openarm.dev/api-reference/ros2/control/>.
3. `openarm_teleop`: leader/follower unilateral and bilateral force-feedback binaries/configuration. Docs: <https://docs.openarm.dev/1.0/teleop/>.
4. `openarm_mujoco`: safest dynamics/control-development target; v1 entry files are `v1/openarm.xml`, `v1/openarm_bimanual.xml`, and `v1/scene.xml`. Docs: <https://docs.openarm.dev/1.0/simulation/mujoco/>.

**Firmware fact and licensing uncertainty.** The OpenArm firmware-update page links a pinned DAMIAO vendor repository directory, <https://github.com/dmBots/motor-firmware/tree/77a7c91cd5263cde42d931fe7f84619c7fc9e1f0/V3>, containing binary images. It lists tested V3 files including `APP_DM4310(V3)_V5017_04.bin`, `APP_DM4340(V3)_V5117_04.bin`, and `APP_DM8009(V3)_V6417_04.bin`: <https://docs.openarm.dev/setup/openarm-setup/motor-firmware-update/>. The vendor repository's observed default branch is `master`, current HEAD `267911465a403ccc4aceac890fe21d855b9a3c91`, but it exposes no README or LICENSE at repository root. Therefore these are vendor binaries, **not established open-source firmware**, and redistribution/vendoring rights are uncertain.

**Critical uncertainty.** The firmware page applies specifically to hardware-version-3 motors. The v1 BOM's “V1.1” suffix on DM-J4310 is not enough evidence that an installed motor is hardware V3. Never infer firmware compatibility from the arm generation or product-name suffix.

**Safety facts.** The official v1 motor-configuration guide says zero calibration moves the robot automatically and requires cleared space, PPE, and readiness to emergency-stop: <https://docs.openarm.dev/1.0/software/setup/motor-config/>. It also says motor parameters have a finite write limit. Current ROS control docs call the hardware bridge unstable/under update, particularly gripper bridging: <https://docs.openarm.dev/api-reference/ros2/control/>. Accordingly, OpenArmIK development should default to parsed model/fake hardware/simulation. No setup demo, motor enable, zeroing, baud-rate flash, firmware flash, or trajectory command should ever be run as part of installation, import, tests, or CI.

## Hardware/CAD and BOM assets

The canonical hardware repo is <https://github.com/enactic/openarm_hardware>. Its current Git tree contains metadata/scripts and points v1 assets to the official Drive folder <https://drive.google.com/drive/folders/1a9ec9vzBV_D-AX9s_LOkBVy3ZXDC1kJT?usp=sharing>; the repo says that folder contains STEP full assemblies, STL printable parts, attachments, and wiring. The versioned docs also state that STEP, STL, attachments, and wiring are under CERN-OHL-S-2.0: <https://docs.openarm.dev/1.0/hardware/assembly-guide/find-cad-files/>.

**Uncertainty.** Current CAD payloads live outside Git, so `git clone openarm_hardware` alone does not reproduce the CAD bytes, and the Drive folder is mutable. Prefer immutable GitHub release assets from <https://github.com/enactic/openarm_hardware/releases/tag/1.0.1> when the required asset is available, preserve downloaded checksums locally, and record which artifact was used. The docs/BOM themselves are hosted in the `openarm` documentation source and remain the authoritative build description.

## Exact recommended clone list for OpenArmIK

### Required: model/IK only

Clone exactly one upstream repository, outside this project's own source namespace, and pin the peeled release commit:

```sh
git clone https://github.com/enactic/openarm_description.git third_party/openarm_description
git -C third_party/openarm_description checkout --detach 5db5232d4bbf7396222437a568c625176bac1139
```

Use `assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro`. Retain upstream `LICENSE.txt` and attribution. Better still, if the build permits, fetch this dependency at configure time or consume a system ROS package rather than copying its contents into OpenArmIK.

### Optional: deterministic v1 simulation reference

```sh
git clone https://github.com/enactic/openarm_mujoco.git third_party/openarm_mujoco
git -C third_party/openarm_mujoco checkout --detach 8955afb54e4adfb59a236e2b4d15192b7a02865c
```

Only add this if tests actually consume the `v1/` MJCF; it is not needed to derive URDF kinematics.

### Optional: ROS 2 integration workspace, not the IK library

```sh
git clone https://github.com/enactic/openarm_ros2.git src/openarm_ros2
git -C src/openarm_ros2 checkout --detach 73ef89838763496b94da30ede38fc92218bea18e
git clone https://github.com/enactic/openarm_description.git src/openarm_description
git -C src/openarm_description checkout --detach 5db5232d4bbf7396222437a568c625176bac1139
git clone https://github.com/enactic/openarm_can.git src/openarm_can
git -C src/openarm_can checkout --detach 7549401e65366d89cfed793fa6038490eec94efb
```

This is a source snapshot recommendation, **not a claim that these independently released tags form a vendor-certified compatibility bundle**; no first-party lockfile pins all three together. Validate only against fake hardware/simulation before any separately authorized hardware procedure.

## Related repositories/assets that should not be vendored into OpenArmIK

- `enactic/openarm`: hub/site/docs, not the canonical model package.
- `enactic/openarm_hardware`: acquire a specific CAD/release artifact only when geometry work requires it; cloning current Git does not bring the external CAD payload.
- `enactic/openarm_can`, `enactic/openarm_ros2`, `enactic/openarm_teleop`: operational motor/control stacks are out of scope for a pure IK library and greatly expand safety and platform obligations.
- `enactic/openarm_dataset`, `enactic/dora-openarm`, and the newer Enactic `dora-openarm-*`, `openarm_driver`, and `openarm_control` repositories: data/inference/current application-stack components, not sources of v1 kinematic truth.
- `enactic/openarm_isaac_lab`: heavyweight optional training environment; reference externally instead of vendoring.
- Archived `enactic/openarm_simulation`, `enactic/openarm_isaaclab_experiment`, plus `openarm_mujoco_hardware` and `openarm_maniskill_simulation`: superseded/experimental integrations; not official v1 model authorities.
- Old `reazon-research/*` URLs: historical aliases/redirects; do not create duplicate vendor copies.
- `dmBots/motor-firmware`: opaque vendor binaries with no explicit repository license; never vendor or redistribute, and never flash merely to satisfy software setup.
- Google Drive CAD folder: mutable external distribution, not a Git dependency. If a file is necessary, archive the exact artifact/checksum and its CERN-OHL-S-2.0 notices rather than mirroring the whole Drive.

## Bottom line

For this repository, the only authoritative upstream needed for OpenArm 1.0 IK is `enactic/openarm_description` pinned to `5db5232d4bbf7396222437a568c625176bac1139`; the v1 entry xacro is under `assets/robot/openarm_v1.0/urdf/`. `openarm_mujoco` is the only sensible optional second clone for simulation cross-checks. CAD, ROS control, CAN, teleop, dataset/Dora, and vendor firmware are separate concerns and should remain external unless a clearly scoped feature requires them.
