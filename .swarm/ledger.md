# OpenArm 1.0 Control Stack Run Ledger

## Objective

Research and assemble the OpenArm 1.0 open-source stack, render and control a dual-arm model in ROS 2/RViz, and implement a hardware-safe C interface for CAN discovery, configuration, forward kinematics, and Cartesian inverse-kinematics targets.

## Safety invariants

- No physical actuator command is transmitted during discovery, tests, or simulation.
- Hardware output requires an explicit arm/enable call and valid joint/velocity/torque limits.
- Simulation and SocketCAN virtual interfaces are the default backends.
- Emergency stop and communication timeout behavior are part of the public interface.

## Environment

- Host: Ubuntu 26.04 x86_64
- ROS: ROS 2 Lyrical desktop and RViz 2
- GPU: NVIDIA RTX 5060 Laptop GPU; OpenGL 4.6 verified
- Initial hardware scan: no CAN interface or USB CAN adapter detected

## Status

- [x] Requested `SKILL.md` read completely.
- [x] Missing referenced skill support files documented; using equivalent local contracts.
- [x] Initial ROS and hardware inventory complete.
- [x] Authoritative research independently cross-verified.
- [ ] Sources downloaded and pinned.
- [ ] URDF built and verified in RViz.
- [ ] Dual-arm simulated control verified.
- [ ] C API implemented and reviewed.
- [ ] Full test/coverage/sanitizer verification green.
- [ ] Final independent sweep clean.

## Decisions and evidence

- 2026-07-28: Physical hardware is not currently connected; develop and validate against simulation/vcan first.
- 2026-07-28: `robot_state_publisher` and `rviz2` are present; joint-state publisher, xacro, ros2_control, and MoveIt availability must be resolved after source requirements are known.
- 2026-07-28: Three independent reports identify Enactic's `openarm_description` as the canonical v1.0 kinematic source and `openarm_can`/`openarm_ros2` as the control sources.
- 2026-07-28: Pin v1.0 description release 1.0.4 (`5db5232d4bbf7396222437a568c625176bac1139`) for reproducibility; current docs/source paths supersede archived v1 documentation paths.
- 2026-07-28: Use `openarm_{left|right}_hand_tcp` as the default claw/tool target. The canonical model has no frame literally named `claw`.
- 2026-07-28: Physical discovery remains read-only and disarmed. It cannot infer joint assignment, polarity, zero, or duplicate IDs; those require a commissioning manifest.

## Open items

- Reconcile reported J3/J4 motor-model discrepancy between BOM and current ROS defaults before codec defaults are frozen.
- Source-build and validate upstream packages on ROS 2 Lyrical; upstream v1 officially targeted Humble and does not promise Lyrical compatibility.
