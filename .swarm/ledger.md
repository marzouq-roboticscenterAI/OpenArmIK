# OpenArm 1.0 Control Stack Run Ledger

## Objective

Research and assemble the OpenArm 1.0 open-source stack, render and control a dual-arm model in ROS 2/RViz, and implement a hardware-safe C interface for CAN discovery, configuration, forward kinematics, and Cartesian inverse-kinematics targets.

## Safety invariants

- No physical actuator command is transmitted during discovery, tests, or simulation.
- The delivered runtime is virtual-only; the C CAN layer provides read-only discovery,
  protocol codecs, and an in-memory fake, but no physical motion backend.
- Any future hardware output must require commissioned identity/mapping data, explicit
  arming, validated limits, a watchdog, and a physical emergency stop.

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
- [x] Sources downloaded and pinned.
- [x] Current bimanual URDF generated deterministically and verified in RViz.
- [x] Atomic dual-arm Cartesian virtual control verified through ROS 2 and TF.
- [x] C CAN and kinematics/IK APIs implemented and independently reviewed.
- [x] Strict, sanitizer, deterministic-generator, ROS, and coverage suites green.
- [x] Final fresh independent whole-branch audits clean.
- [x] RViz live-resize renderer workaround, centered camera, single-instance guard, and graceful Ctrl+C/window-close lifecycle verified.
- [x] Encoder-closed-loop controller protocol research independently cross-verified.
- [x] C ABI/C++ OOP controller design independently critiqued and approved for staged implementation.
- [ ] Stage A fail-closed runtime, simulator, calibration workflows, and measured-state joint/TCP planning in implementation.

## Decisions and evidence

- 2026-07-28: Physical hardware is not currently connected; develop and validate against simulation/vcan first.
- 2026-07-28: `robot_state_publisher` and `rviz2` are present; joint-state publisher, xacro, ros2_control, and MoveIt availability must be resolved after source requirements are known.
- 2026-07-28: Three independent reports identify Enactic's `openarm_description` as the canonical v1.0 kinematic source and `openarm_can`/`openarm_ros2` as the control sources.
- 2026-07-28: Pin the audited current `openarm_description` commit
  `6c7b720f1ba48e8bafa3a3dc752c45f397b42221`; the old release's checked-in
  example URDF is not the current xacro output.
- 2026-07-28: Use `openarm_{left|right}_hand_tcp` as the default claw/tool target. The canonical model has no frame literally named `claw`.
- 2026-07-28: Physical discovery remains read-only and disarmed. It cannot infer joint assignment, polarity, zero, or duplicate IDs; those require a commissioning manifest.
- 2026-07-28: All ten repositories listed by the canonical OpenArm hub were fully cloned under `upstream/`, detached at the coherent audited current commits, with licenses and repository state recorded in `UPSTREAM_SOURCES.md`.
- 2026-07-28: Current v1.0 xacro generation produces and `check_urdf` validates a bimanual tree. The generated wrist-to-hand transform is 0.1025 m and hand-to-TCP is 0.0835 m (0.1860 m total); the checked-in upstream `example/v1.urdf` is stale and must not be used as kinematic truth.
- 2026-07-28: Independent first-pass reviews rejected both implementation branches. CAN required stale/enable/ABI/netlink fixes; model required deterministic Jacobian construction, hard-bound feasibility, generator provenance, task-priority and strict-C11 fixes. Neither initial branch was integrated.
- 2026-07-28: Corrected branches passed independent final review and were merged to
  `main` at `13f7610`.
- 2026-07-28: Fresh strict and ASan/UBSan builds pass for both C libraries; model
  generator determinism passes with `/usr/bin/python3` and the pinned xacro.
- 2026-07-28: ROS reports 10/10 tests passing. Authored line coverage is 97.87%
  (`paired_transaction.cpp`) and 99.13% (`openarm_ik_ros_node.cpp`); every
  instrumented transaction branch executed.
- 2026-07-28: Core C coverage is 99.65% of model lines and 96.88% of portable
  CAN codec/probe lines. The Linux netlink unit is 60.73% because deterministic
  tests exercise its parser but deliberately do not mock kernel socket failures;
  a separate live read-only enumeration succeeded and found zero CAN interfaces.
- 2026-07-28: Desktop RViz loaded the bimanual model with NVIDIA OpenGL 4.6.
  A paired target `(0.20, 0.30, 0.85)` / `(0.20, -0.30, 0.85)` committed with
  approximately `6.06e-8 m` residual on each TCP, and TF matched the target.
- 2026-07-28: RViz emits four inherited finger-inertia warnings. It also exited
  with `SIGSEGV` during Ctrl-C teardown after successful operation; both project
  ROS nodes exited cleanly. This shutdown-only issue does not affect the verified
  virtual transaction but remains disclosed.
- 2026-07-28: Two fresh independent final passes reported CLEAN for the
  hardware-free scope. See `final_audit.md` and `final_verification.md` for exact
  commands, results, coverage, and limitations.
- 2026-07-29: GNOME Wayland/Qt selected DPR=2 and hardware GLX flickered while
  resizing. Process-local DPR=1 prevented persistent post-resize failure, and the
  user confirmed Mesa software rendering was better during live resize. Native
  Qt Wayland could not initialize Ogre's required GLX parent window.
- 2026-07-29: The launcher now defaults to software OpenGL on Wayland, rejects a
  second instance, centers the bimanual model, separates RViz from the ROS signal
  group, and closes RViz through the window manager before stopping ROS. Three
  Ctrl-C checks exited in under one second with both ROS nodes clean; window-close
  cleanup also passed. Independent lifecycle review, fresh sweep, and verification
  are CLEAN; ROS remains 10/10 and model/CAN strict and sanitizer suites pass.
- 2026-07-29: A fresh paired virtual request reached left/right hand TCP targets
  `(0.20, 0.30, 0.85)` / `(0.20, -0.30, 0.85)` with approximately
  `6.06e-8 m` residual per side under the stable renderer.
- 2026-07-29: Independent pinned-source audits confirmed that DaMiao feedback is
  output-shaft encoder q/dq/torque-estimate/status/temperature. Host code must not
  apply the integrated reduction again, must verify configured PMAX/VMAX/TMAX,
  and must never treat command position as measured state.
- 2026-07-29: Encoder feedback cannot infer side, joint identity, URDF zero,
  sign, or TCP. First-time manual calibration requires a known fixture/reference;
  automatic hard-stop calibration is separately supervised and recipe-gated.
  Upstream's incomplete precise-home sequence will not be copied.
- 2026-07-29: The approved architecture is a C++17 object model behind a
  versioned C ABI with one owner, two bus workers, immutable measured snapshots,
  bounded trajectories, explicit lifecycle/watchdogs, fail-closed collision
  policy, a deterministic motor simulator, and a separate commissioning product.

## Open items

- Physical motion is intentionally unvalidated and unavailable until two CAN-FD
  adapters, both arms, commissioned side/joint/ID/model/polarity/zero data, and a
  physical emergency stop are supplied.
- Reconcile the reported J3/J4 motor-model discrepancy between the BOM and current
  ROS defaults during physical commissioning; the codec does not silently choose.
- The upstream v1 ROS hardware stack targeted Humble. Its sources are pinned here,
  but it is not represented as a validated Lyrical physical-control backend.
- The user has now requested a compiled, modular physical controller. Until it is
  implemented and hardware acceptance is performed, the prior physical-motion
  boundary remains in force.
