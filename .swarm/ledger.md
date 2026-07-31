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
- [x] Stage A control codecs, fail-closed commissioning, query-only SocketCAN
  transport, encoder-driven controller, deterministic simulator feedback, and
  measured-state joint/TCP planning implemented and independently reviewed.
- [x] Native/ROS packaging unified; installed strict C11/C++ consumers and
  dependency-prefix reuse regressions pass.
- [x] Secure compiled portal and measured ROS virtual actions implemented; ROS
  control/session paths use `OpenArm::Runtime` as their sole authority.
- [x] Unified Runtime facade, authenticated checkpoint persistence, bounded
  planning epochs, and physical-query fail-closed boundary completed.
- [x] Public builds have bounded parallelism, sequential ROS package execution,
  canonical resource locks, bounded signal cleanup, and secure lock directories.
- [x] Final pin/freshness/ABI hardening and independent Runtime ABI review clean
  at `1ece782`.
- [x] Cache provenance, constrained cleanup, launcher binding, and physical
  cache-tree containment independently verified clean through `59590d1`.
- [x] Strict-C binary64 coordinate units, additive native unit-aware ingress,
  and centimetre/inch portal presentation completed through `a6c011d`.
- [x] Browser-native measured-pose WebGL visual proxy, audited per-arm presets,
  pinned viewer assets, bounded request lanes, and 15-test closure completed
  through `bd06a30`; standalone RViz remains available separately.
- [x] URDF-measured wide m/cm/in presets, guarded best-reachable straight-ray
  projection, pole/inter-arm stop mitigation, and portal feedback completed and
  exercised through the measured virtual controller; final independent review
  round is in progress.
- [ ] Physical hardware acceptance remains intentionally unperformed and physical
  motion remains unavailable.

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
- 2026-07-29: The staged physical-control foundation is complete: verified
  DaMiao codecs, fail-closed calibration sessions, query-only SocketCAN,
  coherent encoder-driven control, cause-aware fault stops, watchdog gating,
  and deterministic feedback retirement/overflow behavior all passed their
  independent reviews. This is a software safety result, not hardware acceptance.
- 2026-07-29: The unified native/ROS build installs the reviewed dependency graph
  and strict consumers. The compiled portal and ROS actions consume measured
  virtual state. Runtime facade persistence uses authenticated checkpoint-backed
  recovery, and breaking V1 API/transitive Commission changes require V2 or an
  explicit compatibility adapter.
- 2026-07-29: Physical Runtime discovery was deliberately disabled. Inventory
  queries clear outputs and return `OA_RUNTIME_EUNSUPPORTED`; preview is invalid
  and non-armable; apply/calibration/motion remain unsupported. Runtime has no
  CAN/transport linkage. At most bounded local `/sys/class/net` CAN-link metadata
  is exposed, never motor identity or evidence.
- 2026-07-29: ROS production control was migrated to a single
  `OpenArm::Runtime` session authority. Its archive has required `oa_runtime_*`
  references and no direct controller/plan/manifest APIs; startup, completion,
  cancel, stop, heartbeat, event, and shutdown arbitration are bounded and
  independently reviewed. No physical ROS endpoint is exposed.
- 2026-07-29: Bounded-memory build supervision merged at `53bfd80`: positive job
  limits propagate through CMake/CTest, ROS packages build sequentially, and a
  waiting supervisor owns canonical output/build/install locks while callbacks
  run in an isolated process group with bounded signal escalation.
- 2026-07-29: Final hardening through `1ece782` pins and authenticates the detached
  description source, recreates launcher-facing installs from empty while
  retaining build caches, binds full install/source/toolchain closure in an
  atomic stamp, holds shared launch leases, enforces exact installed package
  authority, and freezes Runtime V1 plus Commission 0.1.0 and all 50 Runtime
  symbols. Hardware-free focused suites, Runtime 9/9, and the final one-job
  three-package ROS build passed.
- 2026-07-30: `669ab88` integrated transactional requested/actual CMake cache
  provenance. Fresh reviewers cross-confirmed C1 arbitrary cleanup reachability,
  I1 incomplete compiler/linker launcher provenance, and I2 missing physical
  component/cache containment.
- 2026-07-30: `59590d1` remediated C1/I1/I2 with owned/allowlisted cleanup APIs,
  requested/actual launcher argument, path, and byte binding, and canonical
  component/cache containment. A same-round independent re-review and fresh
  verification were CLEAN.
- 2026-07-30: Focused cache/resource/launch/pin suites and Runtime ABI CTest 9/9
  passed. The default one-job path (`OPENARM_BUILD_JOBS=1`) rebuilt all three ROS
  packages; completed cache records, the V2 stamp, exact prefix, and no-build
  launch gate then passed. These remain hardware-free results.
- 2026-07-30: Commits `1f2d543..a6c011d` added the Model-owned strict-C
  `oa_vec3d`/`oa_length_unit` binary64 contract for metres, centimetres, and
  inches, plus additive unit-aware Model, Control, and header-only Runtime
  entry points. Runtime V1's frozen header and exact 50-symbol archive remain
  unchanged.
- 2026-07-30: Portal XYZ presentation defaults to centimetres and can switch to
  inches. The v2 request carries an explicit unit and is converted exactly once
  at the server boundary; canonical portal state, ROS actions, Runtime, model
  calculations, and stock RViz remain binary64 metres.
- 2026-07-30: The final independent unit/portal sweep was CLEAN. Targeted native
  suites, portal 24/24, full ROS 14/14, and production launch-integrity checks
  passed. These tests used no connected CAN hardware and do not enable physical
  discovery, calibration, or motion.
- 2026-07-30: Commits `6f645d5..bd06a30` replaced RViz/X capture only in the web
  launcher with a browser-native WebGL2 measured-pose visual proxy. The separate
  stock RViz launcher is retained; the portal starts no VNC/noVNC service and
  forwards no browser pointer or keyboard input to X11. Exact labels, IDs, and
  coordinates for nine field-fill presets per arm were validated, including the
  1,800-case endpoint matrix and all 18 virtual selected-arm sessions.
- 2026-07-30: The portal exposes a bounded 30 Hz latest measured-pose snapshot;
  a visible Firefox run measured 60.01 WebGL draw FPS at a 1920x1080 backing
  buffer. That result measures foreground `requestAnimationFrame`/draw submission,
  not compositor presentation or physical display scanout. The viewer uses the
  pinned Stage-A URDF and 11 allowlisted collision-proxy STLs, with exact
  size/SHA-256 checks, manifest provenance, and the redistributed pinned upstream
  Apache-2.0 license.
- 2026-07-30: One loopback listener now separates bounded static and API work;
  pre-route intake is asynchronous with a real 500 ms deadline, at most 16 retained
  sessions, and oldest-incomplete eviction. Sixteen partial bodies left authenticated
  stop available in 12 ms, deadline cleanup restored baseline FD/thread counts, and
  SIGTERM completed in 121 ms. Full ROS CTest passed 15/15. These remain virtual,
  hardware-free results; every physical-control limitation below is unchanged.
- 2026-07-30: The former centimeter-scale portal examples were replaced by nine
  URDF-derived wide targets per arm. High far is `[0.28, +/-0.67, 0.52]` m,
  over 89% of the pinned 0.747--0.748 m shoulder-to-TCP centreline reach and
  about 0.736 m from neutral. All 1,800 endpoint/cross-state transitions retained
  at least 0.0265 m sampled nominal clearance.
- 2026-07-30: Finite impossible/unsafe portal requests now submit only the
  farthest sampled, validated prefix on their original straight ray. The actual
  first failing pole/inter-arm waypoint is a hard search boundary across every
  later grid; three fixed refinement scans continue across isolated IK failures,
  and sub-1-mm projections fail without submission. An in-progress guard now
  holds a cancelable command reservation, while stop has a separate admission
  lane. Invalid/currently unsafe scenes fail closed. The genuine pole regression
  is `[0.40, +/-0.05, 0.40]` m; the legacy `[0.28, 0.80, 0.60]` m left ray now
  proves less than 1 mm before its retained keepout and is explicitly rejected.
  Focused reach/pole commands completed from measured virtual feedback in
  28.706 s. On the final source-built tree, the 273.50 s virtual suite and
  34-case portal core suite passed; the other 13 registered tests also passed
  (the six ROS-dependent cases were rerun with the installed ROS environment
  sourced). Two independent final safety/code sweeps reported CLEAN. Controller
  collision reporting remains false and no physical safety claim is made.

## Open items

- Physical motion is intentionally unvalidated and unavailable until two CAN-FD
  adapters, both arms, commissioned side/joint/ID/model/polarity/zero data, and a
  physical emergency stop are supplied.
- Reconcile the reported J3/J4 motor-model discrepancy between the BOM and current
  ROS defaults during physical commissioning; the codec does not silently choose.
- The upstream v1 ROS hardware stack targeted Humble. Its sources are pinned here,
  but it is not represented as a validated Lyrical physical-control backend.
- The compiled modular controller foundation is implemented, but it has not been
  connected to or accepted against physical arms. No result in this ledger
  validates torque/position output, real bus timing, limit behavior, collision
  avoidance, stop distance, thermal behavior, or emergency-stop effectiveness.
- The portal's sampled nominal virtual prefilter is not a verified scene or
  physical collision certificate; Runtime reports `collision_checked=false`.
- Physical enablement requires a separately reviewed backend plus witnessed
  commissioning and acceptance with correct adapters, both arms, exact
  side/joint/ID/model/polarity/zero mapping, verified electrical/mechanical
  limits, watchdogs, and a physical emergency stop.
