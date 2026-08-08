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
- [x] Read-only per-joint robot-left/right encoder-to-RViz direction/reference/
  offset calibration implemented with binary64 persistence and a compiled CLI.
- [x] Native-C paired Cartesian routing, post-Cross escape, measured-state
  per-leg revalidation, and full virtual regression closure completed.

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
- 2026-07-30: Portal movement limits became adjustable from the legacy 0.5
  through 1.0, defaulting to 0.8. The strict v3 JSON boundary, additive
  `MovePairedTcpScaled` action, and session boundary all preserve and validate
  the binary64 scale; the original action and other legacy paths remain at 0.5.
  The same scale bounds velocity, acceleration, and jerk, while the existing
  synchronized seventh-order profile remains unchanged. The WebGL proxy now
  has a command-free blue/neutral palette toggle. Focused portal 35/35,
  scale-duration, Firefox, direct scaled-action, and live v3 portal gates passed;
  the final full ROS suite passed 15/15 in 363.47 s. Its 44.94 s ROS contract
  exercised live HTTP v3 through the scaled action to measured completion,
  rejected 0.49/1.01/NaN/+Inf/-Inf at the action boundary, and verified scaled
  cancellation plus race-tolerant process cleanup. Two independent final
  reviews reported CLEAN. These are virtual, hardware-free results.
- 2026-08-06: Implemented narrowly scoped nominal claw-tip contact for clap and
  both heart cusps. Only mutually facing terminal 6/6 caps are exempted; every
  other arm pair and the pole retain the 25 mm portal gate and 10 mm measured
  feedback floor. Contact succeeds only with a Runtime CONTACT report. The
  post-contact paired path is promoted to the scoped policy only after all 17
  controller waypoints prove a monotonically opening retreat that exits the
  corridor. Ordinary moves remain strict.
- 2026-08-06: The first terminal-cap implementation routed contact through
  `/api/v3/converge` with a model-derived 0.073 m stop distance (superseded
  later the same day by the branch-specific exact-mesh search) and the following step through
  `/api/v3/retreat`. Full quantized-feedback clap/heart geometry passed. Actual
  converge -> CONTACT -> retreat passed from measured virtual feedback. Portal
  speed is bounded to 50--100%, default 100%, and real RViz MJPEG pixels have a
  command-free regular/blue-light-blue browser filter.
- 2026-08-06: The one-job test build passed every native suite, installed
  consumer, header matrix, Runtime freeze, and exact 57-symbol manifest; 16 ROS
  tests registered. The 313.15 s virtual session, portal core, RViz capture, and
  all other ROS tests passed. Direct CTest must source both ROS Lyrical and
  `ros2_ws/install/setup.bash`; five cases first invoked without that environment
  failed only at dynamic loading, then passed 5/5 once sourced. These remain
  hardware-free results; physical contact and safe speed are not certified.
- 2026-08-06: Superseded the fixed terminal-cap stop for Heart/Clap with exact
  pinned OpenArm v1.0 claw geometry. `hand.stl` and `finger.stl` triangles are
  embedded as binary64 literals by a C generator and evaluated through a C ABI
  using FCL. Only hand-housing to hand-housing contact may enter a bounded 4 mm
  endpoint corridor; all eight other cross-claw mesh pairs, arm capsules, and
  the pole retain 25 mm. The portal searches 15--70 mm for the actual measured
  IK branch and the monitor evaluates the same meshes on every feedback cycle.
- 2026-08-06: Fresh-start focused regression passed the full entry, two Clap
  contacts, all three Heart contacts, and every retreat. Live loopback portal
  testing selected 41.5 mm at the top and 41.0 mm at the Heart bottom; both
  stopped with Runtime contact cause 1/outcome 1, then completed their scoped
  retreats. This is fixed-opening hand-housing mesh contact, not physical or
  fingertip certification; no physical arm moved.
- 2026-08-06: Final one-job test build completed with every native/component
  suite passing and exactly 16 ROS tests registered. The fully sourced ROS
  CTest sweep then passed 16/16 with zero failures in 409.54 s; the long virtual
  control integration passed in 315.09 s and portal core passed in 15.02 s.
- 2026-08-06: Fixed the post-restart blank portal viewer. RViz was aborting its
  GLX setup because the launcher forced NVIDIA PRIME while `nvidia-smi` could
  not reach the driver. The panel-free portal now defaults to accelerated AMD
  integrated GLX and has no palette/filter control. Live capture showed the
  complete 1368x768 raw RViz render, delivered 107 frames in three seconds, and
  visibly rotated after three accepted pointer events. Focused portal/capture
  regression passed 2/2; shutdown remained clean.

- 2026-08-06: Replaced the Heart/Clap contact target with an expanded gripper-
  rail envelope. Exact pinned hand/finger STL separation now targets 24 mm and
  may not fall below 23 mm; every other cross-claw pair and the ordinary gate
  retain 25 mm. The measured-feedback monitor stops both virtual and physical
  backends on entry into the 25 mm rail envelope. No STL touch is permitted.
  Native model/control tests passed; focused ROS completion is still pending.
- 2026-08-06: Adapted the collected DREAD wizard to the current pinned OpenArm
  v1.0 URDF and new `motor_map.openarm_v10.yaml`. Two-stop encoder sweeps now fit
  scale as well as offset to the complete URDF range, both grippers use 0..44 mm,
  missing-motor preflight aborts before enable, and incompatible old rotary-
  gripper mappings cannot be preserved. DREAD hardware-free tests pass 24/24.
- 2026-08-06: After the operator resolved the visible spark source, four rounds
  of read-only `0xCC` refreshes found can1 IDs 1..8 and can0 IDs 1,3..8; can0 ID
  2 / reply 0x12 remains absent. Both links are error-active with zero CAN error
  counters. No motor has been enabled or commanded.
- 2026-08-06: Brought up the physical read-only observer and RViz. Fixed a live
  mapping bug first: seven arbitrary replies no longer count as J1..J7, and a
  missing J2 cannot shift the gripper into J7. The observer refreshes exact IDs
  1..8 continuously and publishes any complete arm while leaving an incomplete
  arm neutral. Focused observer tests pass. A live wire audit recorded 768
  refresh requests, 720 replies, and zero unexpected/enable/MIT frames. can1
  publishes in RViz; can0 stays neutral.
- 2026-08-06: The operator's live motion check corrected the earlier viewpoint-
  ambiguous LED label: can0 is physical robot-right and can1 is physical
  robot-left. The running observer was swapped immediately without CAN writes,
  and the confirmed assignment is now pinned in the real launch, portal launch,
  DREAD launcher/defaults, tests, and handoff documents.
- 2026-08-06: Operator accepted robot-right J1 display direction `+1` and
  offset `0 deg`. Immediately afterward can0 J2/reply 0x012 disappeared again;
  can0 IDs 1,3--8 remained live and status-disabled, and both links retained
  zero errors/drops/bus-off. The read-only observer now updates exact-ID joints
  independently and freezes only a missing joint. Focused mapping tests pass.
- 2026-08-06: Operator accepted robot-right J3--J7 and robot-left J1--J7 with
  direction `+1` and offset `0 deg`. Right J2 resumed status telemetry at
  491/492 replies but exposed a stale-reference `-42.6866 deg` startup jump;
  its local display offset is now atomically saved at `+42.686576 deg` and the
  displayed current pose is zero. Its raw encoder did not change during the
  subsequent manual-motion window, so direction/range remain unaccepted.
- 2026-08-06: The live engineering RViz config now explicitly selects the
  MoveCamera tool. The read-only launch was restarted with the saved display
  calibration intact so the operator can orbit, pan, and zoom while locating
  the remaining visual offset.
- 2026-08-06: Multi-angle inspection found robot-right J1 visually high by
  roughly 5--10 degrees. Its live display value was `+9.070350 deg`; the local
  per-joint calibration atomically saved `-9.070350 deg` and now displays zero.
  This offset remains provisional pending operator visual confirmation.
- 2026-08-06: The next multi-angle pass implicitly accepted right J1, judged
  right J2 about 5 degrees overcorrected, and found right J6 offset. J2 was
  reduced to `+37.686576 deg`; J6's measured `-4.830385 deg` was zeroed with a
  saved `+4.830385 deg` offset. J2 then again returned 0/24 replies, so its
  adjustment awaits renewed telemetry and both J2/J6 await visual acceptance.
- 2026-08-06: Operator visually accepted the final right J2 and J6 offsets.
  The atomically written V2 file was reread and verified: right J1
  `-9.070350 deg`, J2 `+37.686576 deg`, and J6 `+4.830385 deg`; all other
  accepted display offsets remain zero. J2 direction and range are not implied
  by this visual-offset acceptance.
- 2026-08-06: Prepared and launched the strictly read-only C11/GTK directional
  range recorder beside the live observer. It labels can0 robot-right and can1
  robot-left, records the complete ordered binary64 encoder trace, instructs a
  relaxed--safe-limit-A--safe-limit-B--relaxed hand path without forcing hard
  stops, and atomically saves schema V2 data under `calibration/`. The focused
  target rebuilt cleanly with warnings treated as errors; no range has yet been
  accepted and no motor was enabled or commanded.
- 2026-08-06: Robot-right J1 completed the first hand sweep: 816 samples at
  49.3 Hz, zero drops, 279.66 deg extent, and 544.96 deg ordered round trip.
  Mesa software RViz simultaneously reached about 905% CPU and starved the
  desktop, so only RViz was stopped and restarted on integrated GLX at about
  33% CPU while the observer and in-memory capture remained live. The compiled
  recorder update now atomically auto-saves after every Stop; the already-
  running old instance needs one manual Save before restart.
- 2026-08-06: Fixed robot-right J2 RViz flicker caused by cross-socket CAN local
  loopback. The observer had decoded the recorder's `7FF#0200CC...` read-only
  request as feedback (`-12.42 rad`, minimum velocity/torque). It now accepts
  only the exact expected reply arbitration ID and matching payload motor ID.
  The focused regression test passed, the fixed observer was installed and
  replaced live without stopping the recorder, and 199 J2 samples stayed within
  a 0.000381476 rad band around -0.088 rad. One `/joint_states` publisher was
  confirmed; no enable or motion frame was sent.
- 2026-08-07: Completed the virtual post-Cross routing checkpoint. Added the
  public C11/binary64 `oa_route_plan_paired` graph planner, 17-sample IK/FK plus
  conservative arm/tool-capsule and finite-pole validation on every edge,
  monotonic 10--25 mm clearance recovery that must finish outside 25 mm, and
  portal execution of all returned legs under one cancelable command. Every
  next edge is re-proved from fresh measured feedback immediately before
  submission. The exact nine-step Cross sequence completed, its swap used two
  guarded legs, and a distant tenth paired target immediately afterward ended
  at left `[0.22005,0.30000,0.35002]` m and right
  `[0.22000,-0.30007,0.34999]` m. Native suites and all 17 ROS tests passed;
  top-level `run.sh` readiness and clean Ctrl+C shutdown passed. The exact old
  lock was not deterministic on the pre-fix clean sequence, and software Stop
  remains a separate disable-and-restart lifecycle by design.
- 2026-08-07: Fixed Heart's first post-contact retreat. Its HTTP intake had the
  correct terminal-contact escape proof, but route execution reapplied the
  ordinary keepout and rejected the already-inside contact start. The executor
  now preserves the dedicated monotonic terminal-retreat policy. Ordinary
  routing is dynamic: after each completed leg it replans the full remaining
  tail from current measured joints, re-proves the selected edge, and validates
  equivalent encoder/diagnostic evidence at handoff. A nonblocking 20 ms timer
  waits for fresh idle diagnostics between legs. The native controller still
  checks shared measured geometry every 5 ms during execution.
- 2026-08-07: Live loopback endpoint tests strictly checked terminal command
  outcomes. Heart completed both top contacts, the lower contact, every retreat,
  and the full curve. Cross completed all nine poses; its swap used two dynamic
  legs/two replans, its return escaped a one-code quantized joint-limit endpoint,
  and a distant paired move then completed. Startup-to-Clap-open separately
  proved a two-leg/two-replan transition. No physical backend or motor motion
  was used; results continue to report `collision_checked=false`.
- 2026-08-07: Final clean test-profile verification passed every native suite,
  including the 5 ms bimanual measured-geometry monitor regressions, followed
  by all 17 ROS tests in 395.32 seconds with zero failures.
- 2026-08-07: Final launch-ready verification ran the normal-profile
  `run.sh --no-browser --no-rviz`, received fresh idle `/api/state`, passed
  `openarm_assert_current_launch_tree`, and shut down the controller and robot
  state publisher cleanly with Ctrl+C. No OpenArm process was intentionally
  left running.

## Open items

- 2026-08-07: Physical calibration was explicitly checkpointed and paused for
  a virtual post-Cross routing problem. No OpenArm observer, recorder, RViz, or
  portal process was running at checkpoint time. The right-J1 summary remains
  in `calibration/openarm_range_diagnostics.log`, but the old recorder process
  ended before Save JSON; the full ordered trace is absent and J1 must be swept
  again. The accepted V2 visual offsets remain persisted separately in
  `~/.openarm_real_zero`.
- The formerly missing robot-right/can0 J2 now replies. The live observer saw
  IDs 1--8 on both buses and the follow-up one-second audit counted 800 refresh
  requests and 800 replies per bus with zero other frames. Per-joint direction,
  offset, and then directional range commissioning remain operator-guided work.
- The user paused the collision/demo/DREAD/research track for a live-RViz
  tangent. `codex.md` now contains the authoritative resume checkpoint,
  completed changes, existing evidence, hardware state, and seven-step final
  verification sequence. The paused track is not complete until that sequence
  passes.
- Physical motion is intentionally unvalidated and unavailable until the current
  dual-channel CAN-FD adapter, both arms, commissioned
  side/joint/ID/model/polarity/zero/range data, and the physical emergency stop
  pass a separately witnessed powered acceptance procedure.
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

## 2026-08-07 gripper commissioning and guarded physical acceptance

- Added a binary64 C11 gripper mapping/codec API, ROS MoveGripper action, CLI
  open/close/grasp commands, J8 position-force configuration/confirmation, and
  two encoder-derived finger joints in `/joint_states`. The saved commissioned
  closed reference is `~/.openarm_real_gripper`; 1.0472 rad maps to 44 mm.
- Physically verified both no-load claws through full open and closed travel.
  Both claws were closed before retesting arm motion. Position-force torque
  limits worked over the no-load stroke; object-contact grasp force remains
  unaccepted because no test object was supplied.
- Physically completed Return-to-Neutral and a paired home target. Cross reached
  its stack transition, where robot-right J2 repeatedly stopped replying only
  under motion. The complete-feedback watchdog aborted and transmitted
  disables, but a later status audit proved J2 had recovered while still
  enabled. Targeted disable plus refresh then confirmed status 0. can0 remained
  ERROR-ACTIVE with zero bus errors and a 30 second idle soak was healthy. Full
  Cross, Heart, Clap, Box, and range acceptance remain blocked on
  inspection/replacement of robot-right J2/controller or its CAN/power harness.
- Hardened intentional Disconnect/E-stop races so an interrupted in-flight
  exchange cannot overwrite the operator-requested terminal state with a false
  watchdog fault. Gripper calibration replacement now synchronizes both the
  control and CAN encoder readers.
- Connect now writes/read-confirms volatile register-9 value 4000 (200 ms at
  50 us/tick) on all 16 motors before enable. Teardown repeatedly disables and
  polls status for up to three seconds, and never claims confirmed disable
  unless every returned status nibble is zero. A disabled hardware audit
  received exact timeout echoes and status 0 from all 16 drives. The E-stop
  result now propagates an unconfirmed teardown as `physical_disabled=false`.
  The arms were left disabled and the operator hand-placed them in neutral;
  no saved calibration was captured or changed.

## 2026-08-07 production physical C ABI acceptance

- New objective: an installed, standalone C11 caller must control the existing
  `run-real.sh` production safety session. It must detect controller
  availability, explicitly connect/configure all 16 motors through that sole
  authority, read encoder-derived joint/TCP/gripper state, move one joint or
  one/both TCP targets with binary64 coordinates and explicit units, command a
  torque/speed-limited gripper, Stop, Disconnect, and assert/clear E-stop.
- Audit result: the existing public native ABIs intentionally expose physical
  SocketCAN as query-only. Physical actuation currently exists only behind the
  internal C++ ROS session and therefore is not callable from an installed C
  program. This is the gap being closed; raw public SocketCAN motion will remain
  blocked.
- Safety requirement discovered during the first physical Cartesian attempt:
  a single-arm target currently enters the paired route graph and can move the
  unselected arm. The old pair's robot-right J2 then lost feedback and the live
  watchdog aborted. Before hardware ABI Cartesian acceptance, single-arm route
  planning and execution must preserve every unselected-arm joint exactly.
- Current state before implementation: the real portal/controller/RViz process
  tree is running passively, the controller is disconnected, independent status
  queries reported all 16 motors disabled, and both CAN links were ERROR-ACTIVE
  with zero bus errors. The physical pose after the aborted route must not be
  assumed neutral.

## 2026-08-07 physical C ABI implementation and left-only acceptance

- Added the installed `openarm_real.h` C11 ABI, `libopenarm_real.so`, and the
  pure-C `openarm_real_cli`. The ABI delegates lifecycle and motion to the sole
  ROS physical controller instead of opening a second raw SocketCAN authority.
  It covers readiness/detection, Connect, encoder-derived snapshots, individual
  joints, one or both TCPs with explicit metre/centimetre/inch units, J8 gripper
  position/torque commands, Stop, Disconnect, and E-stop. Coordinate and joint
  values remain IEEE-754 binary64 until the documented DaMiao wire codec.
- A small robot-left J4 `+0.05 rad` command and return passed through the C ABI;
  all seven reported robot-right joint values remained bit-for-bit unchanged.
  A subsequent paired High-far attempt aborted through the complete-feedback
  watchdog when the known robot-right J2 (`can0:2`) stopped replying.
- Added explicit `active_side_mask` controller recovery mode: `1` opens and
  controls robot-left/can1 only, `2` robot-right/can0 only, and normal
  `run-real.sh` remains strict bimanual mask `3`. Commands involving an inactive
  side are rejected. Snapshot consumers can distinguish live encoder state from
  the inactive fixed planning placeholder through the same mask.
- After the operator removed robot-right motor power, left-only mode used the C
  ABI to move the robot-left TCP from approximately
  `[2.22, 12.96, 7.81] cm` to encoder-derived
  `[28.47, 61.79, 40.75] cm`. Two requests for `[28, 67, 52] cm` ended safely as
  `segment_settle_timeout_holding_measured_pose`, with approximately `0.0306
  rad` residual joint error on the second attempt. The requested endpoint is
  therefore not claimed as achieved.
- Verification for the final sources used an isolated output root while the
  physical controller remained untouched. Every native component suite passed;
  the ROS package registered 19 tests, and the focused
  `test_real_control_session`, `test_portal_core`, and `test_real_c_abi` set
  passed 3/3. The entire 19-test ROS suite was not rerun against the final source
  while a live domain-0 physical controller was holding the arm.
- At this checkpoint robot-right motor power is off. The robot-left controller
  remains connected, enabled, and holding the measured far/high pose. Do not
  terminate it abruptly unless the arm is supported/clear to relax or its motor
  power has been removed.
