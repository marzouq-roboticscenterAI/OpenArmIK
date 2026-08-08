# OpenArmIK Codex handoff

Last updated: 2026-08-07. This file is the starting point for future Codex
sessions. Work only inside `/home/signalprocessing-dev/OpenArmIK` unless the
user explicitly changes scope. Read the root `SKILL.md` completely when the
user asks to use it, and maintain `.swarm/ledger.md` and `.swarm/learnings.md`
as that skill requires.

## Latest checkpoint: powered `run-real.sh` controller and portal

On 2026-08-07 the former read-only `run-real.sh` launch was replaced by a
passive-at-construction physical controller in
`openarm_ik_ros::real::RealControlSession`. The historical observer sections
later in this file remain incident history, not the current launch contract.

- `openarm_real.launch.xml` runs `openarm_ik_ros_node` with
  `physical_hardware=true`; can1 is robot-left and can0 robot-right.
- Construction opens no CAN socket. `/openarm_real/connect` requires the saved
  `~/.openarm_real_zero` calibration and fresh exact replies from IDs 1--8 on
  both arms before enabling anything.
- Connect seeds every J1--J7 MIT target through the exact binary64 inverse
  calibration and holds each J8 gripper at its measured raw encoder position.
  It sends zero-gain targets, enables all 16 motors, then ramps the official
  OpenArm gains over 50 cycles/500 ms.
- The worker is 50 Hz with an 18 ms complete-reply deadline and a 100 ms
  watchdog. Fault, Disconnect, E-stop, shutdown, and Ctrl+C disable; faults and
  E-stop also close both sockets. E-stop latches before locks and releasing it
  never reconnects.
- Portal Connect is a real Connect/Disconnect toggle. Stop also reaches direct
  Return-to-Neutral commands through `/openarm_real/stop`. E-stop calls the
  physical service synchronously and the UI surfaces failed disable
  confirmation.
- All existing ROS actions now dispatch to the real session in physical mode.
  The real session uses the C route planner, smooth binary64 seventh-order
  references, measured leg reseeding, live measured collision intervention,
  exact claw contact policy, and persistent encoder-torque contact evidence.
- Return to Neutral routes to the neutral TCP and then validates/appends the
  exact all-zero J1--J7 joint pose at no more than 0.02 rad between collision
  samples. It never substitutes an IK-equivalent redundant posture.
- Physical diagnostics are now distinct from virtual diagnostics. The portal
  accepts motion state only when the physical session reports fresh complete
  masks, idle/armed lifecycle, physical authority, and performed collision
  mitigation; it no longer relies on a false virtual-backend report.
- The Box marker is RViz-only in real mode. The physical arms mimic the path in
  empty space and J8 holds its measured opening; marker deletion remains in the
  sequence `finally` path.
- `test_real_control_session` proves construction/Stop remain passive and
  E-stop supremacy works without a connection. There are now 18 registered ROS
  adapter tests; update `scripts/build.sh` if that count changes.
- Root `README.md` was rewritten as the detailed operator/developer guide.

At this checkpoint, compilation and the focused display/physical-session tests
passed. Do not claim powered hardware validation until the incremental
Connect, Stop/E-stop, tiny move, Neutral, and demo procedure has actually been
completed on the attached arms and recorded here.

## Latest checkpoint: dynamic routing and Heart contact escape

Implemented and live-tested on 2026-08-07 for the normal virtual `run.sh`
workflow. This supersedes the earlier post-Cross checkpoint below.

- `model/include/openarm_route.h` and `model/src/openarm_route.c` add the public
  C11 `oa_route_plan_paired()` API. Inputs, graph state, IK/FK, collision values,
  returned joint solutions, and TCP endpoints are binary64 `double` throughout.
- The C planner tests actual edges through 17 IK samples plus conservative
  arm/tool capsules and finite-pole geometry. Exact claw-mesh contact remains
  confined to the dedicated Clap/Heart contact and retreat path. It retains
  the 25 mm planning gate and can leave a 10--25 mm start only while clearance
  is monotonically non-decreasing; the edge must finish outside 25 mm and that
  gate then resumes irreversibly.
- `NominalPathGuard::route_or_project()` calls that C planner. The portal keeps
  one final target and one command active, but after each completed leg it
  discards the unexecuted prediction and replans the entire remaining route
  from the latest measured joints. It independently re-proves the new first
  edge and revalidates encoder plus diagnostic evidence at action handoff.
  Between legs a 20 ms asynchronous gate waits up to two seconds for fresh idle
  controller diagnostics without blocking state, Stop, or E-stop callbacks.
  Cancel or E-stop clears every unsubmitted leg. A routed single-arm request may
  temporarily reposition both arms, then restores the nonselected claw at the
  requested final pose.
- The Heart failure was a policy mismatch: the HTTP retreat was proved with the
  dedicated terminal-contact escape, then the executor incorrectly rechecked
  it as an ordinary path starting inside the normal claw keepout. Terminal
  retreats now retain their exact monotonic escape proof; ordinary legs still
  use the C router. Heart completed both top contacts, the bottom contact, all
  three retreats, and the complete traced curve through the browser endpoints.
- DaMiao feedback can quantize an in-limit terminal solution by one
  `25/65535 rad` encoder code beyond a mathematical joint limit. The measured
  guard, C router, and final handoff now consistently allow only that one-code
  tolerance; planned IK states remain strictly in bounds. This fixed Cross
  becoming permanently stranded after its swapped pose.
- Final endpoint tests used strict command-result assertions, not merely an idle
  state. Startup-to-Clap-open completed two legs with two measured-feedback
  replans. The exact nine-step Cross completed, including a two-leg swap, then
  a distant paired target completed immediately afterward. Portal JavaScript
  now stops a demo when an accepted asynchronous goal later terminates without
  the expected completed/contact result.
- The native controller's shared-geometry keepout runs every 5 ms from measured
  joint feedback during motion. Dynamic route topology is recalculated at leg
  boundaries; it is not claimed to replan every control cycle. Runtime still
  discloses `collision_checked=false`, so none of this is physical or
  safety-rated collision certification.
- Software Stop is a separate behavior: it deliberately uses a disable stop,
  leaving the virtual session in `stopped_requires_restart`. Do not misdiagnose
  that expected post-cancel state as a collision-route failure.
- Focused route/contact and native real-time keepout tests passed. A clean
  test-profile build passed every native suite, then all 17 ROS tests passed in
  395.32 seconds with zero failures. The final normal-profile
  `run.sh --no-browser --no-rviz` rebuilt successfully, reported fresh idle
  state, passed the launch-tree fingerprint/manifest check, and stopped both
  ROS children cleanly on Ctrl+C. Source `ros2_ws/install/setup.bash` before
  invoking ROS CTest directly so generated message libraries are discoverable.

No physical motors were used or authorized by this routing work.

## Paused checkpoint: per-joint live RViz calibration

Implemented and brought live on 2026-08-06. This section is the current task;
the collision/demo/DREAD track immediately below remains deliberately paused.

- `openarm_real_observer` now holds independent binary64 calibration for
  robot-left/right J1--J7:
  `displayed = direction * (encoder - relaxed_reference) + display_offset`.
  Direction is exactly `+1` or `-1`; reference and offset are C++ `double`.
- The persisted file is versioned V2. The previous two-arm zero/sign file was
  migrated exactly and atomically replaced at `~/.openarm_real_zero`. Reconnect
  does not recapture zero: `capture_zero_on_connect` now defaults false.
- `/openarm_real/adjust_display_joint` supports query, direction flip, signed
  offset delta in degrees, and assigning the latest encoder reading an exact
  displayed angle. It changes only local `/joint_states` mapping and has no
  motor-write path.
- Prefer the compiled client, not a Python helper:
  `ros2 run openarm_ik_ros openarm_display_calibration_cli query|flip
  robot-left|robot-right J`, plus `offset ... DELTA_DEGREES` or
  `set-current ... DISPLAY_DEGREES`. `capture-relaxed` captures both complete
  arms without writing hardware.
- Both buses currently reply on IDs 1--8. A read-only query of all 14 joints
  reported live encoder data. `/joint_states` measured 100 Hz. A subsequent
  one-second wire audit counted can0 800 refresh/800 reply and can1 800/800,
  with zero other frames. This supersedes the earlier missing-can0-J2 note.
- Build passed. Focused tests passed 15 assertions: observer exact-ID mapping,
  per-joint isolation, sign behavior, binary64 persistence without narrowing,
  and exact migration of the old format. The compiled CLI queried both sides.
- The last sanitized real launch used `can0=robot-right`, `can1=robot-left`,
  all motors torque-disabled, and only DaMiao `0xCC` status refreshes. No
  observer, recorder, RViz, portal, or virtual-control process was running when
  this checkpoint was written on 2026-08-07.
- The full engineering `openarm_ik.rviz` explicitly starts with
  `rviz_default_plugins/MoveCamera`, so left-drag orbits, middle-drag pans, and
  the wheel/right-drag zooms while inspecting joint offsets.

Operator workflow: first confirm both arms are completely relaxed in the pose
that should be the reference, then run the compiled client's `capture-relaxed`
command once. For each joint, gently move only that physical joint and report,
for example,
`robot-left J1: wrong direction, offset by about 8 degrees`. Flip only that
joint for “wrong direction”; do nothing to its sign for “right direction”. Add
the estimated signed display offset, query it, and let the operator say whether
the offset correction went the correct way. If not, undo/reverse that delta.
Never enable a motor for this procedure. Do not start the range-of-motion trace
until all per-joint directions and offsets are visually accepted.

Current operator acceptance table (range remains a later, separate trace):

| Side | Joint | Direction | Display offset | State |
| --- | --- | --- | --- | --- |
| robot-right | J1 | `+1`, operator-confirmed correct | `-9.070350 deg`, operator-confirmed | Accepted |
| robot-right | J2 | `+1`, unverified | `+37.686576 deg`, operator-confirmed | Visual offset accepted; direction/range still blocked by intermittent telemetry |
| robot-right | J3--J5, J7 | `+1`, operator-confirmed correct | `0 deg`, operator-confirmed | Accepted |
| robot-right | J6 | `+1`, operator-confirmed correct | `+4.830385 deg`, operator-confirmed | Accepted |
| robot-left | J1--J7 | `+1`, operator-confirmed correct | `0 deg`, operator-confirmed | Accepted |

After right J1 was accepted, can0 J2 disappeared again while IDs 1 and 3--8
continued replying with disabled status 0 and normal temperatures. Both CAN
links remained error-active with zero errors/drops/bus-off events. The observer
was changed to publish each exact-ID joint independently: right J1 and J3--J7
now continue updating while only missing J2 holds its last known pose (or
neutral if it has not answered since launch). The exact-ID mapping test proves
the gripper cannot shift into J7. This is a truthful partial view, not a claim
that J2 feedback existed during that outage.

Later in the same session right J2 resumed normal telemetry: 491 replies for
492 refreshes over five seconds, status 0, and about 28/25 C MOS/rotor. The
apparent startup transition was not movement-triggered. RViz publishes neutral
for 1.5 seconds before auto-connect, then the old reference (`-7.77017 deg`)
was applied to the live raw angle (`-50.4567 deg`), yielding `-42.6866 deg`.
`set-current robot-right 2 0.0` corrected and atomically saved an offset of
`+42.686576 deg`. During an 18-sample operator motion window the raw J2 encoder
remained between `-50.4786` and `-50.4567 deg`; no other right motor moved
either. Treat J2 direction and all J2 range claims as blocked until a physical
motion produces a measured encoder delta.

Multi-angle refinement subsequently accepted right J1 at `-9.070350 deg`.
The operator judged right J2's correction about 5 degrees too far, so its
offset was reduced from `+42.686576` to `+37.686576 deg`. Right J6 measured
`-4.830385 deg` and was set to display zero, producing a saved
`+4.830385 deg` offset. Immediately afterward J2 again returned zero replies
to 24 refreshes; those J2/J6 refinements remain provisional until visually
confirmed, and J2 direction/range remain blocked.

The operator then accepted the final right J2 and J6 visual alignment. The V2
persistence file was reread and verified to contain right J1 offset
`-0.15830747615777785 rad`, right J2 `+0.65775483861337614 rad`, and right J6
`+0.084306125360494172 rad` (respectively `-9.070350`, `+37.686576`, and
`+4.830385 deg`). These are saved display offsets, not motor firmware changes.

Directional range capture began after that acceptance. The installed
`openarm_calibrate_gui` is a C11/GTK read-only recorder: it transmits only the
DaMiao `0xCC` status-refresh request for the selected motor, stores every
timestamped raw and wrap-unwrapped binary64 encoder sample in acquisition
order, and never enables or commands a motor. It labels
`can0=robot-right`/`can1=robot-left` and atomically saves schema V2 data to
`calibration/openarm_hand_range_calibration.json`. Launch it from the repository
root so the relative output stays in scope. For each joint, begin relaxed,
press Start, move slowly to one operator-chosen safe limit without forcing a
hard stop, pass through relaxed to the opposite safe limit, return to relaxed,
then press Stop. The ordered trace—not min/max endpoints alone—establishes the
direction. Do not accept any joint whose encoder does not visibly change;
robot-right J2 remains especially suspect because its telemetry is intermittent.

The first robot-right J1 capture completed with 816/816 successful reads at
49.3 Hz, 279.66 degrees of encoder extent, and 544.96 degrees of ordered round-
trip path. The old running GUI still requires one explicit Save to preserve
that in-memory path. During the capture, the default Mesa software RViz process
rose to about 905% CPU and starved the desktop; it was stopped without stopping
the observer. RViz was then restarted alone on integrated GLX at about 33% CPU.
`openarm_sanitize_snap_environment` now chooses integrated rather than software
GLX by default on Wayland, and the newly compiled recorder atomically saves
after every Stop so future completed captures survive renderer failures.
The operator did not press the old instance's explicit Save before the process
ended: only the one-line diagnostic remains. The full ordered trace is absent,
so right J1 range/direction was not accepted and its sweep must be repeated
with the auto-saving build when this tangent resumes.

The subsequent robot-right J2 flicker was not encoder instability or a second
ROS publisher. With the recorder selected on J2, its other CAN RAW socket sent
`7FF#0200CC0000000000`; Linux local loopback delivered that request to the
observer. `collect_replies` had not restricted arbitration IDs before decode,
so those request bytes were accepted as J2 at about `-12.42 rad`, `-45 rad/s`,
and `-54 Nm`, producing `-11.6288 rad` after display calibration. The observer
now requires the exact `send_id + 0x10` reply ID and matching payload motor ID
before decoding. A regression test pins rejection of `0x7ff`. The focused test
passed, the installed observer was replaced without stopping the recorder, and
199 live J2 samples then stayed between `-0.088029414` and `-0.087647938 rad`.
There is exactly one `/joint_states` publisher. All observed traffic remained
status refresh/reply only; no motor was enabled or commanded.

## Pause checkpoint: resume the pre-live-RViz work here

Recorded 2026-08-06 at the user's request. The user is deliberately pausing the
earlier collision/demo/DREAD/research track for a live physical-RViz tangent.
Do not confuse tangent fixes with completion of the earlier track. This pause
checkpoint supersedes conflicting older same-day wording or verification notes
later in this file.

### Completed implementation on the paused track

- Pick/place box cleanup was fixed in both layers. `portal.js` always posts
  `scene-box=false` from the sequence `finally` block, including cancellation
  and failure. `openarm_ik_ros_node.cpp` now uses the same `kBoxMarkerId=0` for
  Marker ADD and DELETE; the previous DELETE used ID 1 and could not remove the
  latched ID 0 marker. `ScenePropLifecycle` statically pins both requirements.
- Heart/Clap no longer permit claw STL contact. The exact pinned v1.0
  `hand.stl`/`finger.stl` FCL evaluator exposes a 25 mm expanded rail envelope.
  Portal convergence targets 24 mm exact hand-mesh separation, accepts no less
  than 23 mm for quantization tolerance, and keeps all other cross-claw,
  arm/arm, and pole pairs at the ordinary 25 mm planning gate. The measured-
  feedback monitor stops on entry into that 25 mm rail envelope for virtual and
  physical backends. The next path must be a proved monotonically opening
  retreat. No STL touch is permitted.
- The box/rail behavior and public C collision API are implemented in
  `model/include/openarm_collision.h`, `model/src/openarm_collision.c`,
  `control/src/control_core.cpp`, and the portal sources/tests. The hand/finger
  triangle data remain generated C/binary64 with a narrow C++ FCL backend.
- `dread.sh` now invokes the sudo CAN helper through `bash`, cleans up its RViz
  child on wizard exit/Ctrl-C, uses the current installed
  `openarm_v10_bimanual.urdf`, and selects
  `dread/config/motor_map.openarm_v10.yaml`. The collected M1/Ranger maps remain
  reference-only and must not seed the active v1.0 calibration.
- DREAD's active map and solver now use the official v1.0 motor lineup, exact
  current arm limits, and two 0..0.044 m prismatic finger joints. Every safe
  two-stop capture fits binary64 `scale=(urdf span)/(encoder span)` plus offset,
  rather than assuming a 1:1 angle. J4 captures only its safe stop and retains
  its prior scale. Closed/open J8 feedback fits the complete 44 mm travel.
- Before any enable, the live wizard requires read-only status from every
  expected motor on every selected bus. It primes zero `kp`, `kd`, and torque
  while disabled, repeats immediately after enable, checks enabled telemetry
  and faults, and disables on every exit. An old rotary-gripper mapping cannot
  be preserved through the skip path.
- DREAD and all physical launch paths now use the operator-confirmed assignment
  `can0=robot-right`, `can1=robot-left`. The dual-channel DM-USB2FDCAN has one
  shared USB identity, so this must remain explicit unless wiring changes.
- `OPENARM_V1_RESEARCH.md` records the official v1.0 hardware, URDF, CAN, motor,
  gripper, ROS, library, and local-integration audit. All ten official upstream
  repositories remain full local clones. Remote histories/tags were refreshed
  on 2026-08-06 without moving the pinned detached worktrees; the latest
  description commit did not change the v1.0 subtree.
- DREAD documentation was rewritten for this repository/current URDF. Main
  README, ROS README, CLAUDE.md, and this handoff were updated from the old
  “exact touch/4 mm overlap” wording to the no-STL-touch rail envelope.

### Verification already obtained

- DREAD hardware-free suite: `24 passed`.
- A complete both-arm DREAD `--transport sim --yes --dry-run` rehearsal passed
  before the final incompatible-gripper-skip guard; rerun it once on resume.
- The one-job native test build passed CAN 1/1, Model 6/6, Commission 2/2,
  Transport 3/3, Control 5/5, Runtime/installed consumers/header matrix/ABI and
  the frozen symbol checks. The ROS package compiled and registered exactly 16
  tests, but the top-level build intentionally refused to publish its launch
  stamp because source files were edited while that long build was running.
- After the later observer/tangent edits, a focused incremental ROS build
  passed, `test_real_observer_core` passed, and DREAD again passed 24/24.
- No post-rail-change full ROS CTest result exists yet. Do not reuse the older
  16/16 contact-era result as proof of the new 23--25 mm envelope.

### Required resume sequence

1. Review the whole dirty diff; preserve unrelated/user changes. No final commit
   for this checkpoint has been made.
2. Run `bash scripts/build.sh --incremental --tests --jobs 1` without editing
   files during it so a trustworthy launch stamp is produced.
3. Source `/opt/ros/lyrical/setup.bash` and
   `ros2_ws/install/setup.bash`, then run at minimum `test_portal_core` and the
   long `test_virtual_control_session`; preferably run all 16 ROS CTests with
   `--output-on-failure`.
4. Rerun DREAD's 24 tests and the complete simulated both-arm dry-run after the
   final skip guard/default-side changes.
5. Exercise pick/place in the virtual portal through finish and cancellation;
   verify a Marker DELETE for namespace `openarm_scene`, ID 0, and verify the
   box is absent afterward. No real arms are needed.
6. Exercise the complete virtual Clap and Heart sequences. Assert exact hand
   STL gap is positive and at least 23 mm at every accepted endpoint, the target
   is approximately 24 mm, all other pairs retain at least 25 mm, and every
   convergence retreats successfully.
7. Run a stale-wording/source sweep for “touch”, “4 mm overlap”, foreign
   M1/Ranger active paths, and the superseded side mapping. Update ledger and
   learnings with final evidence before declaring the paused track complete.

### Physical safety/blocker state at pause

- No motor enable, LED-enable, MIT motion, zero, or firmware write was sent in
  this work. A live wire audit of the read-only observer saw 768 `0xCC` refresh
  requests, 720 replies, and zero unexpected/enable/MIT frames.
- Both CAN-FD interfaces are UP at 1 Mbit/s arbitration / 5 Mbit/s data with
  zero bus error counters. `can1` (physical left) replies on IDs 1..8. `can0`
  (physical right) replies on IDs 1 and 3..8; J2/ID2/reply 0x12 is still absent.
- Keep physical calibration and all enable/green-LED tests paused until can0 J2
  is repaired and a fresh read-only 16/16 preflight passes. Simulation work can
  continue independently.

## 2026-08-06 delta

The host is fully provisioned with ROS 2 Lyrical and the native toolchain. The
contact work below was compiled and exercised; the earlier unverified/missing-
toolchain note was from a concurrent stale session and was false for this host.

### Expanded claw-rail envelope and retreat

- `model/openarm_collision` has a scoped
  `OA_COLLISION_CONTACT_TERMINAL_CAPS` evaluator backed by the exact pinned
  OpenArm v1.0 `hand.stl` and `finger.stl` triangles. The public boundary is C;
  the narrow `openarm_claw_mesh.cpp` backend uses double-precision FCL. Only the
  mutually approaching hand-housing mesh pair targets 24 mm separation and may
  never be planned below 23 mm. The other eight cross-claw mesh pairs, every
  other arm pair, and the central pole retain 25 mm; non-finite input fails
  closed. No STL mesh is allowed to touch.
- Portal `/api/v3/converge` no longer assumes one fixed claw/TCP width. Starting
  from the freshest encoder-derived joint state, `validate_convergence_contact`
  searches a bounded 15--70 mm stop-radius corridor and accepts only a complete
  17-sample branch-continuous path ending inside the expanded rail envelope. It targets
  a 24 mm gap so encoder quantization cannot miss the 25 mm rail envelope; the
  real-time monitor stops on envelope entry for virtual and physical backends.
  Converge succeeds only on a STOPPED
  event whose runtime contact report says `OA_RUNTIME_STOP_CAUSE_CONTACT`; plan
  completion, another keepout, and missing evidence abort.
- `/api/v3/retreat` is the only post-contact portal path. It must begin in the
  scoped terminal pair, never move deeper, clear the pair, and preserve the
  normal clearance for everything else. The controller independently promotes
  an ordinary paired plan to the scoped policy only after all 17 planned
  waypoints prove the same monotonically opening retreat. Virtual tangent stop
  is restricted to contact-monitored converge, so it cannot stop the retreat.
- Clap converges twice at `[0.34, 0.00, 0.86] m`. Heart uses that top
  convergence and
  `[0.30, 0.00, 0.74] m` for its bottom cusp. From the zero-joint virtual
  startup, JavaScript prepends the tested low/mid/heart-lobe lead-in, and routes
  every immediately following non-contact waypoint through retreat. These are
  hand-housing rail-envelope stops with fixed gripper opening, not proven
  fingertip, force, environment, or physical contacts.
- Normal portal moves remain strict. The 25 mm sampled planning gate and 10 mm
  measured-feedback intervention floor were not lowered. The runtime monitor
  still evaluates every feedback cycle.

### Other current behavior

- Portal movement limits are `[0.5, 1.0]`, default `1.0`; there is no overdrive
  above commissioned virtual limits. All XYZ paths and public unit adapters use
  IEEE-754 binary64. Portal ingress accepts explicit `cm`, `in`, or `m` and
  normalizes exactly once to canonical metres.
- The portal embeds cropped, interactive, unfiltered real-RViz MJPEG pixels.
  It contains no palette/filter control and does not expose RViz option panels.
- Runtime V1 remains frozen and the exported manifest remains exactly 57
  symbols. Centroid and mirrored runtime adapters remain header-only; converge
  is the stateful contact planner.

### Verification completed 2026-08-06

- `./scripts/build.sh --tests --incremental --jobs 1`: all native CAN, model,
  commission, transport, control, Runtime, installed-consumer, header, ABI, and
  57-symbol-manifest tests passed; exactly 16 ROS tests were registered.
- The complete `test_virtual_control_session` passed in 315.09 s. It includes
  proved top and bottom contact, false-contact rejection, contact-to-retreat,
  all portal targets, cancellation, heartbeat, and lifecycle cases.
- `test_portal_core` passed, including a quantized DaMiao-feedback simulation of
  the full clap and heart sequences. The test proves ordinary moves reject the
  same contact endpoints, both contact corridors retain every other keepout,
  and each retreat returns to the strict corridor.
- All remaining 15 ROS tests passed. Five executable tests initially failed to
  load the freshly rebuilt message library because CTest was invoked without
  sourcing `ros2_ws/install/setup.bash`; rerunning those five with both ROS and
  the workspace sourced passed 5/5. Always source before direct CTest.
- After the exact-mesh update, the focused portal regression passed from the
  actual zero-joint virtual startup through the complete entry, two Clap
  contacts, all three Heart contacts, and every retreat. Live loopback API
  testing selected 41.5 mm for the measured Clap branch and 41.0 mm for the
  measured Heart-bottom branch. Both halted in the real-time monitor with
  `cause=OA_RUNTIME_STOP_CAUSE_CONTACT`, `outcome=completed`, and both scoped
  retreats completed back to their open waypoints. No physical arm moved.

### Still requires physical hardware validation

No physical arm was moved in this verification. DaMiao torque/encoder contact
thresholds, calibration, actual finger geometry, mounting tolerances, and safe
speeds must be commissioned and validated on hardware before treating any
virtual target as physically safe.

## What currently works

- On a provisioned dev host with ROS 2 Lyrical, RViz2, xacro, build tools, and
  Firefox installed, `bash run.sh` performs an integrity-gated incremental
  build and opens the loopback-only portal. `bash run.sh --no-build
  --no-browser` is useful for automated checks. Ctrl+C should cleanly stop the
  portal and ROS children.
- `scripts/launch_rviz.sh` launches the separate stock RViz engineering view.
  The web portal now embeds **real RViz pixels** as MJPEG captured from the
  live `rviz2` window, not the older in-browser WebGL proxy.
- The C/C++ model, unit conversion, IK/FK, CAN transport, commissioning,
  runtime, control API, ROS node, and portal server are implemented. Browser UI
  code is JavaScript; do not add Python production code. Existing Python test
  and ROS launch infrastructure is allowed.
- Portal coordinates default to centimetres and can explicitly use `cm`, `in`,
  or `m`. The public unit-aware C ingress and all internal coordinate
  calculations use IEEE-754 binary64 `double`; canonical internal units are
  metres. Do not introduce a float narrowing conversion.
- The OpenArm v1.0 bimanual description/model is pinned to upstream
  `enactic/openarm_description@6c7b720f1ba48e8bafa3a3dc752c45f397b42221`.
  The generated URDF and redistributed viewer collision meshes come from this
  pin and include the upstream license.

## Bimanual motion, real-time keepout, contact, E-stop (commit 5d3eb56)

Four simultaneous-motion planners now share one validated body in
`control/src/control_core.cpp`: paired, centroid (both claws translate by the
vector carrying their measured midpoint to a target), mirrored (one claw
commanded, the other mirrored across y = 0), and converge (both claws advance
on a shared point until contact torque, a keepout violation, or the planned
prefix ends). All four inherit identical all-or-nothing, identity-binding,
freshness and expiry semantics.

`model/openarm_collision.{h,c}` is the single source of truth for the keepout
geometry, in C. The portal guard delegates to it. It was verified bit-exact
against the previous portal implementation over 400k randomized scenes.

Two thresholds exist and must stay distinct: planning requires 25 mm
(`oa_collision_required_clearance_m`), the real-time monitor intervenes at
10 mm (`oa_collision_intervention_clearance_m`). Sharing one value aborted
motions the planner legitimately accepted, because the measured arm always
trails its reference; `test_virtual_control_session` catches that regression.

Simulated contact reports **servo effort while the plant is held**, not
penetration depth. Holding the plant at the obstacle surface keeps penetration
near zero, so a penetration-based reaction stays negligible however hard the
arm pushes. Do not "simplify" it back.

`oa_estop_assert` is a process-wide lock-free latch with no handle and no lock,
safe from any thread or a signal handler, sampled at the top of every control
cycle in every lifecycle state before any other work. It is a software
interlock, not a hardwired E-stop.

Runtime additions live in `openarm_runtime_motion.h`. `openarm_runtime.h` stays
byte-identical to its frozen copy; centroid and mirrored remain header-only
adapters. Stateful converge/contact support stays on the additive motion
surface. The frozen exported-symbol manifest is exactly 57 symbols.

The portal's 3D view is now a real RViz window (`rviz/openarm_bare.rviz`,
`Panels: []`) rather than the in-browser WebGL proxy. Renderer/HiDPI setup is
shared in `scripts/lib/rviz_env.sh`. The Firefox viewer oracle was retired with
the canvas it drove. The ROS test inventory is currently 16.

## Python conversion (commits ae3a383 .. HEAD)

All seven Python test harnesses in the ROS package and the Python launch file
are gone. 1549 lines of Python became 235.

- `test/c/*.c` are C11 sharing `test/c/test_support.h`: test_generated_urdf
  (libxml2), test_no_can_linkage, test_invalid_expiry_parameter,
  test_visualization_urdf (libxml2 + json-c + libcrypto).
- `test/c/*.cpp` are C++ because ROS 2 exposes actions only through
  rclcpp_action: test_active_sigint, test_cli_server_lifecycle,
  test_ros_contract. The last one carries a small loopback HTTP client rather
  than adding a curl dependency.
- `launch/openarm_ik_rviz.launch.xml` replaces the Python launch file.
  robot_state_publisher's robot_description needs `type="str"`; without it the
  launch parameter parser infers a type from the URDF payload and fails.

The 235 remaining Python lines are deliberate and should stay:

- `model/tools/generate_model.py` drives xacro, which is itself Python.
  Replacing it means reimplementing xacro.
- `model/tests/test_reference.py` and `test_generator.py` are an independent
  oracle: they parse the URDF with a different implementation to cross-check
  the C model. Porting them to C would delete the independence that is their
  entire value.

Two ports changed behaviour deliberately, both noted in their headers: the
visualization test's 12 random postures come from a fixed LCG rather than
Python's Mersenne Twister, and XML serialization comparisons use libxml2's
serializer on both sides rather than ElementTree's bytes.

## Historical queued work

This section is older planning context. Read the `2026-08-06 delta` above
first; several items below have since landed partially or fully.

Do these in order. Each is independently committable and testable.

### 1. RViz inside the web portal (replaces the separate window)

The user wants RViz pixels in the portal page, not a second window. The current
separate-window setup in `launch_web_portal.sh` is the wrong shape and should be
removed once this lands.

`.swarm/web_rviz_recon.md` already researched this; follow its recommendation:
capture the live `rviz2` top-level X11 window with XComposite
(`XCompositeRedirectWindow` + `XCompositeNameWindowPixmap`), encode JPEG with
libjpeg, and serve `multipart/x-mixed-replace` from the existing Beast server in
`src/openarm_portal.cpp`. The page shows a plain `<img>`; no new JS machinery.
Reject Xvfb/VNC/noVNC: those components are not installed and drag in Python.

All libraries are present and confirmed on this host: x11 1.8.13,
xcomposite 0.4.6, xext 1.3.4, libjpeg 2.1.5, xfixes 6.0.0.

Two things to prove before building it out:
- RViz's Ogre render widget may be a separate native X11 child window. Verify the
  captured top-level pixmap actually contains the moving 3D scene, not just the
  Qt frame. If it does not, walk the child tree and capture that child. Do NOT
  fall back to a root-window crop; that captures whatever overlaps RViz.
- The portal currently forces software rendering on Wayland. Confirm capture
  works in that mode, since that is the default path.

Integration notes: `dispatch_static`/`dispatch_api`/`dispatch_urgent` are all
one-shot request handlers with `stream.expires_after(...)`. An MJPEG stream is
long-lived and needs its own dispatch lane with no expiry, its own admission
cap, and cancellation on shutdown. Capture on one thread into a single shared
latest-frame buffer; do not capture per client. Capture only while a client is
attached, with a heartbeat so an idle scene still refreshes.

### 2. Expose the new motions through ROS, the portal, and the CLI

`oa_controller_plan_{centroid,mirrored,converge}_tcp` and the E-stop exist and
are tested at the C ABI level only. Needed:
- new actions alongside `MovePairedTcpScaled` in `openarm_control_msgs`;
- handlers in `openarm_ik_ros_node.cpp` routing to the runtime entry points in
  `openarm_runtime_motion.h` (centroid and mirrored are header-only adapters;
  converge is `oa_runtime_plan_converge_tcp_body`);
- `SessionCommand::Kind` variants in `virtual_control_session.hpp`;
- portal routes and CLI subcommands;
- an always-available E-stop route on the existing urgent lane, calling
  `oa_runtime_estop_assert`, plus a CLI `estop` subcommand.

### 3. Clap and cross-arms demos

The claws only need to come close, not touch, so the 25 mm planning gate is not
the blocker it first appeared. Measured clearance stays above the gate down to
roughly 0.30 m between claws at x=0.30, z=0.35; the planner starts refusing
below that. Crossing is NOT characterised: the probe used to sweep it had a
state bug and misread OA_CONTROL_ESTATE from an unsettled controller as
infeasibility. Re-measure before designing the waypoints.

### 4. Box in RViz plus pick / lift / place

The box needs a scene object. The existing `oa_sim_contact` sphere gives the
grasp resistance; a box also needs a visual marker published to RViz and a
keepout entry if it should be avoided rather than grasped.

### 5. Web page demo buttons

`portal_page.cpp` already has a preset mechanism (`nominal_targets` serialized
into `#portal-targets`). Add demo presets through the same path so the buttons
only fill fields and never submit motion, matching the existing contract.

## Open issue: portal shutdown time

Teardown via SIGTERM measured 25.8 s with RViz in the tree. The RViz close
helper accounts for only 1.9 s of that, so most of it is portal/ROS teardown.
Not yet isolated to a cause, and not established whether the RViz addition made
it worse. `test_ros_contract` does not cover this path; it bounds the bare
`ros2 launch` teardown, not `launch_web_portal.sh`.

## Delivered since the bimanual core (99362c0 .. c53a1cb)

- RViz streams **into the portal page** as MJPEG on `/api/rviz/stream`, captured
  from the live rviz2 window with XComposite and libjpeg
  (`src/rviz_capture.cpp`). Two traps: the Ogre content is in the *top-level*
  window, not the render child (the child reads back uniform), and `XGetImage`
  on a Pixmap returns **zero RGB masks**, which silently yields black frames if
  used unchecked. The child's geometry is reused as the crop rectangle to drop
  the Qt chrome.
- Ctrl+C fixed. The launcher's single-instance lock on fd 9 leaked into every
  child, so a surviving grandchild (robot_state_publisher) kept it held and the
  next launch failed with "already running". All spawns now close it. Measured
  through a real controlling PTY: 0.90-0.95 s, no leftovers.
- CLI demos: `clap`, `cross`, `pick-place`, `mirror`, `estop`, `estop-release`.
- E-stop on the portal urgent lane (`/api/estop`, `/api/estop/release`), and
  motion is refused at the portal boundary while latched.
- Graspable scene box marker plus web demo-pose buttons that fill both targets.

### Tool capsule: measured, and it is too small

The collision meshes are **millimetres**. Measured half-extents about the
segment-6 tool axis: hand corner 88.8 mm, finger corner 82.9 mm, link7 44.7 mm.
`kToolRadius` is 75 mm, so the capsule **under-covers the gripper by 13.8 mm in
the jaw direction**. This is a real gap and was not introduced here; it is
recorded rather than silently changed because widening it to ~89 mm makes the
guard stricter and would need every documented preset re-measured against the
2.5 cm gate (`test_virtual_control_session` asserts a 2.6598 cm minimum).
Decide that deliberately.

Shrinking the radius is not an option and does not help crossing. The waste is
the shape: the gripper is a 57 x 168 x 16 mm flat plate, and a circular capsule
sweeps 5.05 L around 0.15 L of actual hardware. Only an oriented box would fit
it tightly.

### Cross-arms: what is actually achievable

A genuine crossing **is** possible, but only stacked in Z, not side by side.
Working waypoint: left claw to y=-0.04 at z=0.62, right claw to y=+0.04 at
z=0.22, both at x=0.30, commanded together, measured at 26.5 mm clearance.

The window is narrow and was mapped empirically over roughly thirty geometries:
- deeper than about 4 cm either side is unreachable, or the leading tool enters
  the trailing forearm (ARM_ARM, segment 6 vs 4) at ~10 mm;
- pulling either arm back in x brings its tool near the central shaft (POLE,
  segment 6) at ~9.7 mm;
- side-by-side passes cannot work at any x-stagger, because two 75 mm tool
  capsules need 175 mm of centre separation.

The current Clap demo no longer stops at that older 24 cm separation. It uses
the scoped exact-mesh corridor documented above, stops on proved hand-housing
contact in the real-time monitor, and then performs a validated retreat. The
ordinary move and cross-arms paths still retain the 25 mm gate.

Do not "fix" a failure here by lowering the gate without saying so.

## Full-surface verification

`scripts/functional_sweep.sh` runs 25 checks against a live stack: portal
routes, the MJPEG stream, every CLI motion and demo, the box, the rejection
paths, and the E-stop. Run it after any change that touches motion; it found
three defects the unit and ROS suites did not, because each only appears when
surfaces are exercised in sequence.

Ordering matters in that script. `converge` ends with the claws inside the
keepout intervention floor on purpose, and the demos after it only pass because
the monitor permits a retreat from there.

### Portal surfaces

Single-arm moves (`/api/v3/move`) pin the other arm to its measured TCP and may
be shortened to a best-effort prefix. The dual move (`/api/v3/move-both`, the
Move Both Arms button) commands both targets in one paired command and is
**never** shortened: stopping one arm early while the other continues would
change the relative geometry the guard's samples were validated against. The
guard interpolates both arms along their own lines when `MoveRequest::dual` is
set, so clearance is sampled on the pair actually in motion.

### Verify the viewer with a client attached

Every check before 7c2d481 ran `run.sh --no-browser`, so nothing attached to
`/api/rviz/stream`, the capture never ran, and a crash on that path went
unseen until a real browser opened. When touching the capture, attach a client
during startup, which is when rviz2 is creating and destroying windows:

    setsid bash run.sh --no-build --no-browser &
    for i in $(seq 1 60); do curl -s --max-time 2 \
      http://127.0.0.1:8080/api/rviz/stream -o /dev/null & sleep 0.5; done

Xlib's default error handler calls `exit()`, so any unguarded X call in the
portal is a process-killer, not a logged warning. `RvizCapture` installs a
counting handler; `test_rviz_capture` guards it.

### Waypoints must clear the GUARD, not just the monitor

Two gates exist and they are not the same. The real-time monitor intervenes at
10 mm; the portal's pre-flight guard requires 25 mm across seventeen sampled
path waypoints and additionally checks IK branch continuity. Waypoints tuned
only against the monitor are silently unusable from the portal, which is how
eight of fourteen demo presets came to be rejected while the CLI demos worked:
the CLI goes through the action, which does not run the guard.

Method: link `libopenarm_portal_core.a` and run
`NominalPathGuard::validate_or_project` over each waypoint, once from neutral
and once step by step through each sequence, before believing a demo works.

Closed poses sit at +/-0.15. Below +/-0.13 the left tool passes inside the
central shaft gate. Some poses are only reachable partway through a sequence,
because the guard's IK branch continuity check refuses a large single jump;
those are labelled mid-sequence in the UI and covered by the Run buttons.

### Frame rate

The portal requests the integrated GPU renderer, unlike `launch_rviz.sh`. The
software-rendering default exists for live-resize flicker on a hand-dragged
window; the captured window is never resized, and llvmpipe gave 3-4 fps. Do not
force NVIDIA PRIME: after the 2026-08-06 restart the NVIDIA kernel driver was
unavailable and PRIME GLX aborted RViz before its first frame, while accelerated
AMD GLX remained healthy. The repaired raw stream delivered 107 frames in three
seconds (about 35.7 fps) against a 28 ms cap. The sweep asserts at least 30 fps.

### Three rules that are easy to break again

1. **A monitor stop is a success only for converge.** Mapping STOPPED to
   COMPLETED for everything makes an ordinary move that was halted short of its
   target exit zero. That hid a pick-place where all seven steps "completed"
   with the arms 10 cm from the commanded pose and the box untouched.
2. **The keepout monitor must intervene only while clearance is worsening**,
   never on the absolute value. Vetoing on the absolute value makes any pose
   inside the floor inescapable, including by a command that moves the arms
   apart. The comparison is "not worsening", not "strictly improving": for the
   first cycles of a retreat the arms have barely moved and clearance is equal.
3. **The box grasp radius is 0.05 m for a reason.** The clap midpoint passes
   0.064 m from the box and clap closes inside the grasp separation, so a
   larger radius makes the clap demo pick the box up and carry it away.

Sequencing note for anyone extending the sweep: the session holds a reservation
until the previous goal is terminal and the portal state is briefly stale on
either side of a command. Post motion back to back and you get exit 4 from the
CLI or 409 from the portal, which looks like a broken command but is a racing
harness. `climove` and the debounced `wait_idle` in the sweep exist for that.

## Outstanding requests not yet implemented

- Nothing from the original request list. Centroid, mirrored and converge are
  now exposed through the `MoveBimanual` action, the session, and the CLI.
- Open: the tool capsule under-coverage recorded above, and the portal
  shutdown-time note further down (Ctrl+C itself is fixed and fast).
- Clap and Heart intentional contact are complete for the virtual fixed-opening
  hand geometry. Cross-arms remains a strict-clearance motion and must not use
  the contact exception.

## Safety and behavior that must not regress

This is still a virtual-only system. No physical arms or CAN-FD adapters were
available, so physical discovery, encoder calibration, zeroing, polarity,
motion, collision behavior, and E-stop wiring have **not** been validated. The
controller deliberately reports `collision_checked=false`; never describe the
portal guard as physical certification. The portal software-stop button is not
a hardwired safety-rated E-stop.

The portal guard validates every accepted target as a complete 17-waypoint
straight-line plan from fresh measured joint state, using public binary64 IK/FK,
joint/branch bounds, arm capsules, tool capsules, and the central-pole model.
For an impossible or unsafe finite target it:

1. normalizes the requested ray without overflow, including near-`DBL_MAX`;
2. searches at most 2 m using 64 coarse samples and progressively denser
   16/32/48 non-monotonic refinement scans;
3. continues after isolated numerical IK failures;
4. records the *actual failing waypoint* of any pole/inter-arm violation as an
   irreversible ray boundary, discarding any previously accepted endpoint past
   it and never routing around the obstacle;
5. submits the farthest sampled, fully revalidated prefix before that boundary;
6. rejects rather than reports success if less than 1 mm of safe progress can
   be proven.

Do not replace the waypoint boundary with the failed candidate endpoint; that
was a real phase-shift safety bug. `CommandReservationGate` reserves a command
before state capture/guard evaluation. Stop invalidates that reservation before
submission, and `/api/stop` has a bounded urgent worker/admission lane separate
from expensive ordinary API work. Preserve both properties.

Portal motion now defaults to 100% of configured virtual velocity,
acceleration, and jerk limits, with a 50–100% slider; the former fixed value was
50%. The percentage is a limit scale, not a linear travel-time promise. The
strict v3 portal request preserves the binary64 value through the additive
`MovePairedTcpScaled` action and `SessionCommand`, while legacy v1/v2 requests,
the original `MovePairedTcp` action, the CLI, and the deprecated PoseArray path
remain at 50%. The synchronized seventh-order motion profile is unchanged.
Any physical speed change still requires hardware commissioning and safety
validation.

## Useful virtual coordinates

All values below are centimetres in body frame. The right-arm Y coordinate is
normally the negative mirror of the left:

| Purpose | Left XYZ | Right XYZ |
| --- | --- | --- |
| Near low | `[15, 22, 15]` | `[15, -22, 15]` |
| Forward mid | `[30, 22, 30]` | `[30, -22, 30]` |
| Forward outer | `[30, 50, 30]` | `[30, -50, 30]` |
| Near-max forward | `[48, 17, 35]` | `[48, -17, 35]` |
| Outer high | `[25, 58, 45]` | `[25, -58, 45]` |
| High far | `[28, 67, 52]` | `[28, -67, 52]` |
| Impossible-ray test | `[5000, 5000, 5000]` | `[5000, -5000, 5000]` |
| Genuine pole test | `[40, 5, 40]` | `[40, -5, 40]` |

High far displaces a TCP about 73.6 cm from neutral. The pinned URDF offsets
give a 74.7--74.8 cm shoulder-to-TCP centreline upper bound, so call this
"near-full audited reach," not an absolute continuous-workspace maximum. All
1,800 quantized preset endpoint/cross-state transitions passed the sampled
2.5 cm nominal gate; the recorded minimum was about 2.6598 cm.

For the inter-arm guard test, first move both arms to their respective Near-max
forward targets, then request the left arm at `[48, -17, 35]`. The legacy left
ray `[28, 80, 60]` is intentionally a no-motion regression: its first retained
keepout occurs before 1 mm of validated progress, so it must be rejected.

## Latest verification evidence

- Native component suites passed in the one-job test build.
- Current source-built ROS inventory is 16/16. The 315.09 s
  `test_virtual_control_session` and portal core passed, including both contact
  corridors and contact-to-retreat. The complete sourced suite passed 16/16 in
  409.54 s on 2026-08-06.
- The 44.94 s ROS contract drove a real `/api/v3/move` request through the
  scaled action to measured completion, rejected 0.49/1.01/NaN/+Inf/-Inf at
  the action boundary, and verified accepted-goal cancellation plus
  race-tolerant portal/launch cleanup.
- Independent final reviews reported CLEAN for the collision boundary,
  non-monotonic IK search, no-op rejection, unit handling, urgent stop, and UI
  wording. Two additional independent reviews reported CLEAN for the adjustable
  movement limits, additive action compatibility, live v3 wiring, raw RViz view,
  and final cleanup behavior.
- Live production portal checks projected `[5000,5000,5000]` cm to roughly
  `[37.94,53.18,45.49]` cm, projected the pole target `[40,5,40]` cm to roughly
  `[6.13,13.76,12.57]` cm, completed from measured virtual feedback, returned
  active software stop in 12 ms, and shut down cleanly on Ctrl+C.
- A huge finite `1e300`-metre ray also returned a positive best-effort prefix
  without overflow and completed from measured feedback.

The Firefox oracle derives its synthetic rollback sequence above the current
live 30 Hz sequence. Keep that relative sequence; a fixed value caused a
transient test-harness race even though rollback rejection itself was correct.

## Physical hardware: read-only observer (commits 41c5d27 .. ba45690)

The arms are now physically connected. Everything below was measured against
them, not inferred.

### What the hardware actually is

- One DaMiao USB-C-to-CAN adapter (gs_usb, OpenMoko 1d50:606f), dual channel,
  presenting `can0` and `can1`. CAN-FD capable, 80 MHz clock.
- **Eight motors per arm, not seven.** IDs `0x01`..`0x07` are the joints,
  `0x08` is the gripper. The gripper has no URDF joint in this model, so it is
  read and reported but not published as a joint state.
- Replies arrive at `send_id + 0x10`, so `0x011`..`0x018`.
- Motor IDs are unique **within** a bus, not across the pair. A stock pair puts
  the same eight IDs on both interfaces.
- Refresh-status is `7FF#<id>00CC...`; the reply's position field is the 16-bit
  value at `data[1..2]` spanning +/-12.5 rad. That range is the same for every
  DaMiao type here, so **position decodes without knowing the motor type**.
  Velocity and torque do not.

`sudo bash scripts/setup_can_interfaces.sh` is the only privileged step.
`run-real.sh` refuses to run as root and refuses to start unless the interfaces
are already up, so nothing that talks to the arms ever runs with root.

gs_usb rejects `restart-ms` outright ("Device doesn't support restart from Bus
Off"). The setup script attempts attribute combinations in order of preference
and takes the first the driver accepts; do not reintroduce a single hardcoded
`ip link` invocation.

### The observer is read-only by construction

`openarm_real_observer` cannot command motion, and that is structural rather
than a policy. Its single socket write takes a motor ID and builds the frame
itself with `oa_can_make_refresh_status`; there is no overload accepting bytes,
and no enable/zero/motion encoder is referenced in the translation unit. Keep
it that way -- adding a frame-taking write would silently remove the guarantee.

It starts passive: nothing is opened and nothing transmitted until
`/openarm_real/connect` is called.

**It still publishes `/joint_states` while passive**, at 100 Hz, holding the
rest pose. This is deliberate and was a bug once: `robot_state_publisher`
derives TF from `/joint_states`, so a silent observer makes the robot vanish
from RViz entirely and an idle stack looks broken.

### Arm identification is explicitly pinned on this pair

The joint-limit heuristic scores a measured pose against each side's mirrored
joint-1/joint-2 limits. On can1 it answered **left** with a 0.819 rad margin.
The operator later confirmed by moving both physical arms under the strictly
read-only observer that can1 is robot-left and can0 is robot-right.

The arithmetic is mirror symmetric and tests verify it, but absolute angles are
not a general identity mechanism. Motor zeros are not commissioned to the URDF
zeros (joint 4 reads -0.978 rad, impossible for either side), so launchers pin
the operator-confirmed mapping instead of trusting the heuristic.

Motor IDs cannot substitute: a stock pair is `0x01`..`0x08` on both buses.

So the assignment carries a `confidence`. `"low"` for the heuristic, `"high"`
only when a human or the `interface_a_side` parameter settled it. The portal
presents an unpinned guess as unverified and offers **Swap arms**
(`/openarm_real/swap_sides`), which is one click and marks the result
operator-confirmed. `test_real_observer_core` pins both the observed score and
the exact-ID mapping behavior.

The sound automatic method, if this is ever revisited, is a **motion-delta**
test: zeros cancel in a difference, so the sign of a joint's change under a
named physical motion identifies the side regardless of commissioning.

### Current physical observer state

- A prior relaxed reference has been migrated, but per-joint direction and
  display offsets remain operator-uncommissioned. The active-tangent section at
  the top gives the current procedure and interface.
- Both can1 (robot-left) and can0 (robot-right) now answer on IDs 1..8. The
  earlier missing can0 J2 condition cleared before the per-joint calibration
  build went live.

### Real mode is a flag, not a fork

`launch_web_portal.sh --real` swaps `openarm_ik_rviz.launch.xml` for
`openarm_real.launch.xml` and passes `--real` to the portal. The real launch
**omits `openarm_ik_ros_node`**: it and the observer both publish
`/joint_states`, and running both renders a blend of a real arm and a simulated
one. `run.sh`'s path is untouched and was re-verified after this work.

`/api/real/status` answers in both modes, so one request tells the page which
stack it is attached to; that is why the same page serves both launchers with
no build-time switch.

### scripts/blink_arm_leds.sh

Enables motors briefly so their LEDs identify the arm, then disables from an
EXIT trap. Defaults to the gripper alone because enabling a DaMiao motor with
non-zero saved gains makes it hold a position target immediately, so a joint
can snap.

`cansend` requires exactly **three** hex characters for a standard CAN ID.
`08#...` makes it print usage and send nothing, which looks identical to a
motor ignoring the frame. Every ID goes through `frame_id()`.

## Planned: powered auto-calibration (NOT YET BUILT)

Requested: connect to the servos, drive each joint on its own to find its range
of motion, derive left/right and per-joint direction from that, reflect it in
RViz, behind a button, with an e-stop that works under all circumstances.

Two constraints shape the whole design, and both are load-bearing.

**This assistant cannot test motion.** The sandbox blocks transmitting enable
and motion frames to physical actuators. Every read-only claim in the section
above was verified by measurement; nothing in this section can be, so it must be
treated as unverified until someone runs it on hardware.

**A hardware e-stop exists and a second person can press it** (confirmed
2026-08-05). That is the real backstop and the powered phase should not run
without someone on it, because a software e-stop cannot stop a motor that has
latched a position target, and cannot act at all if the process dies, the
adapter is unplugged, or the bus saturates.

The motor timeout below is what covers the gap between "the software noticed"
and "a human reacted", so it is still required rather than made redundant by
the hardware stop.

### Build the motor timeout FIRST

`OA_CAN_RID_TIMEOUT` (register 9) is a per-motor watchdog: if no command arrives
within the window, the motor disables itself. The enforcement is in motor
firmware, so it survives the failure modes a software e-stop cannot -- process
killed, cable pulled, bus saturated, code deadlocked.

Set it short (order 100 ms) before enabling anything, and drive motion from a
loop that must keep refreshing. Then the default state of the system is
"disabled", and staying enabled requires continuous proof of life. This is the
single highest-value safety item and it is verifiable read-only: write the
timeout, stop sending, confirm the motor drops out.

Do not build the calibration before this works.

### Calibration sequence, per joint, one joint at a time

- Never move more than one joint at once. A whole-arm move cannot be reasoned
  about and the existing keepout model is not certified for the real robot.
- Command small position increments from the current measured position, not
  absolute targets. An absolute target lets a motor snap across its range.
- Move slowly, and keep the torque limit HIGH. These are not in tension once
  detection stops being torque-based, and both are required: the proximal
  joints need real torque simply to lift the arm against gravity, so a low
  ceiling would trip on gravity load and never reach the true range.

- Detect the limit by STALL, not by a torque threshold. The commanded position
  keeps advancing while the measured position stops changing; that is the
  signal. Concretely, the joint has hit something when the commanded-minus-
  measured following error grows past a few encoder codes for N consecutive
  cycles while the commanded position is still moving.

  Why stall rather than torque, given the torque signal exists: a torque
  threshold has to sit above whatever gravity is demanding at that pose, which
  varies with joint, arm configuration and payload. Set it low and it false
  trips mid-travel; set it high enough to be reliable and the joint is already
  pressing hard into the stop by the time it fires. Following error does not
  depend on the gravity baseline at all, so it fires on the first cycle the
  joint stops tracking, at whatever force it happens to be applying. High
  available torque then becomes harmless, because nothing ever asks the motor
  to use it against a stop.

  Torque stays as a secondary ceiling and an abort condition, not as the
  primary detector.

- Slow motion is what makes stall detection safe rather than merely correct:
  the following error accumulates over distance, so at low speed the joint has
  travelled a fraction of a degree past first contact by the time it aborts.
- Back off on contact before recording, and record the backed-off value, so the
  stored limit is not the hard stop itself.
- Abort the whole sequence on: E-stop, any joint exceeding a torque ceiling, a
  reading gap, or a joint moving when it was not commanded to.

### What calibration yields

- Per-joint range, which is the field of motion.
- Per-joint direction sign, from the sign of the measured delta against the
  commanded direction. This is what the current Flip buttons stand in for, and
  it removes the guessing that got the sign wrong twice.
- Left versus right, from the range asymmetry: joints 1 and 2 have mirrored
  limits, so once real ranges are measured the sides separate. This is the
  motion-delta idea, and it works where absolute angles failed because the
  uncommissioned zero cancels in a difference.

### E-stop must cover the new paths

The existing process-wide E-stop covers the virtual stack. Powered calibration
adds paths it does not know about, so it needs to gate the calibration loop
itself, and asserting it must send explicit disable frames rather than merely
stopping the sending of commands.

## Historical design note: folding converge into mirror (not implemented)

This was explored on 2026-08-05 but is not the current implementation. Converge
remains the dedicated contact-monitored planner; the completed heart demo uses
its narrowly scoped terminal-contact route plus an explicit retreat.

### The premise is right about geometry and wrong about termination

Converge's motion IS a mirror. Two claws closing on a midpoint are mirror images
by construction, and oa_mirrored_tcp_move already describes that trajectory. That
half is genuinely duplicated.

What converge has that mirror does not is not geometry. It is these six fields
from oa_converge_tcp_move:

    double   contact_torque_nm[7];        per-joint |tau| stop threshold
    double   contact_torque_fraction;     fallback as a fraction of tmax
    uint32_t contact_persistence_cycles;  consecutive cycles before stopping
    double   stop_distance_m;             stop short along the approach ray
    double   minimum_progress_m;          reject if too little travel exists

Converge is force-terminated motion. Mirror is purely kinematic: it plans and
runs to completion. So mirror reproduces converge's path today but not its stop
condition, and the stop condition is the entire reason converge exists.

The merge is therefore: move contact termination onto mirror as an option. Then
mirror-with-contact is converge, and converge becomes a wrapper or disappears.

### The ABI freeze dictates where this lands

openarm_runtime.h is byte-pinned against a frozen copy and has an exact exported
symbol manifest. Widening oa_runtime_mirrored_tcp_move in place changes
struct_size and fails the freeze test. So:

- Add a NEW request type in openarm_runtime_motion.h, the additive surface,
  carrying the mirror fields plus the contact fields. Do not touch the frozen
  header.
- Centroid and mirrored are currently static inline adapters over the frozen V1
  planner and export no symbols. A contact-capable mirror needs the monitor, so
  it cannot stay header-only; it becomes an exported symbol like converge did,
  and the manifest count rises accordingly. Update the expected count in the
  same commit or the manifest test fails.

### control_core: one gate to change, carefully

complete_on_contact() currently treats a contact stop as success only when the
command is converge. That narrowness is deliberate and load-bearing. An earlier
version mapped STOPPED to COMPLETED unconditionally and it masked real failures:
pick-place reported all seven steps complete while the arms sat at
(0.333, 0.080, 0.407) instead of (0.34, 0.11, 0.30).

So the gate becomes "was contact termination requested for this command", not
"is this command converge". Do not widen it to all STOPPED events again.

Keep the two clearance thresholds distinct while doing this: the 25 mm planning
gate and the 10 mm real-time intervention floor. Collapsing them aborts motions
the planner legitimately accepted.

### Verification that must pass

- test_virtual_control_session, test_portal_core, test_real_observer_core
- the frozen-ABI hash test and the exported-symbol manifest count
- scripts/functional_sweep.sh, all 32 checks
- a contact test that is NOT vacuous: assert unconditionally on
  OA_STOP_CAUSE_CONTACT. An earlier contact test had a conditional escape hatch
  and never exercised the contact path at all.
- confirm converge's own behaviour is unchanged if the symbol is kept

### Superseded heart sketch

The first sketch proposed a pure mirror sequence without contact termination.
The implemented demo instead uses proved converge contact at both cusps and a
guarded retreat immediately afterward; see the current delta at the top.

Constraints learned from the existing demos:

- Every waypoint must clear the 25 mm portal guard, not just the 10 mm monitor.
  Eight of fourteen presets were originally rejected because they were tuned
  against the monitor. Closed poses ended up at +/-0.15 rather than +/-0.11.
- The midpoint of any near-touching pose must stay outside the 0.12 m box grasp
  radius or the box demo steals it; clap had to move to 0.05 m for this.
- Add it to both demo_targets_json and demo_sequences_json in portal_page.cpp,
  with a sequence button, and give it a lead-in waypoint like cross has.

## Build and test commands

Install dependencies only when needed (this invokes sudo in the user's shell):

```bash
./scripts/install_all_dependencies.sh --apply
```

Resource-conscious test build and full ROS test run:

```bash
./scripts/build.sh --tests --incremental --jobs 1
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
ctest --test-dir ros2_ws/build/openarm_ik_ros --output-on-failure -j1
```

After a test build, publish the normal production (`run_tests 0`) launch tree:

```bash
./scripts/build.sh --incremental --jobs 1
```

Do not edit tracked build inputs while `build.sh` is running; it correctly
refuses to publish a stamp if inputs drift. Launchers hold shared leases on the
installed tree, so a build will also correctly refuse while a demo is running.
The generated `ros2_ws/.openarm-launch-stamp-v2` is untracked runtime state.

## Worktree ownership

The user's pre-existing modification to `transport/tests/test_transport.cpp`
is unrelated and must not be edited, staged, reverted, or overwritten. Preserve
untracked `.swarm/*.md` review reports unless the user asks otherwise. Before
finishing future work, use `git diff --check`, rebuild in one-job mode to avoid
another out-of-memory event, and verify that `bash run.sh --no-build` accepts the
current production stamp.

The verified best-effort portal work was committed and pushed as `d66c44b`.
The user explicitly authorized the subsequent movement-slider/view-style work
to be committed and pushed; later unrelated work still needs fresh authority.

## Real-arm J2 CAN incident (2026-08-07)

The commissioned bus mapping remains `can0=robot-right`, `can1=robot-left`.
The uncased robot-right J2 is command ID `0x02`, feedback ID `0x12`.

Right J2 once stopped replying while disabled even though the other seven
motors and the SocketCAN interface remained healthy. A guarded temporary
enable followed by a double-disable (`scripts/blink_arm_leds.sh --interface
can0 --motors 02`) restored it. It moved less than the status quantization
during the guarded enable and remained disabled afterward. Do not leave it
green as a connectivity workaround: solid green means torque-enabled, not
merely connected.

Read-only register comparison after recovery found no commissioning mismatch:
J2 matches right J1 for firmware bytes `6417`, hardware bytes `300V`, CAN
bitrate register 4 (the working 1 Mbit/s setup), MIT control mode 1, timeout 0,
and expected command/feedback IDs. `can0` was ERROR-ACTIVE with zero bus,
arbitration, warning, passive, or bus-off errors.

A 45-second disabled manual-motion capture then proved J2 feedback continuous:
exactly 100 frames/s throughout a sweep from roughly -50 degrees through +40
degrees and back, including the formerly suspect 45--90 degree region. The
encoder values changed continuously and no reply was lost. This establishes
that RViz is consuming the motor's live absolute encoder, not estimating
motion. If the symptom returns, capture CAN ID `0x12` before changing anything;
a healthy observer produces 100 replies/s.

`RealObserver::read_once()` now clears transient missing-motor diagnostics on
the next complete sample and reports exact missing IDs. It continues polling
IDs 1..8 even when one is absent, allowing a recovered controller to reappear
without restarting. The live integration status after restart was:
`live: can0 and can1 each answered with all expected motor IDs 1..8`.

The operator then placed both relaxed arms as close to physical neutral as
possible and explicitly requested updated display adjustments. On 2026-08-07
at 14:34 local time, `capture-relaxed` captured all seven joint references on
both complete arms and retained the established direction of every joint. The
active file is `~/.openarm_real_zero` (SHA-256
`7f4e7a8b3eeedeb7835169f1bdc8b72fb3d03a892f3517bf5b99f7c55a94637b`).
The immediately preceding calibration is recoverable from
`~/.openarm_real_zero.pre-neutral-20260807T143425` (SHA-256
`10a64cf5f9535f5a13387e553f3e863d80717d0426c4758e35976a1a95a72229`).
The post-capture `/joint_states` check put all 14 revolute joints at zero to
encoder quantization; the largest observed residual was 0.0003815 rad. The
temporary read-only observer was then stopped cleanly.

## 2026-08-07 J8 gripper commissioning and physical retest checkpoint

The C11 CAN ABI now owns double-precision gripper interpolation and DaMiao
position-force encoders: `oa_can_gripper_motor_position()`,
`oa_can_gripper_opening()`, `oa_can_encode_gripper_move()`,
`oa_can_encode_gripper_open()`, `oa_can_encode_gripper_close()`, and
`oa_can_encode_gripper_grasp()`. Only the final DaMiao wire position field is
narrowed because that protocol field is binary32. ROS adds the MoveGripper
action, compiled CLI commands, two measured finger joints in `/joint_states`,
and `/openarm_real/capture_grippers_closed`. Both real and virtual normal RViz
launches use the full movable-finger URDF.

The commissioned calibration is outside the repository at
`~/.openarm_real_gripper` and maps the official -1.0472 rad motor travel to the
URDF's 0.044 m per-finger range:

```text
OPENARM_REAL_GRIPPER_CALIBRATION_V1
0 0.55943389028763413 -0.48776610971236578 0.043999999999999997
1 0.4503318837262551 -0.59686811627374481 0.043999999999999997
```

Robot-left and robot-right both completed a measured full-open stroke at
0.5 N.m. Robot-left closed at 0.5 N.m; robot-right needed 1.0 N.m. Both claws
were then proved closed from encoder-derived finger joints (approximately
0.000032 m left and 0 m right) before the subsequent arm retest. No object was
available, so persistent torque-contact grasping is implemented and tested in
software but not physically accepted against an object.

Return-to-Neutral and one paired home target completed physically. Cross then
proved the lead-in/open/part-height sequence after increasing only the measured
settle tolerance from 0.012 to 0.015 rad and adding an 8 second bounded settle
timeout. During the next stack transition, robot-right J2 (`can0` command 2,
reply `0x12`) reproducibly stopped reporting while the bus stayed ERROR-ACTIVE
with zero bus errors. The 100 ms complete-feedback watchdog aborted motion and
sent disable frames, but a later non-energizing status audit found that J2 had
recovered communications while still reporting enabled status 1. Three
targeted disable frames followed by five refreshes then reported disabled
status 0. This proves that a transmitted disable frame is not sufficient
evidence when the exact motor is offline at that instant.

The real controller now writes and reads back DaMiao register 9 on all 16
motors at every explicit Connect. The volatile value is 4000 ticks; the
documented 50 us register unit makes this a 200 ms firmware communications
timeout. It also keeps both sockets open for up to three seconds after
Disconnect, E-stop, watchdog, terminal-callback fault, or shutdown, repeatedly
sending disable and polling status. It claims "all 16 confirmed disabled" only
after all 16 status nibbles are zero; otherwise diagnostics require the
hardware stop. Fault-status feedback is retained rather than mistaken for a
missing frame. These mitigations are compiled and software-tested, but the
timeout's actual fail-release behavior has not yet been physically induced.
With all motors disabled, the 4000-tick register write was then echoed exactly
by IDs 1--8 on both can0 and can1. A separate 16-motor status refresh returned
status nibble 0 for every drive, including robot-right J2. The E-stop session
return value also now remains false when a connected teardown cannot confirm
all disabled statuses, so the portal's `physical_disabled` JSON field cannot
convert a disable attempt into a false success report.

A reconnect plus 30 second idle soak was healthy, but the dropout recurred
under Cross motion. Do not weaken the watchdog or continue physical demo/range
testing until that motor/controller, CAN lead, and power lead are inspected or
replaced. Heart, Clap, Box, and full-range physical acceptance therefore remain
honestly unverified. The operator also stated that the arms' current pose is
not neutral during the test, then hand-placed both disabled arms in neutral at
the end. No calibration capture or adjustment was made.

The real-session worker now distinguishes an intentional Disconnect/E-stop
overlapping an in-flight CAN exchange from a genuine armed watchdog failure,
and gripper-calibration publication locks both control and I/O readers. The
arms were left disabled; the operator was asked to hand-place them into the
commissioned relaxed neutral pose while disabled and keep both claws closed.

## 2026-08-07 production C ABI and live left-only checkpoint

The repository now installs `openarm_real.h`, `libopenarm_real.so`, and the
pure-C `openarm_real_cli`. This is the physical C11 boundary requested by the
operator. It never opens raw SocketCAN; it calls the one ROS physical controller
that owns calibration, discovery, collision checks, measured-feedback motion,
watchdogs, lifecycle, and confirmed disable. The calls cover readiness, Connect,
read, joint/TCP/paired-TCP motion, J8 gripper movement, Stop, Disconnect, and
E-stop. Cartesian inputs carry an explicit metre/centimetre/inch enum and all
public coordinate/joint values are `double`.

A small robot-left J4 `+0.05 rad` physical move and return passed through this C
ABI without changing any reported robot-right joint. A paired High-far attempt
then encountered the already-known moving robot-right J2 feedback dropout and
was stopped by the controller watchdog.

For recovery with an electrically isolated faulty arm, the real session now
accepts `active_side_mask`: `1` means robot-left/can1 only, `2` means
robot-right/can0 only, and `3` is normal bimanual. Normal `run-real.sh` remains
hard-coded to strict mask 3. Single-side mode does not open the inactive socket,
touch inactive motors, or accept a command involving that side. The public
snapshot reports the mask so inactive fixed-model placeholders cannot be
mistaken for encoders.

With robot-right motor power physically removed, the left-only controller moved
robot-left through the C ABI from `[2.22, 12.96, 7.81] cm` to measured
`[28.47, 61.79, 40.75] cm`. Two `[28, 67, 52] cm` requests finished with
`segment_settle_timeout_holding_measured_pose`; the second residual was about
0.0306 rad, so do not claim the requested pose was achieved. The final source
passed an isolated full native build/test and focused ROS tests
`test_real_control_session`, `test_portal_core`, and `test_real_c_abi` (3/3).
The complete 19-test ROS suite was intentionally not launched on domain 0 while
the live physical controller was enabled.

Live state at this checkpoint: robot-right motor power is off; robot-left is
connected, enabled, and holding the measured far/high pose under a controller
launched with `active_side_mask:=1`. Abruptly killing it may let the arm relax.
Support/clear the arm or remove left motor power before terminating that
controller.
