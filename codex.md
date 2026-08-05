# OpenArmIK Codex handoff

Last updated: 2026-08-03. This file is the starting point for future Codex
sessions. Work only inside `/home/signalprocessing-dev/OpenArmIK` unless the
user explicitly changes scope. Read the root `SKILL.md` completely when the
user asks to use it, and maintain `.swarm/ledger.md` and `.swarm/learnings.md`
as that skill requires.

## What currently works

- Ubuntu 26.04 x86_64 has ROS 2 Lyrical, RViz2, xacro, build tools, Firefox,
  geckodriver, and the portal dependencies installed.
- `bash run.sh` performs an integrity-gated incremental build and opens the
  loopback-only portal. `bash run.sh --no-build --no-browser` is useful for
  automated checks. Ctrl+C cleanly stops the portal and ROS children.
- `scripts/launch_rviz.sh` launches the separate stock RViz engineering view.
  The web portal intentionally uses its own 30 Hz measured-pose WebGL viewer,
  with drag/orbit, wheel zoom, touch pinch, reset, resize caps, and stale-state
  overlays. A local-only button toggles the default blue/light-blue palette and
  an RViz-like neutral palette. It is not RViz pixels and is not collision
  checking.
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
byte-identical to its frozen copy; centroid and mirrored are header-only
adapters adding no symbols, and the manifest moved 50 -> 57 explicitly.

The portal's 3D view is now a real RViz window (`rviz/openarm_bare.rviz`,
`Panels: []`) rather than the in-browser WebGL proxy. Renderer/HiDPI setup is
shared in `scripts/lib/rviz_env.sh`. The Firefox viewer oracle was retired with
the canvas it drove, so the ROS test inventory is 14, not 15.

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

## Next session: execution plan for the queued work

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

Clap closes to 24 cm apart at ~31 mm clearance.

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

The portal requests the GPU renderer, unlike `launch_rviz.sh`. The
software-rendering default exists for live-resize flicker on a hand-dragged
window; the captured window is never resized, and llvmpipe gave 3-4 fps.
Measured: rviz2 33 fps, stream 35.4-35.9 fps against a 28 ms cap. The sweep
asserts at least 30 fps.

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
- Clap and cross-arms demos. The arms need only come close, not touch. Measured
  clearance stays above the 25 mm planning gate down to roughly 0.30 m between
  claws at x=0.30, z=0.35; below that the planner starts refusing. Crossing was
  not yet characterised: the throwaway probe used to sweep it had a state bug
  (it read OA_CONTROL_ESTATE from an unsettled controller as infeasibility).
- Clap and cross-arms only need the claws to come close, not touch, so the
  25 mm planning gate is not the obstacle it first appeared to be.

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

Portal motion now defaults to 80% of configured virtual velocity,
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
- Current source-built full ROS suite: 15/15 passed in 363.47 s.
  `test_virtual_control_session` passed 18/18 in 273.79 s and
  `test_portal_core` passed 35/35 in 1.32 s.
- The 44.94 s ROS contract drove a real `/api/v3/move` request through the
  scaled action to measured completion, rejected 0.49/1.01/NaN/+Inf/-Inf at
  the action boundary, and verified accepted-goal cancellation plus
  race-tolerant portal/launch cleanup.
- Independent final reviews reported CLEAN for the collision boundary,
  non-monotonic IK search, no-op rejection, unit handling, urgent stop, and UI
  wording. Two additional independent reviews reported CLEAN for the adjustable
  movement limits, additive action compatibility, live v3 wiring, viewer palette,
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

### Arm identification does NOT work automatically, and this is settled

The joint-limit heuristic scores a measured pose against each side's mirrored
joint-1/joint-2 limits. On the connected hardware it answered **left** with an
0.819 rad margin. Ground truth from the LED test: that bus is the **right** arm.

The arithmetic is fine -- the scorer is exactly mirror symmetric, and tests
verify it. The input is the problem. Motor zeros are not commissioned to the
URDF zeros (joint 4 reads -0.978 rad, impossible for either side), so absolute
angles carry no dependable side information. **Applying the manifest's
`q_scale`/`q_offset_rad`/`direction` was tried and does not rescue it: every
sign convention still answers left.** Do not spend time re-deriving this.

Motor IDs cannot substitute: a stock pair is `0x01`..`0x08` on both buses.

So the assignment carries a `confidence`. `"low"` for the heuristic, `"high"`
only when a human or the `interface_a_side` parameter settled it. The portal
presents the guess as unverified and offers **Swap arms**
(`/openarm_real/swap_sides`), which is one click and marks the result
operator-confirmed. `test_real_observer_core` pins the falsification, so
restoring the heuristic to authority trips a test that explains why.

The sound automatic method, if this is ever revisited, is a **motion-delta**
test: zeros cancel in a difference, so the sign of a joint's change under a
named physical motion identifies the side regardless of commissioning.

### Not yet established

- Motor zeros are uncommissioned, so RViz renders a pose offset from reality.
  This is the next thing standing between the current state and a faithful
  mirror of the robot.
- `can0` was silent throughout. All 63 IDs `0x01`..`0x3F` swept on both buses;
  only the 8 responders on `can1`. `can0` counters read RX=32/TX=32, pure
  SocketCAN loopback with zero external frames, while both arms were powered.
  The second arm's CAN is not reaching channel 0.

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

**There is no hardware e-stop on this rig** (confirmed 2026-08-05). A software
e-stop cannot stop a motor once it has latched a position target, and cannot act
at all if the process dies, the adapter is unplugged, or the bus saturates.

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
