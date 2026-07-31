# OpenArmIK Codex handoff

Last updated: 2026-07-30. This file is the starting point for future Codex
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
