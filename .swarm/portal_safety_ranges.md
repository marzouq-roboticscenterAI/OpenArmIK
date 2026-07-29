# Portal safety, coordinate, and UX audit

## Bottom line

The repository can support a **local, virtual-only visualization portal**. It
cannot truthfully support browser-authorized physical motion, automatic
calibration, a browser safety E-stop, or a collision-safe workspace. The ROS
adapter has no hardware backend, and the controller's physical backend returns
`OA_CONTROL_EUNSUPPORTED` during verification. Position IK checks only final
joint limits and TCP residual; it checks no self, body, inter-arm, gripper, or
environment collision and leaves orientation free.

Fail closed: every page and command response should say `backend: virtual`,
`collision checked: no`, and `motion authorized: no`. Do not hide those facts
behind a tooltip.

## Truthful frame, units, model, and nominal state

- The command frame is literally `world`. In the generated URDF,
  `world -> openarm_body_link0` is a fixed identity transform. Thus this is the
  **model body frame**, not a surveyed lab/floor/world coordinate system.
  `robot_state_publisher` is the only TF authority, and RViz uses `world` as its
  fixed frame.
- The model places the left/right arm mount frames at
  `(0, +0.031, 0.698)` and `(0, -0.031, 0.698)` metres, with rotations of
  approximately `-pi/2` and `+pi/2` about X. Positive Y is therefore the
  model's left side. Z is model-up. No checked-in commissioning result proves
  that these axes/origin coincide with a physical installation. Avoid claiming
  “height above floor” or facility coordinates.
- Cartesian translations and prismatic joints are metres; revolute joints are
  radians. Mesh source geometry is millimetre-authored and loaded with scale
  `0.001`; that does not change the public Cartesian unit.
- The exact controlled tip is `openarm_{left,right}_hand_tcp`, a fixed 0.186 m
  total offset from link 7 through the hand. IK is XYZ-only; TCP orientation is
  an unconstrained result, not a user command.
- A fresh ROS adapter process initializes all 14 arm joints to zero. FK gives:

  | state | left TCP XYZ (m) | right TCP XYZ (m) |
  |---|---:|---:|
  | initial all-zero model posture | `(0, +0.153497715, 0.075999550)` | `(0, -0.153497715, 0.075999550)` |

  This is a model initial posture, **not a calibrated home or safe pose**. Joint
  4 is exactly on its lower limit at zero. After a successful request, virtual
  “current” means only the last atomically committed joint posture in that
  process; it resets on restart and is not hardware measurement. Offline source
  inspection cannot state a later runtime current pose.
- `/joint_states` is republished at 10 Hz with a new ROS stamp even when the
  virtual values have not changed. Therefore publication freshness is not proof
  of new measurement or movement. For the virtual adapter, tie displayed TCPs
  to the commit diagnostic sequence/achieved values; for any future controller
  integration, use coherent encoder capture sequence, capture timestamp,
  expected/fresh/fault masks, and cross-bus skew.

## Model limits

URDF/model revolute limits are:

| joint | left limit rad (deg) | right limit rad (deg) |
|---|---|---|
| J1 | `[-3.490659, 1.396263]` (-200, 80) | `[-1.396263, 3.490659]` (-80, 200) |
| J2 | `[-3.316125, 0.174533]` (-190, 10) | `[-0.174533, 3.316125]` (-10, 190) |
| J3 | `[-1.570796, 1.570796]` (-90, 90) | same |
| J4 | `[0, 2.443461]` (0, 140) | same |
| J5 | `[-1.570796, 1.570796]` (-90, 90) | same |
| J6 | `[-0.785398, 0.785398]` (-45, 45) | same |
| J7 | `[-1.570796, 1.570796]` (-90, 90) | same |

These are model boxes, not independently verified physical soft limits. The ROS
IK policy uses `limit_margin_rad = 0`, so mere IK success can include a limit.
A physical implementation should require commissioned mappings and additional
validated soft-limit margins.

Each hand is visually/collision-modelled with two prismatic finger joints,
0.000--0.044 m. Joint 2 mimics joint 1. The ROS adapter publishes only the two
master joints, always at `0.0`; it neither commands nor measures a gripper.
The seven-DOF model/control APIs omit finger DOFs. A portal must label them
`modelled, fixed at 0 in virtual view; no gripper feedback/control`, not “open,”
“closed,” or “measured.”

## Recommended virtual-only test target

The sole recommended UI preset is the checked-in regression pair:

| side | target XYZ in model `world` (m) |
|---|---:|
| left | `(0.200, +0.300, 0.850)` |
| right | `(0.200, -0.300, 0.850)` |

It is exercised by both controller and headless ROS contract tests. An
independent run of the exact `continuity-v1` options from the fresh zero state
converged symmetrically with about `6.1e-8 m` residual; its closest model limit
margin was about `0.267 rad` at J6. Label it **“Virtual IK regression target
(collision unchecked)”**.

Do not expose a green “safe XYZ range.” Reachability is not a Cartesian box and
depends on the last committed seed. More importantly, the URDF's simplified
collision meshes are not consulted by IK; RViz's collision display is disabled;
the straight or solver-induced motion can hit the body, the other arm, fingers,
or the environment; and unconstrained orientation can sweep the gripper. The
controller does interpolate 17 Cartesian waypoints, but still reports
`collision_checked == 0`; its default collision policy rejects all plans, and
unchecked planning is virtual-only. No XYZ point in this audit is approved for
hardware motion.

## Critique of proposed portal behaviours

### Calibration

Reject “auto-calibrate.” Automatic CAN discovery cannot establish motor-to-joint
identity, side, sign, zero, gearing, or a safe mapping. The commissioning library
does not drive hardware: manual calibration consumes torque-disabled samples;
the hard-stop recipe is supervised and caller-driven and requires fresh encoder
data, E-stop/deadman state, fixture/qualification revisions, bounded travel,
torque/contact evidence, retreat/reapproach repeatability, explicit review, and
software-only commit. A fault/abort makes commit impossible. The portal may show
a read-only checklist or a guided *manual commissioning* state machine only
after a separately qualified hardware layer exists; it must never start a
calibration motion on page load, reconnect, or a single click.

### Independent arm commands

The ROS contract intentionally accepts exactly two poses ordered left then
right and commits both or neither. The physical-oriented control API likewise
offers paired TCP planning bound to both arms' measured feedback sequences.
Do not let two browser requests race or expose independent left/right “Move”
buttons. If one-side editing is desired, the server must snapshot the other
side's latest committed/measured TCP under one command lock, construct one
paired request, and show both pending targets for confirmation. Never use the
browser's cached other-arm value.

### Browser exposure and CSRF

Default to loopback only. Non-loopback listen must be an explicit, noisy opt-in
with authentication and TLS supplied by a reviewed deployment boundary. All
state-changing endpoints must be POST with strict JSON content type, authenticated
operator session, `SameSite=Strict` cookie (if cookies are used), unguessable
CSRF token, and exact `Origin`/`Host` allow-list validation. Validate WebSocket
Origin and authentication too. No GET side effects, command query parameters,
wildcard CORS, remote third-party assets, or direct browser-to-ROS access.
Rate-limit commands and audit actor/session/request/result without logging
secrets.

### Stale feedback, replay, and acknowledgement

Render desired, accepted, and observed states separately. Never optimistically
move the “current” display. Age feedback using the server's monotonic clock and
the producer capture timestamp/sequence, not browser time or receipt time. Any
missing member, stale capture, fault bit, sequence regression, cross-arm skew,
backend restart, or lost diagnostic must visibly mark state `STALE/UNKNOWN` and
disable motion.

The current `PoseArray` has no caller request ID, ownership, state binding, or
replay nonce. Its timestamp gate (normally 1 s, configurable up to 60 s) still
permits duplicates inside the window, and its receiver-generated diagnostic
sequence cannot safely correlate concurrent clients. A portal command broker
must serialize requests, issue a single-use idempotency key/nonce, bind it to
operator lease, server epoch, latest state sequence, targets, and short expiry,
and consume it exactly once. A browser retry queries command status; it does not
resubmit motion. Rotate the epoch/nonces on process restart.

### E-stop priority

A web button is not a safety-rated E-stop: browser scheduling, Wi-Fi, HTTP, and
the portal process are all failure paths. Require an independent hardwired
E-stop and deadman for physical use. Until then label any UI control **“Request
stop (not a safety E-stop)”**.

The stop request path must bypass IK, image capture, normal command queues, and
operator-lease checks; atomically reject new motion, latch stop/disable, and
retire queued work. E-stop/deadman loss has priority over controlled stop. It
must remain latched across reconnect/restart. Never auto-clear it: the existing
controller reset requires a fresh verification epoch/nonce and returns to
closed, not armed. Note that the present ROS visualization node exposes no stop
or E-stop interface, and the controller physical backend remains unsupported.

### Multiple clients

Use one explicit, expiring operator lease and unlimited read-only observers.
Show the lease holder and expiry to all clients. Lease loss/revocation blocks
new commands and, for a future physical integration, lets the independent
producer watchdog enact the configured stop policy. Never use last-writer-wins.
Only one command may be pending/executing; all clients observe the same
server-authoritative command ledger and terminal result.

### Field validation

Validate again on the server: exact left/right schema, exactly three JSON
numbers each, finite values (reject NaN/Infinity and strings), metres only,
model-world frame only, bounded decimal length/body size, short server-issued
expiry, latest state binding, and exact two-pose order. Reject rather than clamp
or silently swap/sign-convert. Run both IK solves and commit neither unless both
succeed. Treat an optional conservative UI envelope as input-error reduction,
not safety; final acceptance still requires IK residual, joint bounds/margins,
branch/singularity policy, state freshness, and—before any physical motion—a
real collision/trajectory authorization layer that this repository lacks.

### Actual RViz capture

The repository currently launches a real RViz process but has no capture or web
stream endpoint. Do not substitute a static image, CSS/canvas robot, or a second
renderer and label it “RViz.” Call the panel **“Live RViz capture”** only when
frames demonstrably originate from the launched RViz session. Each frame needs
server capture time, session/launch epoch, and associated committed state
sequence; on timeout or process exit, freeze no more than the stated threshold,
overlay `STALE`, then replace it with `RViz unavailable`. Never use image
freshness as control feedback. A static artifact must be labelled “reference
image,” with no live implication.

### Shutdown

On SIGINT/SIGTERM: stop accepting commands, revoke the operator lease, reject
queued work, request disable/disarm through the control owner, and wait for a
fresh coherent disabled acknowledgement before tearing down feedback and the
web server. If that acknowledgement cannot be obtained, report `STOP STATE
UNCONFIRMED—USE PHYSICAL E-STOP`; killing a process is not a safety action.
Then close capture/RViz and ROS processes and reap children. Unexpected portal
disconnect/exit must be covered by an independent producer watchdog. The
existing RViz launcher has orderly TERM/KILL process cleanup, but it controls a
virtual visualization launch only and proves no hardware stop.

## Fail-closed acceptance gates

Do not call the portal ready unless all applicable gates pass:

1. The server asserts virtual backend at startup and aborts if a physical path
   is selected; no CAN interface is opened and no calibration/hardware write is
   reachable.
2. UI and every command/result permanently expose model body frame, metres,
   virtual status, free orientation, and `collision_checked=false`.
3. The fresh-zero nominal TCPs and the symmetric regression target above match
   model FK/IK within `1e-4 m`; either-arm failure leaves both joint states byte-
   for-byte unchanged.
4. Wrong frame/count/order, missing/stale/future stamp, non-finite/out-of-policy
   input, expired nonce, stale state binding, duplicate/replayed request, and
   backend epoch change all reject without state change.
5. Feedback loss/stall, unchanged producer sequence past its deadline, partial
   paired feedback, fault bits, and excessive cross-arm skew all produce
   `STALE/UNKNOWN`, revoke execution authority, and never present a fresh/current
   green state merely because the UI or republisher is alive.
6. Concurrency tests prove one operator lease, one in-flight command, atomic
   paired commits, deterministic observer-only behaviour, and correlation of an
   acknowledgement to exactly one request.
7. CSRF/Origin/auth tests prove cross-site form, fetch, WebSocket, GET, wrong
   content type, unauthenticated, and non-loopback-default attempts cannot mutate
   state.
8. Stop injection while IK, capture, and command handling are blocked proves the
   stop path preempts work, latches, rejects later motion, and cannot be cleared
   by refresh/reconnect. The UI never claims safety-rated E-stop capability.
9. RViz tests prove pixel frames come from the launched RViz process, carry
   session/time/state metadata, and become explicitly unavailable on stall/exit;
   no mock fallback is mislabelled live.
10. Graceful and crash/disconnect tests prove command rejection first and, for
    any future hardware backend, watchdog/disable acknowledgement semantics.
    Absence of acknowledgement remains an alarm, never a successful shutdown.

## Recommended user-facing wording

Banner:

> **Virtual model only — no hardware control.** Position-only IK; orientation is
> free. Collision checking is unavailable. A reachable target is not a safe
> motion.

Coordinate label:

> **Model body frame (`world` = `openarm_body_link0`), metres.** +Y is model-left;
> this frame is not calibrated to the room or floor.

State labels:

- `Virtual last committed TCP` (never `Actual TCP` or `Measured position`)
- `Gripper: modelled at 0.000 m; not commanded or measured`
- `Virtual IK regression target (collision unchecked)`
- `Request stop (not a safety E-stop)`
- `Feedback stale/unknown — commands disabled`
- `RViz unavailable` / `Static reference image` as applicable

Primary action:

> **Preview paired virtual IK**

Avoid “Move robot,” “Go to safe pose,” “Auto-calibrate,” “Safe workspace,”
“Live hardware,” and “E-stop” for the capabilities presently in this repository.

## Evidence inspected

`README.md`; `model/README.md`; generated URDF and model data; model FK/IK code
and tests; ROS launch, node, paired transaction, RViz configuration, and contract
tests; controller API/core/tests and README; commissioning README/core; and the
RViz lifecycle launcher. No GUI, network, CAN, or hardware operation was used.
