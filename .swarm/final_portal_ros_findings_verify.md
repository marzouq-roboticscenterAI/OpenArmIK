# Independent portal / ROS finding verification

Date: 2026-07-29 (America/Los_Angeles)

Repository: `/home/signalprocessing-dev/OpenArmIK`

Reviewed HEAD: `e0d06c86d3fe3cba7c2fc78c4c0b4926de384ad9`

Overall disposition: **MIXED**

I treated the earlier untracked reports as leads rather than evidence. I read
`.swarm/portal-black-capture.md`, `.swarm/portal-guard-geometry.md`, and
`.swarm/final_whole_branch_review.md`, then checked current source and ran fresh
probes. I made no production-source changes, used no CAN or hardware, and made
no external network request or mutation. The only HTTP traffic was GET traffic
to an isolated loopback-only portal instance. The temporary Xwayland display,
ROS domain, RViz, portal, and ROS processes were stopped cleanly afterward.

## Summary verdicts

| Claim | Verdict on current HEAD | Result |
|---|---|---|
| A. Named-pixmap masks are zero and a healthy portal serves a black JPEG | **MIXED** | The raw Pixmap `XImage` masks really are zero, but the current encoder no longer uses them. A fresh live capture returned a visible RViz/Ogre frame. |
| B. The nominal guard falsely rejects neutral and omits material geometry | **MIXED** | The old radius-115 mm guard did falsely reject neutral. Current HEAD accepts neutral, but still checks only a shaft/capsule proxy and is not a complete body/tool collision gate. |
| C. ROS/CLI bypass the runtime and standalone single-arm XYZ is required | **MIXED** | The ROS server directly embeds `openarm_control`; the CLI inherits that path through ROS. The divergence is real. However, paired XYZ already provides a distinct target for each arm, so a single-arm API is only required if posture freeze/one-arm-only execution is a product requirement. |

## A. XComposite capture

### What is confirmed

A fresh raw XComposite probe against the live RViz top-level produced:

```text
window_visual_masks=0xff0000,0xff00,0xff
image_masks=0,0,0
depth=24 bpp=32
min_pixel=0 max_pixel=0xffffff
nonzero=369194 total=373800
```

Thus `XGetImage()` on the named Pixmap returns useful nonzero pixel words while
the resulting `XImage` has zero red/green/blue masks. The mechanism in the old
report is correct: converting those words with `image->{red,green,blue}_mask`
turns every output channel into zero.

Current HEAD contains the relevant correction from `51a602b`:

- `rviz_capture.cpp:232-239` requires the source Window to have a TrueColor
  Visual;
- `:275-293` reads masks from `attributes.visual`, validates them, and converts
  the named Pixmap's pixel words with those Window masks;
- `:295-300` refuses an all-zero RGB buffer;
- `openarm_portal.cpp:581-597` returns 503 JSON on capture failure rather than a
  successful black JPEG.

### Live result: the current black-JPEG claim is disproved

I launched a separate rootful Xwayland `:77`, ROS domain `193`, software GL,
stock `/opt/ros/lyrical/bin/rviz2`, the virtual ROS nodes, and the current
installed portal on `127.0.0.1:18089`. This did not touch the logged-in display
or any existing ROS domain. Xwayland exposed Composite and GLX; RViz reported
OpenGL 4.5.

The isolated portal returned:

```text
GET /api/health -> 200
{"healthy":true,"bind":"127.0.0.1",
 "rviz_process_identity":true,"window_ready":true,"reason":""}

GET /api/rviz.jpg -> 200 image/jpeg, 26797 bytes, 700x534 RGB
SHA-256 1806b0519a2445465b8c9cbd8f5a61b938c8c8db6f5748fff9aaff1c97c5d0cb
YMIN=0 YAVG=172.717 YMAX=255
```

Visual inspection showed the RViz menus/display panel, the dark Ogre viewport,
and the complete rendered OpenArm model. The native Ogre child therefore was
present in the top-level hierarchy capture on this Xwayland/software-rendering
test. Two stationary snapshots were byte-identical, as expected; I did not use
input injection to move the camera.

### Remaining capture issues and narrow fix

`/api/health` is still a process/window-readiness check only
(`rviz_capture.cpp:125-137`, `openarm_portal.cpp:562-574`). It does not prove
that a frame can be read and encoded. The present all-black predicate also
accepts a nearly black frame containing one nonzero channel
(`portal_core.cpp:492-495`). These do not recreate the reported current black
JPEG, but they leave health weaker than its name suggests.

The smallest robust follow-up is:

1. Keep Window Visual masks; never revert to Pixmap `XImage` masks or a default
   Visual.
2. Record the time/result and simple pre-JPEG statistics of the last successful
   capture. Expose `window_ready` and `capture_ready` separately in health.
3. Reject exact black and effectively uniform conversion failures conservatively
   (channel range plus luminance variance), while avoiding a broad threshold
   that rejects legitimate dark scenes.
4. Add explicit Automatic-redirect cleanup. The destructor currently frees the
   Pixmap but does not call `XCompositeUnredirectWindow`
   (`rviz_capture.cpp:109-115,204-212`). Trap destruction races.

Required tests: synthetic Pixmap image with zero image masks and nonzero words;
8:8:8 and 5:6:5 Window-mask conversion; invalid/overlapping masks; all-zero and
near-uniform RGB; 503 rather than JPEG on rejection; live resize, cover,
minimize/restore, and deliberate camera/model motion with changed pixels inside
the Ogre rectangle.

## B. Nominal collision guard and canonical geometry

### Old false reject is confirmed; current neutral rejection is disproved

At `ff38733`, the full-height radius-115 mm cylinder and radius-50 mm arm
capsule produced the reported neutral left segment-2 result:

```text
0.121999192 - 0.115 - 0.050 = -0.043000808 m
```

That was a proxy artifact, not an exact-mesh collision. Current HEAD instead
uses a finite shaft cylinder of radius `sqrt(2)*0.030 = 0.0424264069 m` over
Z `[0.008,0.758]` (`portal_core.cpp:19-28`) and checks segments 0 through 6
against it (`:309-327`). A fresh binary linked to the current portal core gave:

```text
q=0                 accepted=1  nominal minimum=0.029572784 m
q=0.0000667582208   accepted=1  nominal minimum=0.029558099 m
```

The second value is the encoder-quantized startup state used by the regression
test. Current HEAD therefore does not exhibit the neutral false reject.

### Exact URDF/mesh/FK comparison

I rebuilt and ran an FCL 0.7 probe against the pinned binary STLs, URDF mesh
scales/origins, and current public `oa_fk()` transforms. At all-zero q:

| Exact pair | Left | Right |
|---|---:|---:|
| body / fixed link0 | 0.999529 mm | 0.999529 mm |
| body / moving link1 | 29.474558 mm | 29.474558 mm |
| body / moving link2 | 41.450484 mm | 41.450484 mm |
| body / moving link3 | 41.772671 mm | 41.818536 mm |
| body / hand | 39.440017 mm | 39.440017 mm |
| body / closest finger | 81.231973 mm | 81.231973 mm |
| closest moving left/right pair | 138.880034 mm (hand/hand) | |

Neutral therefore clears the requested 25 mm for every tested moving mesh.
Only the two fixed mounting pairs need allowed-contact entries:

```text
openarm_body_link0 <-> openarm_left_link0
openarm_body_link0 <-> openarm_right_link0
```

The exact body collision STL has 5,864 facets and global AABB X
`[-0.155,+0.095]`, Y `[-0.095,+0.095]`, Z approximately `[0,0.773]` m. Direct
vertex bands show geometry far outside the 42.426 mm shaft cylinder: maximum
radial extent is 179.928 mm at the base, 96.473 mm between Z 0.008 and 0.620,
100.329 mm from Z 0.620 through 0.758, and 70.859 mm above Z 0.758. Those are
base/mount/detail regions, not a 60x60 mm central shaft.

### Geometry omission is confirmed, but current evidence does not prove a false negative

The current guard is explicitly a shaft/capsule check:

- arm-arm uses seven joint-axis/TCP capsules (`portal_core.cpp:288-308`);
- body checks use only the finite 42.426 mm shaft cylinder (`:309-327`);
- link1's body-facing radius is set to zero by a source comment rather than a
  mesh-containment test (`:311-319`);
- exact link shapes, body base/upper mounts, hands, fingers through their 44 mm
  strokes, payloads, and continuous swept volume are not represented;
- only 17 IK samples are evaluated (`:357-390`).

The omission is material because those shapes exist in the canonical collision
URDF. It is not enough, by itself, to claim an observed accepted collision. As
a sanity check, a fixed-seed search of 250 paired states within each joint's
limits clipped to +/-0.30 rad found no state that both passed the current guard
and violated the exact 25 mm mesh margin. Previously documented body/tool and
inter-arm adversarial states were rejected by the amended guard. This random
check is not a containment or swept-path proof.

### Safest minimal policy/fix

Do not enlarge exclusions or weaken the physical gate. In particular, do not
skip segment 2, all tools, same-arm body contacts, or a broad mount volume.

For the virtual portal, the current shaft proxy is acceptable only as a named
sampled nominal prefilter. Keep the UI/result wording that it is not physical
certification. Keep runtime/controller physical motion at reject-all unless an
authoritative scene reports collision checked.

For a complete 25 mm gate, retain the cheap shaft/capsule test as an early
reject, then add an authoritative collision stage using the pinned URDF
collision meshes or conservative envelopes with machine-checked containment:

1. check body against every moving link1..7, hand, full finger-stroke envelope,
   and payload;
2. check every moving left/right pair;
3. allow only the two fixed body/link0 mount pairs above;
4. use continuous collision checking between trajectory waypoints, not merely
   17 samples;
5. set `collision_checked=true` and permit a physical policy only when that
   complete stage succeeds for the exact model/scene revision. Any missing
   mesh, transform, finger state/envelope, scene revision, or numerical result
   must reject.

Exact mesh is preferable here because neutral has only about 4.475 mm excess
over the 25 mm requirement at moving link1; a coarse primitive union can easily
reintroduce the false reject. If primitives are retained, tests must prove they
contain every canonical mesh vertex across joint/finger ranges and then measure
their neutral conservatism.

Required regressions: zero and encoder-quantized neutral; exact body/link0 ACM;
all moving body pairs at neutral; base and upper-mount approaches; both prior
adversarial postures; all finger stroke endpoints and swept envelope; payload;
inter-sample collision that 17 discrete samples miss; missing/corrupt geometry
and revision mismatch fail closed.

## C. ROS/CLI versus `openarm_runtime`, and XYZ scope

### Bypass and semantic divergence are confirmed

The package finds `openarm_control`, not `openarm_runtime`, and links the
session directly to it (`openarm_ik_ros/CMakeLists.txt:15,50-54`).
`virtual_control_session.hpp:5` includes `openarm_control.h`; the node binary
contains `oa_controller_*`/`oa_manifest_*` symbols and no `oa_runtime_*`
symbols. The CLI binary itself is only a ROS action client, so “CLI links
control directly” is imprecise; its requests nevertheless execute through the
runtime-bypassing ROS server.

This is not a different motion algorithm today: runtime virtual motion also
wraps the same lower controller (`runtime.cpp:446-470`). It also does not expose
physical motion, so the present split is an integration/authority risk rather
than a demonstrated physical-motion escape.

The concrete semantic differences are real:

- Direct ROS hardcodes the built-in manifest, unchecked virtual collision
  policy, and scene revision 1, then directly verifies and arms
  (`virtual_control_session.cpp:283-316`). Runtime requires an explicit
  manifest/backend, explicit `allow_unchecked_virtual_motion`, interlock/deadman
  policy, and runtime arm authority.
- Runtime reports backend, units, body frame, orientation policy, model/TCP/
  scene revisions, model provenance/hashes, and a coordinate-identity digest
  (`openarm_runtime.h:146-190`, `runtime.cpp:145-208,566-587`). Runtime plans
  require the caller to echo this identity and reject stale/mismatched identity
  (`runtime.cpp:780-880,884-991`). Direct ROS has none of those checks at its
  product boundary.
- Runtime owns a host monotonic clock/deadline translation, one pending-plan
  authority, capability reporting, structured facility/lower-code errors,
  manifest/persistence authority, inventory, calibration leases, and normalized
  events. ROS reimplements a private relative controller clock, reservations,
  heartbeat, stop/restart, and direct `OA_CONTROL_*` result semantics.
- Consequently fixes or future policy changes in runtime do not automatically
  govern ROS. A future physical adapter built by extending the present session
  would be especially likely to bypass runtime identity and fail-closed policy.

### Smallest migration

Keep the ROS actions, TF transformation, measured-state publication,
reservation ownership, feedback callbacks, and CLI unchanged. Replace only the
lower half of `VirtualControlSession::Impl` with a runtime-backed session:

1. `find_package(openarm_runtime CONFIG REQUIRED)` and link
   `openarm_virtual_control_session` to `OpenArm::Runtime`.
2. Create the virtual runtime manifest/options, explicitly opt into unchecked
   **virtual-only** motion, set `(estop=0, deadman=1)`, and arm through runtime.
3. At startup require the expected virtual capability report, SI/body-frame/
   free-orientation identity, model/TCP revisions, scene revision, and digest.
   Populate those exact values in every joint or paired-TCP plan request.
4. Replace snapshot, plan/report, execute, heartbeat, stop/disarm, and event
   calls with their runtime equivalents. Preserve the current ROS owner and
   cancellation state machine around them.
5. Do not silently place `OA_RUNTIME_*` values in the action field named
   `control_status`. Version the message to expose runtime status/facility/
   lower-code, or retain the old field only when a real lower control code is
   available and add truthful runtime diagnostics.

Migration tests should run the existing ROS contract and lifecycle suite
unchanged, then add negative identity digest/model/TCP/scene tests, missing
unchecked-virtual opt-in, deadman/estop, stale feedback/deadline, pending-plan
authority, runtime destruction, event/status mapping, and a link/symbol test
that forbids direct `oa_controller_*` use from the ROS product.

### Paired XYZ already satisfies the stated per-arm target shape

`MovePairedTcp.action` has separate `left_tcp_m` and `right_tcp_m` points. The
node transforms both from one stamped frame and submits both; the CLI exposes
all six coordinates. Runtime likewise exports `oa_runtime_paired_tcp_move` with
separate left/right targets (`openarm_runtime.h:423-445`) and advertises virtual
paired-XYZ motion, but not standalone single-XYZ capability
(`:94-103`, `runtime.cpp:121-130`).

Therefore, if “target XYZ for each arm” means the user can specify a target for
the left arm and a target for the right arm, the existing paired action already
meets it. A new single-arm API is not required for that wording.

A stronger one-arm semantic is genuinely absent. The paired lower planner marks
both arms active and solves 17 position-IK waypoints for each. The portal
emulates a one-side request by setting the other TCP to its freshest measured
position, but that promises TCP hold only, not a bitwise joint-posture freeze or
independent arm authority. If the product requires the nonselected arm's joints
to remain fixed, independent scheduling/preemption, or a one-arm-only active
mask, add a real controller/runtime single-TCP plan with an explicit posture
policy and collision checks against the stationary arm. Do not synthesize it by
serial joint commands or silently relabel the paired operation.

## Final disposition

**MIXED.** The two visible release symptoms in the old reports—black JPEG and
neutral rejection—are fixed and were disproved on current HEAD by live/current
probes. The underlying raw Pixmap-mask mechanism, incomplete proxy geometry,
and split ROS/runtime authority are confirmed. The safest release policy is to
keep capture fail-unavailable, keep the portal guard explicitly virtual and
non-authoritative, keep all physical motion fail-closed, and migrate the ROS
session to the runtime facade before treating runtime policy as product-wide.
