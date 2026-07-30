# Final whole-branch adversarial review

Date: 2026-07-29 (America/Los_Angeles)
Reviewed commit: `315f787f52c42214d0fc6253d71f57ffe5b39c2f`
Disposition: **FINDINGS**

## Bottom line

No Critical finding was found. Five Important product findings and two Minor
documentation/evidence findings remain. The native model/controller/runtime and
the normal ROS measured-feedback path build cleanly and retain their deliberate
physical-motion gates. The merged local web portal is not release-ready: its
RViz stream is deterministically converted to black and its nominal guard
rejects the controller's neutral startup pose. The physical discovery/config
preview surface also does not yet produce the structured authoritative evidence
its public records imply, and a destroy request cannot stop an in-flight
SocketCAN query operation.

No GUI, CAN interface, network service, hardware, configuration, or motion was
started during this review.

## Critical

None.

## Important

### 1. The portal's RViz JPEG path converts named pixmaps to black

`rviz_capture.cpp:267-297` names the XComposite Window pixmap, reads that Pixmap
with `XGetImage`, and converts pixels with `image->red_mask`, `green_mask`, and
`blue_mask`. A core X11 GetImage reply for a Pixmap has no Visual; local libX11
therefore constructs that `XImage` with zero RGB masks. The local `channel()`
helper at `:68-79` returns zero for a zero mask, so every RGB triplet becomes
black before JPEG encoding. There is no uniform/all-zero-frame rejection, and
`/api/health` checks only process/window identity (`openarm_portal.cpp:536-550`),
so it can report healthy while `/api/rviz.jpg` returns a valid black JPEG.

This defeats the portal's principal visualization requirement. The source
Window's validated `XWindowAttributes.visual` masks must be used for the named
Pixmap storage, followed by an uncompressed-frame sanity check. Redirect
ownership also needs explicit unredirect cleanup; the destructor currently
only frees the last pixmap (`rviz_capture.cpp:121-127,216-224`).

The pre-existing untracked `.swarm/portal-black-capture.md` independently
records the same observed symptom and X11 mechanism; it was not altered here.

### 2. The portal guard rejects neutral and is not a faithful body/tool guard

`portal_core.cpp:20-24` models the complete body/support as a radius-115 mm
cylinder through Z=0.775 m. `scene_clear()` checks that proxy only against
joint-axis capsules 2 through 6 (`:322-335`) and starts inter-arm checks at
segment 1 (`:309-320`). At all-zero measured joints, the left segment-2 radial
centerline reaches about 121.999 mm, so the code computes
`0.121999 - 0.115 - 0.050 = -0.043001 m`, below its required +25 mm. Since
sample zero is always checked (`:363-395`), every portal move from the normal
neutral startup state is rejected before IK motion is submitted.

The pinned body mesh is not that cylinder: its shaft is approximately 60 x 60
mm, with separate base and upper-mount geometry. The current capsules also omit
moving link1/link2 body checks and do not contain the actual off-axis links,
hands, or the unknown full finger stroke. The failure is conservative at
neutral, but those omissions also mean the sampled check is not a reliable
complete-body/tool clearance test. It remains virtual-only and cannot change
the controller's truthful `collision_checked=false` result.

The current unit test checks one remote stationary pose
(`test_portal_core.cpp:119-131`) but never the actual startup pose or exact
mesh/primitive containment. The pre-existing untracked
`.swarm/portal-guard-geometry.md` independently measures the exact-mesh neutral
clearances and corroborates this finding; it was not altered here.

### 3. Physical inventory evidence and configuration preview are not
authoritative

The query result is a row per requested candidate, not per observed motor:
`inventory.cpp:292-365` pushes a motor record even when no register response was
received, and `motor_count` is therefore the candidate count (`:367-371`). The
public `register_mismatch_mask`, `duplicate_count`, `enabled_observed`, and
summary `conflict_mask` fields are never populated anywhere. The receive loop
breaks on the first matching response, so it does not detect duplicate
responders. Every nonempty physical result is hard-coded ambiguous and
unresolved, which is safely fail-closed but is not complete discovery evidence.

The fingerprint at `inventory.cpp:372-385` hashes only
`QUERY-<interface>-<send-id>-<serial>` plus the presence mask. It excludes the
observed receive ID, firmware versions, configured IDs, mode, bitrate, timeout,
direction, P/V/T spans, gear ratio, duplicate/mismatch state, and timestamps.
Configuration changes can therefore retain the same purported inventory
fingerprint.

`oa_runtime_configuration_preview_physical()` at `inventory.cpp:440-458` does
not compare any manifest motor to any evidence record, validate backend or
interface kind, or populate mapping/limit diffs. It returns `valid=1` solely for
14 rows with unresolved/ambiguous clear. Consequently the built-in **virtual**
inventory satisfies the function and is reported as a valid physical preview,
while every nonempty inventory the physical query path can actually construct
is unresolved and can never validate. Physical apply remains unsupported, so
this cannot currently configure hardware, but the requested preview/evidence
contract is not implemented.

### 4. Runtime destruction cannot cancel physical query transmission

`oa_runtime_inventory_query()` pins the runtime directly
(`inventory.cpp:213-217`), opens a local transport, and never checks the
runtime's `closing` flag inside its candidate/register loops (`:292-359`).
`oa_runtime_destroy()` only marks the runtime closing and erases its registry
entry (`runtime.cpp:554-564`); it has no reference to that local transport.
Therefore a query already in progress can keep transmitting register queries
after the caller has destroyed the runtime.

With the public maxima, the operation can issue 16 candidates x 14 registers =
224 frames. On a silent bus, each register can consume the allowed one-second
timeout, so teardown may return while read-query traffic continues for roughly
224 seconds. These are typed register reads, not configuration/enable/motion
frames, but continuing physical TX after lifecycle teardown is still an
Important authority defect. A query-wide stop/deadline and closing checks must
close/wake the owned transport before destroy is considered complete.

### 5. The product still has split orchestration and no true individual XYZ
motion API

The newly merged runtime is not the authority used by ROS. The ROS package
finds and links `openarm_control` directly (`openarm_ik_ros/CMakeLists.txt:15,
53-55`), and `VirtualControlSession` includes `openarm_control.h` rather than
`openarm_runtime.h`. Its package manifest has no runtime dependency. Thus the
ROS/CLI product creates a separate built-in controller/manifest and bypasses
runtime coordinate-identity records, persistence authority, discovery,
calibration leases, capability reporting, and runtime event/error semantics.
The compiled CLI exposes only `status`, `move-joint`, and `move-paired-tcp`
(`openarm_control_cli.cpp:308-329`).

There is also no standalone individual Cartesian motion operation. Runtime V1
explicitly reserves but never advertises `OA_RUNTIME_CAP_SINGLE_XYZ_IK` and
exports joint planning plus paired-TCP planning only
(`openarm_runtime.h:94-103,591-596`). The portal's "Move Left/Right" operation
submits the same paired action with the other **TCP** set to its freshest
measured position; redundant IK can change that arm's joint posture, so this is
not a posture-frozen individual-arm Cartesian API. The pure model library does
offer per-arm position IK, but it is not an individual measured-motion API.

The direct-control ROS path is still virtual-only and measured-feedback based,
so this split does not create physical motion. It is an unmet whole-product
integration/individual-XYZ requirement and leaves two lifecycle/identity
authorities to evolve independently.

## Minor

### 1. Top-level build/RViz documentation is stale after merged changes

- `README.md:40-41` says eight ROS tests are registered; `scripts/build.sh:164-168`
  requires 13.
- `README.md:73` says the installed launch description is copied from the
  canonical generated URDF. The launch actually uses the derived Stage-A
  visualization URDF, whose four finger joints are fixed and whose invalid
  finger inertials are removed.
- `README.md:100` still warns that this launch may report the four canonical
  finger-inertia errors and says the adapter does not edit/reinterpret the
  URDF. That is no longer true for the derived visualization-only artifact.
- `README.md:102-104` omits the installed Control and Runtime targets from its
  exported-target list.

The package-local ROS README describes the derived fixed-finger convention
correctly; the top-level README should match it.

### 2. Portal launcher help asserts a collision authority that does not exist

`scripts/launch_web_portal.sh:30-32` says portal motion remains disabled unless
the installed controller reports a verified collision scene containing both
arms and the pole. The controller always reports collision unchecked; portal
eligibility is instead decided by its separate sampled nominal guard. The web
page and package README are more accurate. This help text should not imply a
verified controller collision scene.

## What remains clean

- The canonical model/URDF, C FK/Jacobian/position-IK, side-specific TCP names,
  generated mesh references, and the derived 14-joint/fixed-finger RViz tree
  remain structurally intact.
- Control and normal ROS publication remain encoder-snapshot authoritative.
  JointState contains the 14 measured arm joints, not fabricated finger
  measurements; robot_state_publisher remains the sole TF authority.
- The C++ control, commission, transport, and runtime implementations remain
  behind versioned ISO-C records/opaque handles. Fresh declaration/export
  comparison found 50/50 runtime symbols.
- Physical configuration apply, physical calibration actuation, enable, zero,
  parameter save, and physical motion remain unsupported by runtime. The only
  runtime CAN frame builder reference is the typed register-query builder.
- The production ROS node and portal have no OpenArm CAN/runtime or Python
  library dependency in `ldd`; they use the direct virtual control path. Python
  is still a build/test and ROS launch-file dependency, not a native controller,
  CLI, portal-server, or RViz-close-helper runtime dependency.
- No authored credential/private-key pattern, production `system()`/`popen()`,
  privilege escalation, interface mutation, or installed test-hook export was
  found.

## Fresh checks

- Fresh Release production configure/build of CAN, model, commission,
  transport, control, runtime, and `openarm_ik_ros`: **PASS**.
- `bash -n` for `run.sh`, all scripts, and the native prefix test: **PASS**.
- `cppcheck --enable=warning,performance,portability` for runtime and portal
  core/capture: **PASS**.
- Runtime installed export/declaration parity: **50/50**.
- `scripts/install_all_dependencies.sh --verify`: **PASS**; every declared
  package is installed.
- Tracked `git diff --check`: **PASS**.

Full sanitizer/ROS execution was not duplicated because current whole-stack
verification logs were being generated separately. This review did not use
those untracked status claims as proof of the findings above.

## Worktree reconciliation

The review began with and preserved these unrelated user/parallel changes:

- modified `transport/tests/test_transport.cpp`, widening the expected loopback
  open result from only `EUNSUPPORTED` to `EUNSUPPORTED || EIO`; production
  transport code is unchanged;
- untracked portal investigation reports and screenshot-portal logs;
- untracked `final_integration_*` build/sanitizer/ROS logs created during the
  review window.

This report is the only file written by this review.

## Truthful hardware limitations

No connected OpenArm, CAN adapter, vcan interface, E-stop, or watchdog was
available or exercised. Nothing here proves motor family/serial-to-joint
assignment, bus separation, ID mapping, polarity, zero, gearing, firmware,
timeout behavior, thermal/torque calibration, real-time cadence, emergency
stop, power-loss persistence, physical accuracy, or collision-free motion.
Register-query timing and duplicate/stale responder behavior remain unqualified
on real SocketCAN hardware. Physical commissioning/configuration/motion must
remain disarmed until those facts are independently established on two isolated
buses with a hardwired E-stop, watchdog, complete collision scene/tool model,
and supervised acceptance testing.

Virtual motion is position-only with free orientation and no controller
collision checking. The portal's 17-sample guard is neither a continuous swept
path certificate nor currently a correct complete-geometry proxy. Finger state
is unmeasured and visualized fixed closed; there is no gripper motion/force API.
Persistence V2 additionally relies on caller-owned external checkpoints and a
qualified local filesystem with meaningful file/directory `fsync` and atomic
same-directory operations.
