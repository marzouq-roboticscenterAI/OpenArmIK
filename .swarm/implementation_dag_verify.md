# Portal implementation DAG verification

Date: 2026-07-29 (America/Los_Angeles)  
Checkout inspected: `21ab251ab673618dc46201dc633c5b94aac53379`  
Status: **DAG REQUIRES TWO DESIGN CORRECTIONS; IMPLEMENTATION CAN THEN PROCEED IN THREE PARALLEL LANES AND ONE FINAL INTEGRATION LANE**

This was a read-only whole-design integration audit except for this requested
report. No build, test executable, GUI, listener, network, CAN, or hardware
operation was started. I read the nine untracked portal/motion/collision
reports, the current control/model ABI and implementation, the native and ROS
build scripts, the ROS package/node/launch/helper sources, and the installed
ROS 2 Lyrical rosidl CMake contracts.

## Decision

The overall authority split is sound: one checked virtual controller owns
motion and measured state; ROS is its sole adapter; the portal is only a ROS
client plus stock-RViz capture; Bash only supervises processes. Physical
motion/calibration remains absent and fail-closed.

Do not implement the reconciled synthesis literally until these two internal
contradictions are fixed:

1. **Virtual collision readiness must not depend on an unavailable physical
   survey.** The audited canonical v1.0 body STL already contains the nominal
   60 x 60 x 750 mm central extrusion and the exact body, link, hand, and finger
   meshes are pinned by hashes. That is sufficient for a versioned **virtual
   model scene**, including a redundant square-pole guard. It is not sufficient
   evidence for physical motion. Requiring a physically calibrated pole record
   before *virtual* motion would make the requested virtual portal permanently
   nonfunctional. Name and report the scene as virtual/model-validated, set
   physical calibration/authorization false, and keep the physical backend
   unsupported. Never describe it as an as-built or surveyed scene.
2. **The plant implementation must follow the proof in
   `motion_profile_design.md`, not the simpler clamp proposal in the synthesis.**
   Adding acceleration state and clamping `delta-a` in the current lagging PD
   plant does not prove reversal, velocity saturation, target crossing, or
   snap-free convergence. Use the analytic seventh-order ideal virtual actuator
   with private `(q,dq,ddq)` truth evaluated from absolute monotonic time. It is
   the smallest implementation whose ordinary motion is exactly bounded and
   exactly follows the collision-certified path. A lag plant is a later feature
   requiring a genuine online trajectory generator and a collision tracking
   tube.

A third hidden safety dependency must be made explicit: current C calls on one
controller all take `ControllerSlot::mutex`. A long IK/collision planning call
therefore blocks `oa_controller_set_interlock()`. The claimed urgent stop path
cannot be proven merely by posting work to the same owner thread. Add a narrow,
thread-safe out-of-band software-stop request symbol whose wrapper atomically
latches the controller slot without waiting for that mutex. Planning and
collision loops must observe the latch at bounded checkpoints and transition
the core before any plan can execute. This remains a virtual software interlock,
not a safety E-stop. If this API is not added, remove the 100 ms claim and do not
release state-changing portal routes.

## Corrected dependency graph

```text
                 A0 interface/ABI contract freeze
                  |\
                  | +------ I0 ROS interface package
                  |          |
 A1 virtual manifest          +------ R0 controller/session/node/CLI
 A2 analytic motion -----------------/          |
 A3 virtual collision scene/continuous proof --+---- P2 portal ROS bridge
 A4 true single-TCP planner --------------------/       |
 A5 async stop latch ---------------------------/       |
                                                         |
 P0 capture/JPEG ------------------+                     |
 P1 HTTP/security/UI --------------+------ P2 -----------+
                                                         |
                                      D0 build/install/launcher integration
                                                         |
                         native -> interfaces -> headless ROS -> portal fake-X
                           -> loopback integration -> logged-in actual RViz
```

Contract freeze `A0` is a short review gate, not a code lane. After it, the
control lane, ROS-interface scaffolding, and portal pure modules can start in
parallel. `R0` compiles only after A and I0 install. P2 compiles only after I0.
Launcher/build integration is last because it consumes installed executable and
package names.

There is no dependency cycle if the interfaces and portal are separate ROS
packages. Keeping generated interfaces, controller, portal, and launcher edits
inside the current `openarm_ik_ros` package would create unnecessary file-level
contention and same-package rosidl linkage complexity.

## Non-overlapping worktree/file ownership

The following ownership is intentionally exclusive. Do not let two worktrees
edit the same CMake, manifest, source, test, or generated file.

### Worktree A — native control, motion, manifest, single TCP, collision

Own only:

```text
control/CMakeLists.txt
control/cmake/openarm_controlConfig.cmake.in
control/include/openarm_control.h
control/src/c_api.cpp
control/src/control_core.cpp
control/src/control_core.hpp
control/src/kinematics.cpp
control/src/kinematics.hpp
control/src/virtual_manifest.cpp                    (new)
control/src/collision_scene.cpp/.hpp                (new)
control/src/collision_geometry.cpp/.hpp             (new)
control/generated/openarm_v10_collision_data.inc    (new, reviewed output)
control/tools/generate_collision_geometry.py        (new, build-time only)
control/tests/**
```

This must remain one worktree. Single-TCP planning, plant evaluation, collision
certification, execution binding, urgent-stop checkpoints, and plan reports all
touch `control_core.*`/`c_api.cpp`; splitting them would create semantic and
merge conflicts even if the new geometry generator were developed separately.

Required native contract:

- Add new symbols/records/constants only. Never resize or reinterpret any
  existing V1 record or change any existing symbol.
- Add `oa_manifest_create_openarm_v10_virtual()` using identity mappings,
  `sim_left`/`sim_right`, canonical names, unique simulated serials, URDF limits,
  and fixed `1/2/10` virtual hard limits. The adversarial sign/offset fixture
  remains test-only.
- Add `OA_PLAN_TCP`, `oa_tcp_move`, its prefix-size macro, and
  `oa_controller_plan_tcp()`. It snapshots both measured arms, solves only the
  selected chain, holds inactive planned q bit-identically, monitors advancing
  inactive feedback, and applies completion TCP tolerance only to the active
  side. All planners require both arms at measured rest.
- Add an immutable opaque virtual collision scene, scene report, new controller
  creation symbol that snapshots scene data, and a separate collision-plan
  report. Existing `oa_controller_create()` and unchecked policy remain ABI
  compatible but are forbidden to ROS.
- Use a policy name such as `OA_COLLISION_OPENARM_V10_VIRTUAL`, not
  `...CALIBRATED_POLE`. Report `virtual_model_validated=true`,
  `physical_calibrated=false`, exact model/source/mesh/generator digest,
  scene revision, required margin, included geometry, and pole guard. Physical
  creation/verification remains `OA_CONTROL_EUNSUPPORTED`.
- Use the exact pinned triangle meshes with FCL BVHs, or generated conservative
  proxy unions whose containment of every source triangle is independently
  proven. The reconnaissance recommendation of exact FCL meshes is lower risk.
  The pole guard is a square prism, not a cylinder. Check full-stroke gripper
  envelopes because there is no measured finger state.
- Certify measured start, all 17 IK knots, and every joint-space interval with
  a conservative recursive displacement bound. Budget exhaustion, nonfinite
  distance, FCL/GJK failure, stale digest/revision, or a near-margin numeric
  ambiguity returns collision/indeterminate and publishes no plan.
- Use the analytic seventh-order virtual truth `(q,dq,ddq)` at absolute
  monotonic time. Quantized DaMiao frames remain the published measurement.
  Bind planning to measured q/sequences and privately cover the truth-to-codec
  discrepancy in the collision margin. No ordinary convergence snap is allowed.
  Coalesce duplicate TCP knots, apply the ten-cycle floor only to nonzero
  segments, compute durations in checked `long double`, round upward and
  revalidate the integer duration. `oa_controller_sim_set_state()` is busy
  during accepted/pre-start/executing work; encoder-freeze fault injection
  freezes captured sensor payload while analytic truth continues, rather than
  freezing truth and later introducing a jump.
- Add a new async stop-request symbol rather than changing the blocking meaning
  of `oa_controller_set_interlock()`. The latch lives at the controller slot,
  survives plan return, is checked during every IK/collision budget slice and
  before execute, and is idempotent. Core-transition latency is measured
  separately from latch admission.

### Worktree B — custom ROS interfaces, controller node, compiled CLI

Own only:

```text
ros2_ws/src/openarm_ik_interfaces/**                (new package)
ros2_ws/src/openarm_ik_ros/**                       (replace current adapter)
```

Create `openarm_ik_interfaces` as a pure interface package containing the two
messages, two actions, and four services from the reconciled design. This avoids
a generated-types/controller/portal package knot. Use explicit IDL dependencies
only: `builtin_interfaces`, `geometry_msgs`, `std_msgs`, and
`unique_identifier_msgs` (action expansion supplies its standard action/service
support through rosidl). The package must declare:

```text
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

On this installed Lyrical, `rosidl_get_typesupport_target()` exists and is the
supported way to link same-package generated C++ types. With a separate
interface package, consumers simply `find_package(openarm_ik_interfaces
REQUIRED)` and link its exported targets through ament dependencies. Do not use
obsolete `rosidl_target_interfaces`. Do not use
`rosidl_auto_generate_interfaces()` in a mixed controller package: it derives
`DEPENDENCIES` from every build dependency and would incorrectly feed
non-interface packages such as `openarm_control` to rosidl.

The controller package owns these new implementation files (names may vary but
ownership may not):

```text
include/openarm_ik_ros/virtual_control_session.hpp
include/openarm_ik_ros/command_arbiter.hpp
src/virtual_control_session.cpp
src/command_arbiter.cpp
src/openarm_ik_ros_node.cpp
src/openarm_ik_cli.cpp
test/**
```

Delete the production use of `PairedTransactionProcessor`; do not retain an
instantaneous fallback. The node is the only controller and `/joint_states`
authority. One owner worker makes normal C calls at 5 ms using actual steady
elapsed time. ROS callbacks reserve/reject and validate envelopes. The stop
callback directly invokes only the new thread-safe latch API, then lets the
owner observe/complete the transition.

Startup creates the production virtual manifest and checked virtual scene,
creates/verifies the controller, validates a fresh coherent collision-free
disabled snapshot and sequence-bound FK, publishes `simulation_verified=true`,
and remains DISARMED. It never auto-arms. No ROS parameter or interface can
select a physical backend, CAN interface, commissioning session, manifest path,
or scene revision.

Publish exactly 14 measured arm joints with q/dq/tau and the conservative oldest
capture stamp; publish no fingers and no TF. `robot_state_publisher` remains the
only TF/static-TF authority. PortalState is transient-local depth 1 and includes
session UUID, state sequence, valid/fresh status, scene/collision result,
orientation/gripper limitations, and physical capability false.

The actions use session UUID plus required state sequence, and their result is
the only completion authority. A TF target is transformed at its nonzero fresh
stamp into `openarm_body_link0`. The legacy PoseArray topic may exist for one
migration cycle only through the same arbiter/controller; it must not keep its
own state or direct-model processor.

The compiled CLI is a ROS action/service client, never a second controller. It
provides bounded `state`, `verify`, `enable`, `move-left`, `move-right`,
`move-both`, `stop`, and `reset` operations, binds the observed session/state,
waits for its own goal UUID/terminal result, and uses distinct exit codes for
reject/busy/timeout/abort. It never auto-enables or retries a motion goal.

### Worktree C — compiled portal capture, server, ROS bridge, UI

Own only:

```text
ros2_ws/src/openarm_portal/**                       (new package)
```

Keep this a separate package depending on `openarm_ik_interfaces`, `rclcpp`, and
`rclcpp_action`. Suggested exclusive modules are:

```text
include/openarm_portal/{capture,jpeg,http,security,ledger,ros_bridge}.hpp
src/{main,capture_x11,jpeg,http_server,security,ledger,ros_bridge}.cpp
src/assets/{index.html,app.css,app.js}
cmake/embed_assets.cmake
test/**
CMakeLists.txt
package.xml
```

It owns no controller/model/plan, JointState publisher, TF broadcaster, RViz
input injector, or physical capability. Capture uses only the launcher-supplied
direct RViz PID plus `/proc/PID/stat` start ticks, `/proc/PID/exe`, unique mapped
normal `_NET_WM_PID` top-level identity, and `XCompositeNameWindowPixmap`.
One thread exclusively owns `Display*`; one shared immutable JPEG is encoded at
10 fps; clients have one write in flight and skip frames. Root-window capture,
XTest, browser-native robot rendering, VNC, xpra, and websockify are absent.

The server binds numeric IPv4 `127.0.0.1` only and has no remote-bind option.
Use exact routes/methods, strict Host/Origin, no CORS, compiled assets, bounded
parser/body/header/viewer/ledger sizes, one-use state/payload-bound nonces,
operator lease, CSRF, SameSite/HttpOnly cookie, and security headers. Stop
requires the browser session/CSRF/origin but bypasses lease and normal nonce.
No command route is registered until a compatible, current, collision-checked
controller capability has been observed.

For a stronger multi-user loopback boundary, the portal should generate a
launch secret and print it only as the URL fragment used by the initial page to
bootstrap the cookie/CSRF session; remove the fragment with `history.replaceState`
after bootstrap. Loopback alone is not authentication, and same-UID malicious
processes remain an explicitly documented residual risk.

The UI seeds each side once from its own measured TCP, preserves dirty fields,
sends only the selected side for single moves, has a deliberate paired control,
and gates all motion on current state, explicit enable, lease, checked exact
scene, and no stop/fault/busy latch. It permanently discloses virtual-only,
body-frame metres, free orientation, no gripper feedback, scoped model-scene
collision, unmodelled objects, and software-stop limitations.

The right pane says **Live RViz capture** only after the actual-host gate proves
that the stock top-level redirected pixmap contains both the moving Ogre child
and Qt panels. A separate state poll drives STARTING/LIVE/UNMAPPED/STALE/
UNAVAILABLE because an `<img>` may retain an old decoded frame. There is no
fallback that may be labelled RViz.

Expose a bounded self-probe mode (or a launcher-ready fd) so Bash can wait for
HTTP health plus the first complete live frame without using Python or scraping
logs. Server mode still takes RViz PID, start ticks, and port.

### Worktree D — Bash launcher and build/install integration

Own only:

```text
scripts/build.sh
scripts/build_native.sh
scripts/install_ros_dependencies.sh
scripts/launch_rviz.sh
scripts/test_ros_coverage.sh
tests/test_native_prefix_reuse.sh (only if required)
README.md                         (integration instructions only)
```

This lane starts after target/package/CLI names freeze. Add the interface,
controller, and portal packages to the colcon selection; dependency order then
comes from package manifests. Keep native control installed before colcon. Do
not add CAN/transport/commission as ROS dependencies merely because the unified
native build also installs them.

Portal mode launch order is:

1. acquire the existing UID lock;
2. start the ROS launch group with `rviz:=false`;
3. use the compiled CLI to wait for one valid verified checked DISARMED state;
4. start stock `rviz2` directly under `setsid`, record PID and start ticks;
5. start the portal process group with that identity;
6. use the portal self-probe to require HTTP health and one complete live frame;
7. print the canonical fragment-bearing loopback URL; never auto-open a browser;
8. `wait -n` on ROS, RViz, and portal; any unexpected exit fails the session.

Shutdown is idempotent and authority-ordered: TERM portal first while ROS is
alive; portal rejects mutations, revokes lease, requests disable, waits bounded
time for a newer coherent DISARMED state, closes HTTP, then joins ROS/capture and
frees X resources. Launcher escalates portal TERM/KILL only as process cleanup,
closes RViz through `WM_DELETE_WINDOW` before TERM/KILL, then INT/TERM/KILLs ROS,
waits/reaps all children, and releases the lock. Missing disable acknowledgement
is logged as `STOP STATE UNCONFIRMED — VIRTUAL SESSION ABORTED`, never success.

The existing `rviz:=false` headless exec path may remain. ROS launch itself is
Python; the new controller, CLI, portal, HTTP server, and command path are
compiled. If the product means literally no Python process, that is a separate
launcher redesign and cannot be claimed by this DAG.

## ABI and packaging hazards

1. Existing V1 records use both exact-size and prefix-size validation. Every new
   input record needs an explicit minimum prefix and defaulted optional tail;
   every new output record should require its full independent size. Failed
   calls must leave caller output sentinels unchanged.
2. Do not append collision fields to `oa_motion_plan_report`; old source rebuilt
   against a larger definition and old binaries have different write capacity.
   Use `oa_motion_plan_collision_report` and a new getter.
3. Freeze the exact argument order of `oa_controller_create_with_scene`; the
   reports disagree (`manifest,scene,options` versus
   `manifest,options,scene`). Choose one before parallel work. Recommended:
   `manifest, scene, options, out`, matching ownership order in the ROS RAII
   session.
4. The scene must use its own typed monotonic token registry. Destroy overlap,
   cross-type token misuse, allocation failure, and token exhaustion need the
   same tests as manifest/plan/controller. Controller creation snapshots or
   shared-owns immutable scene data before returning; destroying the scene
   handle cannot dangle the controller.
5. A caller-set integer is not geometry authority. Scene revision is generated
   from/frozen with the content digest; ROS/browser can only echo it. Disable the
   existing revision setter for a checked-scene controller or make it return
   unsupported.
6. `openarm_control` is static. Adding FCL means its exported CMake config must
   `find_dependency(fcl)` and its public target must propagate the required link
   target even though the public ABI remains C. Verify both build-tree and
   installed C11/C++17 consumers.
7. Include order with `openarm_model.h` and `openarm_control.h` still requires
   `OPENARM_DISABLE_LEGACY_GENERIC_STATUS`; the ROS C++ session must define it
   consistently before either header or consume only the control header's
   transitive model target without including both ambiguously.

## Lyrical ROS interface/action hazards

- Lyrical on this host has the required generators/runtime and
  `rosidl_get_typesupport_target`; it requires C++17 for generated interfaces.
- A pure interface package avoids same-package generated-header races and
  typesupport linking. If interfaces remain in `openarm_ik_ros`, generation must
  precede targets and those targets must explicitly link the returned C++
  typesupport target. Merely adding include directories is insufficient.
- `rosidl_generate_interfaces()` must be called before `ament_package()` and
  must list referenced packages in `DEPENDENCIES`. Missing `std_msgs`,
  `geometry_msgs`, or `unique_identifier_msgs` fails generation, often before
  the node target is configured.
- Actions generate hidden SendGoal/GetResult services and FeedbackMessage
  types. Do not hand-roll action UUIDs as acknowledgement authority; use the
  `rclcpp_action` goal UUID and copy it into the explicit result field for the
  HTTP ledger.
- Package.xml needs `rclcpp_action` in controller/portal consumers, while the
  interface package needs generator/runtime/group declarations. `action_msgs`
  is a runtime dependency of the ROS action stack; declare it explicitly in
  consumers if the package linter does not accept the transitive dependency.
- Do not make the non-ament `openarm_control` archive an IDL dependency. The ROS
  controller package finds it via CMake config and documents/provides the local
  build ordering; rosidl only sees interface packages.
- Use `/usr/bin/python3` for colcon/generation as the current build already does;
  this host otherwise has a third-party Python earlier on PATH.

## Missing dependencies

Already installed: ROS Lyrical rosidl generators/runtime, rclcpp_action,
unique_identifier_msgs, tf2_ros/tf2_geometry_msgs, Boost 1.90 headers, X11,
XComposite, XDamage/XFixes/Xext, and libjpeg-turbo's standard JPEG API.

Missing for the lower-risk exact-mesh collision implementation:

```text
libfcl-dev 0.7 (candidate available, not installed)
```

Its Ubuntu dependency closure supplies `libccd`/OctoMap as required. If exact
FCL is selected, add `libfcl-dev` to the dependency installer and CMake/package
metadata. Do not silently fall back to unchecked motion when it is absent.

The installer also needs explicit entries for the newly used development/ROS
surfaces, even if this workstation already has them transitively:

```text
libboost-dev libxcomposite-dev libjpeg-turbo8-dev
ros-lyrical-rosidl-default-generators ros-lyrical-rosidl-default-runtime
ros-lyrical-rclcpp-action ros-lyrical-unique-identifier-msgs
ros-lyrical-std-msgs ros-lyrical-tf2-ros ros-lyrical-tf2-geometry-msgs
```

`libboost-system-dev` has no generic candidate on this Boost 1.90 host; modern
Asio/Beast can use Boost.System header-only. Do not add a nonexistent generic
package merely because older guides list it.

## Test and merge order

1. Freeze new C ABI records/symbol signatures, ROS IDLs, graph names, package
   names, executable names, portal schema/routes, and shutdown protocol.
2. Run collision generator integrity/hash/containment and independent transform
   parity tests before compiling controller acceptance tests.
3. Run native model/control unit tests, new ABI/prefix/sentinel/registry tests,
   analytic motion extrema/continuity tests, collision adversaries, async-stop
   races, then frozen-original and installed C11/C++17 consumers.
4. Build/test `openarm_ik_interfaces` alone. Inspect installed generated C++
   headers and typesupport exports before controller or portal compilation.
5. Run `VirtualControlSession`/arbiter tests with fake clock and fault injection,
   then node/action/CLI tests headlessly. Verify one JointState and one TF
   authority, measured provenance, state/action UUID correlation, no auto-arm,
   reset-to-DISARMED, and stop priority.
6. Run portal capture/JPEG/HTTP/security/UI tests with fake X and fake ROS,
   including sanitizers, slow clients, request smuggling, replay, lease races,
   dirty-field behavior, and shutdown at every phase.
7. Run clean unified build/install and ELF/symbol gates. ROS/controller/portal
   must have no CAN, transport, commission, VNC/xpra/websockify, Python-web, or
   XTest dependency. DDS and the portal's loopback sockets are expected; AF_CAN
   and interface-changing netlink/tool execution are not.
8. Run headless ROS launch, action/CLI end-to-end motion, stop/reset/fault and
   SIGINT/SIGTERM/unexpected-child tests. Confirm the listener is only
   `127.0.0.1` and relaunch is immediate after cleanup.
9. Last, from the logged-in GNOME/XWayland session, run mandatory actual-RViz
   acceptance with software and integrated renderers: full Ogre + Qt capture,
   occlusion privacy, minimize overlay, resize/HiDPI, four-client soak, latency,
   PID/start reuse rejection, WM close, process/socket cleanup. This cannot be
   replaced by CI or a mock.

Stages 2-3, interface Stage 4, and portal pure-module Stage 6 may execute in
parallel after contract freeze. Stages 5 and P2 wait for A/I0; Stage 7 waits for
all code lanes; Stages 8-9 are strictly serial integration gates.

## Current blockers and verified baseline

- **External-input blocker removed for virtual mode:** a surveyed physical pole
  record is not needed to validate the pinned virtual model. It remains a hard
  physical blocker, and physical is unsupported in every layer.
- **Implementation blocker:** no collision engine/checked scene exists today;
  current success uses `OA_COLLISION_VIRTUAL_UNCHECKED` and reports false.
- **Implementation blocker:** no single-TCP planner or production virtual
  manifest builder exists.
- **Implementation blocker:** the current plant steps acceleration and snaps to
  target; it cannot support the jerk claim.
- **Implementation blocker:** current serialized C calls do not provide bounded
  stop admission during a long planner call.
- **Implementation blocker:** current ROS publishes commanded IK as fresh state,
  fabricates two zero fingers, and has no actions/services/session/controller.
- **Dependency blocker:** `libfcl-dev` is not installed if the recommended exact
  mesh path is selected.
- **Empirical release blocker:** stock RViz's redirected parent pixmap must be
  proven to include the native Ogre child on the logged-in host. Failure means
  the product is not releasable as “actual RViz”; it does not authorize a root
  crop or browser renderer fallback.
- **Stale install:** source/CMake contain `close_rviz_window`, but the installed
  package directory contains only `openarm_ik_ros_node`; the current GUI
  launcher fails until a clean workspace rebuild.

With the corrections above, the DAG is acyclic, file ownership is
non-overlapping, full checked virtual functionality is implementable from the
pinned assets, physical behavior remains fail-closed, and the only unavoidable
non-source release gate is actual stock-RViz capture on the user's live display.
