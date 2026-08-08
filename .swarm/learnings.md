# Durable Repository Learnings

- The host runs Ubuntu 26.04 x86_64 with ROS 2 Lyrical installed at `/opt/ros/lyrical`.
- RViz works under OpenGL 4.6 after sanitizing VS Code Snap GUI environment variables in the user's `.bashrc`.
- No physical CAN interface or USB CAN adapter was present during the 2026-07-28 inventory.
- The canonical current OpenArm v1.0 xacro has a 0.1025 m link7-to-hand transform plus a 0.0835 m hand-to-`hand_tcp` transform. Its checked-in flattened example is stale at zero TCP offset.
- OpenArm v1.0 physical arm coordinates cannot be inferred safely from CAN discovery: each unit requires a commissioned side/joint/sign/scale/zero/firmware/timeout manifest before arming.
- On this GNOME Wayland/HiDPI host, RViz/Ogre requires XWayland/GLX. Hardware GLX flickers during live resize; Mesa software OpenGL is stable enough for the OpenArm scene and is the preferred Wayland default.
- RViz should be closed through `WM_DELETE_WINDOW` before ROS receives SIGINT on this host; this avoids the observed Ogre teardown crash and lets both authored ROS nodes exit cleanly.
- DaMiao/OpenArm feedback is output-shaft encoder position in radians. The integrated motor reduction is verified configuration metadata and must not be applied again in host joint-angle conversion.
- DaMiao absolute output encoders do not reveal assembled arm side/joint/URDF zero. First-time calibration needs a known reference pose or a separately qualified, supervised mechanical-stop recipe.
- The completed controller foundation is still not a physical-motion claim. Its
  reviewed codecs, commissioning state machines, query-only transport,
  encoder-driven controller, simulator, and Runtime facade were verified without
  connected CAN hardware or actuator transmission.
- Production `openarm_runtime` is deliberately virtual-only and has no CAN or
  transport linkage. Physical inventory clears the caller's output and returns
  `OA_RUNTIME_EUNSUPPORTED`; bounded `/sys/class/net` rows are link metadata,
  never motor discovery or commissioned identity.
- The ROS production session must use `OpenArm::Runtime` as the sole state,
  planning, execution, heartbeat, stop, and event authority. Direct
  `oa_controller_*`, `oa_motion_plan_*`, or legacy manifest references are an
  authority bypass and are rejected by symbol audits.
- Build parallelism is an end-to-end resource contract: validate a positive job
  count, propagate it to CMake and CTest, and use sequential colcon execution so
  nested package builds cannot multiply peak memory. Canonical output/build/install
  locks belong to the waiting supervisor until its isolated callback group exits.
- Incremental compilation caches are reusable, but installed launch authority is
  not a cache. Recreate the complete install prefix from empty under exclusive
  locks, publish no stamp on failure, and retain shared leases while a validated
  launch consumes the tree.
- Launch freshness must bind actual tracked/untracked/ignored source bytes,
  pinned description identity, compiler/wrapper and backend contents, toolchain
  bytes, canonical output root, and every installed file/directory/internal-link
  target. Broken or escaping symlinks and external portal executable overrides
  fail closed.
- Runtime ABI V1 includes its public transitive Commission 0.1.0 records. Freeze
  both headers, numeric layouts, correctly typed retained references, and the
  current exact 57-symbol archive manifest; capture the installed frozen include path
  before nested `find_dependency()` calls for CMake 3.16-3.29 split prefixes.
- `1ece782` is the completed hardware-free hardening baseline: description pin,
  launch/resource regressions, Runtime 9/9, and the bounded one-job three-package
  ROS build passed. Physical bus timing, motion, collision safety, stop distance,
  thermal behavior, and emergency-stop acceptance remain untested.
- Reusable CMake caches need transactional requested/actual provenance. Bind
  compiler and linker launcher arguments, canonical paths, and bytes, and reject
  component directories or cache files that are not physically contained below
  the selected build root.
- Recursive cleanup must be limited to explicit output children or marker-owned
  trees; caller-controlled generic deletion is not an acceptable internal API.
  `59590d1` is the independently CLEAN cache-hardening baseline after the
  `669ab88` integration and C1/I1/I2 cross-confirmation. Physical limitations
  above remain unchanged.
- Portal movement-rate changes should preserve the original
  `MovePairedTcp.action` contract. Use the additive scaled action, validate the
  binary64 limit scale at JSON, ROS, and session boundaries, and keep legacy
  paths explicitly at 0.5. A single equal velocity/acceleration/jerk scale is a
  movement-limit percentage, not a linear travel-time percentage.
- Native Cartesian inputs use the Model-owned strict-C `oa_vec3d` contract:
  three contiguous IEEE binary64 `double` values and explicit metre,
  centimetre, or inch units. Convert once at the highest ingress and never
  narrow coordinate storage or arithmetic through `float`.
- Unit-aware Model and Control entry points are additive. Runtime's unit adapter
  is a separate installed header so the frozen Runtime V1 header and exact
  exported-symbol archive remain unchanged by unit conversion.
- Portal display/input defaults to centimetres and may toggle to inches, but its
  canonical values, ROS messages, Runtime/model calculations, and stock RViz
  stay metres. The v2 portal request names its unit explicitly and the server
  performs the sole conversion.
- Expanding length conversions must reject magnitudes at or above
  `DBL_MAX / |factor|` before multiplication; post-multiply finiteness alone is
  insufficient under directed rounding.
- The completed binary64/portal stage has a CLEAN final sweep with targeted
  native suites, portal 24/24, ROS 14/14, and production launch-integrity green.
  Physical CAN, calibration, actuator motion, and safety acceptance remain
  unsupported and untested.
- The web launcher uses cropped real RViz pixels over MJPEG, with pointer input
  replayed into the RViz render widget. Keep `scripts/launch_rviz.sh` as the
  separate engineering viewer and do not add VNC/noVNC. The blue filter is
  browser-local and must never issue a robot or RViz command.
- Viewer timing claims must name the measured stage. The stream targets 30 FPS;
  that does not prove compositor presentation or physical display scanout.
- Viewer assets retain pinned Stage-A/model provenance and Apache-2.0 license
  closure. Keep
  static and API work in separate bounded lanes, use asynchronous pre-route reads
  with a real deadline and oldest-incomplete eviction, and test partial-body stop,
  shutdown, FD, and thread cleanup. The completed ROS inventory is 16/16; physical
  CAN, calibration, motion, collision safety, and emergency-stop acceptance remain
  unsupported and untested.
- OpenArm v1.0's pinned kinematic offsets give a 0.747--0.748 m
  shoulder-to-TCP centreline upper bound. The audited symmetric High far portal
  preset `[0.28, +/-0.67, 0.52]` m displaces the TCP about 0.736 m from neutral
  while retaining at least 0.0265 m in the 1,800-transition sampled cross-state
  matrix. Portal presentation supports explicit m/cm/in while all canonical
  coordinate storage and arithmetic remain IEEE-754 binary64 metres.
- Portal best effort is a virtual-only straight-ray projection: validate the
  exact request, scan 64 fixed subdivisions, then use progressively denser
  16-, 32-, and 48-sample non-monotonic refinement scans. Preserve the actual failing path waypoint as
  an irreversible keepout boundary, discard any prior endpoint beyond it, and
  continue across isolated IK failures. Reject projected motion below 1 mm.
  Never search beyond a sampled collision or route around an obstacle. Invalid
  feedback or an already-unsafe scene fails closed; Runtime remains
  `collision_checked=false` and physical motion remains unsupported.

## 2026-08-06 intentional terminal contact

- A shared claw midpoint is not a valid paired TCP endpoint: it drives the two
  tools through one another. Derive one stop-short endpoint per measured TCP.
  Do not use one fixed stop radius: position-only IK can reach the same XYZ on a
  different orientation branch. Search exact pinned hand/finger mesh distance
  from the actual measured joint seed, with a small bounded planned overlap so
  feedback quantization cannot end just short; stop execution at first touch.
- A contact exception must be pair-specific and evidence-bearing. Excluding all
  end links or globally lowering clearance hides unrelated claw/arm/pole
  collisions. The usable scope here is only the approaching hand-housing pair;
  the other eight hand/finger cross-pairs and every capsule/pole pair must be
  evaluated in the same call.
- Portal approval alone cannot make post-contact retreat reliable. The
  controller must recognize the measured start contact and prove its own whole
  path is monotonically opening before carrying the scoped policy. Also gate the
  virtual tangent auto-stop on `contact_monitored`; policy scope by itself must
  not stop a retreat.
- A ROS CTest run from a shell that has not sourced the freshly recreated
  workspace can fail before `main()` on generated message libraries. Source
  `/opt/ros/lyrical/setup.bash` and `ros2_ws/install/setup.bash` before treating
  dynamic-loader failures as test failures.

## 2026-08-06 blank RViz stream after restart

- A healthy portal and MJPEG route do not prove RViz rendered anything. On this
  hybrid laptop, forced NVIDIA PRIME failed GLX context creation when the NVIDIA
  driver was unavailable; the RViz process aborted and left the browser with no
  producer. Probe the renderer itself and attach a real stream client.
- The panel-free captured window does not need the software renderer used to
  mitigate interactive window-resize flicker. Accelerated integrated AMD GLX
  rendered the raw scene at about 35.7 streamed fps and accepted synthetic
  orbit input. Keep the portal raw: no CSS color filter or palette UI.

## 2026-08-06 partial real-arm telemetry

- A motor count is not a joint map. With IDs 1,3,4,5,6,7,8 present, a
  size-based `>=7` test silently maps the gripper into J7. Require the exact
  J1..J7 ID set and index feedback by `send_id`; leave partial arms neutral.
- A read-only observer can continuously refresh IDs 1..8 on both buses while
  publishing a complete arm from only one bus. Re-request missing IDs every
  cycle so a repaired joint appears without reconnecting, but never publish a
  partial pose that invents the missing transform.
- Once records are keyed by exact IDs and carry a per-joint validity bit, a
  partial arm need not be hidden wholesale. Publish fresh exact-ID joints and
  hold only the missing joint at its last exact reading; never compact the
  array, and expire “current reading” eligibility independently per joint.
- Verify a claimed read-only hardware process at the wire: during this launch,
  arbitration IDs were only 0x7FF refreshes and 0x11..0x18 replies. Structural
  source restrictions plus a live candump audit are stronger evidence than a
  UI label.
- On this installed pair, operator-confirmed motion mapping is can0=robot-right
  and can1=robot-left. An LED observation was mislabeled from the opposite
  viewpoint; actual physical-arm motion against named RViz joints is the more
  direct side-identification test. Pin this mapping in launch/calibration paths
  because the dual-channel adapter exposes one shared USB identity.

## 2026-08-06 operator-guided display calibration

- Keep physical encoder calibration factored as
  `direction * (raw - relaxed_reference) + display_offset`. A sign flip then
  reverses motion about the accepted relaxed pose without conflating direction
  with the visual pose correction.
- Persist reference and offset with 17 decimal digits and prove round-trip with
  `EXPECT_DOUBLE_EQ`; ROS interface `float64` generates a C++ `double`, and a
  compiled `static_assert` guards that boundary against narrowing.
- Reconnecting a live observer must not silently redefine the relaxed
  reference. Capture it explicitly once, preserve it across restarts, and keep
  direction/offset changes independently addressable per side and J1--J7.
- A calibration UI/service can be hardware-safe by adjusting only the local
  `/joint_states` affine map. Verify that guarantee separately at the wire; the
  current all-motor run showed only `0x7FF`/`0xCC` refreshes and `0x11`--`0x18`
  replies on both buses.
- A normal-looking pose before a delayed auto-connect is not calibration
  evidence: the observer intentionally publishes neutral while passive. A jump
  at connection means the first encoder-derived affine value differs from
  neutral; compare raw/reference numerically before diagnosing intermittent
  motion.

## 2026-08-06 read-only directional-range capture

- A permissible range is directional data, not merely two extrema. Preserve
  the timestamped acquisition order and require a known hand path from relaxed
  through both operator-chosen safe limits and back; min/max alone cannot prove
  encoder polarity.
- Do not ask the operator to force a mechanical hard stop. The useful limits
  for an unpowered arm are deliberate safe limits with physical clearance;
  conservative control limits can only be derived after reviewing the trace,
  URDF limits, meshes, mounting clearance, and encoder repeatability together.
- GUI programs launched from the Snap-packaged VS Code environment must call
  `openarm_sanitize_snap_environment` before sourcing ROS. Otherwise GTK can
  load `/snap/core20` pthread against the host glibc and fail before `main()`.
- Mesa software GLX was not viable for live physical feedback: RViz consumed
  about nine CPU cores during a hand sweep and starved both GTK and the desktop.
  Integrated GLX reduced RViz to about one third of a core. A little flicker
  during live resize is preferable to making the commissioning UI unresponsive.
- Persist a completed joint trace at Stop, not only at the end of a multi-joint
  session. A renderer failure after one good sweep must not make the operator
  repeat physical calibration work.
- CAN RAW loopback suppresses a socket's own transmission, not transmissions
  from another local raw socket. Every feedback reader must validate the exact
  expected reply arbitration ID before decoding payload bytes. A DaMiao J2
  refresh payload (`02 00 cc 00 ...`) itself resembles a valid-looking frame at
  roughly -12.42 rad with minimum velocity and torque if arbitration identity
  is ignored.

## 2026-08-07 collision-aware Cartesian routing

- Valid endpoint plus valid endpoint does not imply a valid straight Cartesian
  segment. Treat bimanual routing as graph connectivity in seeded IK state, and
  validate every candidate edge from the terminal joint solution of its actual
  predecessor. A canonical anchor is a search hint, never safety evidence.
- A route is only useful if execution follows its edges. Do not validate a
  multi-leg route and then submit the final target to a one-segment controller;
  keep the portal command active and submit the guarded legs in order, clearing
  every pending leg on cancel or E-stop.
- Planning clearance recovery and real-time intervention are distinct. A pose
  between the 25 mm planning gate and 10 mm intervention floor needs a narrowly
  monotonic opening escape; once the path recovers 25 mm, fail closed on any
  later re-entry. This avoids a collision dead-end without weakening approach
  prevention.
- Direct ROS CTest execution must source the installed workspace. Otherwise a
  healthy compiled CLI test can fail only because the dynamic loader cannot
  find generated ROS message libraries.

## 2026-08-07 dynamic execution and contact retreat

- Never feed a terminal-contact retreat back through the ordinary path
  validator. Contact deliberately begins inside the normal claw keepout; only
  an exact, monotonically opening escape proof may authorize the first move out.
- A collision-free predicted route is only a proposal. Replan the unexecuted
  tail from measured joints at every leg boundary, prove the chosen first edge,
  and revalidate encoder plus diagnostic evidence immediately at handoff. Keep
  the lower-level measured keepout monitor active every control cycle.
- An action result can arrive before the next idle diagnostic is processed by a
  single-threaded ROS executor. Do not sleep in that callback and do not bypass
  health checks: schedule a bounded asynchronous readiness poll so live state,
  cancellation, and E-stop callbacks continue running.
- Quantized feedback needs one consistent, model-derived boundary policy at
  guard intake, routing, and final handoff. Allowing one DaMiao position code
  for measured state prevents a valid endpoint from becoming a prison; planned
  IK states must still remain strictly inside model limits.
- HTTP 202 only means an asynchronous command was accepted. Demo automation
  must inspect the terminal command/result and stop on failure instead of
  treating `command_active=false` alone as success.

## 2026-08-07 physical J8 and watchdog lessons

- A DaMiao gripper's motor-to-finger relation must come from a persisted encoder
  endpoint and the official travel, not a guessed joint angle. Keep interpolation
  binary64 and narrow only at the documented wire field.
- Position-force mode is volatile commissioning state: write and read it back
  for both J8 motors at every explicit Connect, seed measured positions first,
  and keep J8 inside the same complete-feedback watchdog as J1--J7.
- A healthy idle soak does not clear a movement-dependent CAN fault. If one
  exact motor reply disappears while the interface reports zero errors, retain
  the fail-closed watchdog and inspect that motor's connectors/controller under
  mechanical motion.
- Intentional socket closure can make an already in-flight exchange incomplete.
  Before classifying it as a watchdog fault, recheck that the controller is
  still armed and connected; Disconnect/E-stop already established the safer
  terminal state.
- Sending disable is an action, not proof of the resulting state. An enabled
  motor that is temporarily unreachable can miss every disable, recover later,
  and remain energized. Keep CAN open, retry, decode fault replies, and require
  an explicit disabled status from every motor before saying shutdown was
  confirmed.
- Program and read-confirm the motor's volatile communications timeout before
  every enable. Keep its period longer than normal command jitter but short
  enough to fail toward disabled when the host process or bus disappears; do
  not flash-save commissioning watchdog experiments.
- A software E-stop can successfully latch while physical disable confirmation
  fails. Keep those outcomes separate in every API: the portal's
  `physical_disabled` field must reflect status feedback, not merely that the
  latch and disable attempt executed.

## 2026-08-07 single-arm physical recovery lessons

- A public physical C ABI should delegate to the one safety-owning controller;
  opening SocketCAN independently would bypass its watchdog, calibration,
  collision checks, command arbiter, and confirmed-disable semantics.
- A single-arm command is not safe merely because only one target changed. The
  planner and executor must preserve the unselected joints exactly, and a true
  recovery mode must avoid opening, configuring, enabling, monitoring, or
  disabling the electrically isolated side.
- Report inactive-side snapshot values explicitly as placeholders. A side mask
  prevents downstream programs from treating fixed model state as live encoder
  feedback.
- Near a kinematic/guarded reach boundary, accepting a command is not evidence
  that its requested endpoint was reached. Preserve the encoder-derived pose,
  velocity, residual, and terminal settle result; on timeout, hold measured pose
  and report the endpoint honestly.
- Keep hardware control and source verification isolated. Building and running
  controller lifecycle tests on the same ROS domain as an enabled physical
  session could contend for its services/actions even when the build itself is
  harmless.
