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
  exact 50-symbol archive manifest; capture the installed frozen include path
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
- Native Cartesian inputs use the Model-owned strict-C `oa_vec3d` contract:
  three contiguous IEEE binary64 `double` values and explicit metre,
  centimetre, or inch units. Convert once at the highest ingress and never
  narrow coordinate storage or arithmetic through `float`.
- Unit-aware Model and Control entry points are additive. Runtime's unit adapter
  is a separate installed header so the frozen Runtime V1 header and exact
  50-symbol archive remain unchanged.
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
