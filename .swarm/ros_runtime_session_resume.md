# ROS Runtime Session Resume

Date: 2026-07-29 (America/Los_Angeles)  
Branch: `refactor/ros-runtime-session`  
Integrated main safety baseline: `28f1e03` (`bd46687` physical discovery
fail-closed and planning-epoch serialization), merged as `f53a9f5`.

## Result

`openarm_ik_ros` now consumes `OpenArm::Runtime` as the only virtual session
authority.  The public ROS action and `openarm_control_msgs` contracts remain
intact, while virtual state, time, manifest/model identity, inventory,
capability exposure, planning, execution, events, persistence metadata, and
motion authorization come from the runtime facade.

The session explicitly creates only the canonical virtual manifest/runtime,
rejects physical motion/configuration capabilities, and uses runtime snapshots
for `/joint_states`, runtime plans/execution/events for actions, and a virtual
unchecked-collision policy for diagnostics.  Paired left/right body-frame XYZ
planning remains one runtime paired request.

## Integration finding fixed

After the main planning serialization merge, the first virtual feedback sample
can arrive asynchronously after `oa_runtime_arm_virtual()`.  An immediate
snapshot validation made every node/session startup fail with `initial runtime
measured snapshot is invalid`; this in turn explained the queued SIGINT test's
action-server timeout.  The session now bounds startup and shutdown feedback
settling to twice the runtime feedback timeout, drains a completion event before
classifying a heartbeat failure, and keeps ownership/result handling correct
through the terminal callback.  This is a runtime-cadence wait, not a ROS
synthetic-state fallback.

TSAN also found an authored-test race: the callback appended recorder states
under a mutex while an assertion iterated the vector without that mutex.  The
test now asserts against a locked copied snapshot.

## Evidence

All compilation used one job (`CMAKE_BUILD_PARALLEL_LEVEL=1`, `MAKEFLAGS=-j1`,
`cmake --build --parallel 1`, and a sequential colcon executor).

* Fresh current-main runtime build/test/install: `openarm_runtime_tests` and
  `openarm_runtime_c11` passed (2/2).
* Fresh current-main ROS package Release build/install succeeded. Its CMake
  cache resolved `openarm_runtime_DIR` to the freshly built verification prefix,
  not the pre-existing workspace runtime.
* Full Release CTest: all 13 `openarm_ik_ros` tests passed, including the
  virtual lifecycle/action session tests, no-CAN linkage check, ROS contract,
  CLI server-loss lifecycle, and active/queued SIGINT handling.
* ASAN+UBSAN: all 13 `test_virtual_control_session` tests passed with
  `detect_leaks=0:halt_on_error=1` and UBSAN halt/stacktrace enabled.
* TSAN: all 13 `test_virtual_control_session` tests passed with
  `halt_on_error=1` after the recorder synchronization fix.
* `nm -u libopenarm_virtual_control_session.a` resolves only runtime facade
  entry points (`oa_runtime_create`, snapshot, planning, execute, heartbeat,
  event polling); it has no `oa_controller_*`, `oa_motion_plan_*`, or
  `oa_manifest_*` bypass reference.
* The node has no OpenArm CAN/transport/control implementation dependency in
  its dynamic linkage. The only `openarm_control` name retained by the ROS
  package is the intentionally preserved `openarm_control_msgs` action ABI.

## Limitations

No physical CAN interface, transmission, physical backend, GUI, RViz capture,
or hardware was used. Runtime remains virtual-only and collision unchecked;
the portal guard is not collision certification. The verification build trees
under `.verification/` are intentionally untracked local artifacts.
