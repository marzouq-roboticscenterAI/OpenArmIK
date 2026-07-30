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

## Follow-up independent-review closure

The cancellation path now rechecks the active command id after `stop` and
event draining. If a completion event has already removed that command,
completion is terminal and cancellation returns without publishing a stale
second result or forcing the adapter into restart/fault state. A captured-cancel
boundary regression asserts exactly one terminal result and stable post-cancel
lifecycle state; the command-id recheck covers the completion-drained branch.

Health notifications are now marked under the session mutex and dispatched by
the owner worker only after it has released that mutex. A reentrant callback
regression calls `health()` from both a reserve notification and an active
worker transition, then verifies bounded shutdown. The focused session suite
contains 15 passing tests after these additions.

### Second-review lifecycle closure

Completion observed only after a successful disable-stop now retains the single
completed terminal outcome but reports the authoritative disarmed state as
`stopped_requires_restart`; it cannot claim idle and later fault on disabled
feedback. The deterministic 60 ms pre-cancel/60 ms captured boundary exercises
that branch. Test-only barrier support is compiled only with
`OPENARM_IK_ROS_TESTING`.

A health callback may request `close()` from the worker: close records shutdown
and skips self-join; a later external close/destructor joins the finished worker.
The expanded session suite passes 16/16, including this self-close regression.

### Final installed-overlay verification

After the follow-up commit, the package was rebuilt and installed with one job,
then verified using `/opt/ros/lyrical`, the base
`/home/signalprocessing-dev/OpenArmIK/ros2_ws/install` action-message overlay,
and the updated local `ros-install/openarm_ik_ros/local_setup.bash` overlay
with an isolated `ROS_LOG_DIR`. `ctest --output-on-failure -j1` completed all
13 registered package tests successfully. The installed-artifact audit again
found runtime facade references only (`create`, snapshot, joint/paired plan,
execute, heartbeat, event polling), no legacy controller/motion-plan/manifest
reference, and no CAN, SocketCAN, transport, or Control implementation dynamic
linkage.
