# Final independent integration verification

Date: 2026-07-29 (America/Los_Angeles)  
Commit: `315f787f52c42214d0fc6253d71f57ffe5b39c2f`  
Verdict: **CLEAN**

## Reassessment of verification assumptions

The initial report treated “seven libraries” and “12 ROS tests” from the
verification request as repository requirements. Commit-level inspection shows
that both were stale external assumptions, not product contracts. They are
**DISMISSED** and do not affect the CLEAN verdict.

The declared native inventory is exactly six components. At commit `315f787`,
`scripts/build_native.sh` says it builds “CAN, model, commission, transport,
control, and runtime,” and `tests/installed_native_consumer/CMakeLists.txt`
finds and links exactly these six targets:

- `OpenArm::Can`
- `OpenArm::Model`
- `OpenArm::Commission`
- `OpenArm::Transport`
- `OpenArm::Control`
- `OpenArm::Runtime`

The clean install and strict C11/C++17 consumers match that inventory and pass.
The seventh package-config path is the intentional lowercase `openarm_can`
compatibility entry point for the same CAN archive, not a seventh library. No
CMake, script, test, or README contract promises seven unique native libraries.

The declared ROS source-test inventory is exactly 13. At commit `315f787`,
`openarm_ik_ros/CMakeLists.txt` contains 13 registrations and `scripts/build.sh`
explicitly requires `registered_ros_tests == 13`. The fresh run passed all
13/13. No current CMake, script, test, or README contract promises 12; that was
an earlier pre-runtime/portal count.

## Safety and repository state

- Source was inspected and exercised read-only. No source file or commit was changed.
- The pre-existing modified `transport/tests/test_transport.cpp` and untracked
  portal reports/logs were preserved.
- All build trees and temporary launcher state were under
  `/var/tmp/openarmik-final-integration.jWHs9v`; that tree was removed after
  evidence was copied to `.swarm`.
- Only the virtual backend was used. No CAN/vcan command, network-interface
  change, physical motion, calibration, or physical-movement claim was made.
- A pre-existing unrelated RViz stack was observed on the shared desktop and
  was not targeted. Verification used private ROS domains and runtime lock
  directories. All verification-owned PIDs and domain participants were gone
  after shutdown.

## Clean Release build, install, exports, and tests

Fresh empty root and unified command:

```bash
verify_root=$(mktemp -d /var/tmp/openarmik-final-integration.XXXXXX)
mkdir -p "$verify_root/tmp" "$verify_root/out"
TMPDIR="$verify_root/tmp" scripts/build.sh --tests \
  --output-root "$verify_root/out"
```

Result: status 0 with GCC/G++ 15.2.0, Release. Native totals including the new
runtime were 16/16:

| Component | CTest result |
|---|---:|
| CAN | 1/1 |
| model | 4/4 |
| commission | 2/2 |
| transport | 3/3 |
| control | 4/4 |
| runtime | 2/2 |

The unified build then built and installed all three ROS packages. The exact
source-built ROS command was:

```bash
source /opt/ros/lyrical/setup.bash
source "$verify_root/out/install/setup.bash"
ROS_DOMAIN_ID=231 ROS2CLI_NO_DAEMON=1 TMPDIR="$verify_root/tmp" \
  ctest --test-dir "$verify_root/out/build/openarm_ik_ros" \
    --output-on-failure
```

Result: status 0, 13/13 passed in 91.84 s.

Installed-consumer commands and results:

```bash
"$verify_root/out/native_build/installed_native_consumer/openarm_installed_c11"
"$verify_root/out/native_build/installed_native_consumer/openarm_installed_cxx17"
```

Both returned 0. They compile under strict C11/C++17 with warnings, pedantic
diagnostics, and warnings-as-errors. The following configs and exported-target
files were present and nonempty: `OpenArmCan`, lowercase compatibility
`openarm_can`, `openarm_model`, `openarm_commission`, `OpenArmTransport`,
`openarm_control`, and `openarm_runtime`. The clean installed-consumer CMake
configure found every one of the six unique native targets through those
configs.

Production-surface checks found no test-hook compile definition on any of the
six production targets, no installed `*test*.a` archive, and zero suspicious
`_test_`, `::test::`, or test-hook symbol matches in every installed archive.
The runtime archive's only undefined CAN builder was
`oa_can_make_register_query_typed`. The ROS no-CAN-linkage test also passed.

## Sanitizers

Fresh Debug component trees were configured individually with the project
options below, built, and run with:

```bash
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1:abort_on_error=1
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1
ctest --test-dir COMPONENT_BUILD --output-on-failure --no-tests=error
```

Options were `OA_CAN_ENABLE_SANITIZERS=ON`, `OA_MODEL_SANITIZERS=ON`,
`OA_COMMISSION_ENABLE_SANITIZERS=ON`,
`OA_TRANSPORT_ENABLE_SANITIZERS=ON`, `OA_CONTROL_SANITIZERS=ON`, and
`OA_RUNTIME_ENABLE_SANITIZERS=ON`. Results: CAN 1/1, model 4/4, commission 2/2,
transport 2/2, control 3/3, runtime 2/2. Install-consumer tests deliberately
guarded out by sanitizer CMake conditions are not included in those counts.

Fresh TSan builds used:

```bash
TSAN_OPTIONS=halt_on_error=1:second_deadlock_stack=1
```

with each project's explicit TSan option. Results: transport 2/2, control 3/3,
runtime 2/2. CAN, model, and commission expose ASan+UBSan options but no TSan
option.

The ROS package was also configured directly with
`-fsanitize=address,undefined -fno-omit-frame-pointer`. The instrumented virtual
session (24.72 s), active-SIGINT (10.97 s), and CLI/server lifecycle (22.41 s)
tests passed. The first lifecycle invocation was a verifier setup error because
the production CLI target had not been built; after building that requested
target, the exact test passed 1/1. A separate `-fsanitize=thread` build of the
ROS virtual-session target passed 1/1 in 24.72 s.

## Isolated live RViz and measured virtual motion

The current launcher was copied byte-for-byte to a temporary launcher tree,
whose `ros2_ws/install` pointed at the fresh `/var/tmp` install. This avoids the
repository's stale/default build products while exercising the current launcher
logic. SHA-256 of both launchers was
`5055a267e440974e51834b50c2aabe4dba606e3ee222e08933838a90ebb2299e`.

Main launch command (domain 201):

```bash
ROS_DOMAIN_ID=201 ROS2CLI_NO_DAEMON=1 TMPDIR="$verify_root/tmp" \
XDG_RUNTIME_DIR="$verify_root/xdg-live" OPENARM_RVIZ_RENDERER=software \
  "$verify_root/live_root/scripts/launch_rviz.sh"
```

It selected `Mesa software rasterizer (XWayland/GLX)`, reported OpenGL/GLSL 4.5,
and remained stable throughout both completed commands. Graph inspection showed:

- `/joint_states`: exactly one publisher, `openarm_ik_ros`;
- `/tf` and `/tf_static`: exactly one publisher each, `robot_state_publisher`;
- TF graph: 25/25 edges, covering all 26 links, including four fixed finger
  links and both named hand TCP links;
- actions: `move_joint` and `move_paired_tcp` present;
- diagnostics: WARN `virtual backend; collision unchecked`,
  `physical_motion_authorized=false`, `collision_checked=false`, zero fault
  masks, and zero left/right timestamp skew.

RViz showed a green check on the OpenArm RobotModel and `RViz is ready.` No
finger, unrealistic-inertia, RobotModel, Ogre, or renderer warning appeared.
The inspected 900x646 screenshot is
`.swarm/final_integration_rviz.png` (64,100 bytes, SHA-256
`e744b2190337d33d64badf0bcaa5871cdb18ed509060346ac82b731e38f092b9`).

### Individual joint via installed compiled CLI

```bash
ros2 run openarm_ik_ros openarm_control_cli \
  move-joint openarm_left_joint4 0.2
```

Result: status 0, `completed command_id=1`, elapsed 2.214 s.

- Baseline measured joint4: `0.0000667582207984907` rad.
- Intermediate measured joint4: `0.03287365529869568` rad with measured velocity
  `0.2368742368742378` rad/s, visibly lagging the 0.2-rad target.
- Intermediate action feedback: progress `0.19461749076997903`, sequence
  13585/13585.
- Executing diagnostic: seed 13487/13487, current 13702/13702, timestamp
  68,500,079,033/68,500,079,033 ns, skew 0, pending/not committed.
- Terminal measured joint4: `0.1999599450675209` rad in two later samples.
- Terminal diagnostic: terminal sequence 13791/13791, plan duration
  1,338,716,917 ns, `reason=completed_measured_feedback`, `outcome=completed`,
  `committed=true`. A later dwell sample retained the same measured position
  while coherent health advanced to sequence 14602/14602.

### Paired named TCP XYZ via installed compiled CLI

```bash
ros2 run openarm_ik_ros openarm_control_cli move-paired-tcp \
  openarm_body_link0 0.20 0.30 0.85 0.20 -0.30 0.85
```

Result: status 0, `completed command_id=2`, elapsed 21.313 s.

- Baseline measured left/right XYZ:
  `(0.079796132, 0.153463844, 0.084001794)` /
  `(0.000080779, -0.153526825, 0.075999557)` m.
- Intermediate measured XYZ:
  `(0.087068152, 0.162302682, 0.128982467)` /
  `(0.036076556, -0.179918722, 0.213212394)` m, still far from both targets.
- Intermediate action feedback: progress `0.12928449915009088`, sequence
  20988/20988.
- Executing diagnostic: seed 20723/20723, current 20952/20952, timestamp
  104,750,067,627/104,750,067,627 ns, skew 0, pending/not committed.
- Terminal measured XYZ:
  `(0.199990286, 0.299922306, 0.850034049)` /
  `(0.200000323, -0.299904228, 0.849973368)` m.
- A later independent TF sample returned exactly the same printed XYZ.
  Terminal sequence was 24867/24867 after a 20,525,467,196-ns plan;
  `completed_measured_feedback`, completed, committed. Coherent health then
  advanced from 26302/26302 to 27902/27902 during terminal dwell.

## Lifecycle bounds and cleanup

The compiled production CLI's `test_cli_server_lifecycle` stops servers in each
of queued, started, and settling phases and asserts exit code 8, the exact
`action server lost after goal acceptance` reason, no terminal-timeout path, no
remaining client, and elapsed time `<=2.0 s`; the full Release and corrected
ASan runs passed.

Idle Ctrl+C used domain 200 and the same fresh-install launcher. The wrapper
returned 130 in 0.7 s observed wall time; launcher, RViz, state publisher, and
adapter PIDs were all absent afterward, and the isolated node list was empty.

For active Ctrl+C, paired command 4 was proven live at progress
`0.044156975975326085`, sequence 40230/40230. Ctrl+C caused the wrapper to return
130 after its orderly RViz-close-first sequence (4.58 s from the last pre-signal
timestamp). The ROS launch record reports both server children finished cleanly
at 1785372008.3845944/1785372008.3864408. The separately invoked compiled CLI
returned code 8 with `action server lost after goal acceptance` at
1785372009.165329338: 0.779 s after the adapter/server finished, below 2 s.
All owned PIDs were gone immediately afterward; the transient DDS participant
listing cleared within 3 s and no window or process remained.

One preliminary idle attempt used invalid Fast DDS domain 240 and failed closed
before a live graph formed (`domainId is over 232`); the valid isolated-domain
run above is the verification result.

## Evidence

- `final_integration_build.log`
- `final_integration_ros_tests.log`
- `final_integration_asan.log`
- `final_integration_tsan.log`
- `final_integration_ros_asan.log` and corrected lifecycle rerun log
- `final_integration_ros_tsan.log`
- `final_integration_production_surface.log`
- `final_live_graph.log`, `final_live_frames.log`, `final_live_frames.gv`
- `final_joint_*`, `final_paired_*`, and `final_active_*` measured evidence logs
- `final_live_ros_launch.log`
- `final_integration_rviz.png`
