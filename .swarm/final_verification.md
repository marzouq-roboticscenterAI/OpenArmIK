# Independent final verification

Date: 2026-07-28 (America/Los_Angeles)  
Verifier branch: `review/final-verification`  
Verified main: `0318bcee2e7e637dc41f87836abcb3c9fbe8781d`

## Verdict

**CLEAN for the implemented hardware-free scope.** Fresh strict, sanitizer, ROS,
URDF, linkage/isolation, and live paired-IK checks all passed. No correctness or
safety regression was found. Physical arm movement was deliberately not attempted.

Two evidence limitations must remain explicit:

1. Coverage is high but not literally 100%. In particular, live Linux netlink and
   kernel failure paths cannot all be forced on this host without fault injection;
   exact authored-source percentages are recorded below.
2. A from-empty online invocation of `scripts/fetch_upstreams.sh` in the verifier
   worktree stalled during the first GitHub transfer and was interrupted after more
   than 60 seconds. The already-downloaded canonical source set was independently
   checked: all ten repositories are clean, non-shallow, at the documented commits,
   and have the documented origins. `bash -n scripts/fetch_upstreams.sh` passes.

## Environment

- Ubuntu host, x86-64; GCC 15.2.0; CMake 4.2.3
- ROS 2 `lyrical`; `/usr/bin/python3` 3.14.4
- Verification used fresh `/tmp/openarmik-verify-*` build trees. No physical CAN
  interface, motor, or arm was opened or commanded.

## C CAN module

Strict Release command:

```bash
d=$(mktemp -d /tmp/openarmik-verify-can-release.XXXXXX)
cmake -S can -B "$d" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build "$d" --parallel
ctest --test-dir "$d" --output-on-failure
```

Result: build passed under the project's `-Werror` warning set; 1/1 tests passed
in `/tmp/openarmik-verify-can-release.l0SNiS`.

ASan/UBSan command used `-DCMAKE_BUILD_TYPE=Debug
-DOA_CAN_ENABLE_SANITIZERS=ON`, followed by:

```bash
ASAN_OPTIONS=detect_leaks=1 \
UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 \
ctest --test-dir "$d" --output-on-failure
```

Result: 1/1 passed in `/tmp/openarmik-verify-can-sanitize.B6l4Pq`; no sanitizer
finding.

Fresh gcov run (`-DCMAKE_C_FLAGS=--coverage
-DCMAKE_EXE_LINKER_FLAGS=--coverage`) passed 1/1 and reported:

| Authored source | Lines | Branches executed | Calls |
|---|---:|---:|---:|
| `can/src/openarm_can.c` | 96.88% of 352 | 99.42% of 344 | 100% of 77 |
| `can/src/openarm_can_linux.c` | 60.73% of 191 | 71.26% of 174 | 36.00% of 25 |

The lower Linux number is concentrated in live socket/bind/setsockopt/send/recv
and kernel-message branches; the pure parser paths and CAN codecs remain covered
by the unit suite.

## C model/FK/Jacobian/IK module

Strict Release plus deterministic generator command:

```bash
d=$(mktemp -d /tmp/openarmik-verify-model-release.XXXXXX)
cmake -S model -B "$d" -DCMAKE_BUILD_TYPE=Release \
  -DOA_MODEL_BUILD_TESTS=ON -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOA_DESCRIPTION_ROOT=/home/signalprocessing-dev/OpenArmIK/upstream/openarm_description \
  -DOA_XACRO_EXECUTABLE=/home/signalprocessing-dev/OpenArmIK/.deps/xacro/root/opt/ros/lyrical/bin/xacro \
  -DOA_XACRO_PYTHONPATH=/home/signalprocessing-dev/OpenArmIK/.deps/xacro/root/opt/ros/lyrical/lib/python3.14/site-packages:/opt/ros/lyrical/lib/python3.14/site-packages \
  -DOA_XACRO_AMENT_PREFIX=/opt/ros/lyrical
cmake --build "$d" --parallel
ctest --test-dir "$d" --output-on-failure
```

Result: 4/4 passed in `/tmp/openarmik-verify-model-release.iEm4XI`, including C
model tests, ABI-v1 no-write canary, independent Python URDF reference comparison,
and byte-for-byte deterministic xacro/model regeneration.

Debug ASan/UBSan with `-DOA_MODEL_SANITIZERS=ON` passed 3/3 in
`/tmp/openarmik-verify-model-sanitize.weTBb3`; no sanitizer finding. A fresh gcov
build with `-DOA_MODEL_COVERAGE=ON` passed 3/3 and reported
`model/src/openarm_model.c`: 99.65% of 288 lines, 99.35% of 308 branches executed,
79.87% taken at least once, and 97.78% of 45 calls.

## ROS 2 build and tests

A clean temporary model prefix was installed, then ROS was built and tested with:

```bash
source /opt/ros/lyrical/setup.bash
export CMAKE_PREFIX_PATH="$verify_root/install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
colcon --log-base "$verify_root/log" build \
  --base-paths upstream/openarm_description ros2_ws/src \
  --packages-select openarm_description openarm_ik_ros \
  --build-base "$verify_root/build" --install-base "$verify_root/install" \
  --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DOPENARM_IK_ROS_COVERAGE=ON
source "$verify_root/install/setup.bash"
colcon --log-base "$verify_root/test_log" test \
  --build-base "$verify_root/build" --install-base "$verify_root/install" \
  --packages-select openarm_ik_ros --event-handlers console_direct+
colcon test-result --test-result-base "$verify_root/build" --verbose
```

Result in `/tmp/openarmik-verify-ros.CmWzkR`: **10 tests, 0 errors, 0 failures,
0 skipped**. The five registered CTest drivers are `test_paired_transaction`,
`test_generated_urdf`, `test_no_can_linkage`, `test_ros_contract`, and
`test_invalid_expiry_parameter`.

Fresh authored-source gcov evidence:

| Authored source | Lines | Branches executed | Calls |
|---|---:|---:|---:|
| `paired_transaction.cpp` | 97.87% of 94 | 100% of 92 | 95.74% of 47 |
| `openarm_ik_ros_node.cpp` | 99.13% of 115 | 91.35% of 370 | 73.13% of 402 |

## URDF/package/mesh and CAN isolation

The project validator resolved every mesh against the pinned description checkout.
`check_urdf model/generated/openarm_v10_bimanual.urdf` parsed successfully. Direct
inventory found 26 unique links, 25 unique joints, 46 mesh references, no duplicate
link/joint names, and both `openarm_left_hand_tcp` and
`openarm_right_hand_tcp`. The pinned description checkout is clean, non-shallow,
and exactly `6c7b720f1ba48e8bafa3a3dc752c45f397b42221`.

`test_no_can_linkage.py` passed against the freshly built node. It checked both
`ldd` and a one-second `strace -f -e trace=socket`; no OpenArm CAN library and no
`AF_CAN`/`PF_CAN` syscall appeared. `readelf -d` likewise showed only ROS/message,
C++, math, and libc dependencies.

All ten source repositories listed by the project were independently checked as
clean, non-shallow, at their exact manifest revisions and expected Enactic GitHub
origins.

## Live hardware-free transaction

Launched without RViz or hardware:

```bash
ros2 launch openarm_ik_ros openarm_ik_rviz.launch.py rviz:=false
/usr/bin/python3 scripts/send_paired_xyz.py \
  0.20 0.30 0.85 0.20 -0.30 0.85 --timeout 5
```

The launch started only `robot_state_publisher` and `openarm_ik_ros_node`. Sequence
1 was acknowledged with `committed=true`, `backend=virtual`,
`collision_checked=false`, and both solver statuses 0. Achieved TCPs were:

- left `(0.1999999805, 0.2999999904, 0.8499999434)`, residual `6.06e-8 m`
- right `(0.1999999805, -0.2999999904, 0.8499999434)`, residual `6.06e-8 m`

The emitted JointState contained all 14 arm joints plus the two driven finger
joints. `tf2_echo` independently reported `(0.200, 0.300, 0.850)` for the left TCP
and `(0.200, -0.300, 0.850)` for the right TCP. Ctrl-C then shut both nodes down
cleanly, and no related process remained.

## Scope boundary

This proves deterministic modeling, virtual paired position IK, ROS/RViz data flow,
CAN codecs/diagnostics, and virtual/CAN isolation on this host. It does **not** prove
motor polarity, zero offsets, IDs, bus timing, watchdog behavior, emergency-stop
behavior, collision-free motion, or physical accuracy. Those require two connected
and commissioned arms, separate CAN buses, an E-stop, and supervised hardware
acceptance testing.
