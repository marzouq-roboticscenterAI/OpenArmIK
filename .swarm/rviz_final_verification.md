# RViz final verification

Date: 2026-07-29 (America/Los_Angeles)  
Branch: `main`  
HEAD: `5313006f9481dd1b6ee5a473ec597b81bfc69db0`

## Verdict

**CLEAN for the hardware-free, non-GUI verification scope.** Fresh canonical
model Release and ASan/UBSan suites pass 4/4, including deterministic xacro
regeneration. The current launcher passes Bash syntax and its `rviz:=` parser
passes accepted-value, last-wins, invalid-value, and argument-stripping checks.
The previously completed CAN and fresh package-scoped ROS suites remain green.
No product file was edited by the verifier.

No GUI or hardware path was launched. Final process inspection found no RViz,
ROS launch, `robot_state_publisher`, or `openarm_ik_ros_node` process.

## Current static checks

Commands:

```bash
git diff --check
bash -n scripts/launch_rviz.sh
/usr/bin/python3 -c 'from pathlib import Path; p=Path("scripts/close_rviz_window.py"); compile(p.read_bytes(), str(p), "exec")'
find model/tools scripts -type d -name __pycache__ -print
```

Results: all checks passed. The Python helper was compiled in memory, and the
final cache inventory was empty. The untracked helper was also checked with
`git diff --no-index --check /dev/null scripts/close_rviz_window.py`; its exit 1
means content differs from `/dev/null`, while empty diagnostic output confirms
no whitespace errors. Its executable mode is `775`.

## Canonical system-xacro model verification

Release commands:

```bash
verify_model_system_release=$(mktemp -d /tmp/openarmik-rviz-final-model-system-release.XXXXXX)
cmake -S model -B "$verify_model_system_release" -DCMAKE_BUILD_TYPE=Release \
  -DOA_MODEL_BUILD_TESTS=ON -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOA_DESCRIPTION_ROOT=/home/signalprocessing-dev/OpenArmIK/upstream/openarm_description \
  -DOA_XACRO_EXECUTABLE=/opt/ros/lyrical/bin/xacro \
  -DOA_XACRO_PYTHONPATH=/opt/ros/lyrical/lib/python3.14/site-packages \
  -DOA_XACRO_AMENT_PREFIX=/opt/ros/lyrical
cmake --build "$verify_model_system_release" --parallel
ctest --test-dir "$verify_model_system_release" --output-on-failure
```

Result in `/tmp/openarmik-rviz-final-model-system-release.Ahn3fx`: build passed;
4/4 tests passed. This includes 3,200 randomized bounded IK cases, the ABI-v1
canary, 600 Python reference FK/Jacobian cases, and byte-for-byte deterministic
model/URDF regeneration.

Sanitizer commands:

```bash
verify_model_system_sanitize=$(mktemp -d /tmp/openarmik-rviz-final-model-system-sanitize.XXXXXX)
cmake -S model -B "$verify_model_system_sanitize" -DCMAKE_BUILD_TYPE=Debug \
  -DOA_MODEL_BUILD_TESTS=ON -DOA_MODEL_SANITIZERS=ON \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DOA_DESCRIPTION_ROOT=/home/signalprocessing-dev/OpenArmIK/upstream/openarm_description \
  -DOA_XACRO_EXECUTABLE=/opt/ros/lyrical/bin/xacro \
  -DOA_XACRO_PYTHONPATH=/opt/ros/lyrical/lib/python3.14/site-packages \
  -DOA_XACRO_AMENT_PREFIX=/opt/ros/lyrical
cmake --build "$verify_model_system_sanitize" --parallel
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 \
  ctest --test-dir "$verify_model_system_sanitize" --output-on-failure
```

Result in `/tmp/openarmik-rviz-final-model-system-sanitize.6gMne0`: build passed;
4/4 tests passed with no ASan/UBSan finding, including deterministic regeneration.

The earlier two-entry `.deps` plus system Python path was a pre-system-install
workaround. After canonical xacro was installed system-wide, that invocation
made `tool_identity()` intentionally hash two xacro copies and therefore produced
a provenance mismatch. The canonical system-only invocation above returns the
recorded `a22ff229...` identity and is the applicable final result.

## Launcher `rviz:=` parser verification

The parser block was extracted directly from the current launcher, from
`rviz_enabled=1` through the line immediately before its launch branch, and
evaluated in isolated subshells. Each result was asserted for both final state
and the exact forwarded `launch_arguments` array:

```text
CASE true         status=0 enabled=1 args=<alpha:=one>
CASE false        status=0 enabled=0 args=<alpha:=one>
CASE one          status=0 enabled=1 args=<alpha:=one>
CASE zero         status=0 enabled=0 args=<alpha:=one>
CASE last_true    status=0 enabled=1 args=<alpha:=one><beta:=two>
CASE last_false   status=0 enabled=0 args=<alpha:=one><beta:=two>
CASE last_one     status=0 enabled=1 args=<alpha:=one><beta:=two>
CASE last_zero    status=0 enabled=0 args=<alpha:=one><beta:=two>
CASE invalid      status=2 output=<rviz must be true, false, 1, or 0 (received yes)>
```

Inputs for the four precedence cases were respectively `false ... true`,
`true ... false`, `0 ... 1`, and `1 ... 0`. Thus the last `rviz:=` occurrence
wins for every accepted spelling. Every `rviz:=` token is absent from the
forwarded array while unrelated arguments preserve order.

The complete launcher was safely invoked with the invalid value only:

```bash
OPENARM_RVIZ_RENDERER=software \
  XDG_RUNTIME_DIR=/tmp/openarmik-rviz-parser-stubs/runtime \
  bash scripts/launch_rviz.sh rviz:=yes alpha:=one
```

Result: exit 2 with the expected rejection text, before a ROS or RViz launch.
Static inspection of both forwarding sites confirms they emit exactly one
internal `rviz:=false` followed by only `${launch_arguments[@]}`; this prevents
the stripped user arguments from overriding the internal launch mode.

## Previously completed C CAN verification

Release:

```bash
verify_can_release=$(mktemp -d /tmp/openarmik-rviz-final-can-release.XXXXXX)
cmake -S can -B "$verify_can_release" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build "$verify_can_release" --parallel
ctest --test-dir "$verify_can_release" --output-on-failure
```

Result: 1/1 passed in `/tmp/openarmik-rviz-final-can-release.FhXjSe`.

ASan/UBSan:

```bash
verify_can_sanitize=$(mktemp -d /tmp/openarmik-rviz-final-can-sanitize.XXXXXX)
cmake -S can -B "$verify_can_sanitize" -DCMAKE_BUILD_TYPE=Debug \
  -DBUILD_TESTING=ON -DOA_CAN_ENABLE_SANITIZERS=ON
cmake --build "$verify_can_sanitize" --parallel
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 \
  ctest --test-dir "$verify_can_sanitize" --output-on-failure
```

Result: 1/1 passed in `/tmp/openarmik-rviz-final-can-sanitize.uUPQEx`; no
sanitizer finding.

## Previously completed fresh package-scoped ROS build and test

A fresh model prefix and colcon build/install tree were created under
`/tmp/openarmik-rviz-final-ros.gctC8T`. Commands:

```bash
cmake -S model -B "$verify_ros/model-build" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DOA_MODEL_BUILD_TESTS=OFF \
  -DCMAKE_INSTALL_PREFIX="$verify_ros/install"
cmake --build "$verify_ros/model-build" --parallel
cmake --install "$verify_ros/model-build"
source /opt/ros/lyrical/setup.bash
export CMAKE_PREFIX_PATH="$verify_ros/install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
colcon --log-base "$verify_ros/log" build \
  --base-paths upstream/openarm_description ros2_ws/src \
  --packages-select openarm_description openarm_ik_ros \
  --build-base "$verify_ros/build" --install-base "$verify_ros/install" \
  --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DOPENARM_IK_ROS_COVERAGE=OFF
source "$verify_ros/install/setup.bash"
colcon --log-base "$verify_ros/test-log" test \
  --base-paths ros2_ws/src --packages-select openarm_ik_ros \
  --build-base "$verify_ros/build" --install-base "$verify_ros/install" \
  --event-handlers console_direct+
colcon test-result --test-result-base "$verify_ros/build/openarm_ik_ros" --verbose
```

Result: both selected packages built; all 5 CTest drivers / 10 authored test
cases passed: **10 tests, 0 errors, 0 failures, 0 skipped**.

## Harness notes and final residue

An initial attempt to stub full launcher commands showed that Bash aliases do
not intercept the `exec ros2` false branch. Two verifier-owned, non-GUI
`rviz:=false` ROS core trees briefly started with the test argument
`alpha:=one`. Both were stopped cleanly with SIGINT, their harness parent was
terminated, and no RViz process was ever started. This was a test-harness issue,
not a product failure; the parser matrix was then tested by direct source-block
extraction as documented above.

Final residue command:

```bash
ps -eo pid=,ppid=,pgid=,sid=,stat=,comm=,args= | \
  grep -E '([/]rviz2|openarm_ik_ros_node|robot_state_publisher|ros2 launch openarm_ik_ros)' | \
  grep -v grep || true
```

Result: empty. Final `git diff --check` also passed. Product status remained:

```text
 M README.md
 M ros2_ws/src/openarm_ik_ros/rviz/openarm_ik.rviz
 M scripts/launch_rviz.sh
?? scripts/close_rviz_window.py
```

Other `.swarm/*.md` files are coordination/evidence artifacts, not product
changes.
