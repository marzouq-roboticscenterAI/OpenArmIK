# Final targeted cache verification — 59590d1

Date: 2026-07-30 (America/Los_Angeles)

## Verdict

**CLEAN** for the requested targeted scope. Every required check passed fresh at
`59590d19c0132a9ba760cc834601af346527490a`, sequentially and without a full
top-level rebuild, GUI/browser launch, network access, or hardware/CAN access.

## Baseline and preservation

- `git rev-parse HEAD`: `59590d19c0132a9ba760cc834601af346527490a`
  on `main`.
- Baseline `git status --short --branch` contained the user's existing
  `M transport/tests/test_transport.cpp`, existing untracked `.swarm` reports,
  and `ros2_ws/.openarm-launch-stamp-v2`. The transport modification remained
  present after verification. No source file was changed by this run; this
  report is the only file rewritten.
- `git diff --check` and `git diff --cached --check`: PASS before testing.
- A separate verifier's resource-control test was detected at baseline. No test
  was started until that process and its children exited, preserving strict
  sequential/no-overlap execution for this run.

## Fresh sequential results

1. `bash -n` over all seven shell files changed by `HEAD^..HEAD`: PASS.
   Covered `scripts/build.sh`, `scripts/build_native.sh`,
   `scripts/lib/build_cache_state.sh`, `scripts/lib/launch_integrity.sh`,
   `tests/test_build_cache_state.sh`, `tests/test_build_resource_controls.sh`,
   and `tests/test_launch_integrity.sh`.

2. `tests/test_build_cache_state.sh`: exit 0,
   `Build cache-state transaction regression passed`.

   This fresh execution explicitly covered:

   - mocked destructive cleanup rejection for repository `.git`, `README.md`,
     `ros2_ws`, repository/workspace ancestors, and `/`, asserting the mocked
     `rm` call count stayed zero;
   - successful deletion only for a disposable, correctly marked owned tree;
   - compiler-launcher environment/list selection, actual cache values, reuse,
     changed launcher path/arguments, and in-place executable-byte mutation;
   - linker-launcher environment/list selection, cache values, and mutation;
   - native-shaped component-directory and cache-file symlink escapes; and
   - ROS-shaped `openarm_ik_ros` component-directory symlink escape.

3. `tests/test_description_pin.sh`: exit 0,
   `Description pin regression passed`.

4. `tests/test_launch_integrity.sh`: exit 0,
   `Launch freshness and authority regression passed`.

   Its expected negative-path diagnostic `Installed launch closure changed
   after validation` appeared before final success. The test also freshly
   verified compiler-launcher environment arguments/bytes, linker-launcher
   arguments, ambiguous launcher rejection, and the incremental no-build/build
   authority paths.

5. `OPENARM_BUILD_JOBS=1 tests/test_build_resource_controls.sh`: exit 0,
   `Build resource-control regression passed (supervisor, pinned fixture, real
   CMake/CTest)`. The real fixture reported 1/1 CTest tests passed. It ran alone
   through a persistent terminal session, so the final exit status was captured
   unambiguously.

## Default workspace assertions without launch

After sourcing the real description, build-lock, cache-state, and launch
integrity helpers:

- Native completed record validation for
  `can model commission transport control runtime`: PASS.
  Request/actual digests:
  `904307f652e8e37a4950c46f3df6bea8471a6280b09db66bc9c33fd90e0a0d94`
  / `ed4aa425a4d08cb248659d0d0bb66e7e09004a217b3c4ef9b3dbb328680af5ad`.
- ROS completed record validation for
  `openarm_control_msgs openarm_description openarm_ik_ros`: PASS.
  Request/actual digests:
  `38477805d142dcbefff1636b86f8dfd1af73227d40d30716920d93369a75f459`
  / `1cd4779b1da9a25b32c4f08c9ee288ff6b872270049ff1e2e655723b38f71cfa`.
- `openarm_assert_current_launch_tree "$PWD" "$PWD/ros2_ws" Release`:
  PASS. The default `OPENARM_LAUNCH_STAMP_V2` therefore matched current source,
  provenance, completed CMake records, install closure, authority checks, and
  artifact hashes.
- `openarm_ensure_current_launch_tree "$PWD" "$PWD/ros2_ws" never`: PASS.
  This exercised the actual no-build gate without starting ROS, OpenArm, RViz,
  the portal, or a browser.
- After sourcing `ros2_ws/install/setup.bash`, exact command
  `ros2 pkg prefix openarm_ik_ros` returned exactly
  `/home/signalprocessing-dev/OpenArmIK/ros2_ws/install/openarm_ik_ros`: PASS.

## Safety and limitations

- Memory was sampled before, between, and after phases. Every observed
  `MemAvailable` sample was at least 8,295,760 KiB; the post-assertion sample was
  8,416,072 KiB. No unsafe-memory condition was approached.
- No full top-level build or unrelated suite was run, as requested.
- Final process checks found no leftover test, OpenArm node, portal, close
  helper, or RViz process.
- Firefox parent PID 12561 was pre-existing. This run did not open, close, or
  otherwise touch Firefox.
- Final `git diff --check` and `git diff --cached --check`: PASS. Final
  `MemAvailable` was 8,541,012 KiB.
