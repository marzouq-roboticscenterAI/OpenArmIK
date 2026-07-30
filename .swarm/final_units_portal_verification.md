# Final units and portal verification

Target: `a6c011dc2fd63943fef5d7698d9e45da18cbc0b6` (`main`)

## Verdict

**CLEAN — no Critical, Important, or actionable Low findings.**

All verification was hardware-free and sequential. No GUI, browser, network,
ROS launch, CAN interface, physical backend, or transmit path was used. Fresh
review outputs and `TMPDIR` are under
`build/final-units-independent`. The user's modified
`transport/tests/test_transport.cpp` and unrelated modified/untracked files
were preserved.

## Fresh results

- Strict C11 units compile/run with `-pedantic-errors -Wall -Wextra -Werror
  -Wconversion -Wdouble-promotion -Wfloat-conversion -frounding-math`: **PASS**,
  exact output `OpenArm binary64 unit conversion tests passed`.
- Model: **5/5 passed** at current HEAD.
- Control: **4/4 passed**, including the C11 ABI, frozen original-V1 ABI, and
  installed public-header C11/C++17 consumers.
- Runtime: **12/12 passed**, including current/frozen C11/C++17 V1 consumers,
  the header-only units adapter, header policy, symbol manifest, and installed
  split-prefix consumers. Six installed Runtime V1/unit consumer executables
  also passed explicitly.
- Runtime ABI: archive symbols exactly matched the frozen manifest,
  `archive_symbol_count=50`, `expected_symbol_count=50`. Current, frozen,
  installed-current, and installed-frozen Runtime V1 headers all had SHA-256
  `e4b2ed9c3e57bd353e805a09270478341681e7aebbe96062e4bb1e39725fd9e5`.
- Focused portal: **24/24 passed in exactly 12 suites** from a fresh targeted
  one-job build linked to the current Model archive.
- CLI help: exit 0 and exact paired-target operands
  `LEFT_X_METRES LEFT_Y_METRES LEFT_Z_METRES RIGHT_X_METRES RIGHT_Y_METRES RIGHT_Z_METRES`.
- Current ROS inventory: `ctest -N` reported exactly **14** registered tests,
  including `test_openarm_control_cli_help`. The top-level build gate and root
  README both now require 14.
- Definitive repository-root verification at this same commit:
  `scripts/build.sh --tests --incremental --jobs 1` exited 0, then the sourced
  installed-overlay ROS CTest passed **14/14 in 96.94 s**. The former 13-test
  evidence was valid before the new CLI help regression was added; the current
  contract and result are 14/14. The following production one-job incremental
  build also exited 0. Its per-process ROS logs remain under
  `build/final-verification/ros-log`; the production rebuild superseded the
  tests-mode CMake result tree.
- Default launch gate: both
  `openarm_assert_current_launch_tree "$PWD" "$PWD/ros2_ws" Release` and the
  real `never`/no-build ensure path passed after the production refresh. The
  stamp records `build_type Release`, `run_tests 0`, fingerprint
  `c095f74b25f4a76630138307a4169539711139463a34ab937c8f6ec60e848387`,
  and build-state digest
  `69bfc44832264ea9fabfbd641221828d9ab93a0b2b0d5cc6f6864a1d41ecd03f`.
- Description-pin, build-cache-state, launch-integrity, and build-resource
  regressions passed. The resource suite included its real CMake/CTest fixture
  (**1/1 passed**) and ended with
  `Build resource-control regression passed (supervisor, pinned fixture, real CMake/CTest)`.
- `git diff --check`, `git diff --cached --check`, and
  `git diff --check 31cab6a..HEAD` passed.

## Contract review

The public native XYZ ingress inventory is complete and additive:

- Model retains metre-native `oa_ik_position`/`oa_ik_position_v2` and adds
  `oa_ik_position_with_units`.
- Control retains metre-native `oa_controller_plan_paired_tcp` and adds
  `oa_controller_plan_paired_tcp_with_units`.
- Runtime V1 retains `oa_runtime_plan_paired_tcp_body`; the separately installed
  `openarm_runtime_units.h` provides the header-only
  `oa_runtime_plan_paired_tcp_body_with_units` adapter without changing the
  frozen 50-symbol ABI.

All unit-aware entries accept metres, centimetres, and inches through the same
fixed-width unit IDs and `oa_vec3d`. Coordinate fields, factors, temporaries,
Model/Control calculations, Runtime adapter values, ROS action fields, portal
state, and JSON normalization remain binary64 `double`; no coordinate-path
`float` storage, cast, or narrowing was found. Tolerances and reports remain
metres/SI.

The conversion helper validates first, converts into a temporary, and commits
once, so every error and exact aliasing preserve the output contract. Expanding
conversions now conservatively check each magnitude at or above
`DBL_MAX / |factor|` before multiplication. The independent probe verified both
`DBL_MAX` and the negative quotient boundary under every available
`FE_TONEAREST`, `FE_TOWARDZERO`, `FE_DOWNWARD`, and `FE_UPWARD` mode: each
returned `OA_UNITS_EOVERFLOW` and left the sentinel unchanged. The post-product
finite check remains as defense in depth.

The portal defaults to centimetres, toggles to inches, preserves canonical
metre targets, and sends the selected `unit` plus selected-unit-scaled numeric
XYZ values to `/api/v2/move`. Submission does not parse rounded DOM display
text. The strict server parser accepts only explicit `m`/`cm`/`in`, stores XYZ
as `double`, and calls `oa_vec3d_convert(..., OA_LENGTH_UNIT_METRES)` exactly
once before the unchanged guard/action path. State declares
`"coordinate_unit":"m"` and serializes with `max_digits10`.

Canonical sample values remain exactly:

- left small `(0.019973, 0.143469, 0.096000)` m;
- left medium `(0.029973, 0.143469, 0.106000)` m;
- right small `(0.020081, -0.143527, 0.096000)` m;
- right medium `(0.030081, -0.143527, 0.106000)` m.

The page still says virtual simulation only, collision checked **NO**, sampled
nominal guard rather than physical collision certification, presets fill fields
without submission, ROS/RViz remain metric with no portal-switchable grid,
calibration is nonmoving simulation verification, and the software stop is not
safety-rated or a replacement for a hardwired E-stop.

## Retained evidence

Key logs are in `build/final-units-independent/logs`, notably:

- `model-ctest-a6c011d.log`, `strict-units-c11-run-a6c011d.log`, and
  `rounding-overflow-run-499f9de.log`;
- `control-ctest.log`, `runtime-ctest.log`, `runtime-abi-explicit.log`, and
  `runtime-installed-consumers-explicit.log`;
- `portal-gtest-499f9de.log`, `cli-help-499f9de.log`, and
  `ros-ctest-inventory-a6c011d.log`;
- `description-pin-a6c011d.log`, `build-cache-state-a6c011d.log`,
  `build-resource-controls-a6c011d.log`, and
  `current-launch-tree-a6c011d.log`.
