# Final units/portal fresh re-sweep

Target: `a6c011dc2fd63943fef5d7698d9e45da18cbc0b6` (`main`), reviewed
independently over `31cab6a..HEAD`, with all findings from the prior
`499f9de` sweep rechecked.

## Verdict

- Critical: **none found**
- Important: **none found**
- Actionable Low: **none found**
- Result: **CLEAN**

The ROS inventory-count Important and optional-`FE_*` portability Low are
closed. The earlier directed-rounding overflow and CLI metre-semantics fixes
remain closed. No production file was edited and no build, GUI, browser,
network, ROS, CAN, or hardware action was run. The user edit in
`transport/tests/test_transport.cpp` was preserved. The only write from this
re-sweep is this report update.

## Finding closure

### ROS test inventory: closed

- `ros2_ws/src/openarm_ik_ros/CMakeLists.txt` statically registers exactly 14
  tests: two `ament_add_gtest` entries and twelve ordinary `add_test` entries.
- `scripts/build.sh` now requires 14 tests, so a correct `ctest -N` inventory no
  longer fails the top-level tests-mode transaction.
- The root README now documents the same 14-test invariant.

### Optional floating-point rounding modes: closed

- `model/tests/test_units.c` conditionally includes each `FE_TONEAREST`,
  `FE_TOWARDZERO`, `FE_DOWNWARD`, and `FE_UPWARD` fixture under its own
  preprocessor guard, and the upward-specific proof is guarded as well.
- The fixture exercises every ISO-C rounding mode exposed by the implementation
  without naming unsupported optional macros. Its at-least-one-mode assertion
  is explicit and consistent with the rounding-mode regression's purpose.

### Directed-rounding overflow and CLI units: remain closed

- Expanding vector conversions conservatively reject components at or beyond
  the `DBL_MAX / |factor|` boundary before any output write. The post-product
  finite check, transactional error behavior, exact aliasing, strict C11
  implementation, and documented contract remain intact.
- The CLI handles exact `--help` before ROS initialization, identifies all six
  paired XYZ operands as metres, and its CTest checks that help contract.

## Rechecked areas remaining clean

- The fixed-width unit/status API and binary64 vector ABI remain intact. All
  public XYZ ingress remains `double`, with no new float field or narrowing
  conversion, and Model/Control/Runtime/portal boundaries still convert once
  while retaining canonical metre tolerances and reports.
- Runtime V1 current and frozen Runtime headers are byte-identical, as are the
  current and frozen Commission headers. The 50-symbol ABI manifest and the
  header-only Runtime units adapter are unchanged.
- Portal centimetre default, inch toggle, touched/dirty-state preservation,
  strict parsing, selected-unit v2 request, server-side single conversion,
  metre-only v1/state contracts, precision-preserving JSON, canonical samples,
  and virtual-only safety wording are unaffected and remain clean.
- Run/build launch integrity, shared/exclusive leases, source/toolchain/cache
  fingerprints, atomic completion stamps, installed-manifest validation, and
  live Runtime/session authority remain fail-closed. The changed source, test,
  and documentation bytes are covered by the existing source fingerprints.
- The exact detached description commit/tree/origin and clean byte-level
  worktree policy still validate offline. ROS production motion remains routed
  through installed `OpenArm::Runtime`; no Control bypass, transport, CAN,
  socket, physical backend, commission, or activation path changed.

## Lightweight diagnostics

- `git diff --check 31cab6a..HEAD` passed.
- `bash -n` passed for the relevant build/run/cache/lock/pin scripts.
- Offline `openarm_validate_description_pin upstream/openarm_description`
  passed.
- Static CMake enumeration produced 2 gtests plus 12 ordinary tests, matching
  the build gate and README value of 14.
- Runtime current/frozen headers share SHA-256
  `e4b2ed9c3e57bd353e805a09270478341681e7aebbe96062e4bb1e39725fd9e5`;
  Commission current/frozen headers share
  `40a7289b7245de27a492b78d07473d6acd48f0e93c37727de3418cde99d9423e`.
