# Runtime V1 ABI hardening final independent review

## Verdict

**CLEAN — no Critical or Important findings.**

Reviewed `1ece782` atop `6484e24`, limited to the Runtime ABI/release contract.
The final package-prefix finding and both earlier ABI findings are closed. No
production file was modified by this review.

## Verified closure

### Supported CMake and split-prefix package behavior

- `openarm_runtimeConfig.cmake.in` now calls
  `set_and_check(openarm_runtime_V1_INCLUDE_DIR, ...)` immediately after
  `@PACKAGE_INIT@`, before `CMakeFindDependencyMacro` is included and before
  every `find_dependency()` call. The path is expanded and validated while
  `PACKAGE_PREFIX_DIR` still names Runtime's own prefix. Nested package configs
  may overwrite `PACKAGE_PREFIX_DIR` under CMake 3.16-3.29 without changing the
  already captured absolute Runtime V1 include path.
- The installed regression reads the generated config and requires the include
  capture to precede the first dependency lookup. In the fresh output the
  capture begins at byte 889 and the first `find_dependency` at byte 1028.
- The installed test deliberately orders the dependency prefix before the
  Runtime prefix in `CMAKE_PREFIX_PATH`, supplies Runtime's config directory
  explicitly, and compares the real exported include path to the exact Runtime
  install prefix. The configure/build/run test passed.
- Compiler dependency files confirm that frozen installed C11/C++17 consumers
  selected Runtime's versioned frozen `openarm_runtime.h` and
  `openarm_commission.h`. Current installed consumers selected Runtime's normal
  header and the dependency prefix's current Commission header. No duplicate
  or stale header was selected.
- Runtime build and installed configs still require
  `openarm_commission 0.1.0 EXACT`. The installed negative test requests 0.1.1,
  requires configuration failure, and requires the diagnostic to identify
  installed 0.1.0. The test passed.

### Frozen transitive ABI and full symbol contract

- Runtime current/frozen headers remain byte-identical at SHA-256
  `e4b2ed9c3e57bd353e805a09270478341681e7aebbe96062e4bb1e39725fd9e5`.
  Commission current/frozen headers remain byte-identical at SHA-256
  `40a7289b7245de27a492b78d07473d6acd48f0e93c37727de3418cde99d9423e`.
  Literal hash and complete-content checks cover both pairs.
- All eight Commission record definitions are frozen. Every record has numeric
  size, 8-byte alignment, and meaningful offset assertions; these cover all six
  Commission record types directly exposed by Runtime plus nested/supporting
  public records.
- The frozen Runtime header declarations, `expected_symbols.txt`, and retained
  typed function references remain the same exact 50-name set. Fresh C11/C++17
  current/frozen build-tree and installed binaries each link all 50 Runtime
  definitions; both production archives pass the exact 50-symbol manifest.
- The retained reference implementation remains protected with compiler
  `used`, GNU `retain`, and its dedicated section. The previous independent
  inspection confirmed a 400-byte retained section (50 pointer slots) in all
  eight linked canaries, so optimization/section garbage collection cannot
  erase the link contract on the tested supported GNU toolchain.
- Manifest creation/summary behavior, the authority-based
  `oa_runtime_manifest_save` signature and call convention, and short-output
  `OA_RUNTIME_EABI` semantics remain exercised by both current and frozen
  canaries. No Runtime implementation changed in this final fix.
- The prior fresh `BUILD_TESTING=OFF` sequential build/install remains
  applicable: this final change only captures the installed config variable
  earlier and strengthens test-only checks. Production target construction and
  install contents are otherwise unchanged.
- The documented V2 rule remains truthful: breaking Runtime or transitive
  Commission record changes require V2 names/versioning or an explicit V1
  compatibility adapter.

## Release-baseline reconciliation

Observed local pre-`987f512` installed headers and ordinary archives already use
the current authority-based save signature; the earlier header preprocesses
identically to the frozen current API. Those artifacts do not evidence release
of the obsolete initial persistence signature. Absence of an external release
remains a release-owner attestation because this checkout has no tags or remote
refs, but no local ABI artifact contradicts it.

## Fresh verification

All work was sequential (`-j1`) and used no GUI, ROS session, browser, network,
CAN, or physical interface.

- Fresh Release Runtime configure/build with `BUILD_TESTING=ON`: PASS.
- Seven targeted `^openarm_runtime_v1_` tests: **7/7 PASS**.
  This includes frozen/current C11 and C++17 consumers, dual-header hash/layout
  policy, exact archive symbol manifest, split-prefix four-consumer install,
  static config-order validation, exact include-prefix validation, and the
  Commission version-mismatch negative test.
- Independent hash, include dependency, and 50-name set comparisons: PASS.
- `git diff --check` for this review file: PASS.
