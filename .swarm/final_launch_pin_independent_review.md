# Final launcher-cache independent re-review

## Verdict

**CLEAN — no Critical or Important findings.**

Reviewed `59590d19c0132a9ba760cc834601af346527490a` against its parent
`669ab88f1b8b5afecff86c62349e31e899ec9e55`, with targeted emphasis on the
previous CMake compiler/linker launcher provenance finding. The finding is
closed. No code or user-owned file was modified by this review.

## Launcher provenance closure

- Requested build state now binds the complete raw values of
  `CMAKE_C_COMPILER_LAUNCHER`, `CMAKE_CXX_COMPILER_LAUNCHER`,
  `CMAKE_C_LINKER_LAUNCHER`, and `CMAKE_CXX_LINKER_LAUNCHER`, plus the canonical
  path and SHA-256 bytes of each selected argv0 executable.
- Effective state reads the same four keys from every native and ROS
  `CMakeCache.txt`. It binds key presence, exact raw cache value, canonical
  executable, and executable bytes. Duplicate/malformed entries and missing or
  changed selected executables fail closed.
- Launcher values are parsed according to CMake's semicolon-list shape without
  shell evaluation. The complete raw list binds fixed arguments; only the first
  element is resolved as argv0. Empty, option-like, whitespace-ambiguous,
  relative path, newline/CR, backslash, and generator-expression ambiguity is
  rejected. Resolution uses quoted `command -v`, `realpath`, and `sha256sum`.
- The launch source fingerprint incorporates the complete requested-state
  digest and independently records the four launcher selections. Raw argument
  changes and in-place launcher byte changes therefore invalidate both build
  reuse and no-build launch freshness.
- CMake-version handling is compatible with the project's supported floor.
  Compiler launchers are supported from CMake 3.4. Linker launchers, introduced
  in 3.21, are optional effective-cache keys: their absence is represented in
  the actual digest rather than treated as malformed on pre-3.21 CMake. The
  real effective-linker regression is conditionally exercised only on CMake
  3.21 or newer, while requested/fingerprint binding remains deterministic on
  every version.

The real-CMake regression proves the full transition: a semicolon launcher list
is selected in both C and C++ caches; an identical request reuses the tree; a
different raw launcher/argument list recreates it and selects the new cache
value; and mutation of launcher bytes at the same path recreates it. Compiler
and linker launcher cases pass on the local CMake 4.2 installation.

## Related hardening re-check

- Native/ROS component directories and cache files must be physical,
  non-symlinked children of the canonical cache root. Native- and ROS-shaped
  directory/cache escape regressions pass.
- Destructive cleanup now uses positive ownership and exact-child policies.
  Direct native cleanup refuses repository/VCS paths and nonempty unowned
  trees; top-level cleanup accepts only `native_build`, `build`, `log`, and
  `install` under a validated output root. The mocked-`rm` regression proves
  rejected repository/root targets never reach deletion.
- The resource regression's `/tmp` branch does not weaken production. No
  production lock helper changed. The test now covers both valid environments:
  a root-owned sticky `/tmp` must be selected as fallback, while an unavailable
  or insecure `/tmp` must produce status 2 and the exact refusal message.

## Fresh sequential verification

All test artifacts and `TMPDIR` were under
`/home/signalprocessing-dev/OpenArmIK/build/cache-final-rereview`. No GUI,
browser, network, ROS session, hardware, physical interface, CAN operation, or
full project build was run.

- Targeted `bash -n`: **PASS**.
- `tests/test_build_cache_state.sh`: **PASS** — `Build cache-state transaction
  regression passed`.
- `tests/test_launch_integrity.sh`: **PASS** — `Launch freshness and authority
  regression passed`.
- `OPENARM_BUILD_JOBS=1 tests/test_build_resource_controls.sh`: **PASS** — its
  real fixture passed CTest 1/1 and the full script ended with `Build
  resource-control regression passed (supervisor, pinned fixture, real
  CMake/CTest)`.
- `git diff --check`, `git diff --cached --check`, and
  `git diff --check 669ab88..59590d1`: **PASS**.

The pre-existing modification to `transport/tests/test_transport.cpp`,
untracked reports, and the untracked default launch stamp were preserved.

