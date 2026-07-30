# Final independent review: interactive WebGL viewer build/package

Reviewed final commit `bd06a301095243325bbe4cf6fc3e94cdf702653f` in
`build/worktrees/interactive-webgl-viewer`, including the complete series from parent
`658174aece8aa09096da1d05d32d5cc74e9e8abf`.

## Verdict

**CLEAN.** No remaining build, package, provenance, resource, inventory, ABI, or
launcher-lifecycle finding in the reviewed scope.

## Final narrow verification

- CMake registers exactly **15** `openarm_ik_ros` CTests.
- `scripts/build.sh` now fails closed unless the exact registered count is 15 and its
  diagnostic also says 15.
- The root README documents exactly 15 freshly registered ROS tests. No stale
  test-inventory assertion for 14 remains; the remaining source-level value 14 is the
  robot's motor count and is unrelated.
- `tests/test_build_resource_controls.sh` passed sequentially with repository-local
  temporary output, including the real one-job CMake/CTest fixture, lock contention,
  lock-FD noninheritance, signal cleanup, clean install recreation, failed-transaction
  stamp removal, and incremental reuse checks.
- `bd06a30` changes only the README and exact count/diagnostic in `scripts/build.sh`;
  it does not change Runtime, ROS package dependencies, either launcher, or portal
  behavior.

## Retained license/provenance closure

- The bundled `openarm_description-LICENSE.txt` remains byte-identical to the pinned
  upstream Apache-2.0 license: 11,357 bytes, SHA-256
  `c71d239df91726fc519c6eb72d318ec65820627232b2f796219e87dcf35d0ab4`.
- `license_source` is in `CMAKE_CONFIGURE_DEPENDS`. The cached same-size mutation
  regression previously passed: incremental build forces reconfiguration, rejects the
  changed pinned hash, and emits no asset stamp. The production build removes the old
  launch stamp before the transaction and cannot publish a new one after this failure.
- The clean install contains the exact license beside the manifest and mesh directory.
  The manifest records its package-relative path and hash, the generated portal asset
  table verifies the same bytes, `viewer_license` is an explicit launch artifact, and
  the full install-tree digest independently closes it. The upstream has no NOTICE
  file to redistribute.

## Retained regression evidence

- All 11 viewer STL hashes, sizes, binary triangle counts, totals, routes, URDF
  references, upstream commit/tree/origin, and installed paths matched their pins.
- The Firefox/geckodriver browser gate passed, including WebGL fidelity, resident-byte
  serving, corrupted-startup SHA-256 rejection, fragmented/slow intake, and prompt API
  stop availability. Dependency verification passed without adding a Node/Java
  runtime.
- Launch-integrity and clean-install checks passed. The web launcher starts no RViz;
  standalone RViz and its X11 close helper remain. Portal health and SIGINT shutdown
  passed with no child/RViz process or JPEG/XComposite portal linkage.
- Runtime sources and Runtime ABI are unchanged. Source/install freshness, sequential
  ROS builds, global job limits, and build/install/GUI locks remain intact.

## Final commands/checks

- `ctest --test-dir build/interactive-check -N` — `Total Tests: 15`.
- Exact inventory search across README, scripts, tests, and CMake registration.
- `TMPDIR="$PWD/build/tmp" bash tests/test_build_resource_controls.sh` — passed.
- Shell syntax, final diff-scope, Runtime/package/launcher no-change, bundled/upstream
  license `cmp`/SHA-256, manifest/integrity entry, install rule, and launch-artifact
  checks — passed.

