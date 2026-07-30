# Independent final cache/provenance re-review

## Verdict: CLEAN

Reviewed remediation commit `59590d1` against `669ab88`, with the surrounding
build/launch integration. No Critical or Important issue remains in C1, I1, or
I2. No GUI, browser, network, hardware/CAN operation, or heavyweight build ran.

## C1 — Closed: destructive cleanup is constrained

- `openarm_build_state_remove_owned_tree()` now rejects the repository root,
  every repository descendant (including `.git`, `README.md`, and `ros2_ws`),
  repository ancestors, `/`, `$HOME`, `.git` paths, files, symlinks, special
  targets, non-physical parents, and nonempty unmarked trees before `rm`.
- Direct native callers pass only the exact build/install path plus the expected
  build/install ownership marker. Top-level callers use the separate
  `openarm_build_state_remove_output_child()` API, whose child is restricted to
  `native_build`, `build`, `log`, or `install` under a validated output root.
- Paths are lexically normalized with `realpath -ms`, then required to equal
  their physical parent/target spelling. Existing child/owned targets must be
  real, non-symlink directories; marker paths must be regular non-symlink files.
- A mocked-`rm` probe confirmed that `.git`, `README.md`, `ros2_ws`, the repo
  root/parent, `/`, a file, FIFO, target symlink, symlink-parent target, and
  nonempty unmarked directory caused zero deletion calls.
- Temporary probes confirmed successful cleanup of missing, empty, marker-owned,
  and all four fixed output-child classes.

## I1 — Closed: CMake launcher provenance is bound

- Requested provenance records the raw semicolon-list specification and the
  canonical path/hash of the first executable for all C/C++ compiler and linker
  launcher variables.
- Actual provenance inertly reads the same four cache keys with duplicate
  detection, preserves the full argument list, resolves the launcher executable,
  and hashes its bytes. Missing values are represented explicitly.
- Launch fingerprints include the complete requested build digest and explicit
  launcher records. Newline, carriage return, backslash, generator-expression,
  empty/option-like, whitespace-ambiguous, missing, nonabsolute slash paths, and
  nonexecutable launchers fail closed without shell evaluation.
- Temporary probes confirmed that launcher argument changes and in-place
  executable-byte changes alter requested and actual digests. Invalid launcher
  forms were rejected.

## I2 — Closed: cache components remain physically contained

- The build root must be a real non-symlink directory whose canonical path is
  exactly the supplied path.
- Every fixed native/ROS component name is syntax-restricted; each component
  directory and `CMakeCache.txt` must independently be non-symlinked and
  canonically equal to the expected path beneath that root.
- The sibling-cache scan uses `find -P` from the canonical root.
- Temporary probes rejected an escaping component-directory symlink, an
  escaping cache-file symlink, and a symlinked build root. The same helper is
  used for native and ROS component sets.

## Additional evidence

- `bash -n` passed for all changed shell entry points, helpers, and targeted
  regression scripts.
- The existing dirty `transport/tests/test_transport.cpp` remains unstaged and
  is absent from both remediation commits.
- The review report is the only file edited by this reviewer.
