# Build-lock supervisor design adjudication

Date: 2026-07-29
Baseline: `fix/bounded-build-memory-resume` at `60b952d`, including merge
`cd4aa67` from `main`
Scope: design only; no production files or builds were changed

## Verdict

The two Important findings in `oom_build_independent_review.md` are valid.
The current `exec` protocol cannot be repaired merely by validating
`OPENARM_BUILD_LOCK_FDS`: the validated descriptors refer to the same open-file
descriptions in the body and all of its descendants, so a recursive
`--locked-body` invocation can reacquire those locks.  The production
`OPENARM_DESCRIPTION_DIR` escape hatch is likewise unjustified and must be
removed.

The smallest sound architecture is:

1. keep each public script as the sole argument parser and lock entry point;
2. put only the native mutating implementation in a source-only function;
3. acquire canonical locks in the public script's parent Bash process;
4. fork a private callback subshell, close every inherited lock FD there before
   calling any mutating function, and have the parent wait while retaining its
   copies; and
5. have the top-level build call the sourced native function directly while its
   supervisor owns the top, native-build, and install locks.

There should be no public `--locked-body` mode, no lock record in the
environment, and no executable internal runner.

## Exact file layout

### `scripts/build_lock.sh`

Keep this as a source-only helper (moving it to `scripts/lib/build_lock.sh` is
cosmetic, not required).  Replace `openarm_run_with_locks` and delete
`openarm_validate_locks`.  The replacement API is:

```text
openarm_run_with_locks RESOURCE... -- CALLBACK ARG...
```

`CALLBACK` is a function already defined in the current Bash process.  This API
is internal and is called as the final, bare simple command of each public
script.

The function must:

- parse resources and callback into local indexed arrays;
- reject a missing resource, separator, or callback with status 2;
- derive the lock files from the already-canonical resource paths;
- de-duplicate resources and acquire every lock nonblocking in the canonical
  caller order (`top`, `native build`, `install` for `build.sh`; `native build`,
  `install` for `build_native.sh`);
- use `exec {lock_fd}>"$lock_file"` and `flock -n -E 75 "$lock_fd"`;
- on contention, close all descriptors already opened, print the conflicting
  resource, and return the public contention status 3;
- retain the successful FD numbers only in a local Bash array, never in an
  exported variable or command-line record;
- fork a private subshell/process group; and
- in that child, before the callback, run

  ```bash
  for lock_fd in "${lock_fds[@]}"; do
    exec {lock_fd}>&-
  done
  unset lock_fd lock_fds
  set -euo pipefail
  "$@"
  ```

`exec {lock_fd}>&-` is Bash's variable-FD close form: it closes the descriptor
whose number is held in `lock_fd` and unsets that variable.  It needs no `eval`,
numeric interpolation, delimiter parsing, or attacker-controlled dynamic
redirection.  The child inherits the local array through `fork`, not through
the environment.  Thus the only process that proceeds to cleanup, CMake,
CTest, colcon, `nm`, or test executables has already closed every build-lock FD.
The supervisor still has its own descriptor-table references to the locked
open-file descriptions.

After the child is reaped and its process group is empty, the parent closes its
FD array with the same variable-FD form, restores traps/monitor mode, and
returns the callback status.  Normal shell exit is a final kernel-backed
cleanup for all parent FDs.

Do not keep `OPENARM_BUILD_LOCK_FDS`, `openarm_validate_locks`, or any equivalent
token/cookie protocol.  Also remove the obsolete
`OPENARM_BUILD_LOCK_HELD`/`OPENARM_NATIVE_BUILD_LOCK_HELD` compatibility code;
ambient variables become irrelevant rather than merely unset.

### `scripts/lib/build_native_body.sh` (new, source-only)

Define one namespaced function and no executable body:

```text
openarm_build_native_body REPO_ROOT BUILD_ROOT INSTALL_PREFIX BUILD_TYPE
                          RUN_TESTS REUSE_BUILD_TREES JOBS
```

Move the current native mutations from `mkdir -p "$build_root"
"$install_prefix"` through the installed-symbol/consumer checks into this
function.  Move or namespace its two helpers (`assert_cache_value` and
`configure_build_test_install`) in the same library.  All state should be local
to the function; nested helpers may use Bash dynamic scoping, but must not read
ambient lock state.

The description path is derived inside the body as exactly:

```bash
description_dir="$repo_root/upstream/openarm_description"
```

It is not a parameter.  `REPO_ROOT` is supplied only by the two scripts from
their own `BASH_SOURCE[0]` location.  This gives tests a source-only seam without
creating a production description option.

Do not make this library executable and do not give it a `--locked-body` or
other dispatcher.  A repository-local sourced function is composition, not a
second public CLI.

### `scripts/build_native.sh`

Retain all current public option parsing and canonical path/safety validation,
including the `0|1` checks and bounded jobs.  Source `build_lock.sh` and
`lib/build_native_body.sh`.  Delete all `locked_body` parsing and validation.

Set and validate only the fixed description checkout:

```bash
description_dir="$root_dir/upstream/openarm_description"
```

After all read-only validation, end the file with the bare call:

```bash
openarm_run_with_locks "$build_root" "$install_prefix" -- \
  openarm_build_native_body "$root_dir" "$build_root" "$install_prefix" \
  "$build_type" "$run_tests" "$reuse_build_trees" "$jobs"
```

Every invocation of this public script therefore acquires the native-build and
install locks.  There is no public bypass argument.

### `scripts/build.sh`

Retain public parsing, canonicalization, output-root safety checks, jobs, and
read-only prerequisite checks.  Add the missing locked-value checks for both
`run_tests` and `clean` while removing locked-body parsing entirely.  Source the
native body library.

Define a private `openarm_build_all_body` function in this file.  Move all
mutations into it: `clean_child`, cleanup, native build, ROS environment setup,
colcon, archive checks, and optional test registration checks.  Its native step
is the direct function call:

```bash
openarm_build_native_body "$root_dir" "$native_build" "$install_prefix" \
  "$build_type" "$run_tests" "$reuse_build_trees" "$jobs"
```

It must not invoke `scripts/build_native.sh`, because that public entry point
must always try to acquire locks and would correctly contend with the top-level
supervisor.  It also must not pass a bypass flag.

End `build.sh` with the bare call:

```bash
openarm_run_with_locks \
  "$output_root" "$native_build" "$install_prefix" -- \
  openarm_build_all_body "$root_dir" "$output_root" "$build_type" \
  "$run_tests" "$clean" "$jobs"
```

The callback child has no lock FDs, while the parent owns all three locks for
the complete native-plus-ROS transaction.

### `scripts/launch_rviz.sh` and `scripts/launch_web_portal.sh`

No build-lock redesign is needed.  Keep GUI FD 9 in managed GUI descendants,
and keep the explicit `9>&-` on the unmanaged browser.  The launchers call only
the normal public `build.sh`; they must not learn about native body functions
or supervisor internals.

## Signal, exit, and Bash semantics

The callback must not be run through `if openarm_run_with_locks ...`, `!`, a
pipeline, or `... || status=$?`.  Bash disables `errexit` dynamically inside
functions invoked in those tested contexts, which would also disable it in the
mutating body.  Each public script must use the lock call as its final **bare**
simple command.  The helper handles contention mapping itself; a body failure
then naturally becomes the script's exact exit status.

For robust signal ownership, the supervisor should put its callback subshell in
a separate process group (temporarily enable Bash monitor mode for the spawn,
record `$!` as both leader PID and PGID, then restore the previous monitor-mode
state).  Install parent traps for HUP, INT, and TERM that:

1. record 129, 130, or 143 respectively;
2. forward the same signal to `-PGID`; and
3. continue waiting rather than returning from the helper.

The child resets those traps to defaults before closing FDs and calling the
body.  `wait` may return early when the parent's trap runs, so use a wait loop:
retry while the leader still exists, reap it once, then retain the locks while
`kill -0 -- "-$pgid"` says any member of the body group remains.  This prevents
a signalled supervisor from releasing locks while CMake/colcon descendants are
still mutating.  On an ordinary completion return the exact body status; after
a forwarded signal return the recorded conventional signal status.  Restore
the caller's traps before returning.  A descendant that daemonizes or ignores
a terminating signal will deliberately keep the supervisor/locks alive; the
build bodies must not daemonize.  SIGKILL of the supervisor cannot be made
transactional by shell code and remains the unavoidable limit.

Using a simple foreground subshell without traps is insufficient: a signal sent
only to the parent PID could release its locks while the body survives.  Using
`flock --close command` alone has the same supervisor-signal problem and would
also require an executable/string-dispatched internal body.  The private
function/process-group design avoids both.

Requirements are Bash-specific: anonymous FD variables require Bash 4.1+, and
the process-group implementation should be tested on the project's Linux/Bash
platform.  No `eval`, `bash -c` command construction, exported functions, or
dynamic FD injection is needed.

## Regression design

Revise `tests/test_build_resource_controls.sh` rather than relying on an
ambient description override.

1. **Immediate and transitive FD noninheritance.** Source `build_lock.sh` in a
   test-only driver.  Hold two synthetic resources and have the callback, then
   a grandchild, enumerate `/proc/self/fd/*`.  Fail if any descriptor resolves
   to either expected lock file.  The callback must signal readiness only after
   these checks.

2. **Adversarial reentrant contention.** While that real callback is held, have
   one of its descendants start a separate test driver that sources the public
   lock helper and requests the same canonical resource.  It must exit 3 and a
   mutation-marker callback must never run.  Then release the first callback and
   prove a new request succeeds.  This is the regression the current forged
   boolean test does not cover; it proves that no inherited open-file
   description makes `flock -n` look reentrant.

3. **Public native has no bypass.** Assert `build_native.sh --locked-body ...`
   is rejected as an unknown option.  Run two copied-fixture public native
   scripts against aliased build paths/shared prefixes and retain the existing
   status-3 contention checks.  Also keep the independent sibling-root success
   case.

4. **Pinned description.** Copy `scripts/build.sh`, `scripts/build_native.sh`,
   and their sourced libraries into a disposable miniature repository below
   `work_root`; put `package.xml` only at that miniature repository's
   `upstream/openarm_description`.  Run the existing narrow CMake/colcon shims
   through those copied public scripts while setting
   `OPENARM_DESCRIPTION_DIR` to a second, valid decoy.  Assert the CMake
   `-DOA_DESCRIPTION_ROOT=` and colcon `--base-paths` records name only the
   miniature repository's pinned path and never the decoy.  In a second case,
   omit the pinned `package.xml` but keep the decoy valid; both public scripts
   must fail before the command log records mutation.  This exercises the real
   public construction without writing the checkout's `upstream/` directory.

5. **Top/native composition.** In the shim log, a top build must show exactly
   one native body sequence followed by colcon.  During a gated top body, direct
   public `build_native.sh` attempts against either its native build root or
   install prefix must exit 3.  This proves the top supervisor owns the
   canonical native/install resources even though it composes the source-only
   body.

6. **Signal lifetime.** Run a lightweight callback with a child in its process
   group, send TERM to the supervisor PID, and require status 143, disappearance
   of the whole callback process group, and immediate successful reacquisition
   afterward.  This protects the parent-only-signal case.

Keep the real one-job tiny CMake/CTest check, bounded `--parallel 2`, sequential
colcon, incremental-tree, GUI mutual-exclusion, and browser-FD assertions.
Run `bash -n` over every changed script and the regression.  No high-parallel,
GUI, middleware, or physical-hardware run is required for this fix.

## Migration order

1. Add the source-only native body and move the native mutating statements
   without behavior changes.
2. Replace the lock helper with parent-retained/private-child closure; add the
   helper-level FD, contention, and signal tests first.
3. Convert `build_native.sh` to its always-locking public wrapper and delete all
   locked-body/environment-record code.
4. Convert `build.sh` to a private top body that calls the native function and
   owns all three canonical locks.
5. Remove both `OPENARM_DESCRIPTION_DIR` reads and migrate the shim to the
   disposable copied-repository fixture.
6. Run syntax/regression checks, then review `rg` results for
   `locked-body|OPENARM_BUILD_LOCK_FDS|OPENARM_DESCRIPTION_DIR|LOCK_HELD`; all
   should be absent from production scripts.

## Remaining risks / non-blocking hardening

- `openarm_lock_dir` still has the review's fixed-name `/tmp` check/create race.
  It should be hardened with create-then-`lstat`/owner/mode verification (or a
  securely provisioned per-user runtime directory), but this is independent of
  the descriptor-ownership architecture.  Never silently mask failures from
  that function.
- Lock identity remains path-based.  All public paths must continue through
  `realpath -m` before the lock helper so aliases coordinate.  Lock files must
  remain outside cleanable output roots.
- Process groups and `/proc` make Linux an explicit platform assumption.  The
  production supervisor does not need `/proc`; only the FD-inheritance test
  does.
- Do not replace the local FD array with delimiter-encoded paths.  Besides
  reopening the inheritance bug, commas/newlines in valid runtime paths would
  reintroduce parsing ambiguity.

This design resolves the root conflict: one parent owns the canonical locks,
no mutating process has a lock descriptor, top-level composition has no public
bypass, direct native builds always lock, and robot-description identity stays
fixed to the fetched repository checkout.
