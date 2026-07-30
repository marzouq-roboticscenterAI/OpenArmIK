# Runtime facade final persistence re-review

Commit: `e09296adba56ff6b8f3b74243f916c904209a709`
Prior reviewed commit: `bafeb78b6662169533af0dbe5bf26ca9726dd8f3`
Disposition: **CLEAN**

No Critical, Important, or Minor findings remain. The three Minors from the
prior review are closed. The earlier repeated authenticated-persistence issue
also remains resolved and is not a convergence blocker.

## Minor closure

1. **Same-object checkpoint input/output:** closed. V2 load, save, and recovery
   validate and copy the input checkpoint before clearing the output. The
   installed strict-C11 consumer passed in-place initial and idempotent save,
   load, and recovery. Malformed aliased input for all three operations returned
   `OA_RUNTIME_EINVAL` and reinitialized/cleared the checkpoint output; load and
   recovery also left the manifest output null. A separate installed-production
   cycle probe used the same object for every recovery and update successfully.

2. **Recovery/save inode aliases and hidden transaction files:** closed.
   Recovery now promotes `.previous` through a file-synced independent copy;
   save creates an independent prior copy and explicitly removes its temporary
   name even for the Linux same-inode rename-no-op case. Cleanup of reserved
   transaction names runs under the fresh-FD directory transaction lock and is
   directory-synced. The regression test repairs a deliberately recreated
   pre-fix hard-link topology, performs eight recovery/save cycles with distinct
   current/previous inodes and no hidden artifacts after each step, and exercises
   exec-kill points before and after recovery rename. Repeated pre-rename kills
   leave at most one artifact, which the next transaction removes. Injected
   pre-rename file-sync failure returns `OA_RUNTIME_EIO` with no visible artifact;
   post-rename directory-sync failure returns `OA_RUNTIME_EDURABILITY`, poisons
   ordinary use, and is cleared only by successful explicit recovery. A fresh
   probe linked against the installed production archive independently recreated
   the pre-fix alias and passed five recover/save cycles, inode and artifact
   checks, and final directory removal.

3. **Whitespace/evidence:** closed. The two Markdown trailing spaces were
   removed. `git diff --check bafeb78..e09296a`,
   `git diff --check 8c92f07..e09296a`, and the working-tree
   `git diff --check` all pass. The follow-up evidence accurately describes the
   checkpoint alias, inode repair, cleanup, kill, fsync, and verification tests.

## Persistence convergence result

- Caller-owned V2 checkpoints remain the freshness authority outside the
  replayable directory; local authenticated files and the static HMAC key are
  not treated as monotonic authority.
- Slot-bound authentication, exact-current CAS, fresh directory-lock file
  descriptions, fork rejection, explicit recovery, legacy/plain arming
  rejection, and fail-closed namespace handling remain covered and passing.
- The durability contract continues to distinguish confirmed rollback
  (`OA_RUNTIME_EIO`) from unknown post-commit durability
  (`OA_RUNTIME_EDURABILITY`) and requires explicit recovery after poison.

## Verification performed

- Fresh GCC 15.2 Release build, install, and CTest: **2/2 passed**.
- Fresh Debug ASan+UBSan with leak detection and halt-on-error: **2/2 passed**.
- Fresh Debug TSan with halt-on-error and deadlock stacks: **2/2 passed**.
- `cppcheck --enable=warning,performance,portability --error-exitcode=1`:
  **passed**.
- Fresh installed all-six-header strict C11 and C++17 consumers: **built,
  linked, and ran**. The C11 consumer includes the alias success/failure probe.
- Installed declaration/export parity: **50/50**. The installed archive exposes
  no `oa_runtime_test_*` symbols, and no test library or test artifact is
  installed.
- Query-only audit: the sole CAN frame builder referenced by the runtime is
  `oa_can_make_register_query_typed`; sends require
  `OA_TRANSPORT_FRAME_REGISTER_QUERY`. No actuation builder or physical motion
  facade is present.
- `bash -n scripts/build_native.sh`: **passed**.
- Full focused persistence coverage includes replay/floor/CAS, thread and exec
  contention, fork rejection, recovery/save cycles, pre-fix alias repair,
  exec-kill cleanup, injected fsync failure/poison/recovery, and installed
  production-archive probes.

No hardware, CAN interface, or network traffic was used.
