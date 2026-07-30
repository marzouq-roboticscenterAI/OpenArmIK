# Runtime facade third independent re-review

Commit: `8c92f078dd5205eddbf835992f3f50cc36794650`
Base: `604ca28`
Prior reviewed commit: `8d01458e238e1d5a5ee2934925893387f7e8a2dd`
Disposition: **FINDINGS**

No Critical findings. Physical behavior remains fail-closed: physical inventory
is unresolved/ambiguous, physical configuration/calibration/motion remain
unsupported, and the production archive references only the typed register-query
CAN builder on the independently query-classified transport send path.

## Important

1. **The authenticated persistence rollback/serialization issue persists on two
   cross-process paths.** `refresh_revision_floor` reconstructs its maximum only
   from authenticated files currently present in the directory plus process-local
   authority memory (`runtime/src/persistence.cpp:218-256`); it does not store a
   trusted durable high-water mark. The `.previous` defense is only a suffix check
   on the requested name (`persistence.cpp:428-434`). In a focused installed-
   Release test, revision 1 was saved, revision 2 was successfully saved, the
   authority was destroyed, the current revision-2 file was removed, and retained
   revision-1 `.previous` was hard-linked as `rollback.oarm`. A newly opened
   authority returned `OA_RUNTIME_OK` for authenticated load of `rollback.oarm`,
   and `oa_runtime_create` accepted that authenticated old ARMABLE manifest with
   `OA_RUNTIME_OK`. Thus removal/replacement of the current artifact resets the
   reconstructed floor and bypasses the retained-prior name restriction after a
   process/authority reopen.

   Cross-process serialization also fails when an authority exists before
   `fork()`. `DirectoryTransaction` applies `flock` to the authority's long-lived
   directory fd (`persistence.cpp:205-216`). Forked children inherit the same open
   file description, which is one `flock` owner, while each child has an independent
   copy of the C++ mutex. Two released-at-once children saving different revision-2
   manifests over revision 1 through that inherited authority both returned
   `OA_RUNTIME_OK` on the first focused run. The resulting current and `.previous`
   files were different authenticated revision-2 contents. Independent authority
   handles opened in each child did serialize correctly in ten races, and separate
   authorities in two threads also produced exactly one `OK` and one `ESTALE`, but
   the public API/README does not exclude post-fork authority use. This is the same
   prior persistence finding, not a new category: revision transactions and the
   accepted revision/content floor are still not unconditionally cross-process or
   rollback safe.

## Resolved prior findings

- **Facade timestamps/staleness:** resolved. Production-time host-steady evidence
  is associated with each feedback sequence under controller synchronization.
  Focused sampling observed continuously increasing recent timestamps; a 120 ms
  plan hold kept sequence/timestamp fixed and cleared freshness after the 50 ms
  timeout; destroying the plan promptly produced a newer sequence, newer facade
  timestamp, and full freshness. Event and calibration paths consume the same
  translation.
- **Mutex error paths:** resolved. Invalid ABI-valid manual and recipe creation
  each returned within 500 ms with runtime `EINVAL`, commission facility, and the
  exact lower `OA_COMMISSION_EINVAL`; no runtime-mutex reentrancy remained on those
  paths. TSan/deadlock-stack coverage passed.
- **Joint-plan identity:** resolved. Individual joint requests now require the
  exact model revision, selected-side TCP revision, coordinate digest, collision
  policy, and scene revision. Focused mutations of every field were rejected, and
  the plan report exposed the verified binding.
- **Semantic last-error:** resolved. Invalid interlock, heartbeat clock, disarm
  clock, and runtime-now clock calls on a valid runtime returned `EINVAL` and
  recorded runtime facility with lower code zero.
- **All first-round findings:** rechecked with the production boundary unchanged:
  truthful capabilities/model identity, live clocks, expiring single-plan
  authority, synchronized/runtime-bound calibration, exception/RAII cleanup,
  exact event evidence, unreachable/detail mapping, ABI-versus-semantic status,
  and CMake 3.16-compatible scoped reset remain covered and pass. Authenticated
  HMAC/file durability behavior passes in ordinary sequential operation, subject
  to the Important rollback/transaction exception above.

## Verification performed

- Fresh GCC 15.2 Release build and CTest: **2/2 passed**.
- Fresh Debug ASan+UBSan with leak detection and halt: **2/2 passed**.
- Fresh Debug TSan with halt/deadlock stacks: **2/2 passed**.
- `cppcheck --enable=warning,performance,portability --error-exitcode=1`:
  **passed**.
- Fresh installed-package all-six-header strict C11 and C++17 consumers: **built,
  linked, and ran**.
- Installed declaration/export parity: **46/46**; no installed test-hook symbols
  or test library.
- Query-only audit: the sole referenced CAN frame builder is
  `oa_can_make_register_query_typed`; the sent frame must be reported as
  `OA_TRANSPORT_FRAME_REGISTER_QUERY`. No write/enable/disable/zero/save/motion
  builder or physical actuation entry point was found.
- Fresh installed-Release focused tests: timestamp cadence/staleness/resume,
  invalid-calibration completion/error detail, all joint identity fields, semantic
  `last_error`, independent-authority thread/process persistence races, authority
  reopen, and `.previous` copy behavior.
- `git diff --check`, `bash -n scripts/build_native.sh`, and source inspection of
  its scoped `cmake -E remove_directory` reset: **passed**. The repository-wide
  native script was not run because this isolated worktree lacks the pinned
  `upstream/openarm_description/package.xml`; the fresh component/install matrix
  above is complete.

No hardware, CAN interface, or network traffic was used.
