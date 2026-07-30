# Runtime facade final persistence re-review

Commit: `bafeb78b6662169533af0dbe5bf26ca9726dd8f3`
Base: `604ca28`
Prior reviewed commit: `8c92f078dd5205eddbf835992f3f50cc36794650`
Disposition: **FINDINGS**

No Critical or Important findings. The repeated authenticated-persistence issue
is resolved and is **not a convergence blocker** at this commit. V2 correctly
makes freshness relative to caller-owned state outside the replayable directory;
legacy/local authenticated files are no longer control authority.

## Minor

1. **The V2 checkpoint input cannot alias its output, but that restriction is
   neither documented nor expressed by the C API.** Load clears
   `out_observed_checkpoint` before validating/reading `trusted_checkpoint`
   (`runtime/src/persistence.cpp:969-975`); save and recovery have the same order
   (`:1018-1024`, `:1099-1105`). Since the parameters are ordinary non-`restrict`
   pointers, updating one checkpoint object in place is a natural legal call.
   A focused installed-Release probe provisioned revision 1, then called
   `oa_runtime_manifest_load_authenticated_v2(..., &checkpoint, ...,
   &checkpoint)`: the function first zeroed the trusted tuple, returned
   `OA_RUNTIME_EPERMISSION`, and left the caller's checkpoint destroyed instead
   of returning `OK`. The behavior is fail-closed, so this is an API usability
   and failure-output issue rather than an authorization bypass. Copy the input
   record before clearing outputs, or explicitly document and validate
   non-aliasing.

2. **Saving after `.previous` recovery leaks a transaction-owned internal hard
   link.** Recovery promotes the prior file by hard-linking it and renaming that
   link over current (`persistence.cpp:1169-1200`), leaving current and
   `.previous` as names for the same inode. The next save links current to
   `.openarm-prior-<pid>-<sequence>` and renames it over `.previous`
   (`:573-582`). On Linux, when rename source and destination already name the
   same inode, `renameat` succeeds without removing the source name. A focused
   installed-Release sequence—recover exact revision 1, then successfully save
   revision 2—returned `OA_RUNTIME_OK` but left
   `.openarm-prior-<pid>-1000002`; directory removal failed until that orphan was
   deleted. Repeated recovery/update cycles can accumulate authenticated hidden
   files and consume directory space. The reserved prefix prevents public V2
   selection, so no rollback or arming bypass was observed.

3. **The committed verification claim for `git diff --check` is false.** The
   command reports trailing whitespace at
   `.swarm/persistence_redesign_b.md:3-4`. This is documentation hygiene only.

## Persistence convergence result

- **External freshness authority:** resolved. V2 authority open and every V2
  operation validate an exact `(revision, content_sha256)` checkpoint. Zero is
  provisioning-only; load/recovery require a nonzero trusted tuple. The README
  accurately states that local files and a static HMAC key are not monotonic
  authority and that the checkpoint must live outside the directory replay
  domain.
- **Replay and slot binding:** resolved. V2 HMAC input binds key ID and logical
  slot. Fresh tests rejected copied old bytes, hard-linked prior bytes restored
  as current, replay under another name, removed-current reopen, same-revision
  equivocation, and direct `.previous` use. Recovery with floor C2 rejected C1;
  recovery with exact C1 explicitly restored and authorized C1.
- **Exact-current CAS:** resolved. Save authenticates current and requires exact
  equality with the caller checkpoint under the transaction lock. Revision-3/4
  proposals based on stale C1 and conflicting revision-2 writers were rejected;
  outputs stayed cleared on the tested non-aliasing failure paths.
- **Serialization and fork:** resolved under the documented process model. Every
  transaction opens a fresh directory file description and holds `flock` through
  verification/final sync. Same-authority threads, separate-authority threads,
  and independently exec'd processes produced one winner and one `ESTALE`.
  Inherited child API calls returned `OA_RUNTIME_ESTATE` before registry mutexes;
  the supported child path is immediate `exec`.
- **Legacy authority boundary:** resolved. Plain and legacy V1 authenticated
  loads report `checkpoint_authorized == 0`; `oa_runtime_create` rejects their
  ARMABLE file handles with `OA_RUNTIME_EPERMISSION`. A V2 checked load reports
  checkpoint authorization and can create the virtual runtime.
- **Namespace and recovery:** component-by-component `openat` with
  `O_DIRECTORY|O_NOFOLLOW` pins authority paths; target opens are no-follow and
  nonblocking. FIFO, symlink, oversized, corrupt, wrong-slot, and missing-file
  cases fail closed. Ordinary load never falls back to `.previous`; recovery is
  explicit and checkpoint-relative, subject only to Minor 2's orphan cleanup.
- **Durability status:** injected failures revalidated confirmed rollback as
  `OA_RUNTIME_EIO`, unconfirmed rollback/sync as `OA_RUNTIME_EDURABILITY`, cleared
  checkpoint outputs, poisoned ordinary operations, and explicit recovery before
  reuse. This validates API state transitions, not universal power-loss behavior;
  the README correctly limits crash guarantees to a qualified local filesystem.

## Other prior findings

All non-persistence fixes remain resolved: continuously translated facade
timestamps become stale during plan quiescence and fresh on resume; invalid
calibration creation has no mutex reentrancy; joint plans bind exact model/TCP,
coordinate digest, collision policy, and scene; semantic failures update runtime
`last_error`; capabilities, model identity, plan expiry, calibration evidence,
event evidence, exception/RAII cleanup, status mapping, ABI-versus-semantic
classification, and the query-only physical boundary remain covered and pass.

## Verification performed

- Fresh GCC 15.2 Release build and CTest: **2/2 passed**.
- Fresh Debug ASan+UBSan with leak detection/halt: **2/2 passed**.
- Fresh Debug TSan with halt/deadlock stacks: **2/2 passed**.
- `cppcheck --enable=warning,performance,portability --error-exitcode=1`:
  **passed**.
- Fresh installed all-six-header strict C11 and C++17 consumers: **built, linked,
  and ran**.
- Installed declaration/export parity: **50/50**; no installed test-hook symbols
  or test library.
- Query-only audit: the only referenced CAN frame builder is
  `oa_can_make_register_query_typed`; the transport send result is required to be
  `OA_TRANSPORT_FRAME_REGISTER_QUERY`. No write/enable/disable/zero/save/motion
  builder or physical actuation facade was found.
- Fresh installed-Release focused checks covered external-floor replay after
  unlink/reopen, hard-link/name replay, explicit recovery above/equal to the
  floor, exact CAS, fork rejection, legacy/plain arming rejection, and FIFO
  nonblocking rejection. Core acceptance passed; the additional alias and
  recovery/update cleanup probes produced Minor 1 and Minor 2.
- `bash -n scripts/build_native.sh` and its scoped CMake reset remain valid.
  `git diff --check 8c92f07..bafeb78` **fails** only as described in Minor 3.

No hardware, CAN interface, or network traffic was used.
