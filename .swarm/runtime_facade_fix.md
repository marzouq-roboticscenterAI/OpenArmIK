# Runtime facade review-fix evidence

Review input: `.swarm/runtime_facade_review.md` against `0f0ea372bddc302ef78837c611ee9c25c9b82fad`.

## Persistence V2 redesign closure

The third review demonstrated that a directory-local authenticated maximum is
replayable after unlink/reopen and that `flock` on a fork-inherited long-lived FD
does not serialize transactions. The additive V2 contract now requires a
caller-owned external `(revision,digest)` checkpoint at open and every operation.
Checked save performs an exact-current CAS, emits its new checkpoint only after
reload verification and final directory sync, and slot-binds the V2 HMAC.
Ordinary load never inspects `.previous`; explicit checked recovery applies the
external floor before promotion. A failed, unconfirmed post-rename rollback
poisons the authority until checked recovery or reopen. Legacy V1 HMAC files are
explicitly replayable and never checkpoint-authorized, so they cannot create an
armable virtual runtime.

Every transaction locks a fresh independently opened directory FD. Authority
paths are opened component-by-component with `openat(O_DIRECTORY|O_NOFOLLOW)`,
special-file reads are nonblocking and rejected, and public internal/recovery
names are reserved. Handles are process-local: an inherited child call fails a
creator-PID guard before registry mutexes, and the supported child path is
immediate `exec` only.

The focused matrix covers byte-copy and hard-link rollback after reopen, slot
relinking, same-revision equivocation, missing-current recovery above/equal to
the floor, exact-CAS revision-3/4 stale writers, same-authority and independent-
authority threads, independently exec'd processes, inherited-handle fork
rejection, FIFO/symlink/oversize/corrupt inputs, confirmed rollback `EIO`, and
unconfirmed rollback-sync `EDURABILITY` with poison/recovery.

Fresh GCC 15.2 Release, ASan+UBSan+leak, and TSan builds pass 2/2 tests;
cppcheck warning/performance/portability passes. A fresh install builds, links,
and runs strict all-header C11 and C++17 consumers; declaration/export parity is
50/50, the installed archive has no test hooks, and its only CAN builder
dependency remains `oa_can_make_register_query_typed`. `git diff --check` and
the native script syntax check pass. The injected fsync matrix proves API status,
poisoning, and visible current/previous reconciliation, not power-loss behavior;
real crash/remount qualification remains filesystem-specific as the README
states.

## Second re-review closure

Review input: the updated `.swarm/runtime_facade_review.md` against
`8d01458e238e1d5a5ee2934925893387f7e8a2dd`.

1. **Controller/facade clock translation — fixed.** Each produced arm feedback
   sequence is now paired with its production-time host-steady timestamp under
   a controller synchronization mutex. Facade snapshots export only that
   evidence and recompute freshness against real facade time. During a plan
   pause the sequence/timestamp remain fixed and become stale after the 50 ms
   timeout; the first resumed sequence receives a fresh, nondecreasing facade
   timestamp. Calibration samples consume the same translated snapshot.
   Controller events are drained into a fixed-capacity facade queue at
   generation cadence and translated before the private controller clock can
   diverge. Tests hold a plan for 70 ms, prove zero freshness and real age,
   release it, and prove prompt fresh/non-backward feedback; event timestamps,
   live wall cadence, plan expiry, and past-deadline rejection are also checked.
2. **Error-path mutex reentrancy — fixed.** Manual and recipe create failures
   clear the calibration lease, release the runtime mutex, and only then record
   the lower commission error. The broader audit moved ABI-valid semantic
   validation behind runtime pinning and records runtime facility/lower-zero
   detail; session-state semantic failures do likewise. Timed asynchronous
   invalid manual and recipe creates must return within 500 ms with exact
   commission facility/lower codes, and the full TSan/deadlock-stack run passes.
3. **Persistence transaction/floor — fixed.** Each authority serializes its
   transaction and also takes an OS directory lock, so independent authority
   handles coordinate. Before authenticated load or save, authenticated regular
   artifacts are scanned under that lock to recover the highest accepted
   directory revision/content floor. Per-name accepted state rejects rollback
   and equivocation; directory-wide same-revision content conflicts fail closed.
   Concurrent conflicting revision-2 saves now produce exactly one success and
   one `ESTALE`. Public `.previous` load is always stale, copied old artifacts
   remain stale after reopening the authority, and plain-loaded artifacts cannot
   arm a virtual runtime. Existing file/directory fsync, atomic rename,
   authenticated reload, rollback, retained-prior, and `EDURABILITY` behavior is
   preserved.
4. **Individual-joint identity binding — fixed.** `oa_runtime_joint_move` now
   requires exact model and side-TCP revisions, coordinate-identity digest,
   collision policy, and scene revision before acquiring plan authority. A
   copied request is rejected across reject-all and unchecked runtimes even
   when feedback sequences coincide. Plan reports expose model, per-side TCP,
   digest, collision policy, and scene verification evidence.
5. **Last-error consistency — fixed.** Invalid interlock booleans and invalid
   heartbeat/disarm/now clock IDs pin a valid runtime first and record
   `OA_RUNTIME_FACILITY_RUNTIME`, `OA_RUNTIME_EINVAL`, and lower code zero.

Second re-review verification: fresh Release, ASan+UBSan+leak, and TSan builds
all pass 2/2 tests; cppcheck passes; fresh installed all-six-header strict C11
and C++17 consumers build/link/run; declaration/export parity is 46/46; the
installed archive has no test hooks; and its only CAN builder dependency is
`oa_can_make_register_query_typed` on the query-classifying transport path.

## Finding disposition

1. **Truthful capabilities and coordinate identity — fixed.** Standalone FK,
   single-IK, and paired-IK bits are never advertised because no such facade
   entry points exist. `oa_runtime_get_model_identity` exposes the exact
   side-specific model ID, provenance, data/source/flattened-URDF digests,
   named TCP and revision. Capability, kinematics, snapshot, paired request,
   and plan-report records bind a combined coordinate identity and explicit
   collision policy/model/TCP revisions.
2. **Clock coherence — fixed.** Every backend initializes a nonzero host-steady
   facade clock and `oa_runtime_now_monotonic_ns` remains live. Physical evidence
   uses that host domain and virtual inventory uses a captured real timestamp.
   The virtual controller has a private exact-cycle counter initialized at the
   same epoch; facade deadlines are translated by remaining duration so host
   scheduling jitter and plan quiescence cannot invalidate the controller's
   hard `delta <= cycle` contract or freeze facade expiry.
3. **Multiple-plan freeze — fixed.** A runtime grants one expiring plan
   authority. Immediate competitors return `OA_RUNTIME_EBUSY`; expiry resumes
   cadence even while the old handle exists; generation IDs prevent an expired
   handle from clearing or executing under a replacement authority. Successful
   execute releases quiescence.
4. **Calibration binding/concurrency — fixed.** Each calibration session has a
   mutex covering sample/step/review/commit/abort and its `finished` state.
   Supervised steps reconstruct input from the coherent runtime snapshot,
   runtime interlocks, and immutable recipe evidence/fixture/posture binding;
   only operator readiness/action/review decisions remain caller assertions.
   Duplicate snapshot generations are rejected before they can latch the lower
   commissioning session. Physical calibration remains unsupported.
5. **Authenticated persistence/failure contract — fixed.** The public magic
   value is removed. An opaque OS-directory authority owns a 256-bit HMAC key
   and key ID. Authenticated load verifies HMAC-SHA-256 in constant time and
   marks the accepted handle; plain load explicitly remains unauthenticated.
   Save prevents rollback and same-revision equivocation, retains
   `<name>.previous`, atomically renames, reload-verifies, and directory-syncs.
   Precommit failures leave the old target. A final sync failure rolls back and
   re-syncs before returning `EIO`; only an unconfirmed rollback returns
   `OA_RUNTIME_EDURABILITY` with explicitly unknown target state.
6. **Allocation containment/cleanup — fixed.** Preview has a complete C-boundary
   exception guard. Physical transport ownership transfers immediately into a
   unique RAII lease, so all later allocation/string/vector failures close and
   destroy it. Test-only allocation, RAII-unwind, and fsync hooks are compiled
   only into the non-installed test library.
7. **Exact event evidence — fixed.** The exact lower aggregate sequence is
   exposed as `source_feedback_seq`. Per-arm sequences and measurement time are
   zero with explicit validity flags clear because the lower event does not
   contain atomic per-arm evidence; no scalar is duplicated into invented data.
8. **Status/detail authority — fixed.** `OA_CONTROL_EUNREACHABLE` maps to the new
   runtime unreachable status. Control, commission, transport, planning,
   execution, heartbeat, stop, disarm, interlock, inventory, and unsupported
   runtime paths record facility and lower status after a runtime is known.
9. **ABI versus semantic errors — fixed.** Record size/version violations return
   `EABI`; valid-record enum/range/frame/clock/policy errors return `EINVAL`.
   Invalid kinematic side is `EINVAL` before backend support is considered.
10. **CMake 3.16 compatibility — fixed.** `cmake --fresh` is replaced by the
    long-supported, build-root-scoped `cmake -E remove_directory` followed by a
    normal configure, for components and installed consumers.

## Adversarial coverage

`runtime/tests/test_runtime.cpp` now exercises false-capability absence, exact
model/TCP identity, query/offline live clocks, real virtual inventory timestamps,
immediate second-plan rejection, clock advance while planning, automatic plan
expiry/replacement with the old handle alive, spoofed calibration interlock and
evidence/posture claims, concurrent abort, HMAC known vector/wrong key,
authenticated-versus-plain load, revision rollback, retained prior revision,
precommit and postrename-fsync rollback, preview allocation failure, transport
RAII exception unwind, explicit event-evidence absence, unreachable lower detail,
and ABI/semantic recovery classes. Query-backend manual and supervised
calibration remain `EUNSUPPORTED`.

## Final verification

- Fresh GCC 15.2 Release build and CTest: **2/2 passed**.
- Fresh Debug ASan+UBSan with leak detection and halt: **2/2 passed**.
- Fresh Debug TSan with halt/deadlock stacks: **2/2 passed**.
- cppcheck warning/performance/portability: **passed**.
- Fresh installed-package all-six-header C11 and C++17 consumers: **built,
  linked, and ran**.
- Installed public declaration/export parity: **46/46**.
- Installed runtime archive: **no test-hook symbols**.
- Physical transmit audit: the only referenced CAN builder is
  `oa_can_make_register_query_typed`; send remains the generic transport path
  that classifies and permits register queries. Physical apply, calibration,
  and motion remain unsupported.
- `git diff --check`, `bash -n scripts/build_native.sh`, and the scoped CMake
  reset probe: **passed**.

The repository-wide native script could not execute because this isolated
worktree does not contain `upstream/openarm_description/package.xml`; its
fail-closed preflight was run and reported the missing pinned upstream before
creating build/install directories. Component and installed-consumer builds
above are complete. No physical hardware/vcan interface was present; physical
transport cleanup is covered by the same production RAII lease under a
test-only exception-unwind probe, while the installed archive remains hook-free.
