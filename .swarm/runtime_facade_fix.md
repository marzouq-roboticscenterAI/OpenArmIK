# Runtime facade review-fix evidence

Review input: `.swarm/runtime_facade_review.md` against `0f0ea372bddc302ef78837c611ee9c25c9b82fad`.

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
