# Runtime facade independent re-review

Commit: `8d01458e238e1d5a5ee2934925893387f7e8a2dd`
Base: `604ca28`
Prior reviewed commit: `0f0ea372bddc302ef78837c611ee9c25c9b82fad`
Disposition: **FINDINGS**

No Critical findings. Physical operation remains fail-closed: physical inventory is unresolved/ambiguous, physical configuration and calibration return `OA_RUNTIME_EUNSUPPORTED`, no physical controller exists, and the production archive references only `oa_can_make_register_query_typed` plus the independently query-classifying `oa_transport_send` path. No arbitrary frame, register write, enable, disable, zero, save-parameters, or physical-motion facade was found.

## Important

1. **Facade clock coherence is still broken after plan quiescence, so exported freshness can contradict exported timestamps.** The worker advances the lower controller on a private exact-cycle counter (`runtime/src/runtime.cpp:383-408`), while `oa_runtime_now_monotonic_ns` returns live host steady time (`runtime.cpp:494-509`). Plan authority pauses the private counter, but snapshot translation exports the unadjusted lower `from.t_ns` as `measurement_runtime_monotonic_ns` under the facade clock ID (`runtime.cpp:252-280`); events likewise export lower `source.t_ns` as a facade timestamp (`runtime.cpp:1066-1115`). A focused Release test held a valid plan for 120 ms, destroyed it, waited 30 ms, then observed `age_ns=121272590`, `fresh_mask=0x7f`, with configured `feedback_timeout_ns=50000000`: feedback was declared completely fresh even though its timestamp was over twice the timeout old in the advertised facade domain. The private/facade offset persists after cadence resumes. Query/offline clocks and virtual inventory timestamps are now live and coherent, but controller snapshot/event/calibration timestamps are not.

2. **Invalid but ABI-valid calibration options deadlock the runtime instead of returning the lower error.** Manual begin acquires `owner->mutex`, clears the lease after `oa_commission_manual_create` fails, and calls `record_error` before releasing the lock (`runtime/src/calibration.cpp:126-143`); `record_error` calls `set_error`, which attempts to lock the same non-recursive runtime mutex. Recipe begin has the identical pattern (`calibration.cpp:253-259`). A focused installed-Release consumer supplied the correct side/joint/revision/serial but a zero/invalid lower manual recipe; `oa_runtime_calibration_manual_begin` failed to return and was killed by `timeout 2s` (`EXIT=124`). Session-level synchronization, runtime-owned recipe interlocks/posture/evidence, concurrent abort serialization, and physical `EUNSUPPORTED` gates otherwise passed.

3. **Authenticated persistence does not serialize revision transactions or prevent authenticated rollback.** `PersistenceAuthorityData` contains the directory fd/key/id but no transaction mutex or accepted revision floor; save performs check/link/rename/verify as an unsynchronized multi-step sequence (`runtime/src/persistence.cpp:339-456`). A focused Release race created two different revision-2 manifests and saved both concurrently over revision 1 through the same authority: both calls returned `OA_RUNTIME_OK` on the first attempt (`both_ok attempt=0`), defeating same-revision equivocation detection and making `.previous`/rollback ownership race-dependent. Separately, authenticated load verifies HMAC/key only (`persistence.cpp:316-336`), and retained `<name>.previous` remains an authenticated, `ARMABLE` old manifest; there is no authority-wide/per-name monotonic accepted revision state, so that prior artifact (or a copied old authenticated artifact under another safe name) can be loaded and passed to `oa_runtime_create`. HMAC correctness, wrong-key rejection, ordinary sequential rollback rejection, prior retention, precommit rollback, post-rename fsync rollback, and `EDURABILITY` semantics are covered and pass, but concurrent equivocation and load-time rollback remain open.

4. **Model/collision identity binding remains incomplete for individual joint requests.** Model identity, snapshot, kinematics, paired-TCP requests, capabilities, and plan reports now carry truthful model/TCP/collision identity, and the standalone FK/IK capability bits are correctly absent. However `oa_runtime_joint_move` still contains only clock, units, feedback sequence, side/joint, and numeric motion fields (`runtime/include/openarm_runtime.h:393-408`); it has no required model revision, coordinate digest, collision policy, or scene revision. Feedback sequence values are runtime-local and can coincide across handles, so an individual-joint request assembled for a reject-all runtime cannot detect accidental submission to an unchecked-collision runtime. Paired TCP correctly rejects this mismatch before planning; individual joint planning should provide the same binding.

## Minor

1. **Last-error detail is still skipped for some semantic errors on a valid runtime.** `oa_runtime_set_interlock` validates booleans before pinning the runtime (`runtime/src/runtime.cpp:605-608`), and heartbeat/disarm validate clock IDs before pinning (`runtime.cpp:994-1000, 1036-1041`), so these calls return `OA_RUNTIME_EINVAL` without recording the otherwise available runtime facility/detail. `OA_CONTROL_EUNREACHABLE` and lower control/commission/transport failures are now mapped and recorded correctly; this is a remaining consistency gap rather than loss of a lower-module cause.

## Prior finding disposition

- Truthful capabilities and exact model/TCP identity: **mostly resolved; residual joint-request binding is Important 4**.
- Query/offline/live inventory clocks: **resolved; residual controller timeline export is Important 1**.
- Single expiring plan authority and stale generation handling: **resolved**; second plans return `EBUSY`, expiry releases authority, and old handles cannot execute replacements.
- Calibration session data-race/runtime evidence/physical gate: **resolved on normal paths; invalid-create deadlock is Important 2**.
- HMAC authority, sequential revision/prior/rollback/durability: **partially resolved; concurrent and load-time rollback are Important 3**.
- C exception containment and transport RAII cleanup: **resolved**.
- Exact absent event evidence: **resolved**; per-arm sequences/timestamp are zero with validity flags clear and the lower aggregate sequence is separately named. Event clock conversion remains part of Important 1.
- `EUNREACHABLE` and lower status/detail translation: **resolved**, subject to the Minor scalar-validation detail gap.
- ABI-versus-semantic status classification: **resolved**.
- CMake 3.16 reset compatibility: **resolved**; scoped `cmake -E remove_directory` replaces `--fresh` and paths retain the script's safety checks.

## Verification performed

- Fresh GCC 15.2 Release build and CTest: **2/2 passed**.
- Fresh Debug ASan+UBSan with leak detection/halt: **2/2 passed**.
- Fresh Debug TSan with halt/deadlock stacks: **2/2 passed**.
- `cppcheck --enable=warning,performance,portability --error-exitcode=1`: **passed**.
- Fresh installed-package C11 and C++17 all-six-header consumers: **built, linked, and ran successfully**.
- Public declaration/export parity: all **46/46** `oa_runtime_*` functions are defined.
- Installed production archive contains **no test-hook symbols** and the install contains no test library.
- Query-only symbol audit: only `oa_can_make_register_query_typed` and generic `oa_transport_send` are referenced; source inspection confirms the sent frame is constructed by that typed query builder and checked as `OA_TRANSPORT_FRAME_REGISTER_QUERY`.
- HMAC RFC known vector, wrong-key rejection, authenticated/plain distinction, symlink/traversal rejection, allocation-failure containment, transport RAII unwind, sequential rollback/equivocation, retained prior, and injected fsync rollback tests: **passed**.
- Handle cross-type/stale/double-destroy and overlapping runtime destroy tests: **passed**.
- Focused adversarial tests reproduced: controller timestamp/freshness contradiction, invalid-calibration deadlock, and concurrent persistence equivocation.
- `git diff --check` and `bash -n scripts/build_native.sh`: **passed**.
- Repository-wide `scripts/build_native.sh` was logically reviewed but not executed, as requested for later main integration.

The exact virtual inventory remains two interfaces/two arms/fourteen fixed motors; physical evidence never assigns side or joint. Manifests remain immutable, handle tokens remain typed/monotonic, collision and motion authorization facts remain false, and the physical authority boundary is clean. These positive properties do not resolve the Important findings above.
