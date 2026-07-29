# Independent corrective re-review of `e225f30`

Verdict: **CHANGES REQUIRED**. The corrective commit substantively fixes the six
original High implementation findings and the mapped-PMAX defect. Release,
ASan/UBSan, and TSan suites all pass. However, adversarial review found two High
C-ABI defects and two remaining Medium robustness gaps.

## 1. High — the corrective change breaks the frozen V1 binary ABI

`OA_CONTROL_ABI_V1` remains 1, but the commit enlarges three V1 input records:

- `oa_controller_options` gains `collision_scene_revision`
  (`control/include/openarm_control.h:212-221`);
- `oa_sim_fault` gains status/send/skew fields (`:272-282`);
- `oa_paired_tcp_move` inserts two fields before its previously existing
  `collision_scene_revision` (`:176-190`).

Every wrapper still calls `valid_record<T>`, which requires
`record->struct_size >= sizeof(the new T)` (`control/src/c_api.cpp:25-29`). A V1
binary compiled against `8bc839e` therefore passes its smaller frozen record and
is rejected with `OA_EINVAL` by the new library. The paired record is not even
prefix-layout-compatible because fields were inserted before an existing field.
The C11 test is recompiled against the new header, so it cannot detect this.

Expected fix/test: either bump the ABI and introduce new record/function names,
or preserve the V1 layouts and accept documented older prefixes with defaults.
Compile a consumer object against the `8bc839e` header, then link and run it
against the current library as a compatibility test.

## 2. High — invalid manifest/plan handles can still cause memory-unsafe reads

Controller handles are now safely looked up in a registry, but manifest and plan
entry points still dereference caller pointers to inspect an in-object magic
value before establishing that the pointer denotes a library-owned object:

- `oa_controller_create` reads `manifest->magic` and `manifest->impl`
  (`control/src/c_api.cpp:138-147`);
- plan report/execute read `plan->magic` and `plan->impl` (`:281-286,311-319`);
- manifest/plan destroy do the same (`:131-135,441-445`).

This does not contain invalid, cross-type, or stale handles. A focused ASan probe
passed a one-byte unregistered allocation as `oa_manifest *` to
`oa_controller_create`; it failed with a heap-buffer-overflow reading eight bytes
at `c_api.cpp:141`, rather than returning `OA_EINVAL`. C++ exception handling
cannot catch this memory fault. This is one of the original ABI acceptance cases
(`invalid handles`) and is not exercised by the in-tree tests.

Expected fix/test: validate all opaque handles through ownership registries (or
an equivalently non-dereferencing token scheme), tombstone stale handles under a
documented policy, and test null, arbitrary, cross-type, destroyed, and
destroy-overlap handles for every handle-taking API under ASan/UBSan.

## 3. Medium — cycle-deadline, measured-dwell, and requested stop policy remain unenforced

`cycle_ns` is validated and used to establish a minimum trajectory segment, but
never compared with successive `advance()` timestamps
(`control/src/control_core.cpp:405-407,719-746,793-909`). `advance()` accepts
equal timestamps and arbitrarily large gaps so long as command/producer expiry
has not passed. Each equal-time call still emits another feedback generation,
and completion is three calls/`settle_cycles_`, not a positive monotonic dwell
interval (`:857-893`). Thus a missed control deadline is not faulted and three
zero-time samples can satisfy completion.

The execute request's `stop_kind` is now copied to `active_stop_kind_`
(`:778-789`), but that member is never read; all watchdog/fault paths still use
the same immediate two-arm disable. This leaves the earlier finding only
partially fixed and makes the requested controlled-stop policy ineffective.

Expected test: reject/fault a cycle gap beyond the configured deadline, prove
equal timestamps cannot create new dwell progress, require a configured elapsed
dwell, and verify distinct controlled versus disable reactions (or remove the
unused execution policy until implemented).

## 4. Medium — `oa_controller_create` is not transactional under allocation failure

Creation inserts the new raw token into `registry.active` and only afterward
pushes ownership into the allocating `registry.tokens` vector
(`control/src/c_api.cpp:148-160`). If that vector growth throws `std::bad_alloc`,
`contained()` returns `OA_ENOMEM`, but the active map entry remains while the
local token is freed. This leaks an unreachable live controller and leaves a
dangling key. If a later allocation reuses that address, `emplace` failure is
ignored and the caller can receive a token mapped to the orphaned controller
rather than the one just constructed.

Expected fix/test: make registry publication transactional/rollback-safe and add
deterministic allocation-failure injection at each create allocation boundary,
checking that `*out` is untouched and no active entry or implementation remains.

## Prior-finding disposition

- **Command-as-state simulator:** fixed. Commands feed a bounded independent
  plant; measured state is decoded from quantized feedback frames
  (`control/src/control_core.cpp:149-251,287-312`). Tests observe post-reference
  lag.
- **Fault bypass during arming/idle:** fixed. Fresh, disabled, healthy feedback is
  required, fault state is separate from enable state, and all status codes 8-14
  are tested (`:489-535,543-617,946-969`).
- **Cross-controller plan replay/start drift:** fixed with instance/verify-epoch
  binding, measured start comparison, and a second check at a queued start
  (`:573-601,632-715,749-790,819-825,1173-1184`).
- **Controller concurrency/destroy overlap:** fixed for controller handles via
  registry pinning plus per-controller serialization; TSan stress passes
  (`control/src/c_api.cpp:59-106,404-472`). Manifest/plan invalid-handle safety is
  separately outstanding as Finding 2.
- **Coherent feedback/skew/partial paired send:** fixed. An incomplete generation
  invalidates immediately, skew is enforced, and partial sends latch global
  fault (`control/src/control_core.cpp:287-353,857-883`).
- **Endpoint-only paired IK:** fixed with 17 Cartesian waypoints, predecessor
  seeds, intermediate residual/bounds/protocol/branch/singularity gates, scene
  binding, and synchronized segment trajectories (`:606-717,827-855`). Physical
  execution remains hard-gated and collision defaults to reject-all.
- **Lifecycle/watchdog:** ESTOP, heartbeat, reset-to-CLOSED/full reverify, event
  deadlines, and queued/settling/aborted events are fixed. The narrower
  cycle/dwell/stop-policy gap remains as Finding 3.
- **Mapped protocol span:** fixed at manifest validation and planning
  (`:126-140,567-571,676-685`). No double gearing was introduced.

## Verification evidence

- Release/Werror build and CTest: **2/2 passed**.
- Debug ASan/UBSan build and CTest: **2/2 passed**.
- Debug TSan build and CTest, including concurrent snapshot/event/destroy stress:
  **2/2 passed**.
- Focused invalid-handle ASan probe: **failed as expected**, heap-buffer-overflow
  at `control/src/c_api.cpp:141`.
- Diff and source review against every finding in the prior report and the frozen
  controller design/protocol requirements.
