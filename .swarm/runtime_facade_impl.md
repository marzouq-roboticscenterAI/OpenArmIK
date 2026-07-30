# Unified runtime facade implementation

Date: 2026-07-29 (America/Los_Angeles)
Branch: `feat/unified-runtime`
Disposition: **DONE_WITH_CONCERNS**

## Delivered boundary

The new `openarm_runtime` C++17 library installs `openarm_runtime.h`, the
`openarm_runtime` CMake package, and `OpenArm::Runtime`. Its ISO-C V1 surface
uses module-prefixed fixed-width statuses and records, explicit SI/body-frame/
orientation/monotonic-clock identifiers, source-facility error detail, and
typed monotonic registry tokens for manifests, runtimes, inventories,
calibration sessions, and plans. Tokens are never dereferenced or reused;
cross-type, stale, overlapping destroy, and double-destroy operations fail
closed.

The runtime composes the existing public control, commission, CAN, transport,
and model dependencies. It does not embed another controller, kinematics
solver, calibration state machine, SocketCAN backend, or CAN codec.

### Stage A

- The built-in immutable virtual manifest has exactly two named virtual buses,
  two arms, and fourteen motors with canonical joint names, pinned family/
  profile/gear/limits, immutable serials, simulation evidence, model revision,
  and manifest revision.
- The virtual inventory reports exactly two interfaces and fourteen exact
  candidates. Physical evidence never receives a side or joint assignment.
- A runtime-owned cadence drives the existing encoder-authoritative controller.
  The facade wraps snapshots, joint coordinates, body-frame joint axes/origins,
  named TCP transforms/XYZ, arming, joint planning, paired body-frame
  position-only TCP planning, execution, heartbeat, stop/disarm, and correlated
  events without changing the control V1 ABI.
- Plans pause the owner cadence between plan publication and execute/destroy so
  the lower controller's exact feedback-sequence binding cannot race the
  facade-owned cadence. Execution resumes the cadence and completion remains a
  measured controller event.
- Collision remains unchecked and `motion_authorized` remains false. Motion is
  available only for an explicitly opted-in virtual runtime. All physical
  configuration, calibration actuation, enable, motion, and collision-validated
  capability bits are false.
- Manual and supervised calibration acquire an exclusive runtime lease. The
  facade supplies encoder samples from coherent runtime snapshots and delegates
  state/evidence checks to `openarm_commission`. Only a successfully committed
  lower-module patch can be applied; exact revision/side/joint/serial/evidence
  checks produce a new immutable manifest and structured preview. No in-place
  manifest mutation or motor zero/save occurs.
- Persistence is a bounded, deterministic, non-executable, fixed-order UTF-8
  format with an in-file SHA-256. The parser rejects missing/extra/reordered
  records, malformed or non-finite values, overlong data, wrong cardinality,
  and digest mismatch before handle publication. Authorized writes use an
  exclusive same-directory temporary, file `fsync`, atomic rename, and directory
  `fsync`. Absolute traversal and symlink paths are rejected.

### Stage B

- Local interface listing delegates to the existing read-only rtnetlink API.
  An unavailable requested interface deterministically returns an immutable
  empty inventory.
- Candidate queries require an explicit interface plus explicit send/receive-ID
  allowlist. The only constructed/transmitted frame is
  `oa_can_make_register_query_typed`; public `oa_transport_send` independently
  enforces query-only authority. The queried schema covers serial, hardware,
  software, sub-version, send/master IDs, mode, bitrate, timeout, P/V/T, gear,
  and direction.
- Physical records report raw presence/timestamps/values, remain
  `unresolved_assignment`, and are always marked ambiguous because dequeue-time
  correlation cannot exclude queued replies or same-ID responders. They never
  become an arm mapping or armable manifest. Physical apply returns
  `OA_RUNTIME_EUNSUPPORTED` without transport activity.

## Verification

- Fresh standalone GCC 15.2 Release/Werror build: PASS.
- Release CTest: 2/2 PASS, including strict C11 and the C++ end-to-end suite.
- Debug ASan+UBSan with leak detection and halt-on-error: 2/2 PASS.
- Debug ThreadSanitizer with halt-on-error: 2/2 PASS.
- `cppcheck --enable=warning,performance,portability --error-exitcode=1`: PASS.
- Installed package consumed from a separate fresh CMake project by strict C11
  and C++17 all-six-header executables: PASS.
- Header declaration/export comparison: all 42 `oa_runtime_*` functions are
  defined.
- Installed archive inspection: no runtime test hook is present. Its only CAN
  frame-builder dependency is `oa_can_make_register_query_typed`.
- `git diff --check` and shell syntax checks: PASS.

The compiled tests cover virtual inventory/configuration, manifest
round-trip/corruption/traversal/symlink/authorization, manual patch application,
supervised recipe ownership/action preview, stale/cross-type/ABA-safe lifetime,
joint and paired XYZ measured completion, unchecked-collision truth, absent
physical interface, physical gates, interlock faulting, overlapping calls and
destroy, and query capability truth.

## Remaining concerns

- The worktree has no pinned `upstream/openarm_description` checkout, so the
  repository-wide `scripts/build_native.sh`/`build.sh` path could be updated and
  syntax-checked but not executed end to end without fetching external source.
  Runtime itself and its installed consumer were built against the previously
  verified installed five-module SDK.
- No live CAN interface was opened and no physical frame was sent. The physical
  query orchestrator is compile-reviewed and structurally query-only, but its
  real SocketCAN timing/evidence behavior remains hardware-gated.
- Allocation exceptions are contained at the C boundary and sanitizer paths are
  clean, but the runtime does not add a production-visible allocation-failure
  injection hook. Existing lower-module test hooks remain absent from the
  installed archive.
- The facade intentionally does not claim single-arm Cartesian motion,
  continuous collision validation, jerk-qualified plant truth, physical
  identity assignment, physical calibration action, or physical motion.
