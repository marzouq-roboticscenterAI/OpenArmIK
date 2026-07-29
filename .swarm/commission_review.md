# Independent re-review: commissioning/calibration (`125ce14`)

Verdict: **CHANGES REQUIRED — one Important finding remains**

## Remaining finding

### Important — stale-handle protection retains every session allocation until process exit

The registry prevents ABA by never erasing an entry and never deleting its
opaque wrapper on destroy. `retire()` only resets the implementation and marks
the entry inactive (`commission/src/c_api.cpp:72-82,275-284,425-434`); both the
wrapper and `unordered_map` tombstone are released only by the registry's static
destructor (`c_api.cpp:42-52`). Consequently every create/destroy cycle grows
resident process state permanently.

An independent optimized stress repro that created and destroyed 500,000 manual
sessions completed with approximately 39,980 KiB maximum RSS. The same repro
under ASan reached approximately 295 MiB. LeakSanitizer reports no end-of-process
leak because the static destructor eventually frees the tombstones, but a
long-running process can be driven to unbounded memory growth before exit.

The ABA requirement is satisfied, but through permanent quarantine rather than
a reclaimable generation/token design. This leaves the C handle-lifetime fix
unsuitable for an unbounded-lifetime library process.

## Original findings re-reviewed

All seven original findings are otherwise corrected:

- Exact four- and eight-byte short records now return `OA_COMMISSION_EABI`
  before later fields are read. The original exact 8-byte ASan recipe-create
  repro exits with `OA_COMMISSION_EABI` and no sanitizer finding; short recipe
  step and output cases also pass.
- Manual stale, enabled, faulted, or malformed samples latch
  `OA_MANUAL_ABORTED`, clear evidence, and permanently exclude commit. Stability
  movement/spread correctly restarts rather than combines a dwell.
- First approach, contact dwell, retreat, reapproach, and second dwell share the
  original qualified envelope; both motion deadlines are enforced in the state
  machine. The original `0.250 rad` retry and late-dwell repros now abort.
- Both contacts require consecutive sample count and dwell evidence before
  repeatability.
- Temperature is checked before WAIT can emit APPROACH, and every active motion
  state requires enabled feedback plus E-stop/deadman interlocks.
- Arm recipes reject simulation mode and require qualification/fixture revisions
  plus the complete other-joint posture; bindings are rechecked on every active
  step. Simulation-only gripper patches carry a distinct evidence kind, record,
  and revision.
- Arbitrary, stale, double-destroyed, and cross-type handles are rejected without
  dereference; use/destroy are serialized. The remaining issue is the unbounded
  tombstone retention described above.

## Verification evidence

- Fresh GCC 15 Debug build with `-Wall -Wextra -Wpedantic -Werror`, ASan, and
  UBSan: passed.
- `ctest --output-on-failure`: 2/2 passed, including the strict C11 consumer,
  canaries, exact short records, injected allocation/exception failures, all
  original adversarial state-machine cases, posture/revision mismatches, and
  stale/cross-type handles.
- Exact original standalone short-record ASan repro: no sanitizer issue; returned
  `OA_COMMISSION_EABI`.
- Independent optimized handle stress repro: 500,000 create/destroy cycles,
  approximately 39,980 KiB maximum RSS.
- No FE/save/flash/register-write/transport surface was introduced.

No source edits were made; this report is the only review update.
