# Commission review corrections

Date: 2026-07-29
Baseline: `ef02cef`
Review: `.swarm/commission_review.md`

## Resolution

All Critical and Important findings from the independent review were corrected.

1. Every public record is checked in two ordered stages: read `struct_size`,
   require the complete eight-byte header, then read `abi_version`; later fields
   are read only after the full V1 size is established. Recipe creation and step
   return `OA_COMMISSION_EABI` for exact four- and eight-byte heap records without
   constructing an action or touching session state.
2. One envelope is established at the start of the first approach and enforced
   through first contact, retreat, reapproach, and second contact. Reapproach
   actions expose only the remaining travel to the original envelope endpoint.
   Phase deadlines are stored once and enforced by the state machine before
   either contact dwell can be accepted.
3. A malformed, stale, enabled, or faulted manual encoder sample latches
   `OA_MANUAL_ABORTED`, clears all accumulated evidence/candidates, and prevents
   review or commit. Stability-only movement/spread still restarts the dwell
   rather than combining discontinuous samples.
4. Reapproach now requires a complete second continuous contact dwell with the
   same low-velocity, torque, minimum-travel, sample-count, deadline, envelope,
   interlock, and energy criteria as the first. Only its averaged stop coordinate
   can enter repeatability.
5. Fault and thermal ceilings are enforced before the WAIT state may emit an
   approach action. Every active motion/dwell state requires the caller-supplied
   drive-enabled state; an unexpected disable aborts and requires a new session.
6. Arm recipes reject simulation mode and bind a nonzero hardware-qualification
   record/revision, fixture record/revision, and exact all-other-joints posture
   mask, coordinates, and tolerance. Every input re-presents those revisions and
   posture. Simulation-only gripper recipes require a distinct simulation record
   and revision, cannot claim hardware qualification, and produce a patch marked
   `OA_EVIDENCE_SIMULATION_ONLY`. Physical patches preserve their admitting
   qualification and fixture revisions.
7. A synchronized, active-only registry maps monotonically issued,
   non-dereferenced token handles to sessions. Destroy erases and frees the
   session; tokens are never reused, counter exhaustion fails closed, and
   arbitrary, stale, double-destroyed, and cross-type values are rejected.
   Output success/failure canaries, short output records, and deterministic
   allocation/exception containment hooks were added.

The product still has no CAN/SocketCAN transport, frame/payload type, register
write, motor enable, `FE` zero, save, flash, netlink, shell, sudo, Python, or ROS
surface.

## Regression coverage

- Exact four-byte and eight-byte heap record ASan regressions for recipe create,
  plus an exact eight-byte recipe-step record and short output record.
- Original review reproductions: reapproach at `0.250 rad`, first contact dwell
  after deadline, manual enabled interruption between stable samples, and hot
  WAIT authorization.
- First and second contact dwell evidence loss, unexpected active-state disable,
  stale/faulted manual interruptions, revision/fixture/posture mismatches, and
  contradictory arm hardware+simulation recipes.
- Explicit abort and injected drive fault from every nonterminal recipe state;
  every path verifies commit rejection and unchanged caller patch.
- Arbitrary, stale, double-destroyed, and cross-type handles; successful guarded
  outputs; injected allocation failure and C++ exception containment; strict C11
  consumer.

## Fresh verification

```text
GCC 15: -Wall -Wextra -Wpedantic -Werror
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1
UBSAN_OPTIONS=halt_on_error=1
ctest: 2/2 passed
cppcheck warning/performance/portability: clean
CMake install/export: passed
git diff --check: clean
```

No physical transport was opened and no hardware command was emitted.
