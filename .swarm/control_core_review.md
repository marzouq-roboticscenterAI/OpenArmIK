# Independent review of `8bc839e`

Verdict: **CHANGES REQUIRED**. Release and ASan/UBSan builds pass, and the
physical backend does fail closed before verification or traffic, but the new
Stage-A core does not yet meet several non-negotiable controller contracts.

## 1. High — the simulator makes commands become measured encoder state

`Controller::advance()` computes the trajectory reference and passes it directly
to `ArmRuntime::sample()` (`control/src/control_core.cpp:588-622`). `sample()`
passes that reference to every motor, and `DamiaoMotorSimulator::update()` writes
it immediately into `MeasuredMotor::raw_q/raw_dq` (`:147-160,201-211`). Thus,
outside the special freeze injection, feedback is exactly the current command;
there is no independently evolving plant/encoder observation. The convergence
tests reinforce this shortcut by asserting those mirrored values
(`control/tests/test_control.cpp:290-316,423-455`). This cannot validate tracking,
following error, delayed/stalled dynamics, or measured completion and violates
the explicit requirement that fake feedback not let command targets masquerade
as measurements.

Expected test: a simulated plant with its own state and dynamics must lag the
reference; advancing reference alone must not change measured state, and only
independently generated encoder samples may satisfy q/dq/TCP dwell completion.

## 2. High — active faults do not gate arming and can be erased by arming

`fresh()` checks only timestamps/masks and ignores motor status/fault masks
(`control/src/control_core.cpp:808-810`). A fault injected while DISARMED sets
status 8, but `challenge()` still succeeds (`:357-371`) and `arm()` then calls
`set_enabled(true)`, overwriting every status byte with 1 before sampling
(`:374-395,135-145`). The controller can therefore arm through an active fault
and make the fault disappear. The same missing gate permits planning/execution
from ARMED_IDLE with a newly injected fault until the next `advance()` notices
it. Arming was required to consume fresh, legal, **disabled and fault-free**
feedback.

Expected tests: inject every status fault before challenge, between challenge and
arm, and while idle; arm/plan/execute must fail closed, latch the fault, and
disable both arms without overwriting the originating status evidence.

## 3. High — plans are replayable across controllers and start drift is not checked

`MotionPlan` carries revisions and per-controller sequence numbers, but no
controller/verification-epoch identity. `execute()` checks only manifest/model
revisions and equality of the two sequence counters (`control/src/control_core.cpp:545-557`);
it never compares the current measured q against `plan.start_q`. Two controllers
with the same manifest and coincident sequence counters can therefore accept one
another's plans even when their measured poses differ. A future-start command is
also not revalidated at its actual start epoch. This defeats immutable plan
ownership and the mandatory immediate pre-execution start-pose drift check.

Expected tests: create two controllers with equal revisions/sequences but
different measured poses and prove cross-controller execution is rejected;
likewise alter measured q after planning/queuing and require rejection before the
first command cycle.

## 4. High — public controller operations have data races and unsafe lifetime overlap

Every C entry point directly reads or mutates one unsynchronized `Controller`;
the class contains mutable lifecycle, optional plan, simulator state, and event
ring with no mutex/atomics (`control/src/control_core.hpp:112-167`,
`control/src/c_api.cpp:103-357`). Concurrent `advance`, `snapshot`, `stop`, or
`poll_event` calls therefore have C++ data races/undefined behavior. Concurrent
destroy versus any call can additionally dereference freed handle/implementation
memory. Neither the public header nor README declares an external serialization
and destruction-quiescence contract. This is incompatible with a coordinator,
watchdog, event consumer, and stop path operating concurrently.

Expected test: a TSan stress test with advance/snapshot/event/stop from separate
threads, plus a specified and enforced close/quiescence protocol. Either provide
internal synchronization/lifetime pinning or make a deliberately single-threaded
API contract explicit and redesign asynchronous use around one owner thread.

## 5. High — feedback is freshness-window aggregation, not a coherent complete generation; skew is unenforced

When one motor is dropped, the other six are updated but `feedback_seq_` merely
stops incrementing (`control/src/control_core.cpp:201-211`). The dropped motor's
old sample remains in `fresh_mask` until the timeout (`:214-242`), so the next
incomplete generation is treated as complete and usable during that window.
This also permits mixed-age joint vectors rather than an immutable complete
snapshot. `max_cross_bus_skew_ns` is reported (`:314-326`) but never compared to
the configured maximum anywhere; no partial-send/skew injection or paired-cycle
enforcement exists. Consequently the requested immediate incomplete-feedback,
cross-bus-skew, and paired stop-both gates are absent (the existing drop test
waits 60 ms specifically to age out the stale sample at
`control/tests/test_control.cpp:345-370`).

Expected tests: the first missing member of a generation must invalidate the
whole generation and fault before another command; mixed timestamps must be
rejected, excessive bus skew and a partial paired send must latch fault and stop
both arms.

## 6. High — paired TCP planning is endpoint IK plus joint interpolation, not a validated Cartesian path

`plan_paired()` performs exactly one IK solve per arm from the measured initial
seed (`control/src/control_core.cpp:451-505`), then execution interpolates a
single straight segment in joint coordinates (`:588-618`). There are no Cartesian
waypoints, predecessor-waypoint seeds, branch-jump tests, singularity policy, or
path residual checks. `collision_scene_revision` is caller-supplied metadata with
no controller scene dependency to revalidate, and the virtual unchecked policy
is the sole planning path. Endpoint FK/IK residual and URDF bounds are correctly
checked, but they do not satisfy the adjudicated path-planning contract.

Expected tests: multi-waypoint TCP paths seeded from measured q then predecessor
solutions, with explicit rejection of branch jumps, singularities, intermediate
residual/bounds failures, and changed collision-scene revision.

## 7. Medium — several lifecycle/watchdog contracts are present only nominally

There is no route to `OA_LIFECYCLE_ESTOP`; fault reset clears injection and
resamples the current simulator state rather than performing the specified full
reverification (`control/src/control_core.cpp:740-756`). `poll_event` ignores its
deadline (`control/src/c_api.cpp:324-341`). The producer deadline is immutable,
must be at least command expiry (`control/src/control_core.cpp:559-564`), and has
no heartbeat/update operation, so it cannot detect a stalled producer earlier
than command expiry. `oa_execute_request.stop_kind` is validated but never stored
or used. The event model also omits the specified queued/settling/aborted states.

Expected tests: ESTOP/interlock loss and physical-reset semantics, reset requiring
a new verification pass, real deadline wait behavior, producer-heartbeat loss
before motion expiry, and observable queued/settling/aborted transitions.

## 8. Medium — a validated manifest can map legal model angles outside the verified protocol span

Manifest validation checks `abs(q_scale)==1`, the codec PMAX, model joint bounds,
and only that the offset itself lies in `[-PMAX,+PMAX]`
(`control/src/control_core.cpp:94-121`). It does not require the full commissioned
joint interval, inverse-mapped by `(q-b)/a`, to fit the raw output span. For
example `a=1,b=12.5` passes, while a legal negative left-joint target maps below
`-12.5` rad. Planning checks only model/mechanical bounds (`:423-446`), and the
simulator accepts the impossible raw value. This would eventually clip/reject at
the codec boundary and invalidates plan feasibility.

Expected test: both endpoints of every joint's commissioned interval must
inverse-map within PMAX (and dynamic/effort/thermal operational limits need
equivalent validated intersections before physical enablement).

## Test/ABI observations

The ISO-C declarations, opaque shared manifest ownership, exception wrapper, and
temporary-before-output pattern are directionally correct; valid output writes
are limited to `sizeof(record)`, preserving trailing caller storage. The physical
backend returns `OA_EUNSUPPORTED` before sampling or traffic, and collision
defaults to reject-all.

However, the C11 test only checks two null/short-record calls
(`control/tests/c11_abi_consumer.c:11-22`), and the C++ "canary" test calls
`snapshot(NULL, ...)`, so it exits on the invalid handle without exercising an
output write (`control/tests/test_control.cpp:254-261`). The required wrong/short
records across all APIs, wrong/cross-type/stale handles, successful and failing
output canaries, allocation/exception injection, event overflow, and close/lifetime
tests are not present. Passing ASan/UBSan therefore does not establish the stated
ABI acceptance criteria.

## Verification performed

- Release/Werror build and CTest: 2/2 passed.
- Debug ASan/UBSan build and CTest: 2/2 passed.
- Independent source review against the adjudicated design, design critic, and
  both hardware/protocol reports.
