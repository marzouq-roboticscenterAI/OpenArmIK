# Final independent review of `b1668b2`

Verdict: **CHANGES REQUIRED — one Medium finding remains.** The final corrective
commit closes both prior High C-ABI findings, allocation transactionality,
cycle-deadline/positive-dwell behavior, and all earlier controller-core findings.
Release, ASan/UBSan, and TSan suites pass 3/3. The only remaining issue is an
incomplete controlled-stop reaction for non-timeout faults.

## Medium — controlled stop does not stop on producer, CAN, or motor faults

`latch_fault()` recognizes `OA_STOP_CONTROLLED`, retains enabled state, and
clears measured velocity only when the cause is exactly `OA_ETIMEOUT`
(`control/src/control_core.cpp:1157-1173`). For the other execution fault paths:

- producer/watchdog expiry calls `latch_fault(OA_ESTALE)` (`:821-824`);
- partial command send or cross-bus skew calls `latch_fault(OA_ECAN)`
  (`:875-892`);
- injected/motor status faults call `latch_fault(OA_EFAULT)` (`:830-833,895-898`).

With a controlled-stop request, those paths only leave the simulator internally
enabled. They do not issue a zero-velocity hold/deceleration state, and they do
not fall back to disable. The controller then enters `FAULT`, where `advance()`
cannot progress a deceleration. The last measured `dq` may consequently remain
nonzero indefinitely. This is not a controlled stop and is less safe than a
documented disable fallback when the bus/fault cause makes controlled motion
unavailable.

The new stop-policy test covers only a missed-cycle `OA_ETIMEOUT`
(`control/tests/test_control.cpp:969-999`), so it does not expose this distinction.

Expected fix/test: define cause-aware stop behavior. When communication remains
healthy, generate and verify a bounded controlled stop/hold before latching the
terminal state; when CAN or motor faults make that impossible, explicitly fall
back to two-arm disable. Test `OA_STOP_CONTROLLED` separately for producer stale,
command expiry, missed cycle, partial send/skew, and motor-fault causes, asserting
measured zero velocity or disabled status as appropriate.

## Closed findings verified

- **Frozen 8bc V1 ABI:** current paired layout preserves the original collision
  revision offset, all three extended inputs accept their original prefix sizes,
  and missing tails receive explicit defaults
  (`control/include/openarm_control.h:176-190,291-299`;
  `control/src/c_api.cpp:29-42,261-303,403-425,505-516`). A consumer compiled only
  against the frozen original header successfully creates, verifies, arms,
  paired-plans, injects a legacy fault, and destroys against the current library.
- **All handle types:** manifest, controller, and plan handles are typed registry
  tokens and caller addresses are never dereferenced
  (`control/src/c_api.cpp:58-193`). Arbitrary, cross-type, stale, concurrent-use/
  destroy, and post-destroy probes return defined errors under ASan/UBSan/TSan.
- **Token memory/ABA/exhaustion:** registries retain only active slots; one global
  monotonic 16-byte-stride token source prevents cross-type and stale ABA; integer
  exhaustion throws `bad_alloc` and is contained as `OA_ENOMEM` (`:78-98`).
  Repeated 4,096-controller and 4,096-plan create/destroy stress returns active
  counts to zero without material RSS growth.
- **Allocation transactionality:** controller publication has no throwing step
  after insertion, checks collision, and deterministic failures at all three
  pre-publication boundaries leave output and active registry untouched
  (`:153-165,285-302`). Manifest/plan publication likewise performs no throwing
  operation after successful registry insertion.
- **Cycle/dwell:** armed cycle gaps greater than `cycle_ns` latch timeout; equal
  timestamps do no work; completion requires three positive cycle intervals in
  measured q/dq/FK tolerance (`control/src/control_core.cpp:795-822,904-927`).
- **Original six Highs:** independent plant/decoded encoder truth, fault-free
  arming, controller/epoch-bound plans with start recheck, serialized lifetime,
  coherent generation/skew/paired stop, and waypoint/predecessor-seeded IK path
  gates remain intact. Mapping/no-double-gearing, physical hard gates, collision
  reject-all default, reset/reverify, heartbeat, events, and PMAX intersection
  also remain intact.

## Fresh verification

- Release/Werror CTest: **3/3 passed**.
- ASan/UBSan CTest: **3/3 passed**.
- TSan CTest: **3/3 passed**.
- Included focused coverage: old 8bc V1 header consumer; arbitrary/cross/stale
  manifest/controller/plan handles; destroy overlap; allocation failure; active
  registry/RSS stress; missed cycle; equal-time dwell; timeout controlled versus
  disable; and all prior safety regressions.
