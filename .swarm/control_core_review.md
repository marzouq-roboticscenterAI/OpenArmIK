# Final stop-policy review of `27cfa72`

Verdict: **CHANGES REQUIRED — one Medium eligibility edge remains.** The commit
correctly adds encoder-visible, two-arm controlled holds for coherent producer/
command/cycle watchdog stops and disable fallback for ordinary stale-feedback,
CAN/partial/skew, and motor-fault paths. Release, ASan/UBSan, and TSan pass 3/3.

## Medium — timeout is declared controllable without checking feedback coherence or motor health

The missed-cycle gate updates `now_ns_` and calls
`latch_fault(OA_ETIMEOUT, true)` unconditionally before checking `fresh()` or
`healthy()` (`control/src/control_core.cpp:805-825`). Command expiry likewise
passes unconditional availability before the later health check (`:829-842`).
`latch_fault` therefore retains enabled hold whenever the request selected
`OA_STOP_CONTROLLED` (`:1166-1182`).

Two unsafe combinations remain:

- With the configured 10 ms cycle and 50 ms feedback timeout, an initial
  `advance(60 ms)` is both a missed cycle and stale feedback. It is classified as
  controllable timeout, holds the last stale q enabled, and synthesizes a new
  complete feedback generation instead of taking the required disable fallback.
- If a motor fault is injected before an advance that also misses its cycle or
  crosses command expiry, timeout wins before `healthy()` is evaluated. The
  fault status is preserved, but the controller still selects enabled hold rather
  than motor-fault disable fallback.

Expected fix/test: compute controlled-stop availability from a coherent, fresh,
fault-free pre-stop snapshot (or prioritize feedback/motor-integrity faults over
timeout). Add controlled-request tests for a cycle gap beyond feedback timeout
and for simultaneous motor-fault-plus-timeout; both arms must show disabled,
quantized-zero dq while the event and fault status preserve the original cause.

## Verified closed behavior

- Coherent producer expiry, command expiry, and ordinary missed cycle materialize
  zero-dq feedback on both arms; controlled requests retain enabled hold and
  disable requests report disabled.
- Missing feedback, partial send, cross-bus skew, and motor status faults normally
  force two-arm disable with quantized-zero dq. Motor fault status/fault mask and
  event cause remain visible.
- `materialize_stop` preserves measured q and fault status while emitting decoded
  feedback (`control/src/control_core.cpp:324-332,1166-1182`).
- All earlier findings remain closed: frozen old-V1 compatibility/defaults;
  typed arbitrary/cross/stale/destroy-overlap handle safety; active-only monotonic
  token storage; allocation transactionality; positive dwell/cycle deadline;
  encoder-independent simulation; arming gates; plan ownership/start drift;
  coherent feedback/skew/paired stop; waypoint IK; mapping/no double gearing;
  collision and physical hard gates; lifecycle, heartbeat, reset, and events.

## Fresh verification

- Release/Werror CTest: **3/3 passed**.
- ASan/UBSan CTest: **3/3 passed**.
- TSan CTest: **3/3 passed**.
- Cause-specific suite covers producer expiry, command expiry, missed cycle,
  missing feedback, partial send, skew, and motor faults for encoder-visible
  two-arm results; the two combined eligibility cases above are not covered.
