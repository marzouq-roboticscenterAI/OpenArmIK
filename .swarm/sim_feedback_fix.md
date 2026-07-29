# Simulator feedback coherence fix

Date: 2026-07-29 (America/Los_Angeles)
Branch: `fix/sim-feedback-coherence`
Disposition: **DONE**

## Result

`oa_sim_fault.feedback_delay_ns` now delays immutable, already-quantized
seven-motor feedback generations. Each capture retains its frame bytes, source
timestamp, member mask, and overflow-checked ready time. Values, timestamps, and
feedback sequence therefore become public together only when the generation is
due.

Each arm owns a fixed 64-generation ring allocated with the controller. The
servo path performs no queue allocation. Ready generations are published in
capture order, delay changes affect only later captures, timestamp order cannot
regress, and arithmetic or capacity overflow returns transport failure so the
controller latches fault. Forced simulator state, verification, disarm, and
materialized fault stops clear pending history and publish a new complete
generation.

An incomplete generation publishes only its missing-member mask. It does not
decode any member, change the last complete timestamp, or advance the sequence.
The controller consequently fails closed without exposing six current joints
under an older generation identity. The subsequent materialized disable is
itself a complete sequence-advancing generation.

Completion dwell now requires three intervals backed by distinct complete
delivered sequences from both arms as well as the original elapsed-time test.
Repeated advances over one still-fresh delayed sample cannot complete a motion.
The public C record layout and symbol set are unchanged; the frozen original-V1
header consumer continues to compile, link, and pass.

## Adversarial coverage

The control tests now cover:

- a public-ABI zero-delay versus 20 ms-delay oracle at a 10 ms cadence, with
  exact historical q/dq/tau, raw decoded values, status, temperature, timestamp,
  and sequence equality after pipeline fill;
- payload inequality against current feedback during motion, exact two-cycle
  completion delay, and completion-event sequence provenance;
- 15 ms non-grid delay, proving the 10 ms capture is absent at 20 ms and first
  public at 30 ms with its original 10 ms timestamp;
- immediate and delayed partial generations with unchanged payload, timestamp,
  and sequence, plus the exact missing-member mask;
- asymmetric per-arm delay causing the existing cross-bus fail-closed path;
- retained ready times across delay mutation and ordered publication;
- inclusive freshness at exactly the timeout and stale classification one
  nanosecond later without current-value leakage;
- checked `capture + delay` overflow and deterministic 64-entry ring overflow;
- delayed completion with a delivery gap, proving dwell needs three distinct
  newly delivered paired sequences; and
- unchanged DaMiao quantization through exact delayed/history comparisons.

## Verification

All builds were freshly configured outside the source tree.

```text
cmake -S control -B /tmp/openarmik-feedback-build-release \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DOA_CONTROL_BUILD_TESTS=ON
cmake --build /tmp/openarmik-feedback-build-release -j2
ctest --test-dir /tmp/openarmik-feedback-build-release --output-on-failure
Result: 3/3 passed

cmake -S control -B /tmp/openarmik-feedback-build-asan \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON \
  -DOA_CONTROL_BUILD_TESTS=ON -DOA_CONTROL_SANITIZERS=ON
cmake --build /tmp/openarmik-feedback-build-asan -j2
ctest --test-dir /tmp/openarmik-feedback-build-asan --output-on-failure
Result: 3/3 passed under AddressSanitizer and UndefinedBehaviorSanitizer

cmake -S control -B /tmp/openarmik-feedback-build-tsan \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON \
  -DOA_CONTROL_BUILD_TESTS=ON -DOA_CONTROL_TSAN=ON
cmake --build /tmp/openarmik-feedback-build-tsan -j2
ctest --test-dir /tmp/openarmik-feedback-build-tsan --output-on-failure
Result: 3/3 passed under ThreadSanitizer

cppcheck --enable=warning,performance,portability --error-exitcode=1 \
  --inline-suppr --std=c++17 -I control/include -I control/src -I model/include \
  control/src/control_core.cpp control/src/c_api.cpp control/src/kinematics.cpp \
  control/tests/test_control.cpp
Result: passed, no diagnostics

git diff --check
Result: passed
```

No Python runtime, ROS GUI, CAN/network interface, or hardware path was used.

## Residual risk

The delay ring intentionally has a fixed 64-generation bound. A caller that
combines a very long feedback timeout with a delay exceeding 64 control captures
will receive fail-closed `OA_ECAN` rather than unbounded buffering. At the
standard 10 ms cycle this bounds queued history to 640 ms; ordinary delays
beyond the standard 50 ms feedback timeout fail freshness first.

## Review-gate correction

Independent review of commit `3e37a9e442c447067da67cc9fcaa9da57d201d46`
found that explicit `OA_STOP_DISABLE` changed the internal enable state without
retiring queued pre-stop generations. The same audit identified equivalent
immediate-snapshot exposure in E-stop and event-overflow transitions, while
reset-to-closed relied on a later verification to retire history.

All authority-reducing transitions now use one coherent materialization rule.
When a verified measured generation exists, the transition preserves its q,
encodes zero velocity with the requested enabled-hold or disabled status, clears
the delay ring, advances the feedback sequence, stamps the transition time, and
only then publishes the transition event. Motor fault status remains
authoritative over the enabled bit. An unverified closed controller retires
history without making invalid state fresh or incrementing its sequence.

Public-ABI regressions configure 20 ms paired delay and exercise explicit
disable and controlled stops, disarm, E-stop, reset/close/reverify, producer
watchdog expiry, missed-cycle timeout, command transport failure, delayed
partial generation, motor fault, and event-queue overflow. They assert the
immediate transition snapshot, the next two no-delivery snapshots, later
post-stop feedback, event sequence provenance, measured-q preservation, disabled
or enabled-hold policy, and retained motor-fault evidence.

Fresh follow-up verification used `/tmp/openarmik-feedback-gate2-release`,
`/tmp/openarmik-feedback-gate2-asan`, and
`/tmp/openarmik-feedback-gate2-tsan` with the same CMake options documented
above. Release, ASan/UBSan with leak/error halting, and TSan with error halting
each passed 3/3 registered tests. Both ABI executables were also run directly
from the fresh Release build and returned zero. Cppcheck and the staged
base-to-HEAD whitespace check passed without diagnostics.

## Re-review event-overflow correction

Re-review of `1d2b9478b91449ccafdcb52fbea0b35f263e40fb` found that
`publish()` could clear `executing_` on event-ring overflow while its caller
continued. The deferred STARTED path then dereferenced a disengaged
`std::optional<MotionPlan>` and returned `OA_OK` despite the public lifecycle and
event both reporting FAULT.

Controller event publication now returns an explicit success result. Every
caller—verification, arm, immediate or queued execution, deferred start,
settling, completion, E-stop, abort, stop, disarm, and fault latching—terminates
its operation on overflow. The triggering public operation returns `OA_EBUSY`,
the delayed queue is retired, the measured plant is coherently disabled, and the
ring contains `OA_EVENT_FAULTED` with `OA_EBUSY` and the triggering command ID.
Fault latching returns its original nonzero cause when its event fits and
`OA_EBUSY` when publication itself overflows.

Public-ABI tests fill the ring exactly and force overflow independently at
QUEUED execute, deferred STARTED, SETTLING, COMPLETED, ABORTED, STOPPED,
heartbeat fault, ARM, VERIFY, DISARM, and E-stop events. They assert non-success
returns, unchanged caller output on failed execute, disabled FAULT snapshots,
coherent timestamps/sequences, measured-q preservation, and fault-event cause,
lifecycle, sequence, and command identity.

Fresh final verification used `/tmp/openarmik-feedback-gate3-release`,
`/tmp/openarmik-feedback-gate3-asan`, and
`/tmp/openarmik-feedback-gate3-tsan`. Release, ASan/UBSan with leak/error
halting, and TSan with error halting each passed 3/3 registered tests. Both ABI
executables returned zero when run directly from the Release build. Cppcheck
and the complete base-to-HEAD whitespace check passed without diagnostics.
