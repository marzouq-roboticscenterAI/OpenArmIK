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
