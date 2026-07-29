# Independent review: simulator feedback coherence

Reviewed commit `3e37a9e442c447067da67cc9fcaa9da57d201d46` against
parent `514e3a8b6ba21a687d18bd761d17c9412a50804c`.

Disposition: **FINDINGS**

## Findings

### MEDIUM — `OA_STOP_DISABLE` does not retire delayed pre-stop generations

`Controller::stop()` changes the simulated motor enable state and enters
`OA_LIFECYCLE_DISARMED`, but it neither clears the new feedback queues nor
publishes a materialized stop generation (`control/src/control_core.cpp:1137-1158`).
Subsequent legal `advance()` calls in the disarmed lifecycle therefore publish
queued generations captured while the motors were enabled
(`control/src/control_core.cpp:325-350`, `850-865`). The public snapshot says the
controller is disarmed while every motor status still says enabled, and its
timestamp and sequence continue to advance through pre-stop history.

This is a regression from the parent. With the same 20 ms fault setting, the
parent's timestamp-only implementation publishes disabled status on the first
post-stop advance; the reviewed commit publishes two old enabled generations and
does not expose disabled status until the third post-stop advance. The internal
`set_enabled(false)` means this is primarily a stop-evidence/coherency defect,
not proof that a hardware write remains enabled. It is nevertheless observable
through the public ABI and makes the disable stop weaker than both
`oa_controller_disarm()` and watchdog/fault stops, which correctly call
`force_state()`/`materialize_stop()`, clear history, and publish a coherent stop
generation (`control/src/control_core.cpp:1162-1178`, `1236-1259`).

Public-ABI oracle executed against the reviewed Release library:

```text
t=20000000 lifecycle=2 status=1 sample_t=0 seq=2
t=30000000 lifecycle=2 status=1 sample_t=10000000 seq=3
t=40000000 lifecycle=2 status=1 sample_t=20000000 seq=4
t=50000000 lifecycle=2 status=0 sample_t=30000000 seq=5
```

The same oracle against a fresh archive/build of `514e3a8` produced:

```text
t=20000000 lifecycle=2 status=1 sample_t=0 seq=4
t=30000000 lifecycle=2 status=0 sample_t=10000000 seq=5
t=40000000 lifecycle=2 status=0 sample_t=20000000 seq=6
t=50000000 lifecycle=2 status=0 sample_t=30000000 seq=7
```

Exact scenario: create/verify/arm, configure 20 ms delay on both arms, execute a
joint plan, advance to 10 ms and 20 ms, call
`oa_controller_stop(..., OA_STOP_DISABLE)`, then snapshot at 20 ms and after
advances to 30, 40, and 50 ms. The reviewed oracle was built and run as:

```text
c++ -std=c++17 -O2 -w -Icontrol/include -Icontrol/src -Imodel/include \
  -x c++ - -x none \
  /tmp/openarmik-review-release.VkUKUd/libopenarm_control.a \
  /tmp/openarmik-review-release.VkUKUd/openarm_model/libopenarm_model.a \
  -pthread -o /tmp/openarmik-review-stop-oracle
/tmp/openarmik-review-stop-oracle
```

The corresponding boundary oracle showed that the other stop paths do flush
the queue:

```text
watchdog lifecycle=7 status=0 t=20000000 seq=3
disarm lifecycle=2 status=0 t=0 seq=3
```

Recommended correction: make explicit controlled/disable stops materialize the
appropriate enabled-hold/disabled state just as fault stops do, thereby clearing
pending history before publishing `OA_EVENT_STOPPED`. Add a public-ABI regression
test with nonzero delay.

### LOW — the committed verification claim for `git diff --check` is false

`.swarm/sim_feedback_fix.md:88-89` says `git diff --check` passed, but the actual
review range fails because lines 3 and 4 contain trailing whitespace:

```text
$ git diff --check 514e3a8b6ba21a687d18bd761d17c9412a50804c \
    3e37a9e442c447067da67cc9fcaa9da57d201d46
.swarm/sim_feedback_fix.md:3: trailing whitespace.
.swarm/sim_feedback_fix.md:4: trailing whitespace.
```

The spaces are Markdown hard breaks, but they still contradict the recorded
command result. Remove them or state that the range check intentionally allows
Markdown hard breaks.

## Adversarial validation that passed

The core value-delay correction is otherwise sound in the exercised paths:

- Each plant capture is encoded once into immutable frame bytes with its source
  timestamp and checked ready time. Publication decodes those stored bytes, and
  sequence/timestamp/value become visible together.
- The fixed 64-generation per-arm ring is allocation-free on the servo path,
  preserves capture order, rejects `capture + delay` overflow, and fails closed
  on capacity overflow.
- Incomplete generations change only the latest member mask. They do not mutate
  payloads, timestamps, or sequence; controller-level partial delivery faults
  and materializes a disabled coherent generation.
- Freshness is inclusive at the timeout boundary, stale data do not leak current
  plant values, cross-bus skew is checked from delivered source timestamps, and
  reverse/equal monotonic calls are handled without queue mutation.
- Completion uses decoded delayed q/dq/FK and requires three distinct paired
  delivered-sequence intervals. The 20 ms oracle completes exactly two 10 ms
  cycles after the zero-delay controller and the completion event identifies the
  final delivered sequence.
- Fault/watchdog materialization uses measured feedback rather than the current
  trajectory command, clears delayed history, and retains the intended
  controlled-versus-disabled policy.
- The public C record layout is unchanged by this commit. Both the current C11
  consumer and frozen original-V1 header consumer compile, link, and return 0.

## Fresh verification evidence

Release:

```text
cmake -S control -B /tmp/openarmik-review-release.VkUKUd \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DOA_CONTROL_BUILD_TESTS=ON
cmake --build /tmp/openarmik-review-release.VkUKUd -j2
ctest --test-dir /tmp/openarmik-review-release.VkUKUd --output-on-failure
Result: 3/3 passed
```

ASan/UBSan:

```text
cmake -S control -B /tmp/openarmik-review-asan.s7UbyA \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON \
  -DOA_CONTROL_BUILD_TESTS=ON -DOA_CONTROL_SANITIZERS=ON
cmake --build /tmp/openarmik-review-asan.s7UbyA -j2
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
ctest --test-dir /tmp/openarmik-review-asan.s7UbyA --output-on-failure
Result: 3/3 passed
```

ThreadSanitizer:

```text
cmake -S control -B /tmp/openarmik-review-tsan \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON \
  -DOA_CONTROL_BUILD_TESTS=ON -DOA_CONTROL_TSAN=ON
cmake --build /tmp/openarmik-review-tsan -j2
TSAN_OPTIONS=halt_on_error=1 \
ctest --test-dir /tmp/openarmik-review-tsan --output-on-failure
Result: 3/3 passed
```

Static and ABI checks:

```text
cppcheck --enable=warning,performance,portability --error-exitcode=1 \
  --inline-suppr --std=c++17 -I control/include -I control/src -I model/include \
  control/src/control_core.cpp control/src/c_api.cpp control/src/kinematics.cpp \
  control/tests/test_control.cpp
Result: no diagnostics

/tmp/openarmik-review-release.VkUKUd/openarm_control_c11_abi
c11_abi_exit=0
/tmp/openarmik-review-release.VkUKUd/openarm_control_v1_original_abi
v1_original_abi_exit=0
```

No GUI, CAN/network interface, or hardware path was launched or touched.
