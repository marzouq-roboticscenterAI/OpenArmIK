# Independent third review: simulator feedback coherence

Reviewed series `514e3a8b6ba21a687d18bd761d17c9412a50804c..e79d613f70e94fb46a828e43baa9cbb560d5b871`.

Disposition: **CLEAN**

## Convergence result

The prior findings do not persist.

- `OA_STOP_DISABLE` and every other authority-reducing transition now retire
  delayed pre-transition history and publish coherent measured stop evidence.
- The base-to-HEAD whitespace check passes.
- The event-ring overflow/disengaged-optional defect is resolved. Publication
  now returns a result, every controller call site terminates on overflow, and
  the triggering public operation returns `OA_EBUSY` instead of continuing with
  cleared execution state.

The exact public-ABI reproduction that previously returned `OA_OK` after
overflow at deferred `OA_EVENT_STARTED` now reports:

```text
return=13 lifecycle=7 before_seq=23 after_seq=24 status=0 t=10000000 event_kind=6 cause=13 event_seq=24 command_match=1
```

`13` is `OA_EBUSY`, lifecycle `7` is `OA_LIFECYCLE_FAULT`, and event kind `6`
is `OA_EVENT_FAULTED`. The transition advances each arm exactly once to a
coherent disabled generation at the triggering time, and the event's sequence
and command ID match the resulting snapshot and failed command. No trajectory
step or optional access occurs after publication fails.

## Event-ring adversarial coverage

Fresh public-ABI capacity tests filled the 64-entry ring exactly and forced
overflow independently at:

- immediate `STARTED`/future `QUEUED` execution publication, including unchanged
  caller command output on failed execute;
- deferred `STARTED` during `advance()`;
- `SETTLING` (the progress-state event) and `COMPLETED`;
- `ABORTED` and `STOPPED` for explicit disable and controlled stops;
- watchdog/fault-event publication through heartbeat;
- `ARMED`, `VERIFIED`, `DISARMED`, and `ESTOP`.

Each path returned `OA_EBUSY`, exposed `OA_LIFECYCLE_FAULT`, retained a final
`OA_EVENT_FAULTED` with `OA_EBUSY`, preserved the triggering command identity,
and materialized a coherent disabled measured state. Reset/reverification after
overflow did not reuse a cleared plan or delayed generation.

The transition-only/value-delay/overflow executable returned:

```text
delay_transitions_event_overflow=PASS
```

## Delay and transition audit

The earlier simulator-feedback conclusions remain sound:

- Complete seven-frame feedback generations are encoded once at capture and
  queued immutably. Values, source timestamp, member mask, and sequence become
  public together only when due.
- Partial delivery changes only the missing-member mask. It cannot leak six new
  joint values under the prior timestamp or sequence, and controller handling
  fails closed.
- The fixed 64-generation rings are allocation-free in the servo path. Checked
  ready-time arithmetic and capacity overflow fail closed without unbounded
  memory or stale publication.
- Delivered timestamps remain monotonic in capture order. Freshness is inclusive
  at its boundary, cross-bus skew uses delivered source timestamps, and current
  plant values cannot appear in an older sample.
- Completion consumes decoded delayed q/dq/FK, requires three distinct paired
  delivered-sequence intervals, and preserves exact historical quantization.
  Equal 20 ms delay at 10 ms cadence delays measured completion by two cycles.
- Explicit disable/controlled hold, disarm, E-stop, producer and cycle watchdogs,
  bus/command failure, delayed partial feedback, motor faults, event overflow,
  reset-to-closed, and destroy/close all retire pending history appropriately.
  No queued enabled generation can publish after an authority-reducing state
  transition.
- Controlled watchdog holds use the last coherent measured q with zero velocity,
  never the current trajectory command. Skew, partial, transport, and motor
  faults override the hold request and materialize disabled evidence.

## Fresh build and runtime evidence

Release:

```text
cmake -S control -B /tmp/openarmik-third-release.txUzDh \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DOA_CONTROL_BUILD_TESTS=ON
cmake --build /tmp/openarmik-third-release.txUzDh -j2
ctest --test-dir /tmp/openarmik-third-release.txUzDh --output-on-failure
Result: 3/3 passed
```

ASan/UBSan:

```text
cmake -S control -B /tmp/openarmik-third-asan.rtjwC0 \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON \
  -DOA_CONTROL_BUILD_TESTS=ON -DOA_CONTROL_SANITIZERS=ON
cmake --build /tmp/openarmik-third-asan.rtjwC0 -j2
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
  ctest --test-dir /tmp/openarmik-third-asan.rtjwC0 --output-on-failure
Result: 3/3 passed, no sanitizer diagnostics
```

ThreadSanitizer:

```text
cmake -S control -B /tmp/openarmik-third-tsan.9hYVZM \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON \
  -DOA_CONTROL_BUILD_TESTS=ON -DOA_CONTROL_TSAN=ON
cmake --build /tmp/openarmik-third-tsan.9hYVZM -j2
TSAN_OPTIONS=halt_on_error=1 \
  ctest --test-dir /tmp/openarmik-third-tsan.9hYVZM --output-on-failure
Result: 3/3 passed, no race diagnostics
```

ABI and static checks:

```text
/tmp/openarmik-third-release.txUzDh/openarm_control_c11_abi
c11_abi_exit=0
/tmp/openarmik-third-release.txUzDh/openarm_control_v1_original_abi
v1_original_abi_exit=0

cppcheck --enable=warning,performance,portability --error-exitcode=1 \
  --inline-suppr --std=c++17 -I control/include -I control/src -I model/include \
  control/src/control_core.cpp control/src/c_api.cpp control/src/kinematics.cpp \
  control/tests/test_control.cpp
Result: no diagnostics

git diff --check 514e3a8b6ba21a687d18bd761d17c9412a50804c \
  e79d613f70e94fb46a828e43baa9cbb560d5b871
Result: passed
```

The public header layout is unchanged by the follow-ups; both the current C11
consumer and frozen original-V1 consumer compile, link, and pass.

No GUI, CAN/network interface, or hardware path was launched or touched.
