# Independent re-review: simulator feedback coherence

Reviewed series `514e3a8b6ba21a687d18bd761d17c9412a50804c..1d2b9478b91449ccafdcb52fbea0b35f263e40fb`, including the follow-up
`1d2b947` for the findings against `3e37a9e`.

Disposition: **FINDINGS**

## Finding

### MEDIUM — event overflow during queued-command start faults the controller but `advance()` continues through a destroyed plan and returns success

`Controller::advance()` publishes `OA_EVENT_STARTED` and then immediately uses
`executing_` to obtain waypoint data (`control/src/control_core.cpp:903-939`). If
the event ring is full, `publish()` resets `executing_`, clears `command_id_`,
materializes a disabled stop, and changes the lifecycle to fault
(`control/src/control_core.cpp:1220-1242`). `advance()` does not check that state
change. It dereferences the now-disengaged `std::optional<MotionPlan>`, performs
another simulator step after the fault transition, and returns `OA_OK`.

A public-ABI ASan/UBSan oracle filled the 64-event ring with 21 immediate
execute/controlled-stop cycles (three events each), queued a future-start joint
command as the 64th event, and advanced to its start time with 20 ms feedback
delay configured. The result was:

```text
advance=0 lifecycle=7 before_seq=23 after_seq=24 status=0 t=10000000
last_event_kind=6 cause=13 lifecycle=7 event_seq=24 command=22
```

Thus the call reports `OA_OK` (`0`) while its snapshot and final event report
`OA_LIFECYCLE_FAULT` (`7`) and `OA_EVENT_FAULTED` (`6`) with `OA_EBUSY` (`13`).
The optional storage happened to retain readable bytes in this build, so the
sanitizers did not diagnose the C++ lifetime violation; dereferencing a
disengaged optional is nevertheless undefined behavior.

Exact compile/run command:

```text
c++ -std=c++17 -O1 -g -fsanitize=address,undefined \
  -fno-omit-frame-pointer -Icontrol/include -Icontrol/src -Imodel/include \
  -x c++ - -x none \
  /tmp/openarmik-rereview-asan.VDPoWT/libopenarm_control.a \
  /tmp/openarmik-rereview-asan.VDPoWT/openarm_model/libopenarm_model.a \
  -pthread -fsanitize=address,undefined \
  -o /tmp/openarmik-rereview-event-overflow
ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
  /tmp/openarmik-rereview-event-overflow
```

The checked-in event-overflow regression reaches overflow from E-stop, whose
caller does not subsequently dereference `executing_`; it therefore does not
cover this path. Make `publish()` report overflow and require every caller to
terminate its operation immediately, or explicitly test lifecycle/plan validity
after each publication that can fault.

## Resolution of prior findings

The prior delayed-stop finding is resolved. All requested authority transitions
now retire pre-transition feedback and publish one coherent measured hold or
disable generation before their transition event. The earlier whitespace/report
finding is also resolved; the complete base-to-HEAD range passes
`git diff --check`.

The original public-ABI disable-stop oracle now produces:

```text
t=20000000 lifecycle=2 status=0 sample_t=20000000 seq=3
t=30000000 lifecycle=2 status=0 sample_t=20000000 seq=3
t=40000000 lifecycle=2 status=0 sample_t=20000000 seq=3
t=50000000 lifecycle=2 status=0 sample_t=30000000 seq=4
```

The transition generation is immediately disabled at 20 ms, the two queued
pre-stop generations never become public at 30 or 40 ms, and the first later
publication at 50 ms is a post-stop disabled capture from 30 ms.

Targeted public-ABI transition coverage passed for:

- explicit disable and controlled enabled-hold stops;
- disarm and E-stop;
- producer watchdog and missed-cycle faults;
- command/bus failure, delayed partial generation, and motor fault evidence;
- event-overflow materialization on the tested E-stop path;
- reset-to-closed followed by re-verification; and
- close/destroy while a public event waiter is pinned.

The close oracle returned the documented statuses without exposing any further
state after destruction:

```text
close_waiter=3 stale_snapshot=1
```

Here `3` is `OA_ESTATE` for the already-pinned waiter and `1` is `OA_EINVAL` for
a call begun using the destroyed handle.

The transition-only and value-delay public oracle returned:

```text
public_value_delay_and_transitions=PASS
```

## Remaining semantic audit

Apart from the event-publication control-flow finding above, the delayed
feedback implementation remains sound:

- Captures are immutable, already-quantized seven-frame generations. Values,
  source timestamp, complete member mask, and sequence become public together.
- Partial generations do not mutate any prior payload, timestamp, or sequence;
  controller-level handling fails closed and preserves motor-fault evidence.
- Ready-time addition and sequence publication are checked, the fixed 64-entry
  ring bounds memory without servo-path allocation, and capacity overflow
  materializes a disabled fault stop.
- Source timestamps remain monotonic in capture order. Freshness is inclusive at
  the configured boundary, cross-bus skew is based on delivered source times,
  and no current plant value leaks into an old generation.
- Completion consumes decoded delayed q/dq/FK and requires three distinct paired
  sequence intervals. Equal 20 ms delay gives exact two-cycle historical
  equivalence and completion delay at a 10 ms cadence.
- Transition holds use last coherent measured q with zero measured velocity,
  never the current trajectory command. Fault/transport paths cannot retain an
  enabled hold unless the configured watchdog-controlled-stop gate is coherent,
  fresh, and healthy.
- The public C layout and symbol contract are unchanged. Both current-C11 and
  frozen original-V1 consumers compile, link, and return zero.

## Fresh verification evidence

Release:

```text
cmake -S control -B /tmp/openarmik-rereview-release.zMsIok \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DOA_CONTROL_BUILD_TESTS=ON
cmake --build /tmp/openarmik-rereview-release.zMsIok -j2
ctest --test-dir /tmp/openarmik-rereview-release.zMsIok --output-on-failure
Result: 3/3 passed
```

ASan/UBSan:

```text
cmake -S control -B /tmp/openarmik-rereview-asan.VDPoWT \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON \
  -DOA_CONTROL_BUILD_TESTS=ON -DOA_CONTROL_SANITIZERS=ON
cmake --build /tmp/openarmik-rereview-asan.VDPoWT -j2
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
  ctest --test-dir /tmp/openarmik-rereview-asan.VDPoWT --output-on-failure
Result: 3/3 passed
```

ThreadSanitizer:

```text
cmake -S control -B /tmp/openarmik-rereview-tsan.A4zXfW \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON \
  -DOA_CONTROL_BUILD_TESTS=ON -DOA_CONTROL_TSAN=ON
cmake --build /tmp/openarmik-rereview-tsan.A4zXfW -j2
TSAN_OPTIONS=halt_on_error=1 \
  ctest --test-dir /tmp/openarmik-rereview-tsan.A4zXfW --output-on-failure
Result: 3/3 passed
```

ABI and static checks:

```text
/tmp/openarmik-rereview-release.zMsIok/openarm_control_c11_abi
c11_abi_exit=0
/tmp/openarmik-rereview-release.zMsIok/openarm_control_v1_original_abi
v1_original_abi_exit=0

cppcheck --enable=warning,performance,portability --error-exitcode=1 \
  --inline-suppr --std=c++17 -I control/include -I control/src -I model/include \
  control/src/control_core.cpp control/src/c_api.cpp control/src/kinematics.cpp \
  control/tests/test_control.cpp
Result: no diagnostics

git diff --check 514e3a8b6ba21a687d18bd761d17c9412a50804c \
  1d2b9478b91449ccafdcb52fbea0b35f263e40fb
Result: passed
```

No GUI, CAN/network interface, or hardware path was launched or touched.
