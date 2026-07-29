# Independent simulator-feedback verification

Date: 2026-07-29 (America/Los_Angeles)  
Tree inspected: `main` at `19a92de`  
Disposition: **DONE_WITH_FINDINGS**

## Bottom line

The central hypothesis is confirmed. `oa_sim_fault.feedback_delay_ns` does not
delay feedback values. It advances the plant through the current control interval,
encodes and decodes that current plant state immediately, increments the feedback
sequence immediately, and merely stamps the new values with
`now_ns - feedback_delay_ns` (clamped to zero).

This means the field currently models **timestamp age/skew**, not feedback
transport/value latency. It can validly exercise freshness and cross-bus timestamp
skew, but it cannot prove that JointState, FK, or completion follows a delayed
encoder-value stream.

The current control completion tests do not themselves make that false claim:
the completion tests use no feedback delay, and the only two delay uses assert a
one-arm skew fault and disabled fallback. The proposed ROS provenance test in
`.swarm/ros_integration_recon.md:160-169` is also correctly caveated: it explicitly
says the present injection backdates timestamps and does not queue old values.
Any stronger statement that the delay field proves delayed encoder **values**
would be unsupported.

I also found a related coherency defect: after a partial generation, the public
snapshot can contain a mixture of old dropped-joint values and newly decoded
non-dropped-joint values even though `feedback_seq` did not advance. The
`fresh_mask` exposes the incomplete generation, so control fails closed, but this
contradicts the README's stronger claim that state is published only from one
complete coherent generation.

## Direct implementation evidence

The disputed path is unambiguous:

- `control/src/control_core.cpp:167-196` advances `plant_raw_q_` and
  `plant_raw_dq_` through the current `dt_s`.
- `control/src/control_core.cpp:197-211` immediately quantizes that current state
  into the eight-byte frame and assigns the caller-supplied timestamp.
- `control/src/control_core.cpp:213-229` immediately decodes the same frame into
  `measured_`.
- `control/src/control_core.cpp:296-307` computes only
  `feedback_ns = max(0, now_ns - feedback_delay_ns_)`, passes it to the immediate
  step/decode above, and records it as the generation timestamp.
- `control/src/control_core.cpp:307-309` increments `feedback_seq_` in that same
  cycle whenever all seven `step()` calls return true. There is no queue, history,
  ready time, or delayed dequeue anywhere in `control/`.

Consequently a nonzero delay changes timestamp metadata and freshness/skew
classification, but not q, dq, status, quantization, sequence progression, or
completion timing when applied equally to both arms below the freshness timeout.

## Independent reproduction

I configured and built the current control source outside the repository at
`/tmp/openarmik-sim-feedback.1aXx6l`. A public-ABI harness created two identical
armed controllers and identical J1 plans. One controller had zero delay; the
other had 20 ms delay on both arms. Both were advanced at 10 ms. Every cycle the
harness compared q, dq, raw q/dq, fresh masks, and sequences for both arms.

Observed output:

```text
cycles=148 payloads_bit_identical=yes delayed_differs_from_baseline_2_cycles_old=yes
completion_baseline_ns=1480000000 completion_delayed_ns=1480000000
final_baseline_t_ns=1480000000 final_delayed_t_ns=1460000000 timestamp_age_ns=20000000 feedback_seq=150
```

Thus the delayed controller matched the baseline's **current** payload bit for
bit, differed from the baseline payload two cycles earlier, completed at exactly
the same time, and only reported an older timestamp. The sequence also advanced
on every current cycle; it was not withheld behind a two-cycle pipeline.

A private-core harness then injected a one-joint drop during a generation:

```text
seq_before=2 seq_after=2 fresh_mask=0x7e snapshot_t_ns=20000000
dropped_joint_q_before=6.6758220798490697e-05 after=6.6758220798490697e-05
nondroppped_joint_q_before=-0.00031471732661891849 after=-0.00069619287403632768
```

(`nondroppped` is only a spelling error in the harness label.) The dropped joint
remained old, a non-dropped joint became new, `t_ns` became the new generation's
timestamp, and `feedback_seq` stayed old. This follows from per-motor mutation in
`DamiaoMotorSimulator::step()` before `ArmRuntime` knows whether all seven members
arrived (`control/src/control_core.cpp:298-309`), while `snapshot()` publishes all
motor payloads regardless of their fresh bits (`control/src/control_core.cpp:340-360`).

A fresh Release/Werror build of the unmodified current control tree passed all
three registered tests. Passing is expected because no test asserts value-delay
semantics:

```text
openarm_control_tests           Passed
openarm_control_c11_abi         Passed
openarm_control_v1_original_abi Passed
100% tests passed, 0 tests failed out of 3
```

## Semantics audit

### Plant lag versus feedback lag

The plant lag is real and independent of the disputed field. The simulator uses
a bounded velocity/acceleration response (`command_raw_dq + 8 * position_error`,
an 80 ms velocity-response denominator, and manifest clamps), then encodes the
result. The existing convergence and frozen-value non-completion tests therefore
do establish command/state separation and decoded-plant completion. They do not
establish delayed-value delivery.

`freeze_mask` also freezes the plant value, not frame delivery: a fresh frame,
timestamp, and sequence are still produced each cycle. That is sufficient to
model a stuck encoder value in otherwise live packets and explains why the move
does not complete.

### Quantization

Feedback is genuinely quantized before completion consumes it. Position spacing
is `25 / 65535 = 3.814755e-4 rad`, so maximum position quantization error is about
`1.90738e-4 rad`. Velocity half-LSB errors are approximately:

- DM8009: `45 / 4095 = 0.010989 rad/s`;
- DM4340: `10 / 4095 = 0.002442 rad/s`;
- DM4310: `30 / 4095 = 0.007326 rad/s`.

Because the 12-bit map has 4096 levels spanning symmetric endpoints,
`llround(2047.5)` maps mathematical zero to code 2048, which decodes to a positive
half LSB for dq and tau. Existing joint/TCP completion tolerances of 0.0005 rad
and 0.02 rad/s are larger than the worst relevant half LSB, so their convergence
is achievable. A delay queue must store already encoded frames (or exact frame
bytes), not re-encode historical doubles, to preserve these semantics exactly.

### Freshness and skew

`snapshot()` calls a member fresh only when it is in the latest generation mask,
valid, non-future, and `now - measured.t_ns <= feedback_timeout_ns` (inclusive).
Therefore:

- equal delays on both arms remain cross-bus coherent and, below the timeout,
  fresh despite exposing current values;
- unequal delays synthesize timestamp skew even though both value streams are
  current; the 2 ms one-arm test exceeds the configured 1 ms skew limit and
  returns `OA_ECAN` for that reason alone;
- a delay greater than `now_ns` stamps successive current values at zero; their
  sequence still advances until age exceeds the timeout;
- a delay equal to the timeout is accepted, while greater age is stale;
- `oa_arm_snapshot.t_ns` is the maximum timestamp among members still classified
  fresh. With a partial generation it can describe the new non-dropped members
  while the sequence still describes the prior complete generation.

The controller checks prior feedback integrity before stepping and current
integrity afterward. Partial/drop and excessive age fail closed. That safety gate
does not repair the provenance mismatch in snapshots or make current values old.

### Completion/dwell

Completion uses decoded measured q/dq (and measured FK for TCP) only after plan
duration. The first in-tolerance sample starts `settle_start_ns_`; completion
requires three full positive cycle intervals after that sample. Equal-time calls
return before producing feedback or dwell. These semantics are sound for the
current immediate-feedback simulator.

With a real delayed-delivery queue, completion must additionally require a newly
delivered complete sequence. Otherwise repeated cycles with no newly due frame
could count the same still-fresh sample multiple times and falsely satisfy dwell.

## Minimal correct correction

There are two honest choices:

1. If timestamp-age/skew injection is all that is intended, keep the implementation
   and rename/document the field as `feedback_timestamp_age_ns` (the binary tail
   layout can remain unchanged). Update every test/report to say timestamp age,
   never value delay. This is the smallest correction.

2. If the field is intended to prove delayed encoder values, the minimal semantic
   fix is a bounded per-arm **complete-generation queue**:

   - integrate the plant at capture time and encode all seven frame bytes once;
   - enqueue the generation with immutable `capture_ns` and
     `ready_ns = capture_ns + delay` using checked arithmetic;
   - publish/decode only complete generations whose ready time has arrived;
   - increment `feedback_seq` only on complete delivery and preserve `capture_ns`
     as the sample timestamp;
   - on an incomplete delivered generation, preserve every payload from the last
     coherent snapshot while exposing the missing mask/fault; do not leak six new
     members under the old sequence;
   - gate dwell on delivery of a new complete sequence;
   - preallocate the ring outside the servo path. Constrain delay to a documented
     maximum (normally no greater than the feedback timeout), size the ring from
     cycle/timeout policy with overflow and allocation checks, and fail closed on
     queue overflow.

I recommend choice 2 only if delayed-value provenance is a required product/test
claim. Otherwise choice 1 is less code and accurately describes the existing,
useful skew injector. A separate timestamp-skew field may still be useful because
true delivery delay sampled at a 10 ms control cadence produces cycle-quantized
observable age, not necessarily an exact 2 ms skew.

## Exact adversarial regressions

For a real queue, add these deterministic tests:

1. **Two-cycle equivalence oracle:** run zero-delay and 20 ms-delay controllers at
   10 ms. After pipeline fill, assert every delayed frame byte/q/dq/status and its
   sample timestamp equals the zero-delay controller from exactly two cycles ago,
   and explicitly assert it differs from the current zero-delay payload during
   motion. Sequence and completion must lag by two delivered generations/cycles.
2. **No early non-grid delivery:** with 15 ms delay and 10 ms advances, prove a
   frame captured at 10 ms is unavailable at 20 ms and first appears at 30 ms with
   capture timestamp 10 ms. Document the resulting 20 ms observable age.
3. **No duplicate-sample dwell:** hold the last delivered sample at goal while no
   queued generation is due; repeated positive-time advances must not increment
   dwell or emit `COMPLETED` until three full intervals of distinct complete
   delivered sequences have occurred.
4. **Freshness boundary:** arrange no delivery and prove age exactly equal to the
   timeout remains fresh, while the next legal advance beyond it returns
   `OA_ESTALE`. Ensure no newly integrated current value appears in the stale
   snapshot.
5. **Atomic incomplete generation:** drop one member of a due generation. Assert
   no q/dq/tau/status member changes, sequence and timestamp remain those of the
   last complete snapshot, the missing mask identifies the member, and control
   fails closed. This test fails the current implementation.
6. **Quantized history identity:** compare delayed and historical baseline frame
   bytes exactly, including positive half-LSB decoded zero for each motor family.
   Also use completion tolerances below half an LSB to prove quantization cannot
   be bypassed by unquantized historical plant values.
7. **Delay mutation and saturation:** queued frames retain the delay/ready time
   assigned at capture when the injection changes; checked `capture + delay`
   overflow and ring overflow fail closed without sequence/payload mutation.
8. **Cross-bus cases:** equal delays preserve zero source-timestamp skew; unequal
   delays produce the documented cycle-quantized delivered-generation skew; no
   current-value/backdated-timestamp shortcut may satisfy the assertion.
9. **Completion provenance:** the completion event's feedback sequence must equal
   the final complete delayed snapshot used for q/dq/FK dwell, and completion must
   occur later than the zero-delay oracle by the expected delivered-cycle count.

For the current timestamp-age implementation, add the inverse contract test:
assert delayed and zero-delay payloads/completion are identical while timestamps
differ, and name the test `timestamp_age_does_not_delay_values`. That prevents a
future report from accidentally claiming more than the simulator implements.

## ROS source/build/install 8-versus-5 discrepancy

The discrepancy is real, with one terminology correction: CTest drivers live in
the **build** tree, not the install tree.

- Current `ros2_ws/src/openarm_ik_ros/CMakeLists.txt:53-87` registers eight CTest
  drivers: one GTest driver, four existing Python drivers, and three new
  `close_rviz_window` argument drivers.
- `ros2_ws/build/openarm_ik_ros/CTestTestfile.cmake` contains only the older five;
  `ctest -N` reports exactly five.
- The source CMake file is timestamped 12:40:17 and comes from commit `0ed82c4`
  (`refactor: replace RViz Python shutdown helper`). The generated CTest file is
  timestamped 12:08:31. Workspace build logs end at that 12:08 configure/build.
  The source change therefore postdates the workspace build.
- The ignored workspace install is stale too: it has no
  `lib/openarm_ik_ros/close_rviz_window`, and its installed `package.xml` hash
  differs from source (the source now declares X11). Tests are not supposed to
  be installed, so absence of an installed CTest file is normal; absence of the
  newly installed runtime helper is the meaningful install discrepancy.
- An existing fresh out-of-tree build at
  `/tmp/openarmik-rviz-c-helper/ros2_ws/build/openarm_ik_ros` contains all eight
  drivers, and its install contains the helper. `ctest -N` there confirms tests
  1 through 8. This proves the source declarations are effective when freshly
  configured.

No ROS rebuild is needed to diagnose the cause: the checked-in source was changed
after the last workspace configure, while `ros2_ws/build` and `ros2_ws/install`
are ignored generated artifacts. Reconfigure/rebuild the package before treating
the workspace install or its five-test result as current evidence.

## Scope and residue

No repository implementation file was edited, no GUI was launched, and no CAN or
network interface was opened or transmitted to. Builds and executable harnesses
were isolated under `/tmp`. The only repository write from this task is this
requested report. Pre-existing untracked `.swarm` reports were left untouched.
