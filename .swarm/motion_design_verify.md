# Independent adversarial verification: analytic virtual motion design

Date: 2026-07-29 (America/Los_Angeles)  
Disposition: **DONE_WITH_CONCERNS**  
Scope: read-only inspection of the motion proposal, portal critic, current
controller implementation, public ABI, tests, and prior simulator provenance,
except for this requested report. No source edit, build, executable, GUI,
network, CAN, or hardware action was performed.

## Verdict

The seventh-order rest-to-rest law is mathematically sound and is the smallest
single polynomial that can join stationary holds with continuous position,
velocity, acceleration, and jerk. Replacing the present acceleration-step,
snap-to-target lag plant with an explicitly labelled **ideal kinematic virtual
actuator** is the smallest sound first-release way to make ordinary virtual
motion follow that law. It is preferable to trying to repair the lag servo with
one jerk clamp.

That conclusion is conditional. The proposal is truthful only if all of the
following are implemented together:

1. The virtual actuator's unquantized truth is the absolute-time analytic
   `(q,dq,ddq)` state. It is not reconstructed from decoded feedback and is not
   integrated from the previous sample.
2. Delayed feedback continues to queue immutable, already-quantized frames.
   Public snapshots, FK used for completion, start binding, and the three-new-
   generation completion dwell remain decoded-frame authoritative.
3. The smoothness claim is explicitly limited to normal unquantized kinematic
   truth. Quantized samples have code steps; delayed samples describe truth at
   their capture timestamp, not current truth; stop/fault/test-injection
   transitions are separately classified discontinuities.
4. Plans retain a private truth origin in addition to their public measured
   seed. Pre-start and idle cycles hold truth, rather than assigning decoded q
   back into truth. Otherwise every move or scheduled start can introduce a
   one-code position jump.
5. Planning requires stationary measured feedback and stationary private truth.
   The current API accepts planning after injected nonzero velocity and permits
   `oa_controller_sim_set_state` during execution; both conflict with a
   rest-to-rest ideal actuator.

Without those conditions, changing only `capture()` to copy `q_reference` is
not a truthful no-sudden-jerk fix. With them, the result is honest as a virtual
trajectory player, but not as a physical servo model, following-error model, or
proof of hardware dynamics.

## Exact polynomial and bounds

For displacement `D=q1-q0`, duration `T>0`, and
`u=clamp((t-t0)/T,0,1)`, let

```text
s(u) = 35u^4 - 84u^5 + 70u^6 - 20u^7
q    = q0 + D s(u)
dq   = D s'(u)/T
ddq  = D s''(u)/T^2
jerk = D s'''(u)/T^3
```

Writing `x=2u-1` gives convenient exact derivative forms:

```text
s'(u)   =  (35/16) (1-x^2)^3
s''(u)  = -(105/4) x (1-x^2)^2
s'''(u) = -(105/2) (1-x^2)(1-5x^2)
```

Therefore

```text
max |s'|   = 35/16          = 2.1875              at u=1/2
max |s''|  = 84 sqrt(5)/25  = 7.513188404...      at x=+/-1/sqrt(5)
max |s'''| = 105/2          = 52.5                 at u=1/2
```

The existing factors `2.2`, `8`, and `60` are conservative. Exact necessary
bounds for a monotone segment are

```text
T >= (35/16)|D|/V
T >= sqrt((84 sqrt(5)/25)|D|/A)
T >= cbrt((105/2)|D|/J)
```

and the synchronized segment duration is the maximum of these values over all
participating joints. The existing scale semantics are consistent: use
`V=velocity_scale*max_velocity_rad_s`,
`A=acceleration_scale*max_acceleration_rad_s2`, and
`J=jerk_scale*max_jerk_rad_s3`.

The degree is minimal: stationary C3 endpoints impose eight independent values
(`q,dq,ddq,jerk` at each end), so a polynomial needs at least eight
coefficients, hence degree seven. The septic solution is unique. Its snap is
not continuous at a stationary hold boundary; that does not invalidate the
continuous-jerk claim, but the product must not claim snap limiting.

At both endpoints `s'`, `s''`, and `s'''` are zero. Consecutive rest-to-rest
segments therefore meet with exactly equal q and zero dq/ddq/jerk even if their
durations and displacement directions differ. Explicit `u<=0` and `u>=1`
branches are required so those endpoint values are exact in floating point.

## Comparison with current generation and execution

The current controller already has the right basic time law and conservative
duration constants, but not the proposed plant semantics:

- `smoothstep7()` and `smoothstep7_derivative()` generate q and dq only.
- `trajectory_duration()` starts every segment at ten cycles, computes extrema
  in `double`, casts only the final nanosecond ceiling to `long double`, and
  returns `UINT64_MAX` rather than a checked failure on overflow.
- The plant selects acceleration directly, so acceleration may step every
  cycle. It never consumes `max_jerk_rad_s3`.
- Its crossing and near-target branches directly assign q and dq.
- The plant has no acceleration state and no analytic acceleration reference.
- `advance()` evaluates from absolute `now-start`, which is the correct basis
  for the proposed ideal truth and makes legal dt jitter path-independent.
- A `cycle_ns+1` jump faults before reference evaluation/capture. Equal time
  returns without a feedback generation or dwell increment. Preserve both
  behaviors.
- Segment selection uses the preceding segment at exact equality and the next
  segment one nanosecond later. That is correct because both evaluate the same
  endpoint state. A missed exact boundary sample does not break the underlying
  continuous analytic curve.
- Paired TCP generates 17 IK knots and 16 independently timed rest-to-rest
  joint segments. Both arms share each segment time, so they are synchronized,
  but every knot is a full stop. It is continuous through jerk and deliberately
  slower than a blended path.
- The ten-cycle floor is currently charged even to a zero-distance segment. A
  zero joint request takes a fake 100 ms with the default cycle, and duplicate
  TCP knots can add fake segments. The floor should apply only when at least one
  synchronized joint has nonzero displacement. Coalesce duplicate waypoint
  intervals; an all-zero plan may have duration zero and proceed directly to
  measured settling/dwell.

Duration computation should return a checked result, calculate with
`long double`, round upward to integer nanoseconds, and re-evaluate the chosen
integer T against all three limits. Keep `2.2/8/60` as named conservative
constants to preserve existing planning behavior; do not silently switch to
the exact constants unless duration/report expectations are intentionally
versioned. Check `10*cycle_ns`, segment sums, start+duration, and the product
duration/horizon before publishing a plan.

## Truth, measurement, delay, and quantization

The present complete-generation delay queue is compatible with the ideal
actuator and should be retained unchanged in principle: capture analytic truth,
encode it once, enqueue the seven immutable frames with source timestamp and
ready time, and decode/publish only when due. Sequence and timestamp must remain
delivery-coherent.

The error statement in the proposal needs a timestamp qualifier. The sound
normal-mode bounds are

```text
|q_meas(t_capture)  - q_truth(t_capture)|  <= |q_scale| pmax/65535
|dq_meas(t_capture) - dq_truth(t_capture)| <= |q_scale| vmax/4095
```

They are not bounds between a delayed snapshot and truth at controller `now`.
During motion that difference also contains all motion over the feedback delay.
With the validated unit-magnitude mappings, maximum q error is about
`1.9074e-4 rad`; worst decoded-zero dq bias is the DM8009 value
`45/4095 = 1.0989e-2 rad/s`. The proposed fixed completion tolerances
`5e-4 rad` and `2e-2 rad/s` cover those errors.

For the caller-configurable joint API, a blanket statement that every tolerance
below the maximum half-LSB is impossible is too strong: a target can lie on an
encoder code. Admission should reject when position tolerance is smaller than
the target-specific endpoint error
`abs(mapped_decode(encode(target))-target)`, or impose and document the more
conservative global floor. Velocity tolerance must exceed the motor-specific
decoded-zero magnitude. TCP admission needs either a conservative FK bound for
all endpoint quantization errors or a fixed tolerance proven against the model;
checking only joint half-LSBs does not by itself prove a 1 mm Cartesian bound.

Decoded q/dq remain the only public `oa_snapshot` state. Do not append private
truth to that public record or use truth for completion. Tests of the analytic
envelope should use an internal evaluator/test accessor. This preserves the V1
ABI and the distinction between model truth and measurement.

The current freeze injection freezes the plant while emitting live frames.
Under an ideal actuator that would either violate ideal following or cause a
truth jump when unfrozen. Interpret `freeze_mask` as a stuck encoder source:
analytic truth continues, but the selected captured sensor payload remains
frozen. Such injected feedback is outside the truth/measurement error bound and
must be labelled fault-test behavior. Unfreezing may produce a measured code
jump; no measured-jerk claim is permitted.

## Required internal fields and semantics

No public ABI addition is required for this plant change. The minimum internal
state is, per joint or per arm:

```text
truth_q[7]
truth_dq[7]
truth_ddq[7]
```

and in each `MotionPlan`:

```text
measured_start_q[2][7]   // existing start_q; staleness and public binding
truth_start_q[2][7]      // private analytic origin/hold configuration
```

Use one pure segment evaluator returning q/dq/ddq from integer elapsed and
duration. Ordinary capture takes that complete analytic state; it must not
accept dt for integration. In idle, settling, and before a future start, capture
the retained truth q with zero derivatives. Never feed decoded measured q back
into ordinary truth.

For a joint plan, waypoint zero and every nonmoving coordinate use private
truth, while the requested moving coordinate ends at the requested target. For
TCP, IK admission and stale/start checks remain measured-seed authoritative;
the first executed segment begins at private truth and all collision/path
validation must cover the bounded discrepancy to the first IK knot. For an
inactive arm, every executed waypoint is its private truth hold. Reported target
and measured completion behavior remain as documented.

At planning, require every decoded `abs(dq)<=2e-2 rad/s` and private
`truth_dq==truth_ddq==0`. A plan request in the wrong lifecycle retains the
current lifecycle error; a fresh idle state with injected motion should return
`OA_CONTROL_EBUSY`. Reversal is then safe only as a second command after normal
measured completion/dwell: the previous truth is exactly stationary and the new
segment starts with zero acceleration and jerk. Requests during the first
command remain rejected rather than blended or preempted.

`oa_controller_sim_set_state` should be rejected with `OA_CONTROL_EBUSY` while
executing, including the queued pre-start phase; it currently succeeds and a
test depends on that behavior. Outside execution it may atomically set q/dq,
reset private ddq to zero, clear delayed history, and publish a classified test
generation. A nonzero injected dq must persist as test state and block planning;
an idle advance must not silently force it to zero through the ideal actuator.
The existing future-start drift test should inject before execute or use a
dedicated fault path instead of mutating an accepted execution.

## Completion and exceptional discontinuities

At and after planned duration, analytic truth is exactly `(target,0,0)` and
remains a stationary hold. There is no convergence branch or target snap.
Delayed endpoint frames arrive later; only then may decoded q/dq/FK become in
tolerance. Preserve the current requirement for three distinct complete
delivered generations from both arms and at least three positive cycle
intervals. Equal-time calls must not count. Multiple queued generations
published in one owner cycle may count as at most one dwell interval, as now.

An execution expiry must include planned duration, maximum permitted feedback
delay, and settling allowance. The proposed portal `duration+0.5 s` allowance
is adequate for the stated 50 ms timeout/10 ms cycle policy, but the generic
core should validate checked arithmetic rather than assume portal policy.

Explicit stop, E-stop, disable, watchdog fallback, event overflow, reset,
fault injection, and simulator-state injection currently materialize q/dq
changes from measured state. Those are discontinuities relative to analytic
truth, especially with delayed feedback. They are legitimate safety/test
exceptions only if lifecycle/events classify them and ordinary envelope tests
exclude them. `OA_STOP_CONTROLLED` is not a controlled deceleration in the
current API; it is an instantaneous enabled measured hold and must not be
presented by the portal as smooth stopping.

## Provenance impact

Changing from the lag plant to ideal kinematic following intentionally
invalidates several previously verified claims and tests; it is not a
provenance-neutral refactor:

- `control/README.md` says commands update only an independent lagging
  bounded-velocity/acceleration plant.
- `.swarm/control_core_impl.md` records command/measurement separation through
  a real lag plant and explicitly lists a real plant-lag proof.
- `test_mapping_kinematics_and_joint_convergence()` requires lag beyond planned
  duration (`lag_observed`). That test must be replaced, not weakened.
- Direct `ArmRuntime::command_and_step` delay-history oracles currently depend
  on the lag response. Preserve their immutable delayed-frame provenance but
  change the oracle to compare analytic historical samples.
- Freeze currently means plant freeze in implementation even though the test is
  named as a frozen encoder. Changing it to sensor freeze is a semantic change
  that needs an explicit regression.
- The current `oa_controller_sim_set_state`-during-execution stale-start test
  must change because execution-time mutation becomes rejected.

The prior delay/coherence evidence remains valid only for the queue mechanism,
atomic generation publication, timestamp/sequence provenance, lifecycle queue
retirement, and measured completion. It does not by itself verify the new truth
source. All lag, following-error, servo convergence, or physical-dynamics
claims must be retired. No collision or hardware safety claim becomes stronger
merely because ideal truth is smoother. Conversely, an ideal zero-following-
error path can simplify a virtual collision certificate, but only after the
full swept-path validator exists; current unchecked collision mode remains a
portal release blocker.

## Exact release tests

1. Symbolically or independently evaluate endpoint derivatives and exact
   extrema above; test both displacement signs and all mapping signs.
2. For each segment, sample endpoints, the velocity/acceleration/jerk extrema,
   random interior times, and one nanosecond before/at/after every waypoint.
   Assert q/dq/ddq/jerk against direct evaluation and `2.2/8/60` bounds.
3. Exercise 1 ns, irregular, alternating, and `cycle_ns` legal advances. Truth
   at a timestamp must depend only on absolute time. Equal time is unchanged;
   decreasing time rejects; `cycle_ns+1` faults before an ordinary sample.
4. Verify multi-joint and two-arm synchronization for unequal distances and
   limits. At every knot assert exact common q and zero dq/ddq/jerk. Assert
   duplicate knots are coalesced and a zero-distance request has no fake moving
   interval.
5. Run zero-delay and delayed controllers. Every delayed public payload must
   equal the quantized analytic baseline payload at the same capture timestamp,
   not current truth. Completion must lag until endpoint frames plus three new
   delivered generations arrive.
6. Test targets on a code, half-way between codes, and below one code for every
   motor family/mapping sign. Assert target-specific tolerance admission,
   decoded-zero velocity bias, stable endpoint codes, and no truth snap.
7. Plan immediately after normal completion and reverse direction. Assert the
   new private origin equals prior truth, public seed is decoded measurement,
   and the second trajectory starts at zero dq/ddq/jerk without a q reset.
8. Inject nonzero simulator dq while idle and require planning busy. Require
   `sim_set_state` during immediate and future-start execution to return busy
   without changing truth, measurement, queues, or the accepted plan.
9. Freeze one encoder during analytic motion: truth continues smoothly,
   measured payload stays fixed, completion never succeeds, and expiry/stop is
   classified exceptional. Unfreeze is never evaluated as ordinary measured
   jerk.
10. At duration and for thousands of later cycles, assert truth remains exactly
    target/zero/zero and captured endpoint bytes remain stable. No convergence
    or near-target assignment may exist.
11. Test duration nanosecond upward rounding, post-round revalidation, minimum
    nonzero duration, all-zero duration, scale endpoints, nonfinite inputs,
    `10*cycle`, segment-sum, and start+duration overflow, plus portal 30 s and
    command-horizon rejection.
12. Retain all existing V1 ABI, delayed-generation atomicity, partial/drop,
    freshness/skew, event overflow, stop-history retirement, sequence-bound FK,
    and three-generation dwell tests, updating only their truth oracle.

## Final disposition

**DONE_WITH_CONCERNS.** The ideal analytic virtual actuator is a defensible and
minimal no-sudden-jerk fix for ordinary virtual truth, and the exact septic
bounds validate the existing `2.2/8/60` duration factors. It is not a drop-in
one-line plant replacement: private truth origin/hold semantics, stationary
admission, acceleration evaluation, injection rules, timestamp-qualified
quantization assertions, checked duration handling, and provenance/test updates
are required. It preserves encoder-authoritative delayed/quantized public state
and completion, but necessarily retires every prior claim that the simulator
models independent lag, servo convergence, following error, or physical
dynamics.
