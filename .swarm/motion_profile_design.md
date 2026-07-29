# Virtual OpenArm jerk-limited motion and plant design

Date: 2026-07-29 (America/Los_Angeles)  
Status: **DESIGN COMPLETE; IMPLEMENTATION REQUIRED**  
Scope: read-only review of the controller, simulator, planner, public ABI, tests,
and `.swarm/portal_control_critic.md`, except for this requested report. No
build, executable, GUI, network, CAN, or hardware action was performed.

## Decision

Keep the existing seventh-order rest-to-rest S-curve as the trajectory law, but
do **not** keep the current acceleration-limited lag plant. It can change
acceleration in one sample and its two snap-to-target branches directly assign
position and velocity. It therefore cannot support a no-sudden-jerk claim.

For the first virtual product, the smallest mathematically sound plant is a
deterministic ideal virtual actuator whose unquantized `(q, dq, ddq)` is the
analytic S-curve state at the controller's absolute monotonic time. Quantized
DaMiao frames remain the only published measurement. This is deliberately a
kinematic virtual plant, not a claim about physical servo dynamics. It avoids a
second, lagging trajectory generator, gives exact convergence without a target
snap, and makes collision validation apply to the motion actually executed.

If an independently lagging plant is a product requirement, use a genuine
online jerk-limited trajectory generator with state `(q,dq,ddq)` and braking-
feasibility logic. Merely adding `clamp(delta_acceleration, J*dt)` to the present
PD servo is not enough: velocity saturation, reversal, target crossing, and
final convergence still need a proof and can otherwise introduce hidden state
discontinuities. Of the two alternatives in the question, an online jerk
limiter is the only correct lag-plant alternative, but it is not the minimal
first-release implementation.

Portal motion must also be collision-validated. The existing
`OA_COLLISION_VIRTUAL_UNCHECKED` mode remains useful only for low-level tests;
the portal must refuse readiness and every move unless a validator proves the
complete joint path clear of the other arm and the central pole in the exact
bound collision-scene revision.

## Existing behavior that must change

- `smoothstep7` and its derivative already generate a valid position/velocity
  reference. `trajectory_duration` conservatively uses factors 2.2, 8, and 60.
- The plant computes a desired velocity, selects acceleration directly, and
  therefore permits an acceleration step on every `capture`.
- The overshoot and near-target branches assign `plant_raw_q_` and
  `plant_raw_dq_` directly. Both bypass acceleration and jerk.
- The plant has no acceleration state and the controller does not calculate or
  pass reference acceleration.
- Multi-waypoint TCP motion runs an independent rest-to-rest curve on every IK
  segment. It is continuous through jerk, but it comes to rest at all 16
  internal boundaries and can be conservative in duration.
- `advance` correctly rejects decreasing time, treats equal time as no step,
  and faults when a positive interval exceeds `cycle_ns`. Those fail-closed
  semantics should be retained.
- Current collision policies are only reject-all and virtual-unchecked. Neither
  satisfies portal collision avoidance.

## Reference law and exact bounds

For one segment from `q0` to `q1`, duration `T > 0`, and
`u = clamp((t-t0)/T, 0, 1)`, use

```text
s(u) = 35u^4 - 84u^5 + 70u^6 - 20u^7
q(t) = q0 + (q1-q0)s(u)
dq(t) = (q1-q0)s'(u)/T
ddq(t) = (q1-q0)s''(u)/T^2
dddq(t) = (q1-q0)s'''(u)/T^3
```

Evaluate position, velocity, and acceleration from the same clamped `u` and
absolute controller time; do not estimate either derivative from the prior
sample. Use Horner/factored forms and explicit endpoint branches so `u<=0`
returns `(q0,0,0)` and `u>=1` returns `(q1,0,0)` exactly.

The polynomial has `s'(0,1)=s''(0,1)=s'''(0,1)=0`. Thus a hold, segment, next
segment, and final hold meet with equal q and zero velocity, acceleration, and
jerk. There is no target snap. Its normalized peak velocity is exactly 2.1875;
the existing constants conservatively bound the normalized peaks by 2.2, 8,
and 60. For each active joint choose

```text
T >= max(2.2 |D| / V,
         sqrt(8 |D| / A),
         cbrt(60 |D| / J))
```

and use the maximum across synchronized joints. Compute in `long double`, round
duration upward to integer nanoseconds, require at least ten control cycles per
nonzero segment, reject overflow, and re-evaluate the integer-duration bounds
before publishing the plan. A zero-distance plan should not execute a fake
motion; either reject it as already satisfied or complete it through the normal
measured dwell without a moving phase.

### Product defaults

The fixed canonical **virtual** hard limits should be, per joint:

```text
velocity      1.0 rad/s
acceleration  2.0 rad/s^2
jerk         10.0 rad/s^3
```

Normal portal requests fix all three scales at `0.5`, yielding reference
ceilings of `0.5 rad/s`, `1.0 rad/s^2`, and `5.0 rad/s^3`. These are virtual
product-policy values, not qualified physical limits. Do not expose hard-scale
motion in the first portal UI. A 10 ms control cycle, 50 ms feedback timeout,
and 1 ms maximum cross-arm timestamp skew are reasonable existing defaults.

Keep `5e-4 rad` joint position, `2e-2 rad/s` velocity, and `1e-3 m` final TCP
tolerances. Cap planned trajectory duration at 30 s and require an execution
expiry of at least `duration + 0.5 s` and at most the fixed 60 s command horizon.
The portal should refresh the producer deadline from its owner thread; a 100 ms
rolling deadline is compatible with the 10 ms cycle but remains separate from
the stricter missed-cycle check.

## Minimal virtual plant

Replace the servo state with unquantized model-coordinate truth:

```text
truth_q[7], truth_dq[7], truth_ddq[7]
```

The controller evaluates and passes `(q_ref,dq_ref,ddq_ref)` for every joint and
cycle. In ordinary execution the virtual actuator records that analytic state,
then maps it to raw coordinates and quantizes a feedback frame. It must not
derive acceleration from `dq/dt`, clamp a later velocity independently, cross a
target, or snap to a setpoint. Idle hold retains the last truth q and analytic
zero derivatives; it must not repeatedly replace truth q with decoded q.

At planning, measured encoder q remains the public seed and plan binding. The
virtual plan additionally captures its private unquantized truth origin. The
two can differ only by the encoder error bound. The first analytic segment is
constructed from private truth, while IK and all public stale/start checks stay
bound to measured q. Collision validation includes the resulting bounded
initial discrepancy. This internal detail changes no C ABI and avoids an
encoder-grid position jump at a subsequent move. Planning is accepted only
when every measured `|dq| <= 2e-2 rad/s` and private virtual `dq=ddq=0`; a
nonzero injected velocity returns busy until smoothly stopped or explicitly
disabled.

`oa_controller_sim_set_state` is test injection, not normal motion. It may set
truth q/dq immediately only while not executing and must reset acceleration to
zero; tests classify the injection boundary separately. It cannot be used by
the portal.

This ideal plant intentionally has no following lag. If visual realism is later
needed, add it only behind a separate simulator mode and only after a jerk-
limited online generator, tracking-tube collision proof, and convergence tests
exist. Do not weaken the default mode.

## Multi-waypoint continuity and speed

For the minimal release retain 17 measured-start Cartesian IK knots and the
per-segment rest-to-rest law. At every boundary both neighboring segments
produce the exact same q and zero dq/ddq/dddq, so continuity through jerk is
immediate even under variable sampling. Segment selection at an exact boundary
must return that common endpoint.

This choice stops at each knot and is slower than a blended spline. It is still
reasonable under the 0.5 normal scale when accepted by the 30 s duration cap,
and it is substantially easier to prove than assigning guessed waypoint
velocities. The UI should show the checked planned duration before execution.
Reject, rather than silently accelerate, a path that exceeds the horizon.

A future speed improvement may use an interpolating piecewise septic spline,
but only if q, dq, ddq, and dddq are continuous at every knot, extrema are
bounded over every polynomial interval (not merely at samples), and the new
curved joint path receives a fresh swept-collision certificate. A cubic spline
is insufficient because its jerk can jump at knots.

## Quantized measurement

Maintain separate names and tests for smooth unquantized plant truth and decoded
feedback. For one motor,

```text
epsilon_q  = |q_scale| * pmax / 65535
epsilon_dq = |q_scale| * vmax / 4095
```

are the maximum round-to-nearest errors (the velocity codec has a nonzero
decoded zero bias because of its code count). With the current unit mappings,
the worst position error is about `1.91e-4 rad`; the DM8009 zero-velocity bias
is about `1.10e-2 rad/s`. The proposed `5e-4` and `2e-2` completion tolerances
cover these values. Reject caller tolerances below the applicable observable
error floor rather than accepting a plan that cannot complete.

No finite-resolution encoder can publish a mathematically continuous value:
one-code steps are unavoidable. Therefore do not claim that naive first,
second, or third finite differences of decoded q obey the unquantized dynamic
limits. The sound observable assertions are

```text
|q_meas - q_truth|   <= epsilon_q
|dq_meas - dq_truth| <= epsilon_dq
```

and, between samples, corresponding difference bounds that add both endpoint
errors. Proof of velocity/acceleration/jerk limits uses the analytic truth.
Portal rendering should interpolate between coherent truth-backed samples or
use measured q directly; it must never promote a noisy numerical derivative to
authoritative state. Completion continues to use decoded q/dq and measured FK
for three distinct complete generations.

## Time, deadlines, and progress

- All controller times, plan starts, expiries, heartbeats, and progress use one
  `steady_clock`/monotonic epoch. ROS time and browser time never enter the C
  controller. ROS timestamps are publication metadata only.
- The owner thread samples monotonic time once per cycle and passes the same
  value to both arms. Reference evaluation uses `now-start`, not accumulated
  floating-point dt, so legal jitter cannot change the path or terminal state.
- A decreasing timestamp is invalid. An equal timestamp performs no plant
  step, publishes no feedback generation, and advances no dwell.
- A positive `dt <= cycle_ns` is legal. A `dt > cycle_ns` faults before
  evaluating or publishing another ordinary trajectory sample. Do not
  subdivide a late call and pretend deadlines were met.
- Preserve V1's current inclusive boundary convention: a command/deadline is
  still valid at equality and expires when `now > deadline`. Test equality and
  `+1 ns` explicitly. Use checked addition for start plus duration.
- Before a future scheduled start, retain the same stationary truth state.
  Revalidate both measured starts, sequences, controller/verification epoch,
  manifest/model revision, and collision-scene revision at actual start.
- Authoritative trajectory fraction is
  `min(max(now-start,0)/duration,1)`. At fraction 1 the phase is SETTLING until
  decoded q/dq, measured FK, feedback health, and the three-generation dwell
  produce the matching COMPLETED event. Display completion below 100% until
  that event.

Normal completion never writes q/dq/ddq to their targets. It simply samples the
analytic endpoint, whose preceding continuous curve already approaches
`(target,0,0)` with zero jerk, then continues the identical stationary hold.

## Stop exception

E-stop and torque-disable are deliberate safety-priority exceptions to the
ordinary smooth-motion envelope. They retire delayed feedback, remove command
authority, and may make decoded finite differences violate normal acceleration
or jerk bounds. Such generations must carry ESTOP/disabled or fault lifecycle
state and can never be counted as ordinary trajectory samples.

The first portal should use disable for cancel, deadline failure, feedback
failure, and shutdown; it must not describe this as a smooth stop. A future
`OA_STOP_CONTROLLED` implementation must generate a separately collision-
checked jerk-limited braking trajectory from current q/dq/ddq. The current
instantaneous zero-velocity enabled hold is not a controlled deceleration and
should not be exposed as one.

## Mandatory collision validation

Add an append-only collision policy value such as
`OA_COLLISION_VIRTUAL_VALIDATED`; adding a numeric constant changes no V1
layout or symbol. The standard virtual controller in this mode owns an immutable
collision scene containing:

- conservative capsules/convex bounds for every moving link of both arms;
- the fixed central pole as a body-frame cylinder/capsule with a conservative
  radius and height;
- excluded adjacent-link pairs only where mechanically justified; and
- a pinned nonzero geometry/model/scene revision and numeric safety margin.

The portal never creates a controller in unchecked mode. Startup verification
must fail if the validator, central-pole geometry, either arm geometry, or scene
revision is absent. Successful plan reports set `collision_checked=1`.

Checking only the 17 IK knots or fixed-time samples is not sufficient. For every
joint-path segment, recursively certify its swept volume. At an interval sample
q, compute signed clearance for all required robot--pole, left--right, and
non-excluded self pairs. Bound any point's motion over the interval using the
sum of downstream link radii/lengths times the joint-angle interval. For a
moving pair use the sum of both bodies' bounds. An interval is certified only
when sampled clearance exceeds the relative-motion bound plus geometry,
floating-point, and tracking/quantization margins; otherwise subdivide. Reject
with collision status if actual collision is found, the recursion/interval
limit is reached without proof, or any geometry result is nonfinite. This is
adaptive sampling with a conservative bound, not sampling-as-proof.

Time parameterization does not change the geometric joint path, but validate
the final time-stamped trajectory too: all waypoint times strictly increase,
the evaluator selects only certified segments, and every normal virtual truth
state lies on that path. The inactive arm is part of every certificate at its
captured hold q. A single-arm plan that would strike the other held arm or pole
is rejected just like a paired plan.

Plans bind the exact scene revision already present in the V1 report. Execution
rechecks it before acceptance and at the scheduled start. The scene object and
revision are immutable during execution; a revision setter cannot merely change
a number without replacing validated geometry, and scene replacement is
idle-only. Any detected scene/plan epoch mismatch while executing faults and
disables both arms before another ordinary sample. The urgent disable is a
safety-stop exception and is reported as such, never as successful completion.

Because the default virtual plant follows the certified path exactly, its
tracking tube is zero apart from declared numeric representation. Any future
lag plant must enlarge every clearance certificate by a proven joint following-
error tube and fault before that tube is exceeded.

## Proof-oriented invariants

For every ordinary, non-stop execution sample:

1. The selected plan/controller/verification/manifest/model/scene epochs match.
2. Both feedback generations are complete, fresh, coherent, fault-free, and
   produced from the same controller timestamp.
3. Each unquantized truth state is the analytic evaluation of exactly one
   certified segment or stationary hold.
4. `|dq|<=V`, `|ddq|<=A`, and `|dddq|<=J`; q remains inside joint and codec
   position limits over the whole monotone segment.
5. q, dq, ddq, and dddq are continuous across start, every waypoint, and end.
6. Every truth configuration lies inside a swept-collision certificate clear
   of the other arm and central pole with the declared margin.
7. Decoded feedback differs from truth only by the proven codec bounds and is
   never replaced with a command or IK result.
8. COMPLETED implies elapsed duration, measured q/dq/FK in tolerance, and three
   distinct new coherent generations; elapsed duration alone never completes.

These invariants are inductive because absolute-time analytic evaluation does
not depend on the previous dt, while admission, start, and each cycle gate all
epochs and health before producing the next normal sample.

## Adversarial acceptance tests

### Trajectory and plant

1. Symbolically/numerically verify endpoint derivatives and the exact or
   conservative 2.2/8/60 extrema. Test positive and negative distance, all
   joints, all mapping signs, zero distance, minimum duration, huge duration,
   nan/inf, and nanosecond rounding.
2. Sample each analytic segment at endpoints, extrema, random interior times,
   one nanosecond before/at/after every waypoint, and prove hard and normal
   envelopes. Verify q/dq/ddq/dddq continuity across all 17 knots.
3. Run 1 ns, irregular, alternating short/long, and maximum-legal dt sequences;
   compare every truth state bit-for-bit or tightly to direct absolute-time
   evaluation. Equal time changes nothing. Decreasing time rejects. `cycle+1ns`
   faults before another ordinary sample.
4. Exercise immediate reversal as two commands separated by required measured
   rest/dwell. A request while truth or measured state is moving returns busy;
   after rest, the reversed S-curve begins with zero jerk and no state reset.
5. Test tiny targets below one q code, targets exactly on and halfway between
   codes, every codec family, and both mapping signs. Prove encoder error bounds,
   tolerance-floor rejection, eventual measured dwell, and absence of a hidden
   target snap.
6. At planned duration and for thousands of later idle cycles, truth remains
   exactly `(target,0,0)` and measured codes remain stable. Completion occurs
   only after three delivered coherent generations, including delayed feedback.
7. Inject E-stop/disable at every subphase and show envelope checks are skipped
   only for explicitly classified stop generations, both arms lose authority,
   and no stopped sample completes a command.

### Deadlines and synchronization

8. Test start/expiry/producer deadlines at `-1ns`, equality, and `+1ns`, checked
   arithmetic near `UINT64_MAX`, future start, heartbeat renewal, and a delayed
   owner cycle. No catch-up stepping or progress based on ROS/browser time.
9. Apply ROS clock jumps, browser pause/reconnect, and worker scheduling jitter;
   controller elapsed time, trajectory, completion, and event attribution do
   not change. Both arms' capture timestamps remain identical and delivered
   cross-arm skew remains within policy.

### Collision

10. Validate safe single-left, single-right, paired, and hold-arm cases; then
    generate endpoint-safe paths that intersect the pole between knots, cross
    arms between knots, graze at exactly the margin, and miss by one numeric
    epsilon. Only continuously certified paths pass.
11. Force the adaptive checker to its subdivision limit, return nonfinite
    clearance, remove pole geometry, use stale model geometry, and understate a
    link bound. All fail closed; no plan is published and output sentinels and
    plant state remain unchanged.
12. Change the scene before execute, at future start, and at every execution
    interval. Stale plans reject; an impossible in-flight epoch change faults
    and disables before another normal state. A revision number without matching
    immutable geometry is never accepted.
13. Quantize measured q toward the collision on every joint and verify the
    certificate's representation margin covers it. For any experimental lag
    plant, inject worst-case following error and prove the tracking tube remains
    clear or faults before breach.

### ABI and lifecycle

14. Keep all frozen V1 layout, symbol, C11/C++17 consumer, prefix, canary, and
    handle tests unchanged. New collision policy constants and internal plant
    state do not alter any public record. Existing unchecked mode remains
    test-only and cannot initialize the portal.
15. Run long deterministic campaigns of accepted/rejected moves, reversals,
    quantization edges, feedback delay/drop/freeze, deadlines, scene changes,
    cancel, E-stop, reset/reverify/re-enable, and restart. No queue, handle,
    event, duration, or recursion bound grows without limit.

## Release gate

Release the portal only when the acceleration-step/snap plant has been removed,
normal motion proves the analytic hard/normal limits, completion is quantization-
aware and measured, and every portal trajectory has a continuous swept-clearance
certificate for both arms and the central pole at the bound scene revision.
`collision_checked=false`, unchecked policy, knot-only collision sampling, or a
lag plant without a jerk and tracking-tube proof are hard release blockers.
