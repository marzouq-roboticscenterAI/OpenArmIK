# Portal nominal target expansion reconnaissance

Date: 2026-07-30

Scope was read-only investigation plus disposable artifacts under `build/`.
No production file, GUI, listener, transport test, physical interface, CAN path,
or repository staging/commit operation was used. The pre-existing modified
`transport/tests/test_transport.cpp` was not touched.

## Outcome

The current two static targets per arm can be expanded to the nine-target
family below without guessing. The coordinates are a deterministic integer-cm
offset lattice from the measured, encoder-quantized virtual neutral TCP. All
nine targets:

- pass the current `NominalPathGuard` from the actual canonical measured
  virtual neutral posture on both sides;
- pass the same public IK settings used by Control's paired planner;
- pass 1,800 guarded directed checks from the Cartesian product of neutral and
  quantized candidate endpoint postures, with zero failures and a worst
  sampled clearance of `0.026527860 m` against the guard's `0.025 m` limit; and
- complete in an actual process-local `VirtualControlSession` sequence (left
  then right for every target, 18 measured completions total) from fresh
  virtual feedback. Observed plan durations were about 5.08--9.93 seconds,
  below the adapter's 30-second plan-duration limit.

These remain sampled nominal **virtual test values**, not physically safe
poses. The model/controller still report collision unchecked.

## Exact recommended target family

Frame is `openarm_body_link0`: +X forward, +Y left, +Z up. Metre values are the
canonical values to store. The centimetre values are their exact portal
display equivalents at the current four decimal places.

| ID / suggested label | Left metres `(x,y,z)` | Left cm | Right metres `(x,y,z)` | Right cm |
|---|---:|---:|---:|---:|
| `small` / Small forward/up (existing) | `(0.019973, 0.143469, 0.096000)` | `(1.9973, 14.3469, 9.6000)` | `(0.020081, -0.143527, 0.096000)` | `(2.0081, -14.3527, 9.6000)` |
| `medium` / Medium forward/up (existing) | `(0.029973, 0.143469, 0.106000)` | `(2.9973, 14.3469, 10.6000)` | `(0.030081, -0.143527, 0.106000)` | `(3.0081, -14.3527, 10.6000)` |
| `large` / Large forward/up | `(0.039973, 0.143469, 0.116000)` | `(3.9973, 14.3469, 11.6000)` | `(0.040081, -0.143527, 0.116000)` | `(4.0081, -14.3527, 11.6000)` |
| `forward_low` / Low reach | `(0.039973, 0.153469, 0.086000)` | `(3.9973, 15.3469, 8.6000)` | `(0.040081, -0.153527, 0.086000)` | `(4.0081, -15.3527, 8.6000)` |
| `forward_mid` / Mid reach | `(0.049973, 0.153469, 0.096000)` | `(4.9973, 15.3469, 9.6000)` | `(0.050081, -0.153527, 0.096000)` | `(5.0081, -15.3527, 9.6000)` |
| `forward_high` / Far reach | `(0.059973, 0.153469, 0.106000)` | `(5.9973, 15.3469, 10.6000)` | `(0.060081, -0.153527, 0.106000)` | `(6.0081, -15.3527, 10.6000)` |
| `high` / High | `(0.029973, 0.153469, 0.136000)` | `(2.9973, 15.3469, 13.6000)` | `(0.030081, -0.153527, 0.136000)` | `(3.0081, -15.3527, 13.6000)` |
| `mid_high` / High near | `(0.019973, 0.153469, 0.126000)` | `(1.9973, 15.3469, 12.6000)` | `(0.020081, -0.153527, 0.126000)` | `(2.0081, -15.3527, 12.6000)` |
| `far_high` / High far | `(0.049973, 0.153469, 0.136000)` | `(4.9973, 15.3469, 13.6000)` | `(0.050081, -0.153527, 0.136000)` | `(5.0081, -15.3527, 13.6000)` |

The measured neutral joint value used by the checked-in test and reproduced by
the virtual encoder is `6.67582207984907e-05 rad` for every joint. Public FK
gives:

- left: `(-0.000027102179658462585, 0.15346860855059954, 0.075999552019693412) m`;
- right: `(0.000080779104435049266, -0.15352682530236783, 0.075999557047326471) m`.

Rounding those once to six decimals produces the lattice bases
`(-0.000027, 0.153469, 0.076000)` and
`(0.000081, -0.153527, 0.076000)`. The candidate offsets in cm, expressed as
`(forward, outward, up)` where positive outward follows the arm's local side,
are:

```
small        (2, -1, 2)
medium       (3, -1, 3)
large        (4, -1, 4)
forward_low  (4,  0, 1)
forward_mid  (5,  0, 2)
forward_high (6,  0, 3)
high         (3,  0, 6)
mid_high     (2,  0, 5)
far_high     (5,  0, 6)
```

The coupling between axes is intentional. A scan of the current guard showed
that simple forward-only, up-only, and outward-only points commonly fail
public IK from neutral. Neutral is a difficult/singular seed for independent
one-axis guesses. For example, left `(+1 cm forward, 0, 0)` and `(+1 cm up)`
both fail the public IK waypoint proof. A superficially nearby left pole
approach is already a checked-in negative test. This is why adding hand-picked
coordinates or merely mirroring visual geometry is not defensible.

Initial-neutral minimum sampled clearances for the recommended set were:

| ID | Left m | Right m |
|---|---:|---:|
| small | 0.027404678 | 0.027498560 |
| medium | 0.028110568 | 0.028569833 |
| large | 0.028465837 | 0.029097021 |
| forward_low | 0.029558099 | 0.029558099 |
| forward_mid | 0.029558099 | 0.029558099 |
| forward_high | 0.029558099 | 0.029558099 |
| high | 0.029558099 | 0.029558099 |
| mid_high | 0.029558099 | 0.029558099 |
| far_high | 0.029558099 | 0.029558099 |

## Definition, rendering, submission, and guard map

### Definition and page rendering

- `include/openarm_ik_ros/portal_core.hpp:71-75` defines
  `NominalTestSamples` as exactly two named `Point`s.
- `src/portal_core.cpp:632-638` is the sole C++ coordinate definition. It
  returns the four six-decimal metre triples currently asserted by tests.
- `src/portal_page.cpp:20-21` hard-codes three buttons per side: dynamic
  `current`, static `small`, and static `medium`.
- `src/portal_page.cpp:27-28` embeds the two static samples per side into JS
  through twelve manual placeholders. `portal_page()` replaces those
  placeholders with `json_number()` output later in the same file.
- `src/portal_page.cpp:59` makes preset buttons field-fill-only. `current` uses
  the latest `/api/state` TCP; named entries copy from `samplesM`. No preset
  submits a move.
- `src/portal_page.cpp:45-57` keeps canonical metres in `targetsM`, renders cm
  or inches, and strictly reparses edits. `src/portal_page.cpp:66` sends an
  explicit unit to `/api/v2/move`; the server normalizes exactly once in
  `src/portal_core.cpp:612-629`.

The current shape does not scale cleanly: adding seven more fields to a struct,
three placeholders per side per target, replacement calls, and duplicated
button HTML is error-prone. Prefer an ordered `std::array<NominalTarget, 9>`
where each entry has stable `id`, user label, and `Point`, then serialize the
array and render buttons from that same data. Keep server C++ as the single
source of canonical metre values.

### Fresh measured-state and command path

- `src/openarm_portal.cpp:121-151` requires fresh joint state and diagnostics,
  copies both measured joint vectors, and computes both TCPs by public FK.
- The page only enables Move while state is fresh and no command is active.
- `src/openarm_portal.cpp:169-217` revalidates guard handoff evidence before
  submitting and builds a paired ROS action from the guard's two commanded
  TCPs.
- `src/openarm_portal.cpp:594-631` strictly parses/normalizes, takes a fresh
  state snapshot, evaluates `NominalPathGuard`, and only then invokes the ROS
  action.

### Nominal guard

- Constants at `src/portal_core.cpp:21-30`: arm capsule radius 0.050 m, tool
  radius 0.075 m, required clearance 0.025 m, finite central shaft radius
  0.04242640687119285 m spanning Z 0.008--0.758 m, and 17 samples.
- `src/portal_core.cpp:337-384` rejects nonfinite/out-of-limit joints and runs
  the public position-only IK with the measured/predecessor posture as both
  seed and posture. Settings are tolerance `1e-6 m`, max solver step `0.12
  rad`, damping `1e-5..0.1`, limit margin `1e-5 rad`, and 500 iterations.
- `src/portal_core.cpp:386-430` checks all 7x7 inter-arm capsule pairs and each
  arm segment against the finite central cylinder at every sample.
- `src/portal_core.cpp:433-494` starts from both measured joint vectors,
  changes only the selected arm TCP, linearly interpolates 16 subsequent TCP
  waypoints, re-solves both arms, enforces a `0.35 rad` per-sample branch jump
  bound, then checks the scene at all 17 samples.
- Acceptance is explicitly only sampled nominal virtual protection. It does
  not inspect meshes continuously and does not turn `collision_checked` true.

### Controller/runtime dependency

- Control's public-IK wrapper uses the same solver options
  (`control/src/kinematics.cpp:36-64`).
- The paired planner independently creates 17 Cartesian waypoints from the
  fresh measured start, checks IK/residual/limits/branch continuity, and keeps
  `collision_checked=false` (`control/src/control_core.cpp:660-747`).
- `VirtualControlSession` sends both TCPs with fresh feedback sequences,
  1e-3-m controller tolerance, 2-rad controller branch bound, and zero minimum
  singular-value threshold (`src/virtual_control_session.cpp:615-640`). The
  portal guard is therefore stricter on waypoint residual and branch jump for
  these candidates, while Runtime remains an independent fresh-state planner.

Dependency flow:

```
nominal_test_samples (canonical metres)
        -> portal_page samplesM + field-fill buttons
        -> /api/v2/move strict unit normalization
        -> freshest measured q/FK snapshot
        -> NominalPathGuard (17 public IK/FK + capsule/pole samples)
        -> handoff generation/freshness equivalence check
        -> paired ROS action (selected target + opposite measured TCP)
        -> VirtualControlSession / Runtime / Control 17-waypoint IK planner
        -> measured virtual completion
```

Build dependency is `openarm_portal_core -> openarm_model`; the portal adds ROS
messages/actions and X11/JPEG dependencies. `test_portal_core` links the core
and action message package (`CMakeLists.txt:84-103,119-132`).

## Existing test coverage and gaps

Existing `test/test_portal_core.cpp` covers:

- strict metre and explicit-unit JSON schemas and binary64 round trips;
- page unit/canonical-metre behavior and safety wording;
- exact four current preset triples;
- parsing and guard acceptance for both current presets from the canonical
  quantized neutral joint posture;
- rejection of a nearby central-pole approach;
- stationary/neutral and a nearby joint-three regression posture;
- capsule/cylinder cap/rim math, freshness, and guard handoff equivalence.

The exact current preset assertions and guard loop are at lines 319-352. Page
coverage is mostly substring-based and assumes only the existing hard-coded
shape. `test_virtual_control_session.cpp` has a generic paired TCP completion
test at `(0.20, +/-0.30, 0.85)`, but does not connect portal presets to fresh
measured Runtime planning.

Missing coverage before expansion:

1. every named target being present exactly once in C++ data and rendered once
   per appropriate side;
2. every named target passing strict JSON/unit normalization and the guard from
   the actual canonical measured posture;
3. target-to-target behavior after virtual encoder quantization;
4. a preset-to-actual-`VirtualControlSession` measured-completion test;
5. a structural page test that button IDs resolve to data entries, rather than
   adding many placeholder substring assertions.

## Pairs versus one-arm exposure

Expose these as **per-arm, one-selected-arm presets**, matching current portal
semantics. Internally the ROS command is paired, but the non-selected target is
the freshest measured TCP. That invariant is what the current guard proves.

Do not add a UI button that commands a preselected left/right pair in one click
with the current API. `MoveRequest` carries only one side and one target, and
the guard interpolates only that selected target. Two sequential button clicks
would not be an atomic pair and would be checked/executed as two different
fresh-state moves. A true pair feature needs a new paired request and a guard
that interpolates both requested TCPs simultaneously at all 17 samples, plus
paired-target/pair-to-pair tests. Until then, calling a left and right entry a
"pair" would overstate what is guarded.

## Recommended production tests

1. Replace `NominalTestSamples` with table entries and exact-assert all 18
   metre triples above, stable IDs, labels, finite values, and unique IDs.
2. For every table entry/side, encode through the page JSON path (including cm
   normalization), assert binary64 canonical metre recovery, and run the guard
   from the canonical measured neutral vectors.
3. Reproduce the endpoint-quantization matrix used here: solve the same 17
   waypoint path, quantize each target endpoint with the canonical manifest's
   DaMiao 16-bit position codec, then for every Cartesian product of
   `{neutral + 9 left endpoints}` x `{neutral + 9 right endpoints}`, guard all
   nine destinations for each selected arm. Assert all 1,800 accept and retain
   a recorded minimum above the actual 0.025-m acceptance boundary. The
   observed baseline is `0.026527860 m`; do not silently weaken the real guard
   merely to preserve a preset.
4. Add a slower `VirtualControlSession` integration test that starts from its
   callback's actual measured snapshot, re-runs the guard immediately before
   each submission, commands selected target plus opposite measured TCP, and
   requires measured completion for all 18 side/target cases. Also assert
   collision remains unchecked and no physical-motion capability appears.
5. Page tests should enumerate rendered `data-preset` IDs, verify buttons only
   call `preset(...)`, and verify only explicit Move invokes `/api/v2/move`.
6. Preserve the pole-approach negative test and add a generator rejection
   example for independent one-axis guesses, documenting the neutral IK issue.

Static presets can never promise acceptance from every arbitrary redundant IK
posture. The runtime guard must remain authoritative and may reject a preset
from an unusual fresh posture. The matrix and actual sequential test provide a
strong canonical-virtual regression envelope without relabeling it as safety.

For future additions, keep generation as an offline deterministic test/tool,
not browser/runtime discovery: compute quantized neutral by the canonical
manifest, FK and round the base once, enumerate integer-cm offsets in a bounded
lattice, require both sides to pass the unchanged guard, quantize candidate
endpoints, run the cross-state directed matrix, then execute representative
fresh-state sequences. Check in only the audited output table and its baseline.

## Investigation artifacts and reproducibility

Disposable files are under `build/target-expansion-recon/`:

- `probe.cpp`: current-guard lattice scan, current public IK path-end solver,
  canonical encoder quantizer, and 1,800-case matrix;
- `probe.log`: exact neutral FK, initial per-target results, and matrix result;
- `left-pass.txt`, `right-pass.txt`: exploratory accepted lattice points;
- `session_probe.cpp`: actual fresh-measured virtual-session integration probe;
- `session_probe.log`: all 18 measured completion results;
- `probe`, `session_probe`: local test executables.

Key final result lines:

```
CROSS checked=1800 failed=0 minimum=0.026527860 state=(8,0) side=0 dest=small
PASS small side=0 ...
...
PASS far_high side=1 clearance=0.037331778 duration_ns=5094179976
```

Both probes compile current `src/portal_core.cpp`; the session probe links the
current installed Model/Control/Commission/Runtime libraries and the current
workspace virtual-session library. It performs process-local virtual motion
only and opens no GUI, network listener, CAN interface, or physical transport.
