# Adversarial collision-design verification

Date: 2026-07-29 (America/Los_Angeles)  
Status: **DONE_WITH_CONCERNS**  
Scope: repository/upstream/system inspection and this report only. No build,
GUI, network, CAN, or hardware operation was performed.

## Verdict

A fail-closed collision validator for the pinned **virtual OpenArm v1.0 model**
is implementable. It must not be described as making the real robot safe.

The 60 x 60 x 750 mm support extrusion is not imaginary or absent. The pinned
v1.0 hardware instructions specify it, the pinned collision STL contains its
roughly 60 mm square shaft, and the flattened URDF gives the body mesh an
identity collision origin and `0.001` millimetre-to-metre scale. The nominal
virtual guard is therefore a box in `openarm_body_link0`:

```
min = (-0.030, -0.030, 0.008) m
max = (+0.030, +0.030, 0.758) m
center = (0, 0, 0.383) m
size = (0.060, 0.060, 0.750) m
```

That record is suitable for a digest-bound virtual scene, as a conservative
outer guard used in addition to the complete body mesh. It is **not** a survey
of an installed pedestal. The repository has no as-built pose, tolerance stack,
mesh simplification error, cable/attachment envelope, payload geometry, or
physical tracking-error bound. Physical motion must remain unsupported.

The smallest credible implementation is a hybrid:

* exact canonical triangle BVHs for body, links, and hand;
* the explicit square-pole box above as a redundant conservative guard;
* fixed conservative full-stroke finger boxes because finger state is absent;
* FCL 0.7 for Boolean collision and non-colliding separation distance; and
* deterministic interval subdivision with kinematic displacement bounds for
  continuous path certification.

Do not implement a custom GJK engine or capsule-only robot first. Capsules miss
off-axis motors, wrists, hands, and fingers unless containment is proved. One
OBB per complete link can be conservative but can also make normal arm postures
unusable. Exact source collision meshes plus a deliberately conservative
unknown-gripper envelope are the smaller reviewable design.

## Evidence: pedestal geometry and pose

The audited checkouts match the manifest claims:

| Repository | Inspected commit |
|---|---|
| `openarm_description` | `6c7b720f1ba48e8bafa3a3dc752c45f397b42221` |
| `openarm_hardware` | `12c07510c09b2c10b7dfe48010dae5c05cbe887f` |
| `openarm` docs | `990fda921c82ae9d12b00f23e449793a9a313afd` |
| `openarm_mujoco` | `8955afb54e4adfb59a236e2b4d15192b7a02865c` |

Primary local evidence:

* `website/versioned_docs/version-1.0/hardware/assembly-guide/pedestal-assembly.mdx`
  says “750mm 60x60 aluminum extrusion,” placed on the base plate.
* `general.mdx` identifies the support pillars as MiSUMi aluminium frames.
* `final-assembly.mdx` says to align the top of J1 to the top of the extrusion.
* Historical `openarm_hardware@0403835...` contains
  `SolidWorks/pedestal/pedestal_v1.0_final.STEP`. This is provenance evidence,
  not a measured installation transform.
* The flattened URDF fixes `openarm_body_link0` to `world` at identity. Its body
  collision has identity origin and mesh scale `0.001 0.001 0.001`.

I independently parsed the binary body STL rather than accepting the prior
report's table:

| Property | Observed |
|---|---|
| File | `body_link0_symp.stl` |
| SHA-256 | `6c18bbf7e86b03e3faf802e61e8eb438b38dcbcf146d97cffe6e808c65e9a72a` |
| Bytes / triangles | 293,284 / 5,864 |
| Raw AABB (mm) | `[-155,-95,~0]` to `[95,95,773]` |
| Triangle degeneracy / non-finite values | 0 / 0 |
| Exact-edge incidence | all 8,796 edges have incidence 2 |
| Edge-connected triangle components | one component containing all 5,864 triangles |

Between Z=100 and 600 mm, all observed body-mesh vertices are within approximately
`X,Y = +/-30 mm`; repeated outer-profile values include `+/-29.931852`,
`+/-29.73205`, and `+/-29.414213` mm. At Z=0 and Z=8 mm the mesh spans the
250 x 190 mm plate. Above about Z=620 mm, arm mounting features extend well
outside the 60 mm shaft, and the mesh reaches Z=773 mm.

Consequences:

1. The shaft is part of the body collision geometry, but it is not a separately
   named mesh component. The entire STL is one connected surface, so selecting a
   “pole component” by connected-component analysis is invalid.
2. The 60 mm box is a conservative outer envelope of the nominal T-slot profile,
   not an exact reproduction of its grooves/chamfers. This is desirable for pole
   avoidance.
3. The body mesh must still be queried. The pole box does not cover the plate,
   lower reinforcement/brackets, or upper mounts.
4. `body_link0` identity is software-authoritative only. It says nothing about a
   table/world registration in an actual installation.

Thus the strong A0 claim in `portal_design_synthesis.md` is too strict for a
purely virtual scene: a separately surveyed pole is not needed to validate the
pinned virtual model. It remains correct for any physical-safety claim. The
opposite claim in `collision_geometry_recon.md` also needs qualification: the
canonical STL is exact only with respect to this software asset, not necessarily
a conservative model of manufactured hardware.

## Dependency and licensing audit

### Host availability

| Candidate | This host | Finding |
|---|---|---|
| hpp-fcl / coal | absent | No pkg-config entry, headers, library, CMake config, installed package, or apt candidate was found. Do not design against it on this host. |
| FCL | not installed | Apt metadata offers `libfcl-dev 0.7.0-3ubuntu1` and `libfcl0.7`; no current build can use it without adding packages. |
| Assimp | installed | `libassimp-dev` and `libassimp6` 6.0.4 are installed; CMake and pkg-config metadata are present. Assimp imports meshes but performs no collision checking. |
| Bullet | installed | `libbullet-dev` / `libbullet3.24t64` 3.24 are present. Bullet is a possible engine, not a drop-in equivalent for reviewed exact mesh separation/continuous certification. |
| Eigen | installed | `libeigen3-dev` 3.4.0 is present. |

Adding Ubuntu FCL would add approximately 11 MiB installed size beyond the
already installed Eigen package: FCL dev/runtime, libccd dev/runtime, and
OctoMap dev/runtime. `openarm_control` is a static archive, so hiding FCL behind
a `PRIVATE` CMake edge does not remove the final consumer's link dependency.
The exported config must `find_dependency(fcl)`, installed-consumer tests must
cover it, the dependency installer must include `libfcl-dev`, and ROS package
metadata must express the system dependency.

FCL is generally distributed under a BSD license, but no FCL package copyright
file is installed here. License approval must verify the exact Ubuntu source and
transitive libccd/OctoMap notices before release; this inspection cannot provide
that missing local package evidence. Assimp's installed Debian copyright records
the core as BSD-3-Clause (with multiple licenses in auxiliary files). Bullet's
installed Debian copyright records the core as Zlib and several auxiliary
licenses. If either is linked, ship the applicable binary/source notices rather
than copying a one-line license label into `NOTICE`.

The OpenArm URDF/mesh inputs are Apache-2.0 and already require preservation of
their attribution. A generated vertex/index table remains derived from those
assets and must retain that provenance.

### Engine decision

Use Ubuntu **FCL 0.7**, not hpp-fcl, if adding the dependency is acceptable.
Assimp should not be a runtime dependency: these are known binary STL files, and
a small deterministic generation-time parser can enforce byte length, triangle
count, finiteness, scale, origin, hashes, and indices before emitting C++ arrays.
Runtime file discovery/import would weaken reproducibility.

Bullet is already installed, but adopting it only to avoid FCL is not smaller:
moving concave mesh handling, separation distance, collision margins, and error
semantics would all require a separate validation effort. A future Bullet or
convex-proxy backend is acceptable only behind the same backend-independent test
oracle. A custom GJK implementation is not appropriate for the first release.

Do not promise “signed mesh distance.” The required contract is:

* Boolean collision/intersection;
* nonnegative separation for a non-colliding pair; and
* `INDETERMINATE` on exceptions, non-finite output, unsupported geometry, or
  iteration/resource exhaustion.

Penetration depth is unnecessary to reject a path and must not be fabricated.

## Smallest implementable checked scene

### Generated immutable geometry

Generate one internal `openarm_collision` C++17 target from the pinned flattened
URDF and the eleven STL byte streams. Emit vertices, triangles, collision origins,
left/right signed scales, local AABBs, object/joint ancestry, mesh hashes, model
source/URDF hashes, pole box, margin, pair policy, and per-joint displacement
radii. The build verifies generated output; runtime never opens a mesh file.

Objects are:

* fixed complete body triangle mesh;
* fixed pole `Box(0.060,0.060,0.750)` at body XYZ `(0,0,0.383)`;
* exact link0..link7 triangle mesh for each side, transformed directly from
  `oa_fk().base_in_body` / `oa_fk().link_post[]` and the generated collision
  origin;
* exact hand mesh for each side, fixed from link7; and
* two link7-fixed conservative finger-sweep boxes per side, each derived from
  the finger mesh AABB at both prismatic endpoints `[0,0.044]` m.

The convex hull/AABB of both translation endpoints contains every intermediate
translated finger mesh. This is one deliberate OBB use whose containment is easy
to prove. It is preferable to pretending the unmeasured gripper is closed. A
30 mm link7/TCP-centered keep-out sphere may be added as explicit policy, but it
must be labelled a keep-out, not physical tool geometry. Unknown tools, payloads,
cameras, cables, or carried objects reject motion until added to a new scene.

Use the complete body mesh and the pole box simultaneously. They are not
alternatives. A broad-phase AABB may skip a narrow-phase query only when the
AABBs, expanded with outward rounding by the required margin, are disjoint.

### Allowed-contact policy

Start with the pinned v1.0 SRDF's narrow list, not the broader MuJoCo list. In
the internal composite-tool representation, allow only:

* body vs left link0 and body vs right link0 (both are fixed mounts);
* same-side directly connected `linkN` vs `linkN+1`, N=0..6;
* same-side link7 vs hand/finger-sweep tool components; and
* internal same-side hand/finger/finger pairs.

Always check:

* all non-adjacent same-arm pairs;
* every left object against every right object;
* every moving object/tool component against the complete body and pole box;
* the two tool envelopes against one another; and
* all registered attachments/environment objects in any future scene.

No inter-arm pair and no moving-body/pole pair is allowed. In particular, do not
copy MuJoCo exclusions `(0,2)`, `(0,3)`, `(1,3)`, or `(5,7)` without separate
proof. Do not infer allowed contact from an overlap observed in one posture.

Full adjacent-pair exclusion can hide an unintended collision away from a joint.
For the first virtual release it is acceptable only because it matches the pinned
SRDF and is backed by exhaustive one-DOF adjacent-pair sweeps over each joint's
limits. If those sweeps reveal contacts outside the mechanical joint neighbourhood,
the pair must be split into generated mount and non-mount triangle groups; adding
another blanket exclusion is not an acceptable fix.

### Margin and result semantics

Use one fixed portal release clearance of **0.025 m** for every checked pair,
plus a separately recorded numerical guard of at least `1e-6 m`. A certificate
passes only when the conservative lower bound is at least `0.025001 m`. The
25 mm value is product policy for the virtual model, not a derived physical
safety distance. It may make poses unavailable; false rejection is preferable
to a false clear result.

Internal result states must be at least:

* `CLEAR_FOR_BOUND_VIRTUAL_SCENE`;
* `COLLISION_OR_MARGIN_VIOLATION`;
* `INDETERMINATE`;
* `INVALID_MODEL`; and
* `INVALID_SCENE`.

Only the first permits publication of a plan with `collision_checked=1`.
Everything else returns no plan. Reports include scene epoch/digest, model
digests, required margin, nonnegative minimum separation, limiting pair,
segment/time interval, subdivision count, and whether the gripper was a full
stroke envelope. Never expose a bare field called `safe`.

### Continuous/swept certificate

Checking 17 IK knots or controller-cycle samples is insufficient. After all
waypoints and exact seventh-order segment durations are finalized:

1. Check the measured two-arm start, every knot, and the final state.
2. For each segment and checked pair, evaluate collision and separation at the
   interval midpoint.
3. Bound each object's maximum point displacement over that interval using its
   generated maximum radius from every influencing joint axis and the monotone
   joint-angle interval. `sum(radius_j * abs(delta_q_j))` is conservative; use
   outward-rounded arithmetic.
4. Certify the interval only if midpoint separation is greater than required
   margin + both displacement bounds + numeric guard.
5. Otherwise subdivide. An exact intersection/margin violation rejects. Maximum
   depth, time, query, allocation, or numeric budget exhaustion is
   `INDETERMINATE` and also rejects.

This avoids depending on FCL continuous-collision support for arbitrary moving
triangle meshes and proves the complete smoothstep path, including a clear-endpoint
but colliding-midpoint case.

At execution, validate every fresh coherent measured two-arm state and the next
reference interval under the same scene. The runtime check must include a
conservative tracking tube or measured-to-next-reference displacement bound;
checking only the nominal plan cannot cover plant deviation. Any stale/non-finite
feedback, margin loss, scene mismatch, or validator error latches disable-stop.

## Scene authority and epochs

The existing scene integer is not authority. Today any caller can set any
nonzero revision while non-executing, revisions can be reused or decreased, no
geometry is loaded, paired plans explicitly set `collision_checked=false`, joint
plans leave it false by default, and execute does not require it to be true.

The smallest safe ABI is an immutable scene passed at controller creation:

```c
typedef struct oa_collision_scene oa_collision_scene;

oa_control_status oa_collision_scene_create_openarm_v10_virtual(
    oa_collision_scene **out);
oa_control_status oa_controller_create_with_scene(
    const oa_manifest *, const oa_collision_scene *,
    const oa_controller_options *, oa_controller **out);
```

Scene creation assigns a process-monotonic nonzero epoch and a SHA-256 content
digest over every geometry byte/hash, transform, pole record, margin, numerical
policy, allowed pairs, gripper envelope, model/source hashes, and generator
identity. The controller snapshots the immutable scene. For the first release,
there is no scene-update API: changing geometry requires disarm, controller
destruction, new scene/controller creation, and re-verification. This is smaller
and eliminates an update race.

Each plan binds controller instance, verification epoch, scene epoch **and
digest**, manifest/model revisions, both feedback sequences, both measured starts,
and certificate policy version. Execute rechecks all bindings and requires a
complete clear certificate. The legacy revision setter may remain for ABI
compatibility only under `OA_COLLISION_VIRTUAL_UNCHECKED`; a checked controller
must return `OA_CONTROL_EUNSUPPORTED` from it.

ROS/browser fields echo the controller-issued epoch/digest; they never create or
select authority. RViz should receive the existing exact robot/body model and a
pole `visualization_msgs/Marker` generated from the same compiled scene record
(size `0.06,0.06,0.75`, pose `0,0,0.383`). Marker content is display-only and is
tested against the scene report/digest. The current RViz config has collision
display disabled; visual display settings must not affect collision validation.

## Exact adversarial test set

All safety-relevant tests are compiled C/C++ tests unless a test only checks the
generated ROS description. Required cases:

1. **Source integrity:** assert all eleven mesh hashes, exact binary lengths,
   triangle counts, AABBs, finite vertices, zero-area count, flattened URDF/source
   hashes, scale/origin, and generated-table digest. Mutate one byte, sign of a
   mirror scale, collision origin, pole dimension, or generator version and
   require build/generation failure.
2. **Body/pole proof:** assert body identity, full AABB, 250 x 190 x 8 mm plate,
   shaft containment by the 60 mm box over the declared shaft range, pole center
   Z `0.383`, and top/bottom `0.008/0.758`. Prove upper mounts and brackets exist
   outside the pole box, so omission of the body query fails a mutation test.
3. **Transform parity:** for zero, every joint limit, and thousands of fixed-seed
   random configurations, compare every collision object transform to independent
   flattened-URDF FK. Include rounded base rolls and every negative mesh scale.
4. **Pair-policy mutation:** enumerate all checked/excluded pairs. Removing any
   checked inter-arm/body/pole pair, or adding MuJoCo's non-adjacent exclusions,
   fails. Sweep each excluded adjacent pair over its one-DOF relative limit and
   verify contact remains in the approved joint neighbourhood.
5. **Margin boundaries:** with test-only rigid-transform fixtures, place a link
   mesh at pole side, corner, bottom, and top separations of margin-minus-epsilon,
   exact margin, and margin-plus-epsilon. Repeat against plate/body, left-right
   forearms, wrists, hands, and tool boxes. Exact/tangent and subnormal/non-finite
   cases must not pass.
6. **Gripper containment:** for both mirrored sides and positions 0, 0.022, 0.044
   m plus fixed-seed random positions, prove every transformed finger-mesh vertex
   is inside the generated full-stroke box. Place obstacles where only an open,
   only a closed, and only an intermediate finger would hit; all must reject with
   unknown finger state. Unknown tool/payload must reject scene creation.
7. **Trajectory tunnelling:** use a synthetic sphere/box path with clear endpoints
   and a colliding midpoint, then an actual arm golden crossing the pole. Exercise
   seventh-order segments, zero/small/large deltas, irregular advance times,
   reversed joint deltas, and simultaneous two-arm motion.
8. **Certificate budgets:** force maximum subdivision depth, query budget,
   allocation failure, FCL exception/non-finite distance, and radius overflow.
   Every case is indeterminate, returns no plan, and never sets checked true.
9. **Scene binding:** absent scene, wrong model digest, zero/reused caller revision,
   stale epoch, same epoch/different digest, destroyed source scene after controller
   snapshot, controller recreation, reset/reverify, and plan from another controller.
   Only exact bindings execute.
10. **Runtime deviation:** drive measured q toward body/pole/other arm while the
    reference remains clear; test stale, skewed, and non-finite feedback. The next
    reference is not issued and disable-stop/fault is latched.
11. **No-false-clear invariant:** fuzz finite/non-finite transforms and q, malformed
    generated indices, near-contact triangles, and backend failures under
    ASan/UBSan. Assert that every exit path other than a complete certificate has
    no plan handle and `collision_checked=0`.
12. **Engine differential:** compare FCL Boolean/separation results on small
    tetrahedron/box fixtures against an independent brute-force triangle oracle;
    include coplanar, edge-edge, vertex-face, degenerate test fixtures, and large
    coordinates. Disagreement is indeterminate.
13. **Install/package:** build installed C and C++ consumers from a clean prefix,
    verify exported FCL dependency discovery and notices, and verify the ROS node
    links no CAN/transport/commission library.
14. **ROS/RViz contract:** parse installed URDF/marker output and assert the exact
    body mesh URI/scale, pole pose/size, body frame, and scene digest. Changing an
    RViz setting, hiding the marker, or publishing a forged marker cannot alter
    controller authority.
15. **Physical gate:** every unchecked, uncalibrated, attachment-incomplete,
    zero-margin, or virtual-only scene returns no physical-motion capability.

## Release boundary

This design can truthfully claim only: “the complete commanded and monitored
trajectory was collision-checked against the pinned virtual OpenArm v1.0
body/arms, nominal support-pole guard, full-stroke gripper envelopes, and explicitly
registered scene objects at scene epoch/digest X with 25 mm policy clearance.”

It cannot claim hardware safety, obstacle completeness, collision avoidance for
unregistered payloads/cables/people, safety-rated stop behavior, or calibrated
world registration. Those omissions remain hard physical-backend blockers.

