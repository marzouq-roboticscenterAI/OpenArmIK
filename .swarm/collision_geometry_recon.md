# OpenArm v1.0 collision-geometry reconnaissance

Date: 2026-07-29 (America/Los_Angeles)  
Status: **DONE_WITH_CONCERNS**  
Scope: read-only source/asset/history inspection; this report is the only write. No build,
test executable, GUI application, network, CAN, or hardware operation was performed.

## Decision

The pinned OpenArm v1.0 description has canonical collision meshes for the common
body/pedestal, `link0` through `link7` of both arms, each hand, and each finger. The
large central support is **not missing**: it is part of
`openarm_body_link0_collision`, along with the base plate, brackets/reinforcement,
and top arm mount. Its full mesh envelope is 250 x 190 x 773 mm. The primary
hardware documentation independently specifies a 60 x 60 x 750 mm aluminum
extrusion on a 250 x 190 x 8 mm base plate.

Those assets are authoritative for the pinned software model, not yet sufficient
authority for physical motion. Upstream publishes no tolerance stack, collision-
mesh generation recipe, mesh-to-STEP digest mapping, cable/cover/attachment
envelope, or as-built pedestal/base-frame calibration. The arm base rolls are also
stored as rounded `+/-1.5708`, not exact `+/-pi/2`. A validator can be implemented
now for the fixed virtual v1.0 model, but physical portal motion must remain blocked
until geometry and frame placement are measured and signed off.

The smallest robust validator is a compiled C++ narrow phase over the exact pinned
STLs (FCL BVHs), preceded by cheap generated AABBs and supplemented by an explicit
square-pole/base guard. One coarse capsule per link or one AABB for the whole body
is not acceptable: it either misses motors/hands or rejects large valid regions.
Primitive-only geometry can be a later optimization only when every source-mesh
vertex is proven contained by the compiled primitive union.

## Audited sources and provenance

The working manifest pins `openarm_description` at
`6c7b720f1ba48e8bafa3a3dc752c45f397b42221`, `openarm_hardware` at
`12c07510c09b2c10b7dfe48010dae5c05cbe887f`, and `openarm_mujoco` at release
`2.0.1` / `8955afb54e4adfb59a236e2b4d15192b7a02865c`.

Primary upstream references:

- [v1.0 description tree at the audited commit](https://github.com/enactic/openarm_description/tree/6c7b720f1ba48e8bafa3a3dc752c45f397b42221/assets/robot/openarm_v1.0)
- [parallel-link gripper at the audited commit](https://github.com/enactic/openarm_description/tree/6c7b720f1ba48e8bafa3a3dc752c45f397b42221/assets/end_effector/parallel_link)
- [v1.0 pedestal STEP retained at the `openarm_hardware` 1.0.1 release commit](https://github.com/enactic/openarm_hardware/blob/0403835afae64949ede85e440e86a283b747b9dd/SolidWorks/pedestal/pedestal_v1.0_final.STEP)
- [v1.0 hardware dimensions](https://github.com/enactic/openarm/blob/990fda921c82ae9d12b00f23e449793a9a313afd/website/versioned_docs/version-1.0/hardware/specifications/general.mdx)
- [v1.0 pedestal assembly](https://github.com/enactic/openarm/blob/990fda921c82ae9d12b00f23e449793a9a313afd/website/versioned_docs/version-1.0/hardware/assembly-guide/pedestal-assembly.mdx)
- [v1 MuJoCo model and its candidate contact exclusions](https://github.com/enactic/openarm_mujoco/blob/8955afb54e4adfb59a236e2b4d15192b7a02865c/v1/openarm_bimanual.xml)

No internet lookup was needed; all cited primary repositories are complete local
clones at those commits.

The repository model generator is stronger than a typical URDF snapshot. Its
`source_hash()` includes every tracked file under the complete v1.0 robot and
parallel-gripper trees, including collision-mesh bytes. The public model reports:

- flattened URDF SHA-256:
  `dd4b5d5fa0c050b78922bc95850ea65bb15721cb259030cccf106a953f171c55`;
- canonical source-tree SHA-256:
  `3f48ffec1598bebca34f90419521d5e320787746b66bf54937c3faeb7c6cb5fc`.

The collision library should require those exact identities, plus its own generated
geometry-table digest. Runtime path resolution to whichever STL happens to be
installed is not sufficient.

## Exact existing geometry

All collision assets are binary triangle STLs authored in millimetres and scaled
by `0.001`. The current xacro mirrors the left arm in mesh Y only for links 0, 1,
2, 5, 6, and 7. Links 3 and 4 are not mirrored. Each right finger mirrors the one
finger asset in mesh Y. The same arm and gripper assets are instantiated twice.

The table gives the collision origin and resulting local AABB after scale/origin,
in metres. `R` means the right-arm instantiation. `L` differs only where shown by
the Y reflection. Bounds are derived directly from every STL triangle, not from a
render.

| Geometry | Triangles | Collision origin xyz (m) | Local AABB min (m) | Local AABB max (m) |
|---|---:|---|---|---|
| body / pedestal | 5,864 | `0 0 0` | `-0.155000 -0.095000 0` | `0.095000 0.095000 0.773000` |
| R/L link0 | 804 | `0 0 0` | `-0.048936 -0.061000 0` | `0.049000 0.061000 0.062500` |
| R link1 | 354 | `0 0 -0.0625` | `-0.046715 -0.049227 0.000500` | `0.049009 0.049208 0.109318` |
| L link1 | 354 | same | `-0.046715 -0.049208 0.000500` | `0.049009 0.049227 0.109318` |
| R link2 | 266 | `0.0301 0 -0.1225` | `-0.018630 -0.049095 -0.048935` | `0.065030 0.049174 0.066633` |
| L link2 | 266 | same | `-0.018630 -0.049174 -0.048935` | `0.065030 0.049095 0.066633` |
| R/L link3 | 3,138 | `0 0 -0.18875` | `-0.042501 -0.032531 -0.000256` | `0.034781 0.032503 0.182323` |
| R/L link4 | 22,798 | `0 -0.0315 -0.3425` | `-0.042503 -0.070605 -0.028478` | `0.032500 0.007606 0.096501` |
| R/L link5 | 15,028 | `0 0 -0.438` | `-0.035800 -0.038001 0.002300` | `0.035800 0.038001 0.130500` |
| R link6 | 600 | `-0.0375 0 -0.5585` | `-0.075000 -0.038002 -0.028513` | `0 0.028004 0.028513` |
| L link6 | 600 | same | `-0.075000 -0.028004 -0.028513` | `0 0.038002 0.028513` |
| R link7 | 476 | `0 0 -0.5585` | `-0.028492 -0.028500 -0.017500` | `0.028489 0.034500 0.094500` |
| L link7 | 476 | same | `-0.028492 -0.034500 -0.017500` | `0.028489 0.028500 0.094500` |
| hand (each arm) | 364 | `0 0 -0.661001` | `-0.028499 -0.084058 -0.008001` | `0.028501 0.084058 0.007999` |
| left finger (each arm) | 264 | `0 -0.05 -0.661001` | `-0.030465 -0.007761 -0.002501` | `0.030501 0.027061 0.092420` |
| right finger (each arm) | 264 | `0 0.05 -0.661001`, mesh Y mirrored | `-0.030465 -0.027061 -0.002501` | `0.030501 0.007761 0.092420` |

Mesh SHA-256 values that a generated collision table should pin:

| Asset | SHA-256 |
|---|---|
| body | `6c18bbf7e86b03e3faf802e61e8eb438b38dcbcf146d97cffe6e808c65e9a72a` |
| link0 | `baf52578e1d9e6225f3818cae82b6074a0b948d3cef8e9a3e6dfafca78507590` |
| link1 | `066113d13d5cc85098609003bc7ebb73c570015350877f5ed7162ef1b6601852` |
| link2 | `382ab32e4ae0880e8a1512e7a6ca6ce1f478a6c125db4efa977429ffb1d6b02a` |
| link3 | `00c908cefab152c00416a570a48bf9aafed1549085f19ff2d882dc3f355d9f59` |
| link4 | `b54883b8c7c96268a68a5879f95998a53ad0b0c4fe74325fad63a6caef669c73` |
| link5 | `678a2802906eff7b45a836d2f34a2d8e51def50b6599376968f888e05c72739e` |
| link6 | `95529bec23733476dfdbbb266c7db0d25a473a568de73c8337a82440fe4a9ac3` |
| link7 | `434f207f21f75f5f0bd604e390b8e5bc7b62b619265222846770e06b3f9b5cfb` |
| hand | `8e5d373ebbd3fd001b506058644062ad71a68f1ced5ca5d5ed0f6de20137956b` |
| finger | `8e96e1314618cf434908f70df78f68dd2b049c03538964e8d41fc99abe41564d` |

All assets are closed by exact-edge counting except link3, which has one edge
with non-manifold incidence. That does not prevent triangle-BVH collision, but it
is another reason not to assume signed watertight-mesh distance without testing.

### Pedestal and central support

`openarm_body_link0` is fixed to `world` at identity. Its collision mesh is one
connected triangulated surface. The documentation and historical primary CAD say:

- base plate: 250 x 190 x 8 mm;
- pole: MiSUMi `HFSB6-6060-750`, 60 x 60 x 750 mm;
- total model height: 773 mm;
- brackets and a 30/60-degree reinforcement attach pole to plate;
- the arms align to the top of the extrusion.

The STL confirms the base/body full AABB above and a `[-0.030,+0.030]` square pole
cross-section through the central shaft. With the plate at Z `[0,0.008]`, the
750 mm extrusion nominally occupies Z `[0.008,0.758]`; the body mesh continues to
Z `0.773` for the top mount. Therefore a circular cylinder is the wrong exact
primitive for the pole. A 60 mm square box, expanded by the clearance margin, is
the correct conservative guard. The complete body mesh must still cover the
brackets, reinforcement, plate, and top mount.

A single AABB around the complete body is unusably coarse. At the model's zero
posture it overlaps the hand AABBs even though the hands are beside empty space,
which would reject legitimate poses. Use the body mesh as narrow phase and the
separate pole/base boxes as fast guards.

### Frame and FK conventions

The exact default arm roots in the body frame are:

| Side | Base xyz (m) | Base rpy (rad) |
|---|---|---|
| left | `0 +0.031 0.698` | `-1.5708 0 0` |
| right | `0 -0.031 0.698` | `+1.5708 0 0` |

The rounded roll produces the small `3.6732051e-6` off-axis terms visible in the
compiled FK. A collision engine must reuse the compiled model transform, not replace
it with an assumed exact quarter turn.

Per-arm joint origins and axes, before the side base transform, are:

| Joint | origin xyz (m) | axis (right / left difference) |
|---|---|---|
| J1 | `0 0 0.0625` | `0 0 1` |
| J2 | `-0.0301 0 0.0600`, roll `+/-pi/2` | `-1 0 0` |
| J3 | `+0.0301 0 0.06625` | `0 0 1` |
| J4 | `0 +0.0315 0.15375` | `0 1 0` |
| J5 | `0 -0.0315 0.0955` | `0 0 1` |
| J6 | `+0.0375 0 0.1205` | `1 0 0` |
| J7 | `-0.0375 0 0` | right `0 +1 0`, left `0 -1 0` |

`oa_fk()` starts at `base_in_body`, records each pre-joint transform, applies the
joint rotation, and stores `link_post[i]`. Thus collision transforms are deterministic:
link0 uses `base_in_body`; link N uses `link_post[N-1]`; each is multiplied by the
collision origin in the table. No separate ROS TF lookup belongs in the validator.

The hand is fixed `+0.1025 m` along link7 Z. The TCP is another `+0.0835 m` along
hand Z, so compiled FK uses `+0.1860 m` link7-to-TCP. Fingers are attached directly
to link7 at `(0,-0.005,0.1025)` with axis `(0,-1,0)` and
`(0,+0.005,0.1025)` with axis `(0,+1,0)`, range `[0,0.044] m`, with the second
mimicking the first.

The control state contains only 2 x 7 arm joints. It has no gripper position or
feedback. Until that changes, a physical validator must either reject unknown
finger state or collide against each finger's complete prismatic swept volume over
`[0,0.044]`. Assuming zero is not fail-closed. Unknown tools, payloads, camera
mounts, covers not contained by the mesh, and cables likewise reject unless entered
as bound scene attachments.

## Authority assessment

| Fact | Authority level | Finding |
|---|---|---|
| URDF link/joint/collision poses | software-authoritative | Pinned xacro is flattened and hashed; FK is independently compared with the flattened URDF. |
| Collision mesh bytes | software-authoritative | Pinned by the generator's full v1 source hash; identical byte hashes also appear in upstream MuJoCo v1. |
| Pole/base nominal dimensions | strong nominal hardware evidence | Docs, BOM, collision STL envelope, and historical pedestal STEP agree. |
| Mesh-to-CAD simplification error | unknown | No upstream recipe or tolerance report. `symp` meshes are not certified conservative hulls. |
| Body-to-real-pedestal/world transform | unknown physically | URDF says identity; no surveyed installation transform or mounting tolerance is stored. |
| Attachments/cables/carried object | absent | Not represented in this model/scene. |
| Finger state | absent from control | URDF has it; controller snapshots do not. |

The upstream MuJoCo v1 tree corroborates all eleven mesh byte hashes and provides
a useful candidate exclusion list. It is not an authority for this validator: its
body collision geom selects a visual OBJ rather than the declared body collision
STL, it carries older gripper transforms, and its left-link3 mirroring differs from
the current audited xacro. Generate from the audited URDF, not MuJoCo XML.

## Current control collision/scene hook

There is no scene-validation hook today.

- `collision_allowed()` returns true only for virtual
  `OA_COLLISION_VIRTUAL_UNCHECKED`; all normal/default plans return
  `OA_CONTROL_ECOLLISION`.
- Paired TCP planning constructs 17 predecessor-seeded Cartesian IK knots, then
  explicitly sets `plan->collision_checked = false`.
- The executor interpolates each knot pair with a seventh-order joint-space time
  law. It performs no collision query at knots, between knots, or on measured
  feedback.
- `collision_scene_revision` is only a caller/controller integer. The setter checks
  nonzero, virtual backend, and not currently executing; it validates no geometry,
  digest, frame, or monotonicity.
- Plans do bind controller instance, verification epoch, manifest revision, model
  revision, scene integer, both feedback sequences, and both measured starts.
  Execute rechecks those bindings. That is a useful seam, not collision validation.
- Tests prove reject-all blocks a joint plan, unchecked virtual plans report zero,
  and changing the scene integer makes an old plan stale. They contain no collision
  geometry, pole, arm-arm, interpolation, or validator-failure test.

## Recommended compiled validator

### Geometry engine

Add a small `openarm_collision` C++17 library and keep its C ABI opaque.

1. Generate `oa_v10_collision_data.inc` from the audited flattened URDF and exact
   STL bytes. Embed vertices/triangles, collision origins, local AABBs, side scales,
   allowed-pair IDs, mesh hashes, source hash, and generator hash. Build must fail on
   any mismatch or dirty canonical checkout.
2. Build immutable FCL `BVHModel<OBBRSSd>` objects once. At a query, obtain the exact
   arm-link transforms from the same FK constants used by `openarm_model`, then set
   FCL object transforms. Use generated AABBs for broad phase and FCL distance/
   collision for narrow phase.
3. Independently check the pole box `[-.030,+.030]^2 x [.008,.758]`, the plate box
   `[-.155,+.095] x [-.095,+.095] x [0,.008]`, and the complete body mesh. The
   duplicate pole/base checks make accidental body-mesh omission observable.
4. Treat both hands and fingers as robot geometry. Until finger state is measured,
   use the full finger swept volume; do not silently omit the gripper.
5. Return a structured result: OK, COLLISION, INSUFFICIENT_CLEARANCE, UNKNOWN,
   INVALID_MODEL, INVALID_SCENE, or BUDGET_EXHAUSTED, plus pair IDs, minimum signed
   distance, sample time, knot/segment, scene epoch, and geometry digest. Every
   result except OK rejects planning.

FCL is a larger dependency than hand-written capsule math, but is the smallest
implementation that preserves the only canonical geometry presently available.
The assets total about 43k arm triangles per side plus 5.9k body triangles, which is
small for immutable BVHs. Primitive alternatives are acceptable only as generated
conservative unions whose containment is tested against every STL vertex and face.
A sphere/capsule around each joint-to-joint segment alone is unsafe because link0-2
motors, link4/5 bodies, wrist, hand, and fingers extend materially off those axes.

### Pair policy

Always check:

- every left-arm object against every right-arm object;
- each arm object against the complete body, pole, and plate;
- every non-allowed pair within each arm/gripper;
- all registered environment and attached-object geometry.

Never infer allowed contacts from current overlap. Compile an audited pair allowlist.
The initial candidate, to be confirmed against exact mesh queries over joint limits,
is:

- fixed body-to-link0 mount on each side;
- same-arm mechanical nesting pairs matching upstream intent:
  `(0,1), (0,2), (0,3), (1,2), (1,3), (2,3), (3,4), (4,5),
  (5,6), (5,7), (6,7)`;
- fixed link7-to-hand and link7-to-finger mechanism pairs;
- hand-to-own-fingers and own-finger-to-own-finger closure contact.

An allowed pair is not a wildcard: where practical, give it a maximum permitted
penetration/contact region. No inter-arm pair and no moving-link-to-pole/body pair
is allowed. The MuJoCo list is evidence for candidates, not a file to copy blindly.

### Margins

Use signed distance, not just triangle intersection. Margins are fixed policy, not
portal parameters. A reasonable virtual commissioning default is 5 mm for self
clearance and 10 mm for inter-arm/body/environment clearance, with an additional
small numerical epsilon. These values are deliberately provisional, not physical
safety claims. Physical values must be at least the measured sum of mesh
simplification, manufacturing, joint/backlash, calibration, tracking, latency, and
attachment uncertainty; unknown uncertainty keeps the backend disabled.

Do not inflate only the broad-phase boxes and then report success from them. The
narrow-phase distance must certify the required margin.

### Every knot and the full time trajectory

Validation occurs after all IK waypoints and segment durations are finalized but
before an immutable plan is published:

1. Check measured start and all 17 Cartesian IK knots for both arms together.
2. Reproduce the exact seventh-order `q(t)` used by execution and check every
   nominal controller-cycle time, every segment boundary, and the final state.
3. Nominal cycle samples alone do not certify arbitrary `advance()` times. For each
   interval, recursively subdivide and use a conservative bound on maximum point
   displacement from joint delta and each link's radius/reach. Accept an interval
   only when endpoint minimum clearance exceeds required margin plus that motion
   bound. Otherwise subdivide or use FCL continuous collision detection. Hitting a
   depth/sample/time limit is UNKNOWN and rejects.
4. At runtime, check each fresh measured two-arm snapshot and the next reference
   against the same immutable scene. Loss of clearance, scene mismatch, nonfinite
   FK, or validator failure requests disable-stop and latches fault. Planning alone
   cannot cover tracking error or an externally moved obstacle.

This catches the important case where both IK knots are clear but the joint-space
smooth interpolation sweeps an elbow or hand through the pole/other arm.

### Scene object and epoch binding

Do not expose the opaque plan's waypoint array through a callback. Put validation
inside the serialized controller planning path and add immutable scene handles:

```c
typedef struct oa_collision_scene oa_collision_scene;

oa_control_status oa_collision_scene_create_openarm_v10_standard(
    const oa_collision_scene_config *config,
    oa_collision_scene **out);

oa_control_status oa_controller_create_with_scene(
    const oa_manifest *manifest,
    const oa_controller_options *options,
    const oa_collision_scene *scene,
    oa_controller **out);
```

The controller retains immutable scene storage. The scene owns a controller-issued,
nonzero monotonic epoch and a content digest over geometry, transforms, margins,
allowed pairs, model/source hashes, and attachments. Requests bind the epoch;
plans bind epoch plus digest; execute rechecks both. Replacing a scene is allowed
only while non-executing, creates a new epoch even for repeated content, and
invalidates every old plan. Revision integers supplied by a browser or ROS caller
must never manufacture scene authority.

For ABI compatibility, leave existing V1 records and symbols unchanged. Add new
symbols/records or append only optional tails with prefix-size handling. Physical
controller creation must require a real scene; unchecked virtual mode can remain
only as an explicitly named test/demo policy and can never set
`collision_checked=true`.

## Required tests

1. **Generation/integrity:** regenerate byte-identical collision tables; reject
   wrong commit, dirty source, mesh hash, flattened URDF hash, scale, origin, or
   topology; assert the AABBs/triangle counts above.
2. **Transform parity:** for zero, limits, and thousands of deterministic random q,
   compare every collision-link transform with independent flattened-URDF FK,
   including rounded base rolls and all left reflections.
3. **Pole/base goldens:** place every moving link just outside/inside/tangent to the
   60 mm pole, plate, reinforcement, and complete body; verify margin behavior on
   both sides.
4. **Arm-arm goldens:** crossing forearms, wrists, hands, and fingers; include
   tangency, sub-epsilon separation, and clear mirrored configurations.
5. **Self/allowlist:** every allowed pair at representative limits, every neighboring
   non-allowed pair, and mutation tests that removing/adding one allowlist entry
   fails. No inter-arm/body allow entry is possible.
6. **Gripper unknowns:** open, closed, midrange, full swept envelope, missing state,
   unknown tool, and a registered attached object. Missing information rejects or
   uses the explicitly proven conservative sweep.
7. **Trajectory tunneling:** clear endpoints/IK knots with a colliding midpoint;
   irregular controller times; very short/long segments; sample/depth/budget
   exhaustion; all must fail closed.
8. **Hook/epoch:** absent validator, exception/error status, NaN distance, stale
   epoch, same integer with changed digest, scene replacement before execute, and
   replacement while executing. Only a complete bound result may set the report
   flag.
9. **Control integration:** joint, future single-TCP, and paired-TCP planners invoke
   validation over both arms and inactive-arm hold; reports expose checked status;
   execute rejects stale scene/model/manifest/start/verification bindings.
10. **Runtime monitor:** measured deviation toward collision, stale/nonfinite
    feedback, and scene change cause stop/fault before another motion reference is
    authorized.
11. **Differential/fuzz:** compare FCL results with an independent brute-force
    triangle oracle on small fixtures; fuzz finite/nonfinite transforms, degenerate
    triangles, near contact, huge values, and allocation/budget failure under
    ASan/UBSan.
12. **Physical gate:** prove every unchecked, unknown, zero-margin, uncalibrated,
    or attachment-incomplete scene returns no physical-motion capability.

## Bottom line for portal work

The portal/control effort may proceed in virtual mode only after this validator is
inside planning and `collision_checked=true` is backed by its immutable scene
result. The pole is already available in canonical geometry and should be checked
both as its exact body mesh and as an explicit square-box guard. Until the missing
physical uncertainty/frame/attachment evidence is supplied, keep physical motion
fail-closed and label any unchecked virtual result honestly.
