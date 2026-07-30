# Portal nominal-guard geometry investigation

Date: 2026-07-29 (America/Los_Angeles)  
Repository/commit: `/home/signalprocessing-dev/OpenArmIK`, `ff38733`  
Status: **FINDINGS**  
Scope: read-only production inspection plus native temporary probes; this report
is the only repository write. No GUI, screenshot, network, CAN, or hardware was
used.

## Finding

The neutral rejection is a false positive caused by the body primitive, not an
intended body/link contact at guard segment 2. `NominalPathGuard` models the
pedestal as a **230 mm diameter cylinder** from Z=0 through Z=0.775 m. The
canonical pedestal shaft is a **60 x 60 mm square** from approximately Z=0.008
through Z=0.758 m. The complete body also has a 250 x 190 mm base and distinct
upper mounts; it is not a cylinder of radius 115 mm.

The guard's segment 2 is the moving `link3` centerline from J3 to J4. It is not
the fixed arm mount. Skipping segment 2 is therefore not justified, and would
not fix neutral anyway: left segment 3 and both arms' downstream segments also
fail the oversized cylinder.

An exact canonical-mesh query at all-zero q gives, symmetrically:

- body-to-fixed-`link0`: 0.9995 mm (the legitimate fixed mounting exclusion);
- body-to-closest moving link (`link1`): **29.4746 mm**;
- body-to-`link3` represented by guard segment 2: 41.7727 mm left and
  41.8185 mm right;
- body-to-hand: 39.4400 mm;
- closest moving left/right geometry: hand/hand, **138.8800 mm**.

Thus neutral clears the requested 25 mm for actual moving links, the opposite
arm, and the modelled hand/finger tools. Only fixed body/`link0` mounting
geometry needs an allowed-contact entry.

## Reproduction from public FK

`scene_clear()` constructs `points[side][0..6]` from
`oa_fk_result.joint_pre[]` and point 7 from `hand_tcp`. The capsule mapping is:

| Guard segment | End points | Nominal represented geometry | Current use |
|---:|---|---|---|
| 0 | J1 -> J2 | `link1` | omitted |
| 1 | J2 -> J3 | `link2` | arm-arm only |
| 2 | J3 -> J4 | `link3` | arm-arm and body |
| 3 | J4 -> J5 | `link4` | arm-arm and body |
| 4 | J5 -> J6 | `link5` | arm-arm and body |
| 5 | J6 -> J7 | `link6` | arm-arm and body |
| 6 | J7 -> TCP | `link7` plus fixed hand/tool | arm-arm and body |

`link0`, the exact off-axis link shapes, and finger geometry are not represented.
Arm-arm starts at segment 1; body checks start at segment 2.

For all-zero q, public FK gives the left segment-2 endpoints:

```
J3 = (0, 0.153499757, 0.631749550) m
J4 = (0, 0.121999192, 0.477999666) m
minimum radial axis distance = 0.121999192 m
```

The guard calculation in `portal_core.cpp` is exactly:

```
0.121999192 - kBodyRadius(0.115) - kArmRadius(0.050)
= -0.043000808 m
```

A temporary native program that linked `portal_core.cpp` and
`model/src/openarm_model.c`, targeted the stationary neutral left TCP, returned:

```
accepted=0
reason=nominal central pole/body keepout clearance is not proven for arm 0 segment 2 (clearance -0.043001 m)
TCP=(0, 0.153497715, 0.075999550) m
```

The observed `-0.043015 m` corresponds to a radial centerline distance of
0.121985 m, only 0.014 mm from exact all-zero FK. It is the same calculation
with a tiny measured-state deviation/rounding, not a different collision.

At exact neutral, every current body result is below the 25 mm threshold:

| Side | seg 2 | seg 3 | seg 4 | seg 5 | seg 6 |
|---|---:|---:|---:|---:|---:|
| left | -43.001 mm | -43.001 mm | -11.501 mm | -11.502 mm | -36.502 mm |
| right | -11.500 mm | -11.501 mm | -11.501 mm | -11.502 mm | -36.502 mm |

So excluding segment 2 merely changes the first error.

## Canonical body geometry versus the proxy

The flattened URDF binds `openarm_body_link0_collision` at identity to
`body_link0_symp.stl`, scaled from millimetres by 0.001. Direct inspection of
the pinned mesh records:

- complete body AABB: X `[-0.155,+0.095]`, Y `[-0.095,+0.095]`,
  Z `[0,+0.773]` m;
- base plate: approximately 250 x 190 x 8 mm;
- shaft: X/Y `[-0.030,+0.030]`, approximately Z `[0.008,0.758]` m;
- upper arm-mount features outside the shaft above roughly Z=0.620 m.

The current radius-0.115 cylinder is therefore over-conservative by 72.6 to
85 mm around most of the shaft, but does not faithfully cover the asymmetric
base or upper mounting features. There is no source provenance or containment
test for 0.115 m in the guard.

A cheap conservative shaft-only replacement is the square's circumscribed
circle, radius `sqrt(2)*0.030 = 0.042426407 m`, with the shaft's actual Z
interval. Keeping all current moving capsules, neutral then gives:

- left minimum shaft clearance: **29.5728 mm**;
- right minimum shaft clearance: **36.0713 mm**;
- current capsule arm-arm minimum: **156.9954 mm** (segment 6/segment 6).

This proves the oversized cylinder is sufficient to explain the false reject.
It does **not** make a radius-only edit a complete body-clearance solution:
the base/upper mounts remain separate geometry, and current capsules omit
moving `link1`, omit `link2` from body checks, and omit the fingers.

## Exact native geometry cross-check

I compiled a temporary C++17 probe against the installed FCL 0.7 and the exact
pinned binary STLs. It parsed the URDF millimetre scales, left-side reflections,
collision origins, and used public `oa_fk()` transforms (`base_in_body` for
`link0`, `link_post[N-1]` for link N). It queried the complete body against
links 0..7, hand, and both closed-model fingers on both arms, plus every
left/right pair.

Neutral results relevant to the 25 mm policy:

| Pair / side | Left distance | Right distance | Result |
|---|---:|---:|---|
| body / fixed `link0` | 0.9995 mm | 0.9995 mm | allowed mount only |
| body / moving `link1` | 29.4746 mm | 29.4746 mm | pass |
| body / moving `link2` | 41.4505 mm | 41.4505 mm | pass |
| body / moving `link3` | 41.7727 mm | 41.8185 mm | pass |
| body / `link4` | 84.3932 mm | 84.3936 mm | pass |
| body / `link5` | 85.4983 mm | 85.4983 mm | pass |
| body / `link6` | 85.4964 mm | 85.4964 mm | pass |
| body / `link7` | 94.9983 mm | 94.9983 mm | pass |
| body / hand | 39.4400 mm | 39.4400 mm | pass |
| body / closest finger | 81.2320 mm | 81.2320 mm | pass |
| moving left / moving right | 138.8800 mm | -- | pass (hand/hand) |

Two ordinary nearby configurations also remain clear:

- left J3 `+0.15`, right J3 `-0.15` rad: closest moving body pair remains
  body/`link1` at 29.4746 mm; moving arm-arm minimum 139.2731 mm;
- both J4 `+0.15` rad: closest moving body pair remains body/`link1` at
  29.4746 mm; moving arm-arm minimum 138.8801 mm.

Adversarial configurations within 0.15 rad of neutral demonstrate that the
correction must retain real body/tool and inter-arm rejection:

1. Right q = `[-0.098875,-0.147888,-0.131569,0.124796,-0.062382,
   0.130508,0.099975]`, left zero: exact hand/finger geometry intersects the
   body; `link7` is only 0.214 mm away and `link6` only 15.034 mm. This must
   reject even though inter-arm clearance is 53.852 mm.
2. Left q = `[0.020063,0.146380,-0.010492,0.138731,-0.141862,-0.131182,
   0.023172]`, right q = `[-0.000158,-0.146966,-0.124840,0.001622,
   -0.148493,0.142470,0.143329]`: hand/right-finger inter-arm separation is
   13.372 mm and hand/finger geometry also reaches the body. Both checks reject.

These are fixed-seed capsule-search adversaries followed by independent exact
mesh queries; they are evidence against blanket moving-link or tool exclusions.

## Narrowest justified correction

Do **not** exclude guard segment 2, and do not add a blanket mount-region,
same-arm, tool, or side exclusion.

The narrow allowed-contact policy is only:

```
openarm_body_link0 <-> openarm_left_link0
openarm_body_link0 <-> openarm_right_link0
```

Those links are fixed to the body and form the arm mounts. Continue checking
every actual moving link against the complete body/support, every left/right
pair, and every hand/finger/tool envelope with the 25 mm threshold.

For the existing nominal guard, the smallest defensible geometry correction is:

1. replace the radius-0.115 full-height cylinder with the exact 60 x 60 mm
   shaft box at Z `[0.008,0.758]` (or its conservative 42.426407 mm
   circumscribed cylinder);
2. separately retain verified conservative geometry for the complete body,
   especially the upper mounts and base;
3. query body geometry against `link1` onward, not only capsule segments 2..6;
4. preserve all moving arm-arm and composite tool checks, including a
   conservative full finger-stroke envelope when finger state is unknown.

The exact pinned body/link/hand meshes are already present and are the narrowest
geometry that both admits neutral and preserves the 25 mm rule. If production
must stay primitive-only, generated primitive unions need containment tests
against those meshes. A one-line change from 0.115 to 0.042426407 is a useful
neutral diagnostic fix, but cannot by itself substantiate complete-body or tool
clearance.

Finally, this investigation only addresses pose geometry. The existing 17-knot
nominal sampling is not a continuous swept-path certificate and continues to
report controller `collision_checked=false`.

## Files inspected / verification

- `ros2_ws/src/openarm_ik_ros/src/portal_core.cpp`
- `ros2_ws/src/openarm_ik_ros/include/openarm_ik_ros/portal_core.hpp`
- `model/include/openarm_model.h`
- `model/src/openarm_model.c`
- `model/src/generated/oa_model_data.inc`
- `model/generated/openarm_v10_bimanual.urdf`
- pinned collision STLs under
  `upstream/openarm_description/assets/robot/openarm_v1.0/mesh/` and
  `upstream/openarm_description/assets/end_effector/parallel_link/meshes/`
- `.swarm/collision_geometry_recon.md` and `.swarm/collision_design_verify.md`
  for previously recorded asset hashes/dimensions, independently exercised here
  by native FK and FCL probes.

Temporary helpers were compiled under `/tmp`; no production source was edited.
The pre-existing dirty `transport/tests/test_transport.cpp` and unrelated
untracked `.swarm` artifacts were not touched.
