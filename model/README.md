# OpenArm v1.0 bimanual model

`openarm_units.h` owns the native length-unit contract. `oa_vec3d` is three
IEEE binary64 `double` values; `oa_vec3d_convert` supports metres, centimetres,
and inches without writing its output on invalid, non-finite, or overflowing
input. Expanding conversions reject values at or above the conservative
`DBL_MAX / |scale|` boundary under every standard floating-point rounding mode.
`oa_ik_position_with_units` converts its target once at ingress, while
all IK tolerances and diagnostics remain metres.

This is a dependency-free ISO C11 forward-kinematics, geometric-Jacobian, and bounded position-IK library. It models exactly two chains from `openarm_body_link0` through the named `openarm_{left,right}_hand_tcp` frames in the flattened canonical bimanual v1.0 URDF.

The immutable data is generated from `enactic/openarm_description` commit `6c7b720f1ba48e8bafa3a3dc752c45f397b42221`. The generator requires a clean checkout, verifies the known full canonical source hash, makes a temporary ament index that points `openarm_description` directly at that checkout, flattens the current entry xacro with `bimanual:=true`, and parses the resulting current `link7 -> hand -> hand_tcp` chain. It never reads the stale checked-in example URDF. The flattened URDF is archived under `generated/`; model-data, flattened-URDF, and full v1 arm/parallel-gripper source hashes are exposed by the API. The model-data hash and provenance also bind generator version/source hash and the xacro version/implementation hash.

```sh
python3 tools/generate_model.py /path/to/openarm_description src/generated/oa_model_data.inc \
  --urdf-output generated/openarm_v10_bimanual.urdf --xacro /path/to/xacro \
  --pythonpath /path/to/xacro-and-ament/site-packages --ament-prefix /opt/ros/lyrical
```

`oa_fk` returns body-relative base, pre-joint, post-link, world/body-frame axes, and the selected tip. `oa_geometric_jacobian` is `[linear; angular]`, 6x7, body-frame, row-major. `oa_ik_position_v2` requires a seed, posture reference/weights, explicit numerical controls, and caller-provided output version/capacity. It validates output metadata before the first write. The legacy ABI-v1 `oa_ik_position` symbol fails closed with `OA_MODEL_EINVAL` and performs no output write. The solver uses adaptive damped least squares, an active feasible set, strict primary-position-error line search, and a secondary posture projection. Every accepted iterate is projected into the effective URDF box including `limit_margin_rad`; `OA_MODEL_OK` revalidates both tolerance and feasibility.

## Collision-aware paired routing

`openarm_route.h` exposes the C11 `oa_route_plan_paired()` API. The caller
provides both measured 7-joint model states and both exact body-frame TCP
targets. A successful result contains ordered paired TCP endpoints and the
seed-dependent terminal joint solution for every edge. Every numeric field and
all internal routing calculations are binary64 `double`.

The built-in graph uses audited portal poses only as connectivity candidates.
Each edge still has to pass 17 public IK/FK samples, joint bounds and branch
continuity, the finite central-pole keepout, and conservative arm/tool
capsules. The exact pinned claw-mesh exception remains confined to the
dedicated terminal-contact/retreat path; ordinary routes cannot request it.
Normal edges retain the 25 mm planning clearance. With
`OA_ROUTE_ALLOW_CLEARANCE_RECOVERY`, a start already inside that gate may leave
only if it is outside the 10 mm intervention floor and every sample is
monotonically opening until 25 mm is restored. `OA_ROUTE_ENOPATH` means this
finite graph did not prove a route; it is not a proof about the continuous
14-DOF configuration space. The API never clamps or projects its target.

Public model results use `oa_model_status` and `OA_MODEL_*` constants. The old
generic `oa_status` and `OA_*` status names remain available to model-only
translation units. A translation unit combining model and control must define
`OPENARM_DISABLE_LEGACY_GENERIC_STATUS` before either header and use the
module-prefixed API; otherwise both include orders fail with a diagnostic
instead of silently assigning generic names different meanings.

Every valid ABI-v2 output buffer receives a finite result carrying `abi_version`, `struct_size`, status, and `collision_checked=0`. Statuses distinguish invalid/non-finite input, invalid bound margins, line-search non-convergence, bound stagnation, undamped rank loss, and iteration-budget exhaustion. `min_singular_value` is the weighted translational active-set Jacobian metric in body coordinates. Public status and flag fields are fixed-width; ABI version 2 requires `OA_IK_OPTIONS_REQUIRED_SIZE`, output version `OA_IK_DIAGNOSTICS_VERSION`, and capacity of at least `OA_IK_DIAGNOSTICS_SIZE`.

Position IK leaves orientation free; its resulting orientation is returned as
`achieved_hand_tcp`. The IK call by itself never checks self, body, other-arm,
or environment collision: `collision_checked` is always zero. Use the routing
API when a sampled nominal paired path is required. Neither API is certified
physical motion authorization or a complete environment-aware planner.

Generated source data and this library retain Apache-2.0 attribution to Enactic, Inc.; the complete license and attribution are shipped as `LICENSE.txt` and `NOTICE`. No upstream code or runtime ROS dependency is required.
