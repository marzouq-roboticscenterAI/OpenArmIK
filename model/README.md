# OpenArm v1.0 bimanual model

This is a dependency-free ISO C11 forward-kinematics, geometric-Jacobian, and bounded position-IK library. It models exactly two chains from `openarm_body_link0` through the named `openarm_{left,right}_hand_tcp` frames in the flattened canonical bimanual v1.0 URDF.

The immutable data is generated from `enactic/openarm_description` commit `6c7b720f1ba48e8bafa3a3dc752c45f397b42221`. The generator verifies Git HEAD, makes a temporary ament index that points `openarm_description` directly at that checkout, flattens the current entry xacro with `bimanual:=true`, and parses the resulting current `link7 -> hand -> hand_tcp` chain. It never reads the stale checked-in example URDF. The flattened URDF is archived under `generated/`; model-data, flattened-URDF, and full v1 arm/parallel-gripper source hashes are exposed by the API.

```sh
python3 tools/generate_model.py /path/to/openarm_description src/generated/oa_model_data.inc \
  --urdf-output generated/openarm_v10_bimanual.urdf --xacro /path/to/xacro \
  --pythonpath /path/to/xacro-and-ament/site-packages --ament-prefix /opt/ros/lyrical
```

`oa_fk` returns body-relative base, pre-joint, post-link, world/body-frame axes, and the selected tip. `oa_geometric_jacobian` is `[linear; angular]`, 6x7, body-frame, row-major. `oa_ik_position` requires a seed, posture reference/weights and explicit numerical controls. It uses adaptive damped least squares, an active feasible set, strict primary-position-error line search, and a secondary posture projection. Every accepted iterate is projected into the effective URDF box including `limit_margin_rad`; `OA_OK` revalidates both tolerance and feasibility.

Every IK call with non-null diagnostics initializes a finite result carrying `abi_version`, `struct_size`, status, and `collision_checked=0`. Statuses distinguish invalid/non-finite input, invalid bound margins, line-search non-convergence, bound stagnation, undamped rank loss, and iteration-budget exhaustion. `min_singular_value` is the weighted translational active-set Jacobian metric in body coordinates. Public status and flag fields are fixed-width; ABI version 1 requires `OA_IK_OPTIONS_REQUIRED_SIZE` and reports `OA_IK_DIAGNOSTICS_SIZE`.

Position IK leaves orientation free; its resulting orientation is returned as `achieved_hand_tcp`. It never checks self, body, other-arm, or environment collision: `collision_checked` is always zero. It is not motion authorization or a collision-safe trajectory planner.

Generated source data and this library retain Apache-2.0 attribution to Enactic, Inc.; the complete license and attribution are shipped as `LICENSE.txt` and `NOTICE`. No upstream code or runtime ROS dependency is required.
