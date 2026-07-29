# Coordinate / example verification (read-only, virtual-only)

## Status

**KINEMATICS VERIFIED; COLLISION-SAFETY REQUIREMENT NOT PROVABLE WITH THIS repository.**

At repository commit `8f3a99915069531e1e96c14798f7d4cf59b5467b`, four conservative target pairs (two effectively single-arm, then two paired) were accepted in sequence by the compiled Stage-A controller. Every controller plan traversed its production 17-waypoint Cartesian planning path, every virtual execution emitted `OA_EVENT_COMPLETED`, and the independently compiled ROS `continuity-v1` transaction processor committed the same four target pairs in the same order. All endpoint joint vectors were within the manifest/URDF limits.

However, all successful controller plans required `OA_COLLISION_VIRTUAL_UNCHECKED`, and every plan/model diagnostic reported `collision_checked=0`. No full-path mesh distance or collision check exists. Consequently, none of the examples below can be represented as demonstrably clear of the opposite arm or central metal support pole, and none is safe or hardware-authorized evidence.

No hardware, CAN, GUI, network, or physical backend was used. All builds and executables were under `/tmp`; temporary probe sources were removed.

## Frames and exact initial TCP

Model FK is expressed in `openarm_body_link0`. The generated URDF has a fixed `world -> openarm_body_link0` transform with zero translation and zero rotation (`model/generated/openarm_v10_bimanual.urdf:19-24`), so body-frame and world-frame XYZ values are numerically identical, in metres.

There are two relevant exact startup states:

| Startup surface | q for each joint | Left TCP XYZ (m) | Right TCP XYZ (m) |
|---|---:|---:|---:|
| ROS virtual visualizer / compiled model (`q = 0`) | `0` | `(0, 0.15349771526864497, 0.07599955003657091)` | `(0, -0.15349771526864497, 0.07599955003657097)` |
| Stage-A virtual controller measured startup (encoder-quantized) | every joint `0.00006675822079849070 rad` | `(-0.00002710217965846259, 0.15346860855059954, 0.07599955201969341)` | `(0.00008077910443504927, -0.15352682530236783, 0.07599955704732647)` |

The second row is the actual controller snapshot used to seed example A. It is intentionally not replaced by ideal all-zero FK.

## Sequential virtual target examples

All coordinates are metres in body/world. A and B hold the other arm at its measured TCP and are therefore the requested single-arm examples through the paired controller API. C and D move both arms. Requests used `tcp_tol_m=0.001`, `max_branch_step_rad=0.35`, `min_singular_value=0`, scales `0.5`, and collision policy `OA_COLLISION_VIRTUAL_UNCHECKED`.

| ID | Kind / start | Left target XYZ | Right target XYZ | 17-waypoint plan | Planned endpoint residual L / R (m) | Max `abs(delta q)` L / R (rad) | Minimum endpoint limit clearance on moving arm(s) (rad) | Virtual execution |
|---|---|---|---|---|---:|---:|---:|---|
| A | Left only, controller startup | `(0.01997289782034154, 0.14346860855059954, 0.09599955201969342)` | hold `(0.000080779104435049, -0.15352682530236783, 0.07599955704732647)` | `OA_CONTROL_OK` | `5.41083e-9 / 0` | `0.364643 / 0` | `0.171141` (left) | completed; measured residual `1.64931e-4 / 0` |
| B | Right only, after A | hold `(0.02012540975976749, 0.14343322643662396, 0.09605142070874839)` | `(0.02008077910443505, -0.14352682530236782, 0.09599955704732648)` | `OA_CONTROL_OK` | `0 / 5.40501e-9` | `0 / 0.364444` | `0.176817` (right) | completed; measured residual `0 / 1.14425e-4` |
| C | Paired, after B | `(0.03012540975976749, 0.13843322643662395, 0.10105142070874840)` | `(0.03017972001249530, -0.13857775617089080, 0.10102620110823265)` | `OA_CONTROL_OK` | `1.43219e-11 / 1.43414e-11` | `0.0376590 / 0.0376209` | `0.164488` (pair minimum) | completed; measured residual `1.50536e-4 / 2.60939e-5` |
| D | Paired, after C | `(0.04027001119517448, 0.14342482575778129, 0.10609241949979897)` | `(0.04016377942864092, -0.14359472008097229, 0.10603799159006971)` | `OA_CONTROL_OK` | `9.88488e-12 / 9.93319e-12` | `0.0344968 / 0.0345984` | `0.172409` (pair minimum) | completed; measured residual `1.09092e-4 / 1.83404e-4` |

The full planned joint changes (J1..J7, radians) were:

- A left: `(0.2421575451, 0.0033250538, 0.0027897622, 0.2586985504, -0.0004473266, -0.0439332426, -0.3646432651)`; right all zero.
- B right: `(-0.2407172377, 0.0022180444, -0.0028176377, 0.2566015608, 0.0011307802, 0.0628524269, 0.3644438933)`; left all zero.
- C left: `(0.0137050144, 0.0065443251, 0.0021622441, 0.0305252198, 0.0004910102, -0.0055595825, -0.0376590308)`; right: `(-0.0136310032, -0.0061573593, -0.0021136652, 0.0304567552, -0.0003171220, 0.0069941063, 0.0376209168)`.
- D left: `(0.0111878946, -0.0078617430, -0.0024261659, 0.0285400868, -0.0013635846, -0.0008116288, -0.0344967646)`; right: `(-0.0112251031, 0.0082017581, 0.0024533828, 0.0285153941, 0.0015356958, 0.0020664817, 0.0345983680)`.

Controller planned durations were A `9.933756945 s`, B `9.931075562 s`, C `4.871426768 s`, and D `4.735962685 s`. Quantized plant endpoints remained under the controller's `0.001 m` TCP tolerance.

The compiled ROS `PairedTransactionProcessor` independently committed A-D sequentially from its true all-zero initial q. Its L/R residuals were:

| ID | ROS `continuity-v1` residual L / R (m) | Both statuses / commit | Collision fields |
|---|---:|---|---|
| A | `1.21814e-5 / 8.58642e-5` | `OA_MODEL_OK / OA_MODEL_OK`, committed | `0 / 0` |
| B | `3.67374e-8 / 7.53776e-6` | `OA_MODEL_OK / OA_MODEL_OK`, committed | `0 / 0` |
| C | `1.33906e-6 / 1.36904e-6` | `OA_MODEL_OK / OA_MODEL_OK`, committed | `0 / 0` |
| D | `9.03544e-7 / 8.80575e-7` | `OA_MODEL_OK / OA_MODEL_OK`, committed | `0 / 0` |

## Why this proves all 17 waypoints, but not collision clearance

`Controller::plan_paired` sets `waypoint_count=17`, solves waypoints 1..16 at evenly spaced Cartesian fractions from measured FK to the target, seeds every waypoint from its predecessor, and rejects the whole plan for IK residual, singularity threshold, joint/raw bounds, or per-waypoint branch-step failure (`control/src/control_core.cpp:693-745`). It returns success only after all of those checks and then times all 16 segments (`:748-768`). Thus each reported `OA_CONTROL_OK` is evidence that the entire production 17-waypoint solve succeeded, not merely endpoint IK.

That loop does not evaluate collision geometry and explicitly assigns `plan->collision_checked=false` (`:747`). The public model likewise specifies `collision_checked` is always zero (`model/include/openarm_model.h:91-103`). The controller's default rejecting policy returns `OA_CONTROL_ECOLLISION`; the test suite confirms this fail-closed behavior and only permits motion in the virtual-unchecked fixture (`control/tests/test_control.cpp:637-660`).

The URDF does contain collision meshes for body, links, hands, and fingers. The metal support is not a separately addressable primitive/link; it is included in the body mesh `body_link0_symp.stl` (`model/generated/openarm_v10_bimanual.urdf:25-38`). Direct binary-STL inspection found 5,864 body triangles and, after the URDF's `0.001` scale, an axis-aligned body-mesh extent of `x=[-0.155, 0.095] m`, `y=[-0.095, 0.095] m`, `z=[0, 0.773] m`. Neither the model nor controller loads these triangles for distance/collision queries. TCP separation or TCP-to-body-AABB separation cannot certify clearance of swept link/hand/finger meshes. No defined safety margin was available to evaluate.

Therefore the requested stronger statement—clear of the opposing arm and pole over the full intermediate path with a defined margin—cannot be made for A-D or any other target using the current API/tests. A collision engine or an independent continuous/sufficiently conservative swept-mesh checker, an explicit margin, and validation of all interpolated configurations would be required.

## Orientation limitation

The IK objective is XYZ position only. Orientation is free and may change along the solve; no target orientation is accepted or held. Even a future position-clearance check at TCP level would not bound the oriented hand/finger geometry.

## Reproduction evidence

- Release model and control test libraries were configured and built from the current checkout under `/tmp/openarmik-coordinate-build` and `/tmp/openarmik-coordinate-prefix`.
- A temporary C++ probe instantiated the same compiled `Fixture` manifest/controller as `control/tests/test_control.cpp`, read the actual startup snapshot/FK, planned each target with `oa_controller_plan_paired_tcp`, read `oa_motion_plan_report`, executed it, advanced only the virtual simulator at 10 ms cycles, required its matching `OA_EVENT_COMPLETED`, and measured endpoint FK from a fresh snapshot.
- A separate dependency-free temporary C++ probe compiled `paired_transaction.cpp` and processed the same A-D world-frame targets in order.
- `openarm_control_tests` rebuilt from the temporary tree and passed: `1/1`, including collision rejection, limit checks, paired TCP measured convergence, and Cartesian path-policy tests.
- Model all-zero FK was evaluated through the compiled `oa_fk` reference driver, not inferred from link lengths.
- Production sources relevant to the result had no git diff. Existing unrelated `.swarm` files were not modified.
