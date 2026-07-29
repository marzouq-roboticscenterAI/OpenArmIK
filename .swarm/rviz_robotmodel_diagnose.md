# RViz RobotModel independent diagnosis

Date: 2026-07-29 (America/Los_Angeles)  
Repository commit inspected: `ea706280f53c15a072fda55f80dc5c21502fbc9d`  
Verdict: **DONE_WITH_CONCERNS**

## Scope and safety

- Read-only diagnosis. No implementation source or commit was changed.
- The only ROS stack used was the measured virtual backend in isolated domains 227 and 228. No CAN interface, physical hardware, or network was accessed.
- RViz and the diagnostic ROS processes were terminated after the checks.
- The pre-existing dirty/untracked worktree was left intact. This report is the only repository file created by this diagnosis.

## Conclusions

There are two independent defects:

1. **The red RobotModel status is caused by four missing finger transforms, not by inertia or mesh/material loading.** The sole JointState publisher emits only the 14 arm joint names and never emits either gripper's driving `finger_joint1`. `robot_state_publisher` therefore cannot emit the two source-finger transforms or their two mimic transforms.
2. **The four console errors are caused by a genuinely non-physical inertia tensor copied from pinned upstream data.** That problem prevents only RViz's equivalent inertia boxes from being constructed. It is not the reason for the red RobotModel status.

The current Stage-A controller has no gripper state. It must not pretend that either gripper is measured at zero. The smallest semantically correct Stage-A/RViz repair is a visualization-only robot description that fixes the four unmodelled finger links at an explicitly documented pose. Upstream documents `q=0 m` as `closed` and also uses `0.0` as the fake-hardware initial position. A future controller that actually models the eighth motor per arm should instead publish its gripper state through the same sole JointState authority.

## Exact live RViz status

I launched the current installed stack with `ROS_DOMAIN_ID=227`, expanded RViz's RobotModel and Status rows, and queried the Qt accessibility tree. The live status was:

- `Topic`: OK (`1 messages received ...`)
- `URDF`: `URDF parsed OK`
- every body, arm, hand, and hand-TCP link: `Transform OK`
- `openarm_left_left_finger`: `No transform from [openarm_left_left_finger] to [world]`
- `openarm_left_right_finger`: `No transform from [openarm_left_right_finger] to [world]`
- `openarm_right_left_finger`: `No transform from [openarm_right_left_finger] to [world]`
- `openarm_right_right_finger`: `No transform from [openarm_right_right_finger] to [world]`

The RobotModel row and nested Status row were therefore `Error`. RViz separately logged exactly four messages of the form:

```text
The link <finger-link> has unrealistic inertia, so the equivalent inertia box will not be shown.
```

No mesh, material, package URI, or URDF parse error appeared. The meshes rendered. `check_urdf model/generated/openarm_v10_bimanual.urdf` successfully parsed the complete tree.

## JointState and TF evidence

`/joint_states` had one normal publisher, `/openarm_ik_ros`, and one subscriber, `/robot_state_publisher`. A live message contained exactly these 14 names:

```text
openarm_left_joint1 ... openarm_left_joint7
openarm_right_joint1 ... openarm_right_joint7
```

The construction is explicit in `openarm_ik_ros_node.cpp:564-578`: it reserves and appends 14 positions, velocities, and efforts. `VirtualControlSession::joint_names()` in `virtual_control_session.cpp:1088-1100` is a 14-element array populated from the seven-motor-per-arm virtual manifest. The manifest in `control/src/standard_manifest.cpp:36-80` defines only motor IDs 1 through 7 and names only arm joints.

The generated URDF has 26 links and 25 parent-child joints. The live TF edge snapshot was:

| Expected edges | Count | Live result |
|---|---:|---|
| `world -> openarm_body_link0` | 1 | present on `/tf_static` |
| body to left/right `link0` | 2 | present on `/tf_static` |
| left/right `link0 -> ... -> link7` | 14 | present on `/tf` |
| left/right `link7 -> hand -> hand_tcp` | 4 | present on `/tf_static` |
| left/right `link7 -> {left,right}_finger` | 4 | **absent** |

Thus all 21 non-finger edges existed and exactly four of 25 expected edges were missing.

The four URDF relationships are:

- `openarm_left_finger_joint1`, prismatic, `openarm_left_link7 -> openarm_left_right_finger`
- `openarm_left_finger_joint2`, prismatic mimic of joint1, `openarm_left_link7 -> openarm_left_left_finger`
- `openarm_right_finger_joint1`, prismatic, `openarm_right_link7 -> openarm_right_right_finger`
- `openarm_right_finger_joint2`, prismatic mimic of joint1, `openarm_right_link7 -> openarm_right_left_finger`

As a controlled diagnostic only, I published the two source joint names at `0.0`. `robot_state_publisher` immediately emitted all four expected TFs in one message, including both mimic children, at the URDF `q=0` origins (`y=-0.005 m` for the right finger and `y=+0.005 m` for the left finger, both at `z=0.1025 m`). This proves that the URDF mimic definitions work and the missing source states are sufficient to explain the TF gap. It does **not** justify adding invented zero values to the production JointState.

## The gripper is a separate eighth actuator

The finger DOF is not mechanically derived from any of the seven arm coordinates:

- Pinned `openarm_can` initializes a distinct DM4310 gripper motor with send ID `0x08` and receive ID `0x18`; arm motors use IDs 1-7.
- Pinned `openarm_ros2/openarm_hardware` declares seven arm DOFs plus an optional gripper, generates `openarm_<side>_finger_joint1` as the eighth state, and reads it from the gripper motor independently.
- Its v1 parallel-gripper mapping is motor `0 rad -> joint 0 m` and motor `-1.0472 rad -> joint 0.044 m`, but the upstream source explicitly labels the mappings approximate/TODO and leaves gripper velocity and torque mapping unimplemented.
- The Stage-A manifest and snapshots have fixed `[7]` arm arrays and no gripper member. The current virtual adapter therefore has neither an eighth simulated state nor a gripper encoder input.

Pinned upstream semantic evidence for pose selection exists, but not for a current measured pose: the v1 SRDF names `0 m` closed, `0.022 m` half closed, and `0.044 m` open. The ros2_control xacro initializes `finger_joint1` to `0.0`. This supports an explicitly fixed **closed visualization pose**, not a claim that a physical or independently simulated gripper was measured closed.

## Exact source and physics of the four inertia errors

The source is pinned `upstream/openarm_description` commit `6c7b720f1ba48e8bafa3a3dc752c45f397b42221`:

```text
assets/end_effector/parallel_link/config/inertials.yaml:32-64
```

Both `left_finger` and `right_finger` define mass `0.03602545343277134 kg` and this tensor in kg*m^2:

```text
[ 2.375e-6  1.0e-6    1.0e-6 ]
[ 1.0e-6    2.375e-6  1.0e-6 ]
[ 1.0e-6    1.0e-6    0.75e-6]
```

`openarm_v10.urdf.xacro` loads that YAML, `openarm_ee_macro.xacro` copies it into each link, and the bimanual gripper macro instantiates left/right finger data for each arm. The frozen generator then copies it verbatim to generated URDF lines 516-520, 535-539, 613-617, and 632-636. This is why there are exactly four errors.

The tensor is symmetric and positive-definite, so a basic URDF parser accepts it. Its principal moments are:

```text
0.133081369945858e-6, 1.375e-6, 3.99191863005414e-6 kg*m^2
```

A physical rigid body's principal moments must satisfy the triangle inequalities. Here the largest moment exceeds the other two combined by `2.48383726010828e-6 kg*m^2`. RViz's equivalent-box calculation consequently produces one negative squared side, `-0.000413680388186116 m^2`, and reports unrealistic inertia. All other link tensors passed the same eigenvalue/triangle check.

## Recommended fixes

### Current Stage-A virtual/RViz stack

Create an explicitly visualization-only robot description derived reproducibly from the pinned model:

1. Replace the four finger prismatic joints with fixed joints at the documented closed pose `q=0 m`; retain their existing joint origins and remove inapplicable axis/limit/mimic elements.
2. Omit the four finger `<inertial>` elements from that visualization description. Inertial data is optional for RViz and robot_state_publisher, and omission is more correct than fabricated dynamics.
3. Keep the canonical generated URDF and seven-DOF FK/IK/control data unchanged. Name/document the variant so it cannot be mistaken for a dynamics or hardware model.

This yields static finger transforms without representing constants as measurements. It also removes the four non-actionable RViz inertia logs.

Do **not** simply append zero-valued finger joints to the current measured JointState. That would make an unmeasured, independently actuated eighth DOF look measured and would weaken the adapter's measured-feedback contract.

### Future gripper-aware virtual or physical control

Extend the manifest/snapshot/control contract with a distinct gripper state per arm, driven by an explicit virtual-gripper model or the actual motor-8 feedback. Convert motor radians to prismatic metres only after the hardware mapping is calibrated/validated; the pinned upstream mapping itself says it is approximate. Publish both source `finger_joint1` names from the existing sole JointState authority. Do not publish the mimic joints; robot_state_publisher correctly derives them.

For dynamics, replace the upstream finger tensors only with values calculated from CAD/mass properties or known geometry and density in SI units, and validate the centre of mass, symmetry/sign changes under mirroring, positive definiteness, and principal-moment triangle inequalities. Until those inputs exist, omission is the only non-guessed repair.

## Required regression tests

- Parse the selected visualization URDF with `check_urdf`.
- Enumerate every URDF link and assert a transform from `world` after launch; expect 25 TF edges and all 26 frames.
- Assert RViz RobotModel status is OK, not merely that the process stays alive or the robot is visible.
- Assert the normal Stage-A launch still has exactly one `/joint_states` publisher and one TF authority.
- Assert JointState name/position/velocity/effort lengths match and that every non-fixed, non-mimic joint in the runtime description has an explicit state source.
- Run the inertia principal-moment triangle check in the generator/URDF test so invalid tensors cannot silently pass basic XML parsing.
- Resolve every `package://` mesh URI and fail on RViz mesh/material errors; the current assets already pass this check.
- If gripper control is later added, test motor-8 encoder endpoints against `0 m` closed and `0.044 m` open using calibrated measurements before accepting the conversion.

