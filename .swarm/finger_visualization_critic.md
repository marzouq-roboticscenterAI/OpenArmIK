# Finger visualization design critique

Date: 2026-07-29 (America/Los_Angeles)  
Repository commit inspected: `ea706280f53c15a072fda55f80dc5c21502fbc9d`  
Verdict: **APPROVE**

## Scope

Read-only review of the proposed design against the pinned canonical URDF/xacro,
the installed-model/digest contract, the Stage-A control manifest and snapshot
shape, the current ROS launch/node/tests, and the prior ROS design/review notes.
No implementation source, GUI, CAN interface, or hardware path was used. This
report is the only file created by this review.

## Decision

Approve a mechanically derived **Stage-A visualization/TF description** with
these two independent, narrowly scoped transformations:

1. Change exactly
   `openarm_{left,right}_finger_joint{1,2}` from prismatic/mimic joints to fixed
   joints, preserving each joint name, parent, child, and origin exactly. Remove
   only the now-inapplicable `axis`, `limit`, and `mimic` children. The retained
   origin is precisely the canonical prismatic transform at `q=0 m`, which the
   pinned v1 SRDF explicitly names `closed`.
2. Remove exactly the direct `<inertial>` child from each of the four finger
   links. Preserve the links and their visual and collision subtrees exactly.
   Omission is valid for URDF consumers used here and is more truthful than
   inventing replacement mass properties for the non-physical pinned tensors.

Do not modify `model/generated/openarm_v10_bimanual.urdf`, the pinned upstream
xacros/YAML, generated C model data, the control manifest, the measured snapshot,
or the adapter's fourteen-name `JointState` contract.

The term "RViz-only" needs one documentation qualification. The description is
loaded by `robot_state_publisher`, not privately by RViz, and therefore defines
the launch's public TF tree and `robot_description` parameter. It is truthful
for this virtual visualization launch as an explicitly declared **unmeasured
closed-pose display convention**. It is not a dynamics model, a calibrated
gripper model, or a safe collision-planning description and must not be exported
or advertised as any of those.

## Evidence for the fixed closed pose

The canonical flattened URDF contains 26 links and 25 joints. Its four finger
relationships are leaf branches:

| Joint | Parent | Child | Canonical `q=0` origin |
|---|---|---|---|
| `openarm_left_finger_joint1` | `openarm_left_link7` | `openarm_left_right_finger` | `xyz="0 -0.005 0.1025"`, `rpy="0 0 0"` |
| `openarm_left_finger_joint2` | `openarm_left_link7` | `openarm_left_left_finger` | `xyz="0 0.005 0.1025"`, `rpy="0 0 0"` |
| `openarm_right_finger_joint1` | `openarm_right_link7` | `openarm_right_right_finger` | `xyz="0 -0.005 0.1025"`, `rpy="0 0 0"` |
| `openarm_right_finger_joint2` | `openarm_right_link7` | `openarm_right_left_finger` | `xyz="0 0.005 0.1025"`, `rpy="0 0 0"` |

These values are at
`model/generated/openarm_v10_bimanual.urdf:541-555,638-652`. For a prismatic
joint, the child transform at `q=0` is the joint origin, and the mimic multiplier
and offset default to `1` and `0`. Converting both source and mimic joints to
fixed while retaining those origins therefore gives exactly the canonical
closed transform.

The pose is not an arbitrary visual guess. Pinned
`upstream/openarm_ros2/openarm_bimanual_moveit_config/config/openarm_v1.0/openarm_bimanual.srdf:88-104`
names `0 m` closed, `0.022 m` half-closed, and `0.044 m` open. The pinned
ros2-control xacro initializes the two source finger joints to `0.0`
(`openarm.bimanual.ros2_control.xacro:95-97,145-147`). Those sources establish a
documented pose convention; they do **not** establish the current gripper pose.

## Why this does not claim measured gripper state

The current adapter publishes exactly fourteen measured arm names and fourteen
positions, velocities, and efforts from `oa_snapshot`:

- `openarm_ik_ros_node.cpp:564-578` constructs the message solely from the two
  seven-element arm snapshots.
- `virtual_control_session.cpp:1088-1100` derives its fourteen names from the
  compiled standard manifest.
- `control/src/standard_manifest.cpp:36-80` defines only IDs 1 through 7 for
  each arm.
- `test_ros_contract.py:15-18,55-58` locks the current fourteen-name contract.

The gripper is independently actuated, not a function of arm joint 7. Pinned
upstream hardware declares a separate DM4310 with send ID `0x08` and receive ID
`0x18` (`openarm_simple_hardware.hpp:98-101`), adds
`openarm_<side>_finger_joint1` after the seven arm joints
(`openarm_simple_hardware.cpp:96-110,137-143`), and reads its own motor feedback
(`:267-279`). The present Stage-A state has no corresponding eighth encoder or
virtual plant coordinate.

Under the approved design, neither finger source joint appears in
`/joint_states`; the four transforms are static consequences of the explicitly
selected visualization model. A fixed model transform is not reported as an
encoder sample and carries no fabricated measurement timestamp, velocity,
effort, freshness, or feedback sequence. The README and generated-file comment
must say "unmeasured, fixed closed visualization pose (`q=0 m`)" rather than
"gripper state is zero" or "grippers are closed."

## Transform, IK, collision, and digest preservation

### Arm IK and control frames

Both hand paths are separate from the finger branches:

```text
openarm_<side>_link7 -> openarm_<side>_hand -> openarm_<side>_hand_tcp
                      \
                       -> left_finger
                       -> right_finger
```

The hand and TCP fixed joints are at canonical URDF lines `492-501` and
`589-598`. Finger edits cannot alter any ancestor, either seven-joint arm chain,
`openarm_body_link0`, either `hand`, or either `hand_tcp`. The C model explicitly
models only `openarm_body_link0` through `hand_tcp` (`model/README.md:3`) and the
adapter transforms goals into `openarm_body_link0`. Consequently FK, Jacobians,
IK, limits, action frames, and measured TCP TF are unchanged.

### Visual and collision geometry

The four visual and collision elements remain on the same named links with the
same mesh URIs, scales, and link-local origins (canonical lines `503-535` and
`600-632`). Their world transforms become exactly the canonical transforms at
`q=0`. Thus the variant preserves closed-pose visual and collision geometry.

This does **not** make collision checking available. The Stage-A adapter already
reports `collision_checked=false` and has no collision engine
(`ros2_ws/src/openarm_ik_ros/README.md:17-21`). Any consumer that uses the
variant for MoveIt, dynamics, or safety collision decisions would incorrectly
treat an unknown aperture as permanently closed. Keep the derived file inside
`openarm_ik_ros`, select it only in this virtual visualization launch, and retain
the existing unchecked-collision warning.

### `robot_state_publisher` and TF authority

The canonical tree has seven fixed and eighteen movable joints. The variant has
eleven fixed joints and exactly fourteen movable arm joints. After one coherent
fourteen-name `JointState`, the one existing `robot_state_publisher` should emit:

- 11 edges on `/tf_static`: the existing seven fixed edges plus the four fixed
  finger edges;
- 14 arm edges on `/tf`; and
- all 25 parent-child edges / all 26 frames reachable from `world`.

No node or publisher is added. `robot_state_publisher` remains the sole `/tf`
and `/tf_static` authority and the adapter remains the sole `/joint_states`
authority, preserving the design contract in
`.swarm/ros_design_synthesis.md:316-355,516-520` and the live test assertions in
`test_ros_contract.py:60-62`.

### Canonical model and digests

The canonical flattened URDF currently hashes to
`dd4b5d5fa0c050b78922bc95850ea65bb15721cb259030cccf106a953f171c55`, exactly
the `flattened_urdf_sha256` embedded for both arms in
`model/src/generated/oa_model_data.inc:6,40`. The model package exports that
canonical artifact through `openarm_model_GENERATED_URDF`
(`model/cmake/openarm_modelConfig.cmake.in:9-10`) and installs it with its
license/notice (`model/CMakeLists.txt:38-39`).

Generate the visualization artifact downstream in `openarm_ik_ros`; do not
replace the exported variable, canonical installed filename, embedded digest,
source hash, model-data hash, or provenance string. A distinct derived filename
and an embedded provenance comment containing the source digest make the
boundary auditable. The visualization file will necessarily have its own hash;
it must never be passed off as the canonical digest-bound model.

## Exact implementation boundary

The smallest maintainable file set is:

1. **New** `ros2_ws/src/openarm_ik_ros/tools/generate_visualization_urdf.py`:
   accept the canonical installed URDF and output a distinct
   `openarm_v10_bimanual_stage_a_visualization.urdf`; require the exact four
   expected prismatic joints and four finger inertials; perform only the edits
   approved above; preserve copyright/license comments; add the source SHA-256
   and the unmeasured-closed-pose warning; fail closed on any source drift.
2. **Modify** `ros2_ws/src/openarm_ik_ros/CMakeLists.txt`: generate the derived
   file in the build tree from `${openarm_model_GENERATED_URDF}`, make it an
   `ALL` dependency, and install it under `share/openarm_ik_ros/urdf`. Continue
   testing the canonical file independently. Do not install the derived file
   from `openarm_model` or change `openarm_model_GENERATED_URDF`.
3. **Modify** `ros2_ws/src/openarm_ik_ros/launch/openarm_ik_rviz.launch.py:15`:
   load only the distinctly named Stage-A visualization file into the existing
   single `robot_state_publisher`.
4. **New or extend**
   `ros2_ws/src/openarm_ik_ros/test/test_visualization_urdf.py` and register it
   from CMake: compare canonical and derived trees semantically and enforce all
   invariants below.
5. **Extend** `ros2_ws/src/openarm_ik_ros/test/test_ros_contract.py`: assert all
   26 frames become reachable from `world`, while publisher counts and the
   fourteen-name `JointState` contract remain unchanged.
6. **Modify** `ros2_ws/src/openarm_ik_ros/README.md`: document the display
   convention, lack of finger measurement, prohibited dynamics/collision use,
   canonical-versus-visualization filenames, and motor-8 upgrade behavior.
7. **If build-time Python is used**, move the existing CMake `find_package(Python3
   ... Interpreter)` outside the `BUILD_TESTING` block and declare the matching
   build dependency in `package.xml`; do not add xacro as a new runtime model
   source.

No change is justified in `model/`, `control/`, `openarm_ik_ros_node.cpp`,
`virtual_control_session.cpp`, action messages, diagnostics, or the RViz config.

## Required generation and regression tests

The change should not merge unless these checks pass:

1. **Strict source/patch cardinality:** exactly four named source joints are
   prismatic with the expected parents, children, origins, axes, limits, and two
   mimic relations; exactly four named links have one direct inertial each. The
   generator fails if any precondition or count differs.
2. **Semantic diff allowlist:** canonical versus visualization files have the
   same 26 link names, 25 joint names, complete parent-child graph, all
   non-finger joint types/elements, all joint origins, all visual/collision
   subtrees, all mesh URIs, and every non-finger inertial. The only semantic
   differences are four joint types, removal of four axis/limit elements and
   two mimic elements, and removal of four finger inertials.
3. **Kinematic equivalence:** over zero, limit, and randomized arm postures,
   compare every canonical frame at finger `q=0` with the corresponding variant
   frame to a tight numerical tolerance. This includes both `hand_tcp` frames
   and all four collision/visual link frames.
4. **URDF validity and resources:** `check_urdf` parses both descriptions; all
   `package://openarm_description/...` meshes resolve; the canonical generator
   byte-determinism and independent FK/Jacobian tests continue to pass.
5. **Digest isolation:** hash the canonical input before and after generation,
   require the value above for this pin, and assert the C API still reports that
   canonical flattened hash. Also test deterministic byte-for-byte regeneration
   of the visualization output.
6. **Live headless TF completeness:** retain exactly one `/joint_states`, one
   `/tf`, and one `/tf_static` publisher; publish exactly fourteen arm states;
   observe 14 dynamic plus 11 static edges, all 26 frames reachable from
   `world`, and no finger names in `JointState`.
7. **RViz acceptance:** RobotModel `URDF parsed OK`; every link reports
   `Transform OK`; overall RobotModel status is OK; no unrealistic-inertia,
   mesh, material, or package error appears. Checking only process liveness or a
   visible arm is insufficient.
8. **Truthfulness guard:** no joint-state publisher, static-transform publisher,
   zero-valued finger state, motor-8 freshness bit, gripper command endpoint, or
   collision-checked claim is introduced.

## Rejected smaller alternatives

- **Append two zero finger sources to the measured `JointState`: reject.** It
  fills the TF gap and allows RSP to derive the mimic links, but presents an
  independently actuated, absent encoder as part of an `oa_snapshot`-measured
  message. It violates the fourteen-joint state-source contract.
- **Run `joint_state_publisher` at default zero: reject.** It adds a second
  `/joint_states` authority and produces guessed state independently of the
  controller's coherent timestamp/freshness contract.
- **Add four `static_transform_publisher` instances: reject.** It creates extra
  TF authorities, duplicates URDF semantics outside the model, and becomes
  ambiguous or conflicting when dynamic gripper feedback is later enabled.
- **Use a robot-state-publisher default position: unavailable/reject.** RSP
  publishes fixed transforms from URDF and movable transforms from received
  `JointState`; its timing/default parameters do not turn absent prismatic
  encoder values into a documented static pose. A hypothetical default would
  still be guessed runtime state.
- **Use existing xacro arguments: unavailable.** The pinned v1 entry xacro has
  arguments for arm/body/control/interface/prefix/base placement, but none for
  fixed fingers or inertial omission (`openarm_v10.urdf.xacro:19-38`). Editing
  pinned xacro/YAML would alter the canonical source/hash. A second handwritten
  wrapper macro would duplicate the gripper model and become a second authority.
  A strict post-process of the digest-bound flattened artifact is smaller and
  mechanically auditable.
- **Delete finger links: reject.** This removes useful visual/collision geometry
  and four established frame names instead of representing the documented
  closed display pose.
- **Keep the dynamic URDF and tolerate red status: truthful but does not meet
  the requested visualization outcome.** It remains the correct canonical model
  and should stay available outside this Stage-A visualization launch.
- **Invent corrected inertia tensors: reject.** Positive definiteness alone is
  insufficient; the current tensors violate rigid-body principal-moment triangle
  inequalities. Replacement requires CAD or validated geometry/density/COM data.
  Omitting optional visualization inertials is the only non-guessed repair.

## Future motor-8 upgrade path

Treat gripper enablement as a capability/contract upgrade, not as removal of a
visual workaround:

1. Add a distinct per-side gripper actuator/state descriptor to a versioned
   manifest/snapshot ABI: actuator identity (motor 8), prismatic position in
   metres, validity/source (`virtual` or measured), sequence, timestamp,
   freshness/fault state, and meaningful velocity/effort semantics. Do not
   squeeze it into the fixed seven-element arm arrays.
2. For virtual operation, implement an explicit eighth-coordinate plant. For
   hardware, read the separate `0x08/0x18` encoder through a separately reviewed
   physical path. Neither is present now.
3. Calibrate and validate the motor-radian-to-aperture-metre mapping and limits.
   Pinned upstream labels the current mapping approximate and leaves velocity
   and torque conversion unimplemented
   (`openarm_simple_hardware.cpp:267-278,375-406`), so it is evidence of topology,
   not production calibration.
4. Publish only `openarm_left_finger_joint1` and
   `openarm_right_finger_joint1` from the existing sole adapter authority when
   each has valid state. Let `robot_state_publisher` derive `finger_joint2`
   through the canonical mimic definitions; never publish mimic state separately.
5. Atomically switch the launch description back to dynamic finger joints. If
   CAD inertials are still unavailable, generate a **dynamic visualization
   variant that only omits the four invalid finger inertials**. Do not retain
   fixed finger joints once motor-8 state is authoritative.
6. Extend tests across `0`, `0.022`, `0.044 m`, boundaries, invalid/stale
   feedback, mimic direction, timestamps, units, and authority counts. Verify
   that loss of motor-8 freshness cannot be masked by the old closed-pose
   visualization convention.

## Final assessment

The proposed derived description is the smallest truthful way to obtain a
complete, clean RobotModel without weakening measured-state authority. It
preserves all canonical arm/control/IK frames and digests, preserves closed-pose
visual/collision transforms, keeps one RSP TF authority, and represents absent
gripper state as an explicit static visualization convention rather than an
encoder reading. Approval depends on the strict downstream-derivation,
documentation, scope, and regression gates above.
