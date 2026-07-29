# Independent Stage-A finger visualization implementation review

Date: 2026-07-29 (America/Los_Angeles)  
Reviewed commit: `9e5d50f9038f4b98b9e2bc4d86f8208236944b42`  
Comparison base: `98edcbe22f504c98a24edfc7ee4b346668cc3977`  
Verdict: **CLEAN**

## Findings

No Critical, Important, or Minor findings in the reviewed change.

The implementation is the approved narrow downstream visualization overlay. It
does not change the canonical model, control state, measured `JointState`, IK,
collision claim, or ROS authority topology.

## Diff and artifact audit

The production change adds a build-time CMake transformation and selects its
distinct output in the existing launch. It changes no file under `model/`,
`control/`, `ros2_ws/src/openarm_ik_ros/src/`, or
`ros2_ws/src/openarm_ik_ros/include/`. `git diff --check 98edcbe..9e5d50f`
passed.

### Canonical model and digest remain authoritative

The source canonical URDF, freshly installed model-package copy, and freshly
installed ROS-package canonical copy all independently hashed to:

```text
dd4b5d5fa0c050b78922bc95850ea65bb15721cb259030cccf106a953f171c55
```

The base commit's canonical URDF has the same digest. That value remains the
flattened-URDF digest embedded for both arms in
`model/src/generated/oa_model_data.inc:6,40`. The fresh native model suite,
including byte-deterministic pinned xacro regeneration and independent
FK/Jacobian comparison, passed 4/4. No model data, IK path, control manifest,
snapshot layout, collision policy, or revision/digest source changed.

The derived artifact has its own stable digest:

```text
4e2838fd90240ed140d5624316fdfbb2ad96f97662aa62f832b39924499dd0de
```

It is installed separately as
`openarm_v10_bimanual_stage_a_visualization.urdf`; the canonical file is still
installed under its original name. `CMakeLists.txt:29-43,86-91` derives from the
exported canonical input rather than creating another kinematic source.

### Transformation is exact and fail-closed

`GenerateStageAVisualizationUrdf.cmake:7-14` refuses any input whose complete
SHA-256 differs from the pinned canonical digest. Its exact-cardinality helper
at lines 18-37 and literal source blocks at lines 39-89 permit only:

- removal of the two positive-y and two negative-y direct finger inertials;
- conversion of the left/right source `finger_joint1` and mimic
  `finger_joint2` from prismatic to fixed;
- retention of the exact joint names, parents, children, and canonical origins;
  and
- removal of only the four now-inapplicable axes, four limits, and two mimic
  elements.

The fixed origins remain:

```text
right_finger: xyz="0.0 -0.005 0.1025", rpy="0 0 0"
left_finger:  xyz="0.0  0.005 0.1025", rpy="0 0 0"
```

These are exactly the canonical prismatic transforms at `q=0 m`; no new
coordinate or guessed offset is introduced.

The fresh `test_visualization_urdf` passed and proved byte-for-byte deterministic
regeneration, an unchanged 26-link/25-joint name and parent-child graph, exact
preservation of every non-finger link/joint subtree, exact preservation of each
finger visual/collision subtree, omission of exactly the four finger inertials,
and equality of every frame at canonical finger `q=0` over zero, arm limits, and
twelve seeded randomized arm postures
(`test_visualization_urdf.py:171-243`). Fresh `check_urdf` also parsed the derived
tree successfully.

Because the finger links are leaf siblings of `hand`, this also confirms both
arm chains, `world`, `openarm_body_link0`, both hands, both `hand_tcp` frames,
and all retained closed-pose visual/collision geometry are identical.

### No new Python runtime

The generator is CMake script mode at build time. Python remains confined to
the pre-existing `BUILD_TESTING` block and test dependencies. No Python
generator or installed runtime helper was added. The freshly installed node and
CLI remain ELF C++ executables; `ldd` contains no Python library.

## Fresh verification

All output was built outside the worktree under a reviewer-created temporary
directory.

### Clean build and tests

Command:

```bash
TMPDIR=/var/tmp/openarmik-finger-review.s1DGQZ/tmp \
  scripts/build.sh --tests \
  --output-root /var/tmp/openarmik-finger-review.s1DGQZ/out
```

Result: exit 0.

- CAN: 1/1 passed.
- model: 4/4 passed, including canonical generation/digest determinism.
- commission: 2/2 passed.
- transport: 3/3 passed.
- control: 4/4 passed.
- installed native consumers built successfully.
- all three ROS packages built and installed.
- exactly 11 ROS tests registered.

Fresh full ROS CTest in isolated valid domain 223:

```bash
source /opt/ros/lyrical/setup.bash
source /var/tmp/openarmik-finger-review.s1DGQZ/out/install/setup.bash
ROS_DOMAIN_ID=223 \
  ctest --test-dir \
  /var/tmp/openarmik-finger-review.s1DGQZ/out/build/openarm_ik_ros \
  --output-on-failure
```

Result: **11/11 passed** in 69.96 seconds. This includes the real headless ROS
contract, deterministic visualization generation, canonical URDF/resource
validation, derived URDF parser validation, no-CAN linkage/syscall isolation,
active SIGINT, and the virtual session/controller tests.

A prior reviewer invocation used domain 243, which Fast DDS rejects because it
exceeds its supported port-derived domain range. Re-running unchanged tests in
valid isolated domain 223 passed completely; that invocation error is not a
product failure.

### Independent live publisher and TF probe

A separate fresh launch in isolated domain 224 produced:

```text
/joint_states publisher: openarm_ik_ros (one)
/tf publisher:           robot_state_publisher (one)
/tf_static publisher:    robot_state_publisher (one)
JointState names:         left joint1..7, right joint1..7 only
name/position/velocity/effort lengths: 14/14/14/14
dynamic TF edges:         14
static TF edges:          11
total unique TF edges:    25
dynamic/static overlap:   none
```

All four expected static finger edges were present from each `link7` to its
left/right finger. All 25 URDF child frames were reachable from `world` using
only the fourteen measured arm states. No finger name was present in
`JointState`. This independently confirms the automated assertions at
`test_ros_contract.py:15-25,64-79` and preserves one state/TF authority.

The normal diagnostic contract also remains truthful: virtual backend,
`collision_checked=false`, `physical_motion_authorized=false`, and
`state_source=oa_snapshot_encoder_feedback`. The fixed finger transforms do not
carry a fabricated encoder sample, timestamp, velocity, effort, freshness bit,
or feedback sequence.

### Fresh RViz

A fresh software-rendered RViz launch in isolated domain 225 initialized the
robot and remained responsive. Its Qt accessibility tree reported:

```text
Global Status: Ok
RViz is ready.
```

The console reported OpenGL initialization and no RobotModel, URDF, missing
transform, mesh, material, package, or unrealistic-finger-inertia message. The
independent live probe above simultaneously establishes all 25 transforms, so
the prior four missing-finger RobotModel statuses are closed. Later Qt AT-SPI
messages were induced solely by this review's accessibility introspection and
were unrelated to the robot model.

Raw `ros2 launch` retains the repository's pre-existing RViz/Ogre SIGINT
teardown behavior on this desktop; the provided `scripts/launch_rviz.sh`
already isolates and window-closes RViz for that reason. This change neither
touches that wrapper nor changes the behavior, so it is not a finding against
`9e5d50f`.

## Truthfulness and future upgrade

The generated URDF itself says at
`GenerateStageAVisualizationUrdf.cmake:91-99` that motor-8 finger state is not
measured, the fixed closed pose is a Stage-A visualization/TF convention, the
invalid inertials are omitted, and the file is prohibited for dynamics or
collision safety. The package README repeats the same boundary and explicitly
states that it is not a gripper measurement, calibrated gripper model, MoveIt
model, or collision-safety model (`README.md:9-24`).

The documented upgrade path is correct: when authoritative motor-8 feedback
exists, publish only the two measured source joints from the existing sole
`/joint_states` authority, restore movable/mimic finger joints for the launch,
and continue letting `robot_state_publisher` derive the mimic transforms. No
current diagnostic or model claim implies that upgrade already exists.

## Cleanup

Both live launches were stopped. No `openarm_ik_ros_node`,
`robot_state_publisher`, RViz, or OpenArm launch process from this review
remained. The temporary build/install tree was removed after evidence was
collected by moving it to the desktop trash (recoverable). The reviewed
worktree was otherwise left unchanged; this report is the only review file
added.
