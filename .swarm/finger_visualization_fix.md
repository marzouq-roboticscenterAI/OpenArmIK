# Stage-A finger visualization fix

Date: 2026-07-29 (America/Los_Angeles)
Branch: `fix/rviz-finger-visualization`
Verdict: **DONE**

## Outcome

The RViz launch now gives its existing sole `robot_state_publisher` a distinct,
reproducibly derived Stage-A visualization/TF description:

- `openarm_{left,right}_finger_joint{1,2}` retain their names, parents,
  children, and exact canonical origins, but are fixed at the canonical
  `q=0 m` closed transforms.
- Only the now-inapplicable `axis`, `limit`, and `mimic` elements are removed
  from those four joints.
- Only the four invalid direct finger-link inertials are omitted. Finger links,
  visuals, collision geometry, mesh URIs/scales, materials, and link-local
  origins remain unchanged.
- The canonical dynamic URDF, generated C model, control manifest/snapshot,
  fourteen-name measured `JointState`, IK/collision behavior, hand/TCP/world
  frames, and TF/JointState authority topology are unchanged.

This is explicitly documented in the derived file and package README as an
**unmeasured closed-pose visualization convention**, not gripper feedback, a
dynamics model, a calibrated gripper model, or a collision-safety model.

## Implementation

`cmake/GenerateStageAVisualizationUrdf.cmake` is a build-time-only CMake
transformation. It:

1. requires canonical input SHA-256
   `dd4b5d5fa0c050b78922bc95850ea65bb15721cb259030cccf106a953f171c55`;
2. fails closed unless every approved source block has the exact expected
   cardinality;
3. applies only the four fixed-joint and four inertial omissions; and
4. writes a distinct
   `openarm_v10_bimanual_stage_a_visualization.urdf` containing provenance and
   truthfulness warnings.

There is no new Python runtime or generator. CMake installs both the canonical
and derived descriptions under separate names; only the visualization launch
selects the derived file. No additional publisher or node was introduced.

## Static and semantic verification

- Canonical source and installed hashes both remain
  `dd4b5d5fa0c050b78922bc95850ea65bb15721cb259030cccf106a953f171c55`.
- Derived visualization hash is
  `4e2838fd90240ed140d5624316fdfbb2ad96f97662aa62f832b39924499dd0de`.
- `test_visualization_urdf` proves deterministic byte-for-byte regeneration,
  26 identical link names, 25 identical joint names and graph edges, fixed
  finger joints with no axis/limit/mimic, absent finger inertials, exact
  preservation of every non-finger subtree and every finger visual/collision
  subtree, and exact canonical-vs-derived FK equivalence with finger `q=0`
  over zero, both arm-limit endpoints, and twelve deterministic randomized arm
  postures.
- `/opt/ros/lyrical/bin/check_urdf` successfully parses the derived tree rooted
  at `world`.
- A direct textual diff contains only the provenance warning, four inertial
  omissions, four type changes, four axes, four limits, and two mimic removals.
- `git diff -- model control ros2_ws/src/openarm_ik_ros/src` is empty.

## Build and automated tests

The worktree-local `/tmp` filesystem had insufficient quota for a clean build,
so the same repository build script was run cleanly with isolated outputs and
temporary files under `/var/tmp`:

```bash
TMPDIR=/var/tmp/openarmik-finger-viz-tmp \
  scripts/build.sh --tests \
  --output-root /var/tmp/openarmik-finger-viz-build
```

Result: exit 0. All native suites passed:

- can: 1/1
- model: 4/4, including canonical generator/digest determinism
- commission: 2/2
- transport: 3/3
- control: 4/4
- installed native consumers built successfully
- all three ROS packages built and installed; 11 ROS tests registered

Full isolated ROS CTest:

```bash
source /opt/ros/lyrical/setup.bash
source /var/tmp/openarmik-finger-viz-build/install/setup.bash
ROS_DOMAIN_ID=189 TMPDIR=/var/tmp/openarmik-finger-viz-tmp \
  ctest --test-dir /var/tmp/openarmik-finger-viz-build/build/openarm_ik_ros \
  --output-on-failure
```

Result: exit 0, **11/11 passed** in 69.84 seconds. The live headless contract
retains exactly fourteen measured arm names and no finger names, observes one
publisher each on `/joint_states`, `/tf`, and `/tf_static`, and resolves all
25 world-to-child transforms/all 26 URDF frames.

Logs:

- `.swarm/finger_visualization_fix_build.log`
- `.swarm/finger_visualization_fix_ros_tests.log`

## Live RViz verification

An isolated virtual-only session ran in `ROS_DOMAIN_ID=190` from the branch
install. RViz used Mesa software rendering and remained responsive. The
expanded RobotModel status showed:

- overall `Status: Ok`;
- `Topic`: OK with messages received;
- `URDF parsed OK`;
- `Transform OK` (with automated coverage of all 25 children); and
- a fully rendered bimanual model.

The RViz console contained no unrealistic-finger-inertia, missing-transform,
mesh, material, package, or URDF errors. Screenshot:
`.swarm/finger_visualization_fix_status.png`.

RViz and the launch were both stopped with Ctrl+C. RViz handled SIGINT; the
adapter and robot-state-publisher reported clean completion. The isolated ROS
domain was empty afterward. A process leaked only by the deliberately failing
pre-fix test setup was identified by exact PID/process group and also stopped;
the test now places TF listener construction inside its cleanup guard.

## Remaining convention / future upgrade

The displayed finger aperture is intentionally unknown and shown closed. A
future motor-8 capability must add authoritative per-side gripper state to the
existing sole JointState authority, publish only the two source finger joints,
and switch the launch back to movable/mimic finger joints. Until then, this
derived description must not be used for dynamics, MoveIt, or collision-safety
decisions.
