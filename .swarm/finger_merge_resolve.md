# Finger visualization merge resolution

Date: 2026-07-29 (America/Los_Angeles)  
Verdict: **DONE**

## Outcome

The paused cherry-pick was resolved without dropping either side. The merged
`scripts/build.sh` retains the current native pipeline, including control and
installed-consumer tests, and the three-package ROS build for
`openarm_description`, `openarm_control_msgs`, and `openarm_ik_ros`. It also
retains deterministic generation, installation, and testing of the distinct
Stage-A visualization URDF.

The combined CMake source registers twelve ROS tests: the ten tests already on
current `main` plus both visualization tests (`test_visualization_urdf` and
`test_visualization_urdf_parser`). The build invariant was therefore reconciled
to the source-derived count of 12.

Cherry-picked commits:

- `9e5d50f` -> `611505f` (`Fix Stage-A RViz finger visualization`)
- `7ef2866` -> `d7289cc` (`docs: record clean finger visualization review`)
- merged test-count correction: `8f48066`

## Verification

The worktree-local `/tmp` tmpfs returned `EDQUOT`, so fresh outputs and
temporary clones were placed on `/var/tmp`, which is on the main filesystem.
No GUI, CAN interface, or hardware was used.

Build command:

```bash
TMPDIR=/var/tmp/openarmik-finger-merge-resolve-tmp \
  scripts/build.sh --tests \
  --output-root /var/tmp/openarmik-finger-merge-resolve-build
```

After the source-derived count correction, the complete incremental validation
of that fresh tree exited 0:

```bash
TMPDIR=/var/tmp/openarmik-finger-merge-resolve-tmp \
  scripts/build.sh --tests --incremental \
  --output-root /var/tmp/openarmik-finger-merge-resolve-build
```

Native results: 14/14 passed.

- CAN: 1/1
- model: 4/4, including generator determinism
- commission: 2/2
- transport: 3/3
- control: 4/4

All three ROS packages built and installed, the visualization URDF target was
built and installed separately from the canonical URDF, and exactly 12 tests
were registered.

Full isolated ROS test command:

```bash
source /opt/ros/lyrical/setup.bash
source /var/tmp/openarmik-finger-merge-resolve-build/install/setup.bash
ROS_DOMAIN_ID=218 TMPDIR=/var/tmp/openarmik-finger-merge-resolve-tmp \
  ctest --test-dir \
  /var/tmp/openarmik-finger-merge-resolve-build/build/openarm_ik_ros \
  --output-on-failure
```

Result: **12/12 passed** in 91.36 seconds. In particular:

- `test_cli_server_lifecycle` passed;
- deterministic visualization generation and `check_urdf` parser tests passed;
- the live headless `test_ros_contract` passed, asserting all 25 URDF child
  transforms from `world`, exactly fourteen measured arm joint states and no
  finger state, and one publisher each on `/joint_states`, `/tf`, and
  `/tf_static`;
- no-CAN linkage/syscall isolation and active SIGINT tests passed.

## Worktree preservation

The pre-existing unstaged `transport/tests/test_transport.cpp` change and the
untracked `.swarm/screenshot_portal_call.log` and
`.swarm/screenshot_portal_response.log` files remain unstaged and unmodified by
this resolution.
