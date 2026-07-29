# Fresh whole-branch audit

Audited implementation through `3067e12353d490cb042daf23ac8b96bd231fc769`,
including the verification-only ledger update at
`748fcc0abd166aae44f1163bf3ac75d5643876e8`.

## Verdict

**CLEAN.** No Critical, Important, or Minor findings remain in the requested
hardware-free scope. The repository truthfully delivers deterministic OpenArm
v1.0 model data, pure-C kinematics/position IK, atomic paired virtual ROS control,
RViz visualization, CAN protocol/read-only diagnostics, and a deliberately
disarmed physical-hardware boundary. It does not claim that unknown physical
arms can be safely auto-commissioned or moved.

## Requirement audit

- **Canonical research and sources:** all ten repositories from Enactic's
  canonical OpenArm index are present as clean, full, non-shallow clones at the
  commits recorded in `UPSTREAM_SOURCES.md`. Origins and current HEADs match the
  manifest. `scripts/fetch_upstreams.sh` now reproduces all ten exact pins. The
  unlicensed vendor firmware repository and unrelated similarly named projects
  are correctly excluded.
- **Deterministic v1.0 model:** generation requires the exact clean
  `openarm_description@6c7b720f`, authenticates the complete relevant source set,
  xacro implementation, and generator, and reproduces both checked-in outputs
  byte-for-byte. The flattened URDF hash is
  `dd4b5d5fa0c050b78922bc95850ea65bb15721cb259030cccf106a953f171c55`.
  `check_urdf` parses the bimanual tree through both named `hand_tcp` tips.
- **Pure C FK/IK:** `oa_fk` exposes base, all seven pre-joint/post-link transforms,
  body-frame axes, and exact TCP. The 6x7 geometric Jacobian and bounded,
  position-only IK use fixed-width/versioned ABI records, validate output capacity
  before writing, enforce effective URDF bounds, return explicit failure statuses,
  and never advertise collision checking. The legacy ABI-v1 symbol fails closed.
- **ROS/RViz virtual control:** one stamped `world`-frame `PoseArray` carries
  exactly two targets ordered left/right. Both IK results commit atomically or the
  prior complete dual-arm state is retained. JointState naming/order matches the
  generated URDF; `robot_state_publisher` is the sole TF authority. Diagnostics
  identify the immutable redundancy policy, virtual backend, success/failure,
  residuals, achieved TCPs, and `collision_checked=false` without fabricating
  failed results. The executable has neither CAN linkage nor a PF_CAN syscall.
- **CAN safety boundary:** the C11 module implements reviewed DaMiao golden-vector
  codecs, full feedback/fault decoding, finite/range policy, fixed-width ABI,
  manifest validation, fresh-disabled expected-ID probing, bounded timestamps and
  receive budgets, synthetic netlink parsing, read-only Linux CAN-interface
  inspection, and a fake transport that rejects control frames. Documentation is
  explicit that probing cannot prove motor family, serial, joint assignment,
  side, sign, zero, gearing, firmware, timeout, or safe configuration. No physical
  transport or auto-arm path is supplied.
- **Truthful hardware scope:** top-level and module documentation consistently say
  the current ROS path is virtual-only, orientation-free and collision-unchecked.
  Physical motion remains blocked on per-arm commissioning, two buses/adapters,
  validated mapping/limits, watchdogs, an independent E-stop, and supervised
  acceptance. No physical CAN interface or arm was available during verification.
- **Git hygiene:** the audited tree was clean, `git diff --check` passed, ignored
  upstream/build products are not committed, no credential/private-key pattern was
  found in project sources, and the local repository has no remote or open-PR
  surface to cross-check.

## Fresh verification evidence

- CAN: fresh GCC 15.2 strict C11 ASan+UBSan build and CTest, **1/1 passed**.
- Model: fresh GCC 15.2 ASan+UBSan build with authenticated generator inputs,
  **4/4 passed**, including 3,200 randomized bounded IK cases, 600 independent
  URDF FK/Jacobian cases, ABI-v1 canary, and byte-for-byte generation.
- ROS 2 Lyrical: fresh standalone model install plus isolated
  `openarm_description`/`openarm_ik_ros` build, **5/5 registered tests and 10 test
  cases passed**. These cover atomic transaction behavior, TF/TCP accuracy,
  generated URDF/mesh resolution, invalid expiry parameters, and CAN isolation.
- RViz evidence was independently reproduced on the actual NVIDIA desktop with
  OpenGL/GLSL 4.6, meshes and TF loaded, and successful paired XYZ motion. The four
  upstream finger-inertia warnings are disclosed. A later Ctrl-C teardown produced
  an RViz-only shutdown signal after successful operation; both project nodes
  exited cleanly, so this is not represented as a project motion/runtime success
  beyond the verified session.

## Acceptance boundary

This is a complete, tested virtual OpenArm v1.0 Cartesian-control foundation and
a hardware-safe C diagnostics/codec foundation. It is intentionally not a
physical robot controller. Claiming physical claw motion before commissioning and
supervised hardware acceptance would be unsupported and unsafe.
