# Best-effort / collision / preset / unit verification

## Status: CLEAN

No Critical, Important, or Minor blocker remains in the reviewed virtual-only
best-effort workflow. This is an audit of the uncommitted tree at `HEAD`
`944c05da9c705e5a5e2e67ac9c510c34c1f9d778`. No production file was edited,
staged, committed, or transmitted to physical CAN by this verifier. The only
repository write made by this verifier is this report.
`transport/tests/test_transport.cpp` was not touched.

## Exact-tree authority

- The final production source fingerprint is
  `a4b303d443993541984ef6f25a6d2d98b04963af88cac8acf512226f3276a6cc`;
  it exactly matches the final launch stamp.
- The final installed-manifest digest is
  `350226138a5a80fc76fe2874fb4903fd18b930b19ad0386fe487622a97b5a984`;
  it also exactly matches the stamp. The stamp records `run_tests=0`.
- `openarm_validate_description_pin` and
  `openarm_assert_current_launch_tree` passed after all source/documentation
  changes and after the final live sessions were stopped.
- `git diff --check` (excluding the user-owned transport test and `.swarm`
  reports) passed. `node --check` passed for `portal.js`, `viewer.js`, and the
  browser oracle.

## Reviewed fixes

- Guard failures now retain the actual failing 17-waypoint path fraction.
  Projection converts that fraction to an absolute ray distance, keeps the
  nearest pole/inter-arm waypoint as a strict irreversible barrier, and drops
  any earlier accepted endpoint at or beyond a newly discovered barrier.
- Projection uses overflow-resistant max-component ray normalization, a 2 m
  physical search cap, 64 coarse samples, then full 16/32/48 refinement scans.
  Numerical IK failures do not cause a monotonic binary-search truncation;
  sampled keepout failures still terminate progress at their retained barrier.
- A projected result is accepted only after at least `0.001 m` of validated
  progress. The legacy `[0.28, 0.80, 0.60] m` ray is consequently a deliberate
  no-motion/HTTP 422 regression rather than a zero-progress success.
- `CommandReservationGate` reserves before measured-state capture and guard
  evaluation. Stop invalidates an unconsumed token; token consumption and goal
  submission are serialized so a later stop cancels the pending/active goal.
  `/api/stop` has its own one-worker, four-admission urgent lane, independent
  of the four-worker ordinary API lane.
- Browser wording says a projected command was **queued only**; it does not
  claim completed movement. Reach wording is **Near-full audited reach**, not
  “Full reach.” The genuine pole regression is `[0.40, +/-0.05, 0.40] m`.
- Public m/cm/in inputs normalize exactly once to binary64 metres. Internal
  guard, IK/FK, action, state, and JSON paths remain `double`; no float
  narrowing was found.

## Build and automated evidence

- `./scripts/build.sh --tests --incremental --jobs 1` completed from the final
  implementation. Native suites passed: CAN 1/1, model 6/6, commissioning 2/2,
  transport 3/3, control 4/4, and runtime 12/12. The tests-enabled source/stamp
  fingerprint matched at
  `7368003120cf404ad9de401b0e43a9ca440e443ab163b9249d4aca015fd119c6`.
- Current source-built `test_virtual_control_session` passed 18/18 in
  273.449 s, including genuine pole projection and measured completion.
  `test_portal_core` passed 34/34 in about 1.31 s, including hard-barrier,
  non-monotonic refinement, huge finite ray, sub-millimetre rejection, units,
  page contracts, presets, and command reservation cases.
- All 15 registered ROS-package CTest entries are covered on the current build:
  the two tests above, seven other entries that passed in the initial full run,
  and the six ROS-dependent entries that passed 6/6 in 72.49 s after sourcing
  both `/opt/ros/lyrical/setup.bash` and `ros2_ws/install/setup.bash`. The same
  six had first failed in a bare shell solely from missing installed shared
  libraries/Python modules; this report does not misstate the split evidence as
  one clean 15/15 invocation.
- The exact preset tables contain nine targets per arm with mirrored Y. High
  far is `[0.28, 0.67, 0.52] m` left and `[0.28, -0.67, 0.52] m` right. The
  1,800-case cross-state matrix passed with minimum sampled clearance
  `0.026598477992 m`; high-far neutral displacement was
  `0.736448405603 m`.
- Description provenance is detached and clean at
  `enactic/openarm_description@6c7b720f1ba48e8bafa3a3dc752c45f397b42221`.
  The pinned URDF-derived shoulder-to-TCP centreline bound is 0.747--0.748 m.

## Installed browser and live API evidence

- The installed browser gate passed on the final runtime artifact manifest:
  URDF parsing, portal health, resident-asset mutation invariance, Firefox
  WebGL/rollback oracle, and corrupted-prefix SHA-256 rejection all passed.
  Stop remained available in 11 ms under eight throttled static clients and in
  12 ms under sixteen partial request bodies; intake resources were reclaimed.
  SIGTERM cleanup completed in 113 ms. The oracle now derives its synthetic
  sequence above live state, eliminating the former fixed-sequence harness race.
- A finite `1e300 m` target returned HTTP 202 without overflow, projected to
  `[0.378859011976, 0.532354722706, 0.454885666175] m`, and completed from
  measured virtual feedback.
- The genuine left pole request `[0.40, 0.05, 0.40] m` returned HTTP 202,
  projected 15.3293609619% to
  `[0.061294496259, 0.137607532064, 0.125666750209] m`, then completed with
  measured TCP `[0.061234278863, 0.137523467537, 0.125696472999] m` and
  `outcome=1`.
- From a fresh neutral session, exact High far was accepted and visibly active
  before stop. Urgent stop returned HTTP 200 in 10 ms and produced
  `canceled_disable_stop`, `outcome=2`, with no active command remaining.
  Both final launch sessions shut down their portal and ROS children cleanly on
  Ctrl+C.

## Deliberate limitations

- This is sampled nominal virtual protection: 17 straight-line waypoints,
  capsule proxies, and a central-shaft keepout. It is not continuous collision
  detection, obstacle routing, environment/self-collision certification, or a
  safety-rated stop.
- TCP orientation is unconstrained. The WebGL view is a measured-pose proxy,
  not collision checking or physical control feedback.
- The controller correctly continues to report `collision_checked=false`.
  No physical arm, CAN-FD adapter, calibration, zeroing, polarity, motion,
  collision response, E-stop wiring, or stop distance was exercised.
