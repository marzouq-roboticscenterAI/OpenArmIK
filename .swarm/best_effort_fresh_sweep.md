# Fresh whole-diff sweep: CLEAN

No remaining Critical, Important, or Minor findings in the current tracked diff (excluding `transport/tests/test_transport.cpp`).

## Reviewed invariants

- The nine symmetric wide presets match the URDF-derived coordinates, pinned reach evidence, browser serialization, docs, all-arm execution coverage, and the 1,800-case endpoint/cross-state matrix (minimum recorded sampled clearance `0.026598477992 m`).
- Portal m/cm/in input is normalized exactly once to binary64 metres; canonical browser state, state JSON, ROS action fields, model/guard arithmetic, and legacy metre-only API behavior remain consistent.
- Huge finite rays use a scaled direction and a conservative 2 m physical search cap, avoiding norm overflow and fraction under-resolution; nonfinite inputs and invalid/currently unsafe measured scenes fail closed.
- Every accepted candidate revalidates its complete 17-waypoint plan. The actual first failing pole/inter-arm waypoint becomes an irreversible absolute ray barrier, previously accepted endpoints beyond it are discarded, and the progressively denser 16/32/48 scans continue across isolated non-keepout IK failures without binary monotonicity assumptions. Sub-1-mm progress rejects as no motion.
- Projected coordinates, requested coordinates, achieved fraction, limiting reason, and virtual-only/no-physical-certification status are propagated truthfully through the action handoff, API, browser, and docs. The revised genuine pole ray and inter-arm cross-preset ray remain before their retained barriers.
- A command reservation exists before state capture/guard evaluation. Software stop invalidates an in-flight guard token before submission; after token consumption it queues cancellation against the pending goal. `/api/stop` also has an independently bounded urgent worker/admission lane, separate from ordinary API projection work.
- Production caller/sibling search found one `validate_or_project()` caller (`openarm_portal.cpp`) plus intended tests; no stale preset IDs, unit paths, alternate command submitters, or unreviewed projection consumers remain.

## Current evidence

- `git diff --check -- . ':(exclude)transport/tests/test_transport.cpp' ':(exclude).swarm'` — pass.
- `ros2_ws/build/openarm_ik_ros/test_portal_core` — 34/34 pass in 1.26 s on the current source-built binary.
- Focused `CommandReservationGate.*:NominalPathGuard.BestEffort*` — 7/7 pass in 0.39 s: impossible targets for both arms, `1e300` finite ray, genuine pole mitigation for both arms, inter-arm barrier, sub-mm no-motion rejection, invalid/nonfinite fail-closed behavior, and stop reservation invalidation.
- The current one-job ROS build compiled and installed `openarm_portal`, `test_portal_core`, and `test_virtual_control_session`. Earlier full-suite evidence records ROS CTest 15/15 in 349.27 s; the current focused/core runs above cover the final projection/barrier/reservation edits.
- Read-only inspection covered the complete tracked diff, all guard/unit/preset/stop callers and siblings, portal API/UI/docs, current build logs, and recorded matrix/live-API evidence. The unrelated transport test and unrelated `.swarm` files were not modified.

Physical motion remains intentionally unsupported and unvalidated; this CLEAN result is limited to the sampled nominal virtual workflow.
