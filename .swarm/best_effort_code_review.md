# Best-effort projection code review

## Verdict: BLOCKING

Read-only review of the current uncommitted diff. I inspected the projection implementation, its only production caller, action handoff, JSON/browser handling, documentation, core tests, browser oracle, and virtual-session test. I did not modify implementation or tests.

## Ranked findings

### 1. [P1] A detected keepout sample is not preserved as a collision barrier

`validate()` samples a candidate plan at endpoint-relative fractions and returns only a boolean that some waypoint collided; it discards the colliding waypoint/fraction (`ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:503`, `ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:532`, `ros2_ws/src/openarm_ik_ros/include/openarm_ik_ros/portal_core.hpp:66`). `validate_or_project()` then records the *candidate endpoint* as `rejected_distance` and breaks at the first keepout result (`ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:632`, `ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:638`). It subsequently refines toward that endpoint using a new 17-point grid for every midpoint and may promote any midpoint whose shifted grid misses the already-observed collision (`ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:644`, `ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:650`).

This is not merely the documented gap between sampled and continuous collision checking. If endpoint `D` collides at waypoint `u*D`, the previous coarse endpoint or a refined endpoint can be greater than `u*D`; re-sampling that longer path at different positions can pass even though the search has already observed a collision earlier on the same ray. The result therefore does not enforce the stated "first sampled keepout stops the search" / "never routes around it" invariant in `README.md:84`, `ros2_ws/src/openarm_ik_ros/README.md:82`, and `ros2_ws/src/openarm_ik_ros/src/portal_page.cpp:41`.

The pole/inter-arm tests only replay the final endpoint through the same endpoint-relative validator (`ros2_ws/src/openarm_ik_ros/test/test_portal_core.cpp:602`, `ros2_ws/src/openarm_ik_ros/test/test_portal_core.cpp:624`, `ros2_ws/src/openarm_ik_ros/test/test_portal_core.cpp:631`, `ros2_ws/src/openarm_ik_ros/test/test_portal_core.cpp:661`). They do not retain the first detected colliding ray position and prove that the returned endpoint remains before it, so they cannot catch this phase-shift failure.

### 2. [P1] Refinement assumes monotonic IK acceptance after the coarse search explicitly rejects that assumption

The coarse loop correctly continues after non-collision IK/branch failures because a farther candidate can validate (`ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:616`, `ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:622`). The 24-step refinement then treats *every* failed midpoint, including numerical IK and branch-continuity failures, as a monotonic upper bound and permanently discards its upper half (`ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:644`, `ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:651`). An accepted island in that discarded half is never considered. Thus the returned point is not generally the farthest guarded/reachable prefix claimed by the result and user documentation (`ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:667`, `ros2_ws/src/openarm_ik_ros/README.md:82`, `ros2_ws/src/openarm_ik_ros/README.md:123`).

The current tests assert broad fractions for three hand-picked rays but contain no nonmonotonic-IK oracle or refinement-island case (`ros2_ws/src/openarm_ik_ros/test/test_portal_core.cpp:538`, `ros2_ws/src/openarm_ik_ros/test/test_portal_core.cpp:602`, `ros2_ws/src/openarm_ik_ros/test/test_portal_core.cpp:631`). The virtual-session test consumes the already-selected endpoint and therefore cannot establish search optimality (`ros2_ws/src/openarm_ik_ros/test/test_virtual_control_session.cpp:226`, `ros2_ws/src/openarm_ik_ros/test/test_virtual_control_session.cpp:237`).

### 3. [P1] Zero-progress projections are accepted and submitted as successful moves

The stationary pose is stored as an accepted `best` result (`ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:557`, `ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:560`). If every positive candidate fails, `accepted_distance` remains zero, yet the function unconditionally marks and returns that stationary result as a projected success (`ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:597`, `ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:659`, `ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:662`, `ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:670`). This occurs for a safe measured pose on a reach/joint boundary with a ray directed strictly outside the locally feasible workspace; there is no minimum-progress or commanded-vs-measured check.

The only production caller treats `accepted=true` as authorization, sends the unchanged measured TCP as an action goal, and returns HTTP 202 (`ros2_ws/src/openarm_ik_ros/src/openarm_portal.cpp:1034`, `ros2_ws/src/openarm_ik_ros/src/openarm_portal.cpp:1040`, `ros2_ws/src/openarm_ik_ros/src/openarm_portal.cpp:1046`). The browser will then say the guard "moved" to a 0.00% prefix (`ros2_ws/src/openarm_ik_ros/web/portal.js:127`). The extreme-finite regression proves positive progress only from neutral for one `1e300` direction (`ros2_ws/src/openarm_ik_ros/test/test_portal_core.cpp:574`); it does not cover a genuine no-positive-prefix state.

### 4. [P2] The browser claims completion when the server has only queued an action

`PortalNode::move()` calls `async_send_goal()` and returns before the goal-response callback knows whether the controller accepted it (`ros2_ws/src/openarm_ik_ros/src/openarm_portal.cpp:476`, `ros2_ws/src/openarm_ik_ros/src/openarm_portal.cpp:481`, `ros2_ws/src/openarm_ik_ros/src/openarm_portal.cpp:535`). The API correctly responds with HTTP 202 (`ros2_ws/src/openarm_ik_ros/src/openarm_portal.cpp:1046`), but projected moves are rendered as "the guard moved" (`ros2_ws/src/openarm_ik_ros/web/portal.js:128`). A later controller rejection therefore leaves the form notice making a false completed-motion claim even though the status area changes.

The browser oracle intentionally asserts that its interactions issue no POST and never exercises a move response (`ros2_ws/src/openarm_ik_ros/test/viewer_browser_oracle.js:59`, `ros2_ws/src/openarm_ik_ros/test/viewer_browser_oracle.js:97`). The C++ asset test only searches for property names (`ros2_ws/src/openarm_ik_ros/test/test_portal_core.cpp:515`, `ros2_ws/src/openarm_ik_ros/test/test_portal_core.cpp:519`). There is no API/JS test covering projected, rejected, malformed, or zero-progress response behavior.

### 5. [P2] The visible "Full reach" label overstates the retained evidence

The portal calls High far "Full reach" (`ros2_ws/src/openarm_ik_ros/src/portal_page.cpp:43`), while the package README and checked-in test establish only that it exceeds 89% of a geometric centreline upper bound (`ros2_ws/src/openarm_ik_ros/README.md:100`, `ros2_ws/src/openarm_ik_ros/test/test_portal_core.cpp:831`, `ros2_ws/src/openarm_ik_ros/test/test_portal_core.cpp:854`). The test pins the chosen coordinate and its neutral displacement but does not reproduce the documented 1 cm search or prove it is the farthest symmetric safe target (`ros2_ws/src/openarm_ik_ros/test/test_portal_core.cpp:844`, `ros2_ws/src/openarm_ik_ros/test/test_portal_core.cpp:864`). The sample may be useful and well-cleared, but "Full reach" is not an honest description of the checked-in evidence.

## Invariants that did hold under inspection

- The revised ray scaling/cap keeps evaluated candidate coordinates finite for finite portal targets, including distances whose mathematical norm overflows; fraction computation avoids multiplying the overflowing norm (`ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:570`, `ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:592`, `ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:603`, `ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:659`). The new `1e300` regression is directionally useful, although `DBL_MAX`/mixed-sign extremes are not tested.
- Invalid/nonfinite measured state and nonfinite targets fail closed (`ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:483`, `ros2_ws/src/openarm_ik_ros/src/portal_core.cpp:494`, `ros2_ws/src/openarm_ik_ros/test/test_portal_core.cpp:667`).
- The projected command, rather than the original request, is copied into the ROS action (`ros2_ws/src/openarm_ik_ros/src/openarm_portal.cpp:469`). Freshness/generation handoff is checked once before reservation and again immediately before `async_send_goal()` (`ros2_ws/src/openarm_ik_ros/src/openarm_portal.cpp:442`, `ros2_ws/src/openarm_ik_ros/src/openarm_portal.cpp:524`).

