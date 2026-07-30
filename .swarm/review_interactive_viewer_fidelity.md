# Independent re-review: interactive WebGL viewer fidelity

Range reviewed: `6f645d5..6203c8a` in the isolated `interactive-webgl-viewer` worktree.

Verdict: **CLEAN**. All three prior P1 findings and both prior P2 findings are closed. No new correctness, safety, or fidelity regressions were found in the remediation diff.

## Finding closure

1. **P1 — exact JointState mapping: closed.** `map_canonical_joint_state()` now requires exactly 14 names and 14 positions, maps by the canonical joint names, and rejects unknown, missing, duplicate, non-finite, and length-mismatched input before committing state. Tests cover a permuted valid set plus extra, missing, conflicting duplicate, unknown, mismatched-length, and infinite-value cases.

2. **P1 — executable real viewer/TF interaction oracle: closed.** The focused browser test launches the actual portal and a headless Firefox/WebGL viewer, exercises camera input, and checks matrices emitted by the running viewer for both arm tips and fixed finger links. An independent C++ oracle checks the same asymmetric all-14-joint pose against the public FK model, including mirrored right-arm scale and fixed-finger transforms. The test also verifies that camera-only interaction emits no control request.

3. **P1 — persisted target coverage: closed.** Tests assert the exact 18 target coordinates, stable IDs and labels, v2 centimetre/inch display normalization, exact selected-arm endpoints, and opposite-arm fresh measured semantics. A persisted 1,800-case quantized endpoint matrix is checked. The virtual-control session executes all 18 selected-side target sessions, starting each from current measured feedback, and verifies the selected endpoint, opposite measured FK, sequence advancement, and fail-closed authority flags.

4. **P2 — pinch isolation: closed.** Pointer handling returns from the multi-pointer pinch path before orbit logic. The real browser oracle confirms that pinch changes camera distance without changing yaw or pitch and exercises pointer cancellation.

5. **P2 — resource and failure caps: closed.** Viewer loading now enforces finite STL vertices, per-file and aggregate byte/triangle limits, manifest mesh/instance cardinality, decoded/GPU byte limits, and bounded canvas dimensions/pixels. Context-loss recovery is session-capped and becomes terminal after the permitted recovery count. Hidden tabs explicitly report `VIEW THROTTLED` and pause FPS reporting. The browser oracle exercises non-finite STL rejection, exact aggregate counts/caps, hidden-tab reporting, and terminal context-loss behavior.

## Regression review

- Transform composition and left/right mirrored model semantics remain consistent with the public model.
- Camera state remains local to the browser and preset controls remain fill-only.
- Installed asset integrity is strengthened by startup hashing and resident immutable assets under a bounded memory cap; corrupt installed assets fail health checks.
- Static and API work use separate bounded pools, and the focused test covers backpressure behavior.
- No production files were modified during this review.

## Verification performed

All focused tests passed in the isolated review build:

- `test_portal_core` — includes exact mapping and the persisted 1,800-case target matrix.
- `test_visualization_urdf_parser` — actual portal plus headless Firefox/WebGL oracle, asset integrity, and backpressure checks.
- `test_visualization_urdf` — URDF provenance, licensing, and visualization closure.
- `test_virtual_control_session` — all 18 sequential target sessions against the virtual backend.
- `tests/test_launch_integrity.sh` — installed launch freshness and authority regression.

These were virtual/read-only review checks; no physical arm or CAN interface was used.

## Review history

The original review of `6f645d5` reported three P1 and two P2 findings. This re-review supersedes that verdict for range `6f645d5..6203c8a`.
