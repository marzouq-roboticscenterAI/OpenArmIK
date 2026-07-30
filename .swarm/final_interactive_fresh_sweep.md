# Final interactive whole-branch sweep

- Target: `8fc4b16ef30c7034c6ba13a0c1e149abdd658314`
- Range: `658174a..HEAD`
- Verdict: **CLEAN**
- Critical: **none**
- Important: **none**
- Actionable Low: **none**

This was a fresh source/caller/test audit of the WebGL viewer, portal server,
launcher/build/install path, nine-target and unit semantics, and virtual-only
authority boundary. No production source was edited, no physical/CAN path was
opened, and no move request was issued. Pre-existing worktree changes were
preserved.

## Areas found clean

- The viewer renders only the pinned Stage-A collision-proxy robot geometry,
  has browser-local mouse/touch orbit, wheel/pinch zoom, reset and resize, and
  sends no camera input to X11 or the motion API. It uses no VNC/noVNC service.
- The viewer applies the exact authoritative 14-name joint mapping, rejects
  rollback/nonfinite state, marks stale/hidden/context-loss states, and bounds
  canvas pixels, STL bytes/triangles, instances, GPU buffers/bytes, metrics,
  fences, and context recovery. Foreground rendering is driven by
  `requestAnimationFrame`; the recorded visible Firefox result is 60.01 draw
  submissions/s at a 1920x1080 backing buffer. This is correctly described as
  draw submission, not compositor scanout or RViz fidelity.
- Portal intake has bounded asynchronous headers/body/deadline/session state;
  static and API work have separate bounded pools. Every request is exact-Host
  gated, mutations additionally require exact Origin, unique JSON content type
  and a `getrandom` CSRF token, and installed assets are exact-size/SHA-256
  verified into bounded resident storage before serving. Worker and intake
  lifetimes remain valid through shutdown.
- Ctrl+C/SIGTERM shutdown is bounded: the launcher terminates the portal first,
  then the ROS process group, and the portal cancels an active/pending software
  goal before ROS teardown. Wording consistently says this is not a hardwired
  or safety-rated E-stop.
- All nine stable IDs/labels/coordinates per arm feed the same guarded path;
  presets only fill fields, the opposite arm comes from the guarded measured
  TCP, and the sampled IK/FK capsule/pole guard is revalidated against fresh
  state and diagnostics immediately before send. Text consistently says this
  is sampled nominal protection, not collision checking or physical
  certification.
- The v2 portal boundary parses binary64 values with an explicit `m`, `cm`, or
  `in` unit and normalizes once through the Model units API. The unchanged
  legacy endpoint, guard, ROS action, controller, Runtime, and model remain
  binary64 metres; no float truncation was found.
- The viewer manifest pins all 11 STL sizes, hashes, triangle counts, upstream
  repository/commit, and Apache-2.0 provenance. The bundled 11,357-byte license
  is byte-identical to pinned `upstream/openarm_description/LICENSE.txt`. The
  license and every served/installed viewer artifact participate in build and
  launch integrity.
- Firefox, geckodriver, jq, and `check_urdf` are present on the acceptance host;
  the dependency installer declares Firefox/jq and verifies geckodriver. The
  production diff adds no Python, Java, or Node implementation. The native
  transport/CAN directories have no committed change in this range.
- The web launcher is loopback-only and keeps the virtual Runtime backend; no
  browser event can authorize physical motion. Stock RViz remains a separate,
  accurately qualified engineering launcher. The corrected process inventory
  says that the web launcher starts the controller, `robot_state_publisher`,
  and portal but not RViz, and the proxy is not called exact RViz pixels or
  collision checking.
- The normal viewer has no autonomous camera rotation. Its initial/reset camera
  values are constant, and the only yaw/pitch write site is the active
  one-pointer `pointermove` handler; distance changes only in the pinch/wheel
  handlers or reset. Neither the draw loop, state polling, pose updates nor a
  timer mutates camera state.
- Both production launchers now export `PYTHONDONTWRITEBYTECODE=1` before
  invoking ROS launch. The ROS contract propagates the same setting, while the
  launch-integrity regression imports the installed launch file twice, rejects
  any new cache/bytecode path, compares the full install digest, and exercises
  repeated `--no-build` validation.

## Focused verification

- `git diff --check 658174a..HEAD`: pass.
- `bash -n` on changed launch/build/dependency and viewer-test scripts: pass.
- On current `8fc4b16`, a real active `--no-build --no-browser` virtual launch
  was observed without interfering with hardware or CAN. After shutdown, the
  full installed-tree digest remained exactly equal to the stamped value,
  `39a41ce9c8c24a690b9abd7214fdccc58d641055d21ff13d8b377fb54ae5c378`,
  and the installed launch directory contained no `__pycache__`, `.pyc`, or
  `.pyo`. Generated message bytecode already present elsewhere in the stamped
  build closure was unchanged and is not launch-time self-mutation.
- Current-commit existing build, focused CTest: `test_portal_core`,
  `test_visualization_urdf`, `test_viewer_configure_dependency`, and the actual
  loopback portal + headless Firefox/WebGL `test_visualization_urdf_parser`:
  **4/4 passed** in 8.17 seconds.
- The focused browser test covered camera input isolation, exact target button
  rendering, left/right transform oracles, resource caps, rollback/stale/
  hidden/context-loss behavior, immutable asset serving, slow/partial clients,
  prompt authenticated stop, and corrupt-prefix fail-closed startup.
- A separate filtered all-target virtual-session rerun was stopped with SIGINT
  after roughly 70 seconds to keep this sweep lightweight; it did not fail or
  finish in this run. Exact all-nine guard/unit/1,800-case coverage passed in
  `test_portal_core`, and the existing full target-session evidence was not
  treated as a fresh execution result.
