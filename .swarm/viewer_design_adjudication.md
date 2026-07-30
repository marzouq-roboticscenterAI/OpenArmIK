# Embedded viewer design adjudication (2026-07-30)

## Decision

Build a **same-origin browser-native WebGL2 measured-pose viewer** and stop launching RViz for the web portal. Do not build the RViz/Ogre render-target bridge in the first release.

The first releasable viewer should be deliberately narrow:

- The C++ portal publishes the latest accepted 14-joint measured snapshot at a bounded 30 Hz. It publishes no TF tree, ROS graph, generic WebSocket, or camera commands.
- Authored browser JavaScript parses the already-derived Stage-A visualization URDF, selects only its collision meshes, and renders them with a small WebGL2 renderer. Drag, wheel, and pinch update only browser-local camera matrices.
- Use the 11 unique pinned collision STL files already supplied by `openarm_description`: 2,498,724 encoded bytes and 49,956 unique triangles. The complete 23-instance robot draws 94,576 triangles. This is a credible laptop WebGL workload and avoids the roughly 69 MiB unique DAE visual input set.
- Do not add Three.js, `urdf-loader`, npm, a CDN, Node, Python, Java, VNC, RFB, X input injection, or another listener to the production viewer. Browser JavaScript is unavoidable, but there is no Java runtime and no new Python/runtime interpreter. Existing ROS launch Python is unchanged.
- Keep `scripts/launch_rviz.sh` and the stock RViz configuration as the separate engineering viewer.

This is the smallest implementation we can build, exercise headlessly in the installed Firefox 153, package in the existing colcon/install-integrity closure, and reason about on this Wayland laptop. It also removes the portal's forced XWayland/GLX/software-RViz process, XComposite readback, RGB copy, and per-request JPEG encode.

It is better than RealVNC **for the requested robot-view task**: it is robot-only, resolution-independent, directly interactive, measured-pose aware, stale-aware, and does not encode pixels or expose a remote desktop/input protocol. It is not more capable than a remote desktop at operating arbitrary RViz panels, and we must not claim that it is.

## Naming and fidelity contract

Do not call the canvas “RViz”, “embedded RViz”, or “actual RViz pixels”. Name it **OpenArm measured-pose viewer** and label the geometry **visual proxy — not collision checking**.

The fidelity contract is:

- **Exact:** accepted measured joint sample, canonical 14-name order, Stage-A fixed-finger convention, URDF joint origins/axes, and link kinematics within a tested floating-point tolerance.
- **Intentionally not exact:** RViz/Ogre materials, DAE visual surfaces, lighting, shadows, view-controller details, antialiasing, focal marker, and pixels. The collision STLs are lower-detail geometry used only as a low-memory visual proxy.
- The existing `collision_checked=false` and “Controller collision checked: NO” text remains unchanged. Loading files whose URDF element is named `collision` must never imply that the displayed pose or requested path was collision checked.

If exact RViz pixels later become a hard requirement, that is a different product and the compiled render-target bridge is the defensible option. It needs an RViz plugin, Ogre-version-specific render-target/readback work, Qt GUI-thread camera integration, a bounded framebuffer/PBO ring, an encoder, browser transport, and driver/platform tests. Even a 1280x720 three-slot RGBA ring is about 10.5 MiB and moves about 111 MB/s at 30 FPS before encoding. It is not the smallest implementation now, and its exactness still depends on codec settings, scaling, and browser presentation.

## Why not the other apparent shortcuts

- Extending the current JPEG endpoint cannot meet the requirement. It is requested at 2.86 Hz, captures top-level Qt chrome, serializes X tree/readback/pixel conversion/JPEG work behind a mutex, and shares the request workers. It has no frame sequence or paint/freshness contract.
- VNC/noVNC is larger and less safe. It adds RFB input, a WebSocket proxy, authentication and lifecycle state, and either exposes the user's display or an entire isolated GUI. It cannot enforce “orbit/zoom only”.
- Browser-to-X11 event injection is display-global/focus-sensitive and structurally incompatible with native Wayland security. Pixel coordinate mapping cannot prove that a menu or dialog was not targeted.
- A first-release Three.js/URDF-loader stack would be easier to prototype but would add a module/bundling and provenance project. The required initial renderer is small enough to implement with browser `DOMParser`, a strict binary-STL parser, fixed shaders, and compact matrix/camera code. Revisit a library only if this authored renderer becomes materially more complex than this boundary.

## Phase 0: close the current HTTP boundary first

Do this before adding any expensive static asset or faster state route.

1. Add a common safe-request policy, applied before route dispatch or any node/capture/asset work:
   - exactly one `Host`, byte-equal to `127.0.0.1:<selected-port>`;
   - if `Origin` exists, exactly one and byte-equal to `http://127.0.0.1:<port>`;
   - `Sec-Fetch-Site` may be absent for the launcher's local `curl`, or exactly `none`/`same-origin`; reject `cross-site`, `same-site`, duplicates, and unknown values;
   - no CORS response headers;
   - preserve the stricter exact Origin + CSRF + JSON mutation policy for every POST.
2. Match raw targets exactly. In particular, `/api/rviz.jpg-anything` and `/api/rviz.jpg?x` must no longer match `/api/rviz.jpg`. The new API needs no query string.
3. Reject missing/duplicate Host, conflicting framing, oversized headers/bodies, and malformed methods before work. Add `nosniff`, `Referrer-Policy: no-referrer`, `frame-ancestors 'none'`, and `Cache-Control: no-store` consistently.
4. Replace `std::random_device` token construction with an explicit Linux `getrandom()` loop that fails closed. Keep the CSRF value out of logs.

Exact Host validation fixes DNS rebinding: a rebound attacker origin still sends its attacker hostname in `Host`. Fetch Metadata prevents a normal cross-site Firefox page from blindly forcing even an unreadable image/state request directly to `127.0.0.1`. Allowing an absent Fetch Metadata header preserves the local health probe; a local process is already inside the loopback/user boundary. If support expands beyond the declared Firefox target, add a per-run capability bootstrap rather than weakening these checks.

## Phase 1: implement the read-only measured-pose viewer

### Files and build/package changes

- `include/openarm_ik_ros/portal_core.hpp`, `src/portal_core.cpp`
  - add `SafeRequestHeaders`/`SafeRequestPolicy` and pure policy tests;
  - add a `ViewerSnapshot` serializer using `max_digits10` for every joint;
  - keep all viewer types read-only and separate from `GuardInput` and mutation parsing.
- `src/openarm_portal.cpp`
  - add `PortalNode::viewer_snapshot()` under the existing short state mutex;
  - add exact read routes listed below;
  - resolve only the installed Stage-A URDF and allowlisted mesh paths at startup;
  - use bounded file responses rather than copying assets into unbounded vectors;
  - accept only `--port`; remove RViz PID/executable/capture construction.
- `src/portal_page.cpp`
  - change the heading and safety caption;
  - replace the image with a canvas, stale/error overlay, reset-view button, and bounded metrics/status text;
  - retain all control wording and POST behavior.
- `web/portal.css`, `web/portal.js`, `web/viewer.js`
  - external same-origin assets; no `eval`, dynamic code, external URL, or inline event handler;
  - `viewer.js` owns strict URDF/STL parsing, matrix propagation, WebGL resources, local orbit/zoom, frame pacing, and a fixed-size metrics ring;
  - `portal.js` owns the existing form/state behavior and the sequential viewer-state poll.
- `cmake/ViewerAssetManifest.cmake`
  - declare each package-relative URDF/mesh path, byte size, SHA-256, triangle cap, and served route;
  - verify the installed/source artifact at configure/build time and generate an installed `viewer/manifest.json`;
  - reject an unexpected Stage-A URDF or `openarm_description` mesh rather than silently serving it.
- `CMakeLists.txt`, `package.xml`
  - install the authored web files and generated manifest;
  - add `ament_index_cpp` for installed share resolution if resolution is kept inside the C++ server;
  - remove JPEG and XComposite from `openarm_portal`; retain X11 and RViz dependencies needed by the standalone RViz launcher/close helper.
- `scripts/launch_web_portal.sh`
  - retain the GUI lock, build/install stamp, loopback-port check, controller lifecycle, Firefox opening, and clean shutdown;
  - remove renderer environment selection, RViz discovery/start/PID proof, and close-helper use from this launcher;
  - health now means server plus pinned viewer assets ready, not “RViz window found”.
- `scripts/lib/launch_integrity.sh`, `tests/test_launch_integrity.sh`, README
  - add installed viewer assets/manifest to required artifacts without narrowing the existing whole-install digest;
  - describe the proxy accurately and keep standalone RViz documented.
- `src/rviz_capture.cpp` and its header
  - leave unused during a short migration if helpful, then delete after the WebGL acceptance gate. Never retain it as a silent fallback that still starts RViz and consumes memory.

The upstream meshes are not an untracked web dependency. They come from the already-audited Apache-2.0 `openarm_description` commit `6c7b720f1ba48e8bafa3a3dc752c45f397b42221`, are already part of the build/install manifest, and have a repository license. The generated viewer manifest must retain per-file hashes/sizes and the upstream notice. Do not copy the 69 MiB DAE set into the portal package. There is no third-party JavaScript to vendor in this phase. If a library or converted GLB is introduced later, require exact upstream commit/version, source URL, SHA-256, license/NOTICE, reproducible offline conversion, installed manifest entry, and no runtime download.

### Exact HTTP API

All routes pass Phase 0 policy before dispatch. Static route names are exact and are the only names mapped to filesystem paths.

- `GET /`
- `GET /web/portal.css`
- `GET /web/portal.js`
- `GET /web/viewer.js`
- `GET /viewer/manifest.json`
- `GET /viewer/stage_a.urdf`
- eleven exact `GET /viewer/mesh/<fixed-name>.stl` routes
- `GET /api/health`
- `GET /api/state` (existing controls/status contract)
- `GET /api/view-state` (new, cheap pose contract)
- existing exact POST routes, unchanged

`/api/view-state` should have this shape (integer timestamps/sequences are decimal strings to avoid JavaScript integer loss):

```json
{
  "schema": 1,
  "have_state": true,
  "fresh": true,
  "sequence": "42",
  "producer_time_ns": "1234567890",
  "receipt_age_ms": 1.25,
  "joint_order": ["openarm_left_joint1", "...14 fixed names..."],
  "position_rad": [0.0, "...14 finite binary64 values..."]
}
```

For no state, omit positions and report `have_state:false`. For stale state, the last finite pose may remain present with `fresh:false`; it is rendered dimmed under a prominent `VIEW STALE` overlay. The browser computes continuing age as server `receipt_age_ms` plus its monotonic elapsed time since response receipt. It does not compare ROS/simulated producer time to the browser wall clock.

The browser uses one sequential fetch at a time on absolute 33.333 ms deadlines. It skips missed deadlines and never starts a catch-up burst. There is consequently no history queue or old response racing a new response. A sequence must be strictly greater than the last applied sequence; duplicate/rollback responses are ignored and surfaced in metrics. If TCP-per-poll overhead alone fails the acceptance gate, Phase 1b may replace only this route with one bounded same-origin Beast WebSocket: one client, maximum payload, one pending latest snapshot, no history. Do not start with that additional server state.

Rendering runs on every foreground `requestAnimationFrame`, normally 60 Hz, and applies the newest accepted pose. Orbit drag, wheel zoom, touch pinch, reset, resize, and redraw are local operations and make **no network request at all**. Clamp pitch, distance, device-pixel ratio, and backing-buffer pixels (initial maximum 1920x1080). Use pointer capture and no generic key forwarding or pan-to-arbitrary-target operation.

The parser accepts the one pinned Stage-A document only: no doctype, unknown element needed for kinematics, nonfinite number, unsupported joint type, unknown package URI, unexpected link/joint/count, or mesh outside the manifest. It selects `<collision>` geometry, reuses one GPU buffer per unique mesh, and caps each mesh and the aggregate before allocating. Negative mesh scales must be tested; disabling culling for this small two-sided proxy is safer than getting mirrored winding subtly wrong.

### Safety separation

`viewer_snapshot()` copies measured state; it never updates state, diagnostics, freshness evidence, guard generations, action state, or motion eligibility. Viewer freshness is informational only. The existing server revalidation at guard handoff remains authoritative. Canvas events cannot name or invoke a POST route. Viewer failure, WebGL context loss, asset failure, or a stale tab may disable/dim the viewer but must not relax or alter motion policy. Preserve the existing software-stop disclaimer; the viewer is not safety-rated feedback.

## Test and release design

### Deterministic unit/contract tests

- Extend `test/test_portal_core.cpp` with a table for missing/duplicate/wrong Host, DNS-rebinding Host, missing/duplicate/wrong Origin, `Origin:null`, every Fetch Metadata value, exact target matching, CSRF separation, and max-digits joint serialization.
- Add a C++ HTTP black-box test around a factored server/backend interface. Send raw requests for duplicate Host, path suffix/query confusion, conflicting length/chunking, oversized/slow body, wrong methods, and cross-site headers. Assert rejection occurs before a counted state/asset callback. Assert no response opts into CORS.
- Extend `test_visualization_urdf.py` or add a viewer-manifest test proving 26 links, 25 joints, 14 dynamic arm joints, four fixed Stage-A fingers, exactly 11 unique allowlisted collision meshes, 2,498,724 bytes, and 49,956 unique triangles. Verify every hash and license/provenance entry.
- Add a Firefox headless browser contract test. A test-only Python standard-library harness is acceptable because this repository already has Python tests and it adds no production runtime; if “no Python” includes tests, write the orchestration in C++ instead. Do not add Node/npm. Test strict parser failures, 23 instances, asymmetric joint matrices, pointer/wheel/pinch camera changes, resize caps, sequence skip/rollback, one in-flight fetch, stale overlay, context loss, and zero POST/camera network messages.
- Compare representative and randomized browser link transforms against the Stage-A URDF/robot-state-publisher TF oracle, including both mirrored arms and fingers, to a declared tolerance. A screenshot alone is not a kinematic oracle.

### Honest 30 FPS/source-to-paint observability

Do not claim 30 FPS from `requestAnimationFrame` counts alone. Browser script cannot prove physical scanout.

Keep a fixed rolling metrics buffer (for example 512 records) containing server sequence, server receipt age at serialization, client receive time, pose-apply time, rAF draw-submission time, WebGL fence completion, visibility state, and context-loss state. Emit `performance.mark` entries keyed by sequence. This gives a clock-safe source-age-to-draw upper bound: server receipt age plus client receive-to-draw elapsed time.

For the release gate, capture a Firefox Profiler/WebRender compositor trace on the declared Wayland laptop and correlate each changing-pose draw mark with the first subsequent canvas composite. That external compositor event is the source-to-paint observation; an optional Firefox paint event may aid diagnostics but is not the sole proof.

After warm-up, run 60 seconds at 1280x720 in a visible foreground tab with continuously changing asymmetric pose and continuous orbit input. Require:

- at least 1,740 distinct painted canvas frames (29.0 average FPS);
- p99 painted-frame gap at most 67 ms;
- p99 accepted-pose source-receipt-to-composite age at most 100 ms;
- monotonically increasing applied sequences, no catch-up burst, and convergence to the latest available sequence within 100 ms after a forced client stall;
- bounded portal/browser/GPU memory and no degradation in stop/control response latency under the single permitted viewer poll.

Repeat with wheel/pinch, resize, software WebGL, failed asset, server pause, 5 FPS throttling, hidden/restored tab, and WebGL context loss. A hidden/throttled tab must say it is throttled/stale; it is excluded from the foreground 30 FPS claim. Record portal CPU/RSS, Firefox content/GPU RSS, bytes, triangle/buffer counts, and peak canvas size. No high-detail mode ships by default until it passes the same memory gate.

### Adversarial/failure tests

- From an attacker origin, try DNS rebinding, `img`, `fetch`, form POST, and framing. Capture/state/asset callback counts must remain zero for rejected traffic.
- Flood malformed and nonreading connections while the legitimate viewer runs. Memory, FDs, thread count, and any pending-view counter remain bounded. If the existing shared four-worker design cannot preserve the declared stop/control latency, reserve executor capacity or move the read-only viewer work off the mutation path before release.
- Verify `ss` shows only the portal loopback listener: no RFB, no extra WebSocket bridge, no `0.0.0.0`, and no IPv6 wildcard.
- Run offline and assert the page makes no non-loopback request. CSP should be `default-src 'none'; script-src 'self'; style-src 'self'; img-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'none'` (add only the narrowly required canvas/font directive, if Firefox proves one necessary).

## Failure modes that must remain explicit

- **Proxy mistaken for safety geometry:** the strongest product risk. Keep “visual proxy — not collision checking” adjacent to the canvas and retain `collision_checked=false` everywhere.
- **Kinematic drift:** a hand-written loader can reverse RPY order, axis order, or mirrored scale. Golden matrix/TF tests are mandatory; visual inspection is insufficient.
- **Memory regression:** malformed STL counts, high DPR, resize churn, retained download buffers, or context restoration can allocate repeatedly. Validate byte/triangle counts before allocation, cap canvas pixels, reuse buffers/matrices, release source ArrayBuffers, and fail closed on repeated context loss.
- **Polling harms controls:** one sequential cheap poll is expected, but a hostile client may still pressure shared request workers. Gate release on mutation/stop latency under load and separate capacity if the test fails.
- **Stale pose looks live:** retain and dim the last pose only under an unavoidable stale overlay; age continues locally during network failure. Never use viewer health to authorize motion.
- **Custom renderer grows into RViz:** do not add ROS topic browsing, plugin loading, generic scene files, arbitrary URLs, GUI input, or a server camera API. Those invalidate the small security boundary.
- **“30 FPS” measures the wrong stage:** distinguish source sample, server serialization, client receive, pose apply, GPU completion, compositor paint, and display scanout. Release wording should say “at least 29 painted frames/s on the declared Firefox/Wayland acceptance host”; it must not imply all machines or physical scanout instrumentation.

## Final release recommendation

Implement Phase 0 and the collision-proxy WebGL viewer as one gated change, then remove RViz/XComposite/JPEG from the web launcher. Keep standalone RViz for exact engineering visualization. Do not spend this release building the Ogre bridge unless the user changes the requirement from “robot-only embedded view” to “exact RViz rendering”.
