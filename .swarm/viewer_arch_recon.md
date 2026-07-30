# Portal viewer architecture reconnaissance (2026-07-30)

## Recommendation

Use a **browser-native, same-origin WebGL robot viewer** in the portal and remove RViz/XComposite/JPEG from the portal runtime. Keep `scripts/launch_rviz.sh` as the separate engineering/debug viewer. This is the only option that cleanly gives a robot-only canvas, mouse/touch orbit and zoom, sustained 30 FPS, loopback-only exposure, low host memory/CPU, and no paid service or Python/Java runtime.

Use vendored, version-pinned Three.js + `OrbitControls` + `urdf-loader` assets, served by the existing C++ Boost.Beast server; do not use a CDN or a ROS-to-WebSocket bridge. JavaScript is unavoidable in a browser, but this adds neither Java nor a server-side JS/Python runtime. Three.js documents orbit/dolly/pan controls ([OrbitControls](https://threejs.org/docs/pages/OrbitControls.html)); the Apache-2.0 URDF loader supports ROS package maps, collision-versus-visual selection, and setting named joint values ([loader README](https://raw.githubusercontent.com/gkjohnson/urdf-loaders/master/javascript/README.md), [loader options/source](https://raw.githubusercontent.com/gkjohnson/urdf-loaders/master/javascript/src/URDFLoader.js)).

For a low-memory first release, load the derived Stage-A visualization URDF hierarchy but set `parseVisual=false`, `parseCollision=true`, recolor the meshes, and label them **visual proxy only—not collision checking**. The 11 unique referenced collision STLs are about 2.4 MiB here, versus about 69.2 MiB of unique source DAE visuals. Add the high-detail visual meshes later as an explicit quality option or an optimized/instanced GLB; Three.js supports glTF mesh compression extensions ([GLTFLoader](https://threejs.org/docs/pages/GLTFLoader.html)). This preserves pose fidelity while avoiding the current large DAE working set. It must not change the existing `collision_checked=false` truth text.

### 30 FPS/state design

- Extend the authoritative C++ snapshot with the exact 14 measured joint radians, fixed joint-name order, `state_sequence`, producer timestamp, receipt age, and freshness. The portal already validates all 14 names/finiteness and stores `measured_q_` from `/joint_states`; expose that same snapshot rather than rebuilding state from TCPs or TF.
- Add one same-origin, loopback WebSocket (on the existing HTTP port) for view state. On every 33.333 ms tick, atomically take the newest snapshot and send it only if its sequence advanced. Maintain at most one pending message per client: overwrite/drop an unsent older pose. Never queue history and never extrapolate. A 14-double JSON message is small; `max_digits10` retains binary64 round-trip behavior already used by the portal.
- Pace WebGL draws from `requestAnimationFrame` with a 33.333 ms accumulator. If a frame is late, render the newest pose once and reset the deadline to `now`; do not issue catch-up frames. Camera interaction uses the same capped loop. Rendering may stay at 30 FPS while a stationary pose has no new messages.
- If state exceeds the existing 500 ms trust window, freeze the last clearly marked pose and show a stale overlay; controls remain visually usable, but the picture must never be presented as current feedback. Camera state is display-only and must remain outside motion/CSRF paths.
- The producer/session polls at 5 ms, so latest-wins 30 Hz delivery does not invent extra robot state. It deliberately decimates authoritative samples while retaining their sequence and timestamps.

At 1920x1080 the current capture allocates roughly 5.9 MiB just for its RGB vector per frame, additionally owns an `XImage`/pixmap/JPEG buffer, calls `XGetPixel` for every pixel, converts every channel, and JPEG-compresses under a single capture mutex. Thirty FPS would process about 62 million pixels/s before encoding and is not a credible low-CPU extension of this implementation. WebGL moves rasterization to the browser/GPU; host work becomes a tiny latest-pose message. Browser/GPU memory still needs measurement, especially before enabling the 69.2 MiB DAE visual set.

## Current evidence

- `src/portal_page.cpp:25,73`: a top-level `<img>` polls `/api/rviz.jpg` every 350 ms, so the requested rate is only 2.86 FPS (not 30); `/api/state` polls every 250 ms. There is no input handler for the image.
- `src/rviz_capture.cpp`: identifies exactly one mapped top-level RViz window by launcher PID, redirects it with XComposite, reads the entire window with `XGetImage`, loops through pixels with `XGetPixel`, builds RGB, and JPEG-encodes quality 75. It captures menus/panels because it captures the top-level window, not RViz's central render widget.
- `src/openarm_portal.cpp:435-560`: the C++ server binds only `127.0.0.1`, serializes capture behind `capture_mutex_`, and serves one newly encoded JPEG per request. Its existing CSP and exact-origin/CSRF mutation policy are good foundations.
- `src/openarm_portal.cpp:344-378`: the portal already accepts only fresh, finite, fully named 14-joint messages and retains the newest measured vectors and sequence. `/api/state` currently emits only two TCP positions, so a browser robot cannot yet reproduce the measured pose.
- `rviz/openarm_ik.rviz:1-40`: RViz is configured for RobotModel, a Displays panel, Orbit view, and internal 30 FPS. The portal loses that interactivity and rate during pixel polling.
- `scripts/launch_web_portal.sh:225-349`: portal startup forces XCB/GLX; on Wayland `auto` selects Mesa software rendering because hardware GLX flickers during resize, then separately launches RViz and passes its PID/executable evidence to the portal.
- The installed derived URDF fixes unmeasured fingers and uses the same arm joints/meshes as RViz. Preserve that exact web input; do not use the canonical dynamic URDF for display.

## Option comparison

| Option | 30 FPS and interaction | Security/dependencies/resources | Verdict |
|---|---|---|---|
| Browser WebGL + direct C++ pose stream | Native orbit/zoom; deterministic latest-wins 30 FPS; exact measured joint samples; no pixel encode | One existing loopback listener; vendored JS only; removes RViz/Ogre/XComposite/JPEG from portal; client GPU memory must be profiled | **Best fit** |
| Stock RViz `--fullscreen` on X display + TigerVNC/noVNC | Exact RViz pixels and pointer input; TigerVNC `FrameRate=30` aggregates faster changes and uses XDamage/SHM; noVNC supports scaling/clipping, touch, Tight/JPEG/H.264 | Still runs RViz + X server + RFB + WebSocket proxy and encodes pixels; more RSS/CPU and attack surface. Existing-display sharing retains the XWayland/hybrid-GPU context; isolated Xvnc avoids desktop exposure but must prove RViz/GL works. noVNC's usual websockify path adds Python unless replaced with a narrow C++ proxy | **Exact-RViz fallback only** |
| Remote X input added to current JPEG capture | Could crop/fullscreen and forward drags/wheel | Current 2.86 FPS path must be rewritten with XDamage/XShm and streaming codec to reach 30. XTest-style input is display-global/focus-sensitive; coordinate mapping and dropped drag events are brittle | Reject |
| Foxglove Bridge + Foxglove 3D | C++ high-performance bridge; capable 3D/debug UI | Exposes ROS topics over another WebSocket and adds a large general UI rather than robot-only portal. Current docs say the bridge automatically makes all topics available; embedded Foxglove is a Pro/Enterprise/Academic feature, conflicting with no-paid-service preference ([ROS 2 docs](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2), [embedding](https://docs.foxglove.dev/docs/embed)) | Use only as optional developer tooling |
| rosbridge + ROS3D.js | Browser 3D and input possible | rosbridge server is Python and gives clients publish/subscribe/service capability; ROS3D.js documents an old Three.js r89 dependency and dead loader CDN links. Extra listener/ACL work is unnecessary ([rosbridge](https://github.com/RobotWebTools/rosbridge_suite), [ROS3D.js](https://github.com/RobotWebTools/ros3djs)) | Reject |

If exact RViz pixels are a hard requirement, prefer open-source TigerVNC + noVNC over RealVNC. Launch RViz at fixed 1280x720 with `--fullscreen`, share only that geometry, bind both RFB and WebSocket proxy to `127.0.0.1` (or a mode-0600 Unix RFB socket), use a per-run credential, and proxy noVNC through the portal's same origin. TigerVNC documents pointer acceptance, geometry cropping, XDamage, SHM, `FrameRate`, `-localhost`, and authentication ([x0vncserver manual](https://tigervnc.org/doc/x0vncserver.html)); noVNC documents modern-browser input, clipping/scaling, codecs, WebSocket requirement, MPL-2.0 license, and loopback binding ([noVNC](https://github.com/novnc/noVNC)). Use damage-driven latest framebuffer updates capped at 30 and discard superseded rectangles; never queue old frames. This is more realistic than making the current JPEG endpoint 30 FPS, but materially heavier than WebGL.

## Dependency/file map and implementation boundary

Current portal-specific runtime:

- `src/portal_page.cpp` — inline HTML/CSS/JS and JPEG polling.
- `src/openarm_portal.cpp` — ROS state/action client, loopback HTTP, API/security, capture endpoint.
- `src/rviz_capture.cpp`, `include/.../rviz_capture.hpp` — X11/XComposite/JPEG capture and process/window identity.
- `scripts/launch_web_portal.sh` — renderer selection, RViz lifecycle/identity, portal health and browser.
- `rviz/openarm_ik.rviz`, `launch/openarm_ik_rviz.launch.py` — RobotModel/Orbit config and RSP/controller/optional RViz.
- `CMakeLists.txt`, `package.xml`, `scripts/install_all_dependencies.sh`, `scripts/install_ros_dependencies.sh` — `rviz2`, X11/XComposite, JPEG and related dependencies. Do not remove dependencies needed by the standalone RViz launcher/close helper when separating portal dependencies.

Expected WebGL change surface:

- Add installed, pinned `web/` assets: Three.js, OrbitControls, URDF loader, license notices, and an allowlisted Stage-A URDF/mesh subset (or verified GLB).
- Replace `<img>` polling with `<canvas>` and a measured-state WebSocket in `portal_page.cpp`; keep command controls and safety text unchanged.
- Extend immutable state serialization/snapshot access and add same-origin WebSocket/static allowlist routes in `openarm_portal.cpp`/portal core.
- Remove RViz PID arguments, capture health, and RViz startup from the **web** launcher; retain standalone RViz behavior and its GUI lock as appropriate.
- Stop linking `openarm_portal` itself to XComposite/JPEG once capture is gone; retain X11 for `close_rviz_window` if standalone RViz still uses it.

Existing tests that constrain this work:

- `test/test_portal_core.cpp` asserts literal page contracts, binary64 JSON, process identity, XComposite helpers, security, units, and safety wording. Replace capture-only assertions; add exact joint ordering/sequence/staleness cases.
- `test/test_visualization_urdf.py` and `test/test_generated_urdf.py` protect the derived/canonical description distinction and mesh resolution.
- `test/test_ros_contract.py` already exercises the headless `rviz:=false` state/TF contract.
- `tests/test_launch_integrity.sh`, `tests/test_build_resource_controls.sh`, and `scripts/lib/launch_integrity.sh` explicitly inventory the portal, close helper, RViz config, and launch tree; update their expected installed web assets without weakening freshness/prefix checks.
- There is no current integration test for actual captured pixels, browser interaction, or frame rate.

## Task-specific verification before acceptance

1. **Pose contract:** C++ unit tests require 14 finite joint radians in canonical names/order, max-digits round trip, monotonic sequence, correct stale flag, and no fresh pose emitted after expiry. Compare browser-applied joint transforms for representative/asymmetric vectors against the derived URDF/robot-state-publisher TF to a documented tolerance; fixed fingers remain at the Stage-A convention.
2. **30 FPS:** foreground Firefox at fixed 1280x720 for 60 s after warm-up. Instrument actual WebGL render submissions and a browser performance trace: at least 1,740 frames (29.0 average FPS), p95 inter-frame interval below 50 ms, no catch-up burst, and no queued pose older than the newest available sequence. Repeat while orbit-dragging, wheel-zooming, and during a virtual motion. Hidden/throttled tabs are excluded but must report throttling rather than claiming 30 FPS.
3. **Fidelity/drop policy:** record server sequence/timestamp and client-applied sequence. Client sequence must be monotonic, may skip under load, must never regress, and must converge to the newest server sequence within 100 ms on loopback. On a forced client stall, exactly the newest pending pose is applied after recovery.
4. **Interaction/UI:** automated pointer/touch/wheel events change camera matrices but never joint state or command endpoints. Screenshot/DOM assertions show only the canvas robot—no RViz menus, panels, toolbar, or remote desktop. Resize does not touch host GL and does not flicker.
5. **Resources:** compare 60 s idle/motion/orbit runs against the current portal using `/usr/bin/time -v`, per-process RSS, CPU time, bytes sent, and browser/GPU diagnostics. Test both the 2.4 MiB collision proxy and any high-detail option; do not enable high detail by default until its peak browser/GPU memory is acceptable.
6. **Security/failure:** `ss` shows only the chosen portal loopback port; no `0.0.0.0`, RFB, bridge, CDN, or cloud connection. Test path traversal and unknown asset rejection, CSP/self-origin behavior, stale/disconnected state overlay, WebGL-unavailable message, clean shutdown, and unchanged CSRF/mutation policy.

