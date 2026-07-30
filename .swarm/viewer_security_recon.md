# Interactive RViz viewer security/design adversary report

## Verdict

Do **not** add VNC/noVNC and do **not** translate browser pointer/keyboard events into X11 events. Both turn a camera viewer into remote GUI control. For an actual embedded RViz image at 30 FPS, the smallest defensible design is a narrow, compiled RViz render-target bridge: capture only the Ogre render panel into a bounded latest-frame ring, encode each source frame once, and expose only typed orbit/zoom/reset commands. Keep robot motion on the existing independently guarded API.

The current snapshot implementation is a useful read-only diagnostic prototype, but is neither a 30 FPS transport nor a safe base for input injection.

## Evidence from the current tree

Strengths worth retaining:

- The listener binds explicitly to `127.0.0.1` (`openarm_portal.cpp:443-450`), mutation requests require exact Host, Origin, CSRF token and JSON type (`portal_core.cpp:304-326`), and request header/body/time/concurrency limits exist (`openarm_portal.cpp:470-505`).
- Motion freshness is independently revalidated at guard handoff; image pixels are not control feedback.
- RViz capture rechecks PID start time, `/proc/PID/exe`, exactly one mapped top-level PID window, dimensions, TrueColor masks, and rejects uniform-black frames (`rviz_capture.cpp:117-299`). These checks reduce accidental window confusion.

Blocking findings:

1. **GET trust boundary is open.** `/`, `/api/health`, `/api/state`, and the expensive image capture run before any Host/Origin/session validation (`openarm_portal.cpp:511-560`). A DNS-rebinding page can read GET state/root content using its attacker Host, although the exact mutation Host/Origin check still prevents it from issuing motion. Any cross-site page can blindly create images against the predictable loopback URL and force capture work. The image route is also a prefix match, so `/api/rviz.jpg-anything` captures.
2. **30 FPS is impossible on the current path.** The page requests one image every 350 ms, at most 2.86 requests/s (`portal_page.cpp:73`). Each request walks the X tree, synchronously copies the whole top-level window with `XGetImage`, calls `XGetPixel` once per pixel, builds an RGB copy, and performs a fresh JPEG encode (`rviz_capture.cpp:214-335`). At the accepted 4096x4096 maximum, RGB alone is 48 MiB and a typical 32-bit XImage is another 64 MiB; 30 FPS would move roughly 3.4 GiB/s before JPEG output. Even 1920x1080 implies about 249 MB/s of 32-bit readback plus 187 MB/s of RGB writes at 30 FPS. The four HTTP workers can all block behind the single capture mutex, starving state/mutation handling (`openarm_portal.cpp:476, 548, 636-640`).
3. **No backpressure or freshness contract.** `setInterval` starts state requests without waiting for the previous request and rewrites `img.src` without waiting for load/decode. Old state responses may arrive after newer ones. On image failure the browser retains the last good pixels indefinitely; there is no source sequence, capture time, decode time, stale overlay, or frozen-frame detection. The black-frame check cannot detect a frozen or partially updated GL frame.
4. **The captured surface is the wrong interaction boundary.** It is the top-level Qt window, so RViz menus, docks, dialogs, and chrome are pixels in the stream. Mapping browser coordinates through CSS `object-fit: contain`, letterboxing, HiDPI, window resize, and XWayland scaling is inherently racy. Raw input could hit a menu or dialog instead of the Ogre panel. `_NET_WM_PID` is a client-owned X property, not authorization against another client already admitted to the same X display.
5. **Platform/fidelity limits are structural.** The launcher forces Qt `xcb`; on Wayland it is actually XWayland and defaults to software GL (`launch_web_portal.sh:229-248`). Native Wayland and headless sessions are unsupported by XComposite. GLX/XComposite named-pixmap behavior varies by compositor/driver and may be black, stale, or torn. Software GL plus readback plus JPEG competes directly with RViz's configured 30 FPS rendering.
6. **Current tests stop below the boundary.** `test_portal_core.cpp` tests policy values, process identity, version predicates, masks, and black-frame helpers, but does not run the HTTP server, a browser, XComposite/GL capture, overload, stale-frame behavior, or renderer fidelity. Page tests largely assert source substrings.

## Why VNC/noVNC and X event injection fail

- VNC exposes a framebuffer **and an input protocol**. Against the user's display it can control more than RViz; against an isolated display it can still operate RViz menus, file dialogs, display/plugin configuration, clipboard, and any future privileged UI. noVNC also adds RFB, WebSocket proxying, authentication/origin handling, a substantial JavaScript dependency tree, and another server lifecycle.
- XTest/`xdotool` moves the session's real pointer and synthesizes trusted-looking global input. `XSendEvent` is inconsistently accepted by Qt/GL. Both race window geometry, focus, popups, compositor scaling, and real user input. Wayland intentionally does not provide equivalent arbitrary injection; forcing XWayland merely preserves X11's weak same-display isolation.
- Neither design can prove “orbit/zoom only.” Browser wheel/trackpad/pinch/button semantics differ, and pixel-coordinate tests cannot prove that a menu never overlays the render region.

## Smallest production-safe architecture for 30 FPS

Use a small RViz plugin/compiled companion with two deliberately narrow interfaces:

1. **Pixels:** capture the Ogre render target, not the Qt top-level window. Use a 2- or 3-slot PBO/shared-memory ring at one fixed supported size (recommend declaring 1280x720 first). Producer and consumer exchange monotonically increasing `frame_sequence` and monotonic capture time. The producer never blocks RViz: if the next slot is busy, overwrite/drop the oldest unpublished frame.
2. **Camera:** accept only a strict finite schema such as bounded `orbit_delta_yaw`, `orbit_delta_pitch`, `zoom_log_delta`, and `reset`, coalesced to one pending update. Apply these through RViz's Orbit view-controller API on its GUI thread. There is no generic key, button, absolute screen coordinate, focus, menu, clipboard, file, ROS-topic, or plugin-loading operation.
3. **Transport:** one authenticated loopback viewer, one encoded latest-frame slot, and one in-flight frame per browser. A pull/long-poll endpoint `frame?after=N` is simpler than RFB: return the newest JPEG, wait at most one frame period when unchanged, and let the browser request the next only after decode/paint. Encode a source frame once, not once per request. A slow client skips directly to newest. Reserve a separate executor/queue for control and stop requests.
4. **Resource caps:** exact route matching; maximum 1280x720 pixels initially; fixed ring size; maximum encoded frame size; one stream; bounded header/body/time; camera token bucket (for example 60 updates/s, burst 2); queue depth one; no unbounded per-client buffers. At 1280x720, a three-slot RGBA ring is about 10.5 MiB and raw 30 FPS readback is about 111 MB/s. JPEG quality/bandwidth must be measured on the minimum supported machine. If JPEG cannot meet the acceptance budget, choose one distro-provided video codec stack deliberately—do not silently add VNC.
5. **Browser/session:** generate the capability with OS `getrandom`, preferably choose an ephemeral port, and put all routes under an unguessable session path. Validate exact Host on **every** request, exact Origin plus capability on every mutation/camera request, emit no CORS allowance, and reject cross-site Fetch Metadata before expensive work. Use external same-origin JS/CSS with CSP `default-src 'none'; script-src 'self'; style-src 'self'; img-src 'self' blob:; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'none'`. Camera interaction must never alter motion eligibility.
6. **Freshness:** show the frame sequence and age. After two frame periods without a newer frame, overlay `VIEW STALE`; after the state freshness deadline, separately overlay `ROBOT STATE STALE`. A stale frame may remain visible for orientation but must be visibly dimmed and may never be described as live. Ignore sequence rollback and late responses.

This preserves actual RViz rendering and Orbit semantics while removing XComposite, window chrome, coordinate injection, and per-request capture. If a compiled RViz bridge is out of scope, the safe smaller product is the existing read-only snapshot plus direct interaction in the local RViz window; there is no small safe shortcut to browser-controlled RViz. A browser-native WebGL robot viewer is safer still, but it is not accurately called an embedded RViz view and requires audited mesh conversion/rendering work.

## Exact trust-boundary/acceptance tests

### HTTP/browser adversary

- Raw-socket matrix for every route: missing/duplicate/wrong Host, attacker Host (DNS-rebinding simulation), wrong Origin, `Origin: null`, wrong/missing capability/CSRF, cross-site `Sec-Fetch-Site`, OPTIONS/preflight, alternate 127/localhost/IPv6 spelling, path suffix/query confusion, chunked body, conflicting Content-Length, oversize header/body, slowloris, and disconnect during write. All fail before capture/camera work; no response contains CORS opt-in.
- Headless Firefox and Chromium test from an attacker origin: attempts to read state/root, force frame production with `img`/`fetch`, and submit camera/motion. Assert zero capture count and zero mutations. Run runtime with outbound network denied and assert the page makes no non-loopback request.
- CSP/header test on all response classes; no inline executable content, framing, MIME sniffing, referrer leakage, or cache reuse of state/frames.
- Flood 100 nonreading/reconnecting clients plus malformed camera requests for 10 minutes. Assert fixed thread count, bounded RSS/FDs/ring occupancy, no frame queue growth, and reserved motion/stop endpoint latency remains within its declared bound.

### 30 FPS/backpressure

- Declare resolution, minimum CPU/GPU/software-renderer host, browser, and quality before claiming 30 FPS. Over 60 seconds require at least 29.0 decoded-and-painted new frames/s after warm-up, p99 source-to-paint latency no more than 100 ms, p99 inter-painted-frame gap no more than 67 ms, and no frame older than two source periods delivered while a newer frame exists. Record source, encoded, received, decoded, and painted sequence/timestamps.
- Repeat while orbiting continuously, robot state changes, tab is backgrounded/restored, browser is throttled to 5 FPS, socket reads stop for 30 seconds, capture is slower than 33.3 ms, encoder returns oversize/error, and client reconnects. The producer must keep bounded memory and drop stale frames; reconnect's first frame must be the newest, never the backlog.
- Measure encode time, readback time, JPEG size distribution, portal/bridge RSS, CPU, and RViz render FPS on the minimum host. Fail if capture lowers RViz below 30 FPS or if control latency crosses its bound. Include 1280x720 and reject—not silently scale—larger requests.

### Camera and visual fidelity

- Property/fuzz test pointer/wheel conversion with NaN/infinity/extremes, pointer loss, multi-touch, HiDPI and resize. Network output can contain only the typed bounded camera schema at the capped rate.
- Golden Orbit tests: initial RViz config yaw/pitch/distance/focal point; drag directions; pitch clamp; logarithmic wheel zoom; reset; resize/letterbox invariance. Read back controller properties, not window pixels.
- Pixel/golden-scene tests capture only the render target: no menu, panel, tooltip, dialog, cursor, or desktop pixels are possible. Compare known poses from the pinned visualization URDF at multiple cameras, including mirrored mesh scales and transparency/background.
- State tests inject duplicate, rollback, delayed and missing sequences. Assert late data cannot replace newer data, stale overlays appear on deadline, and viewer freshness never enables motion; server-side freshness/guard handoff remains authoritative.

### Platform/provenance

- Exercise supported native X11 and Wayland/XWayland/software-GL combinations; fail launch with an explicit unsupported-platform message. The render-target path must not link XTest, VNC, noVNC, websockify, or depend on XComposite window discovery.
- Build and run offline. Every third-party JS/codec artifact needs exact version/commit, SHA-256, source URL, license/NOTICE, installed-file manifest, and inclusion in the existing launch-integrity hash closure/SBOM. No CDN, runtime download, npm install, floating branch, or unaudited generated bundle.

## Release gate

Do not market the feature as interactive or 30 FPS until the source-to-paint test passes on a declared minimum machine under continuous orbit and hostile slow-client load. Do not merge any design that exposes generic GUI input or lets image/camera traffic share an exhaustible queue with motion cancellation.
