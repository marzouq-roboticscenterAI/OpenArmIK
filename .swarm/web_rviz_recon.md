# Live RViz in a local browser portal: read-only reconnaissance

Status: **DONE_WITH_CONCERNS**

No GUI, listener, ROS graph, CAN interface, or hardware process was started during this reconnaissance. The only created artifact is this report.

## Recommendation

Stream the already-running, actual `rviz2` top-level XWayland window from a compiled C++ portal process:

1. The launcher starts the ROS core nodes and `rviz2` exactly as it does now, records the direct `rviz2` PID, and passes that PID to the compiled portal executable.
2. The portal finds the X11 top-level whose `_NET_WM_PID` equals that PID, using the same mechanism as `close_rviz_window.cpp`.
3. It names the compositor-redirected window pixmap with `XCompositeNameWindowPixmap`, captures that pixmap, converts it to RGB, and encodes one shared JPEG frame with libjpeg.
4. A small Boost.Asio/Beast HTTP server exposes the portal assets and a `multipart/x-mixed-replace` MJPEG endpoint. The right pane is an ordinary `<img>` with `object-fit: contain`.
5. It binds **only `127.0.0.1` by default**, handles `SIGINT`/`SIGTERM`, closes listeners and streams, releases X resources, and joins its worker thread before exiting. The launcher then closes RViz through `WM_DELETE_WINDOW` and stops ROS as it already does.

This is the smallest design that literally displays the live pixels of the real RViz application, preserves the known-good XWayland/GLX path on this laptop, needs no Python web runtime, needs no browser decoder, and uses development libraries already installed.

The first implementation should be display-only. Browser-to-RViz input injection is a separate security and correctness feature; it is not necessary to satisfy “display the actual live RViz window.” If interactivity is later required, use a narrow same-origin WebSocket input channel and map coordinates to the selected window, but do not enable general desktop input, clipboard, or keyboard injection by default.

## Why this fits the current host

The current launcher deliberately makes RViz an X11 client even though GNOME is a Wayland session:

- `QT_QPA_PLATFORM=xcb`
- `QT_XCB_GL_INTEGRATION=xcb_glx`
- HiDPI render-target scaling disabled for this process
- Mesa software rasterization selected by default on Wayland to avoid this host's live-resize flicker
- RViz started directly under `setsid`, with its PID retained separately from the ROS launch process
- clean close by a compiled Xlib helper that finds `_NET_WM_PID` and sends `WM_DELETE_WINDOW`

Environment inspection showed `XDG_SESSION_TYPE=wayland`, `DISPLAY=:0`, `WAYLAND_DISPLAY=wayland-0`, and X11 sockets `/tmp/.X11-unix/X0` and `X1`. This is exactly the case where compositor-backed X11 window capture is preferable to Wayland screen capture/portal APIs: the target application is already an XWayland window and is unambiguously identified by PID.

The X.Org Composite API describes `XCompositeNameWindowPixmap` as a reference to a redirected window's off-screen storage. It also states that a new pixmap must be obtained after a map or resize, which must be handled explicitly: [XComposite manual](https://www.x.org/archive/X11R7.5/doc/man/man3/Xcomposite.3.html).

## Minimal compiled design

### Process/lifecycle boundary

Keep Bash as orchestration only. It may set the existing GL/Qt environment, acquire the single-instance lock, and launch compiled executables; it must not serve HTTP, loop over frames, encode images, or act as a watchdog beyond its existing child cleanup.

The launcher should retain three process groups/PIDs: ROS core, RViz, and portal. On any unexpected child exit or `INT`/`TERM`:

1. stop the portal first (`SIGTERM`, bounded wait, then `SIGKILL` only as the last fallback), so no new requests arrive;
2. close RViz with the existing `close_rviz_window RVIZ_PID --timeout 3`, preserving the known workaround for RViz/Ogre teardown;
3. stop the ROS process group with the existing `INT` -> `TERM` -> `KILL` escalation;
4. wait/reap every child and release the flock descriptor.

The portal itself should use `boost::asio::signal_set` (or `sigaction` plus a self-pipe) rather than doing unsafe work in a signal handler. Shutdown must cancel the acceptor, close every MJPEG socket, wake/join the capture thread, call `XDamageDestroy` if used, `XFreePixmap`, destroy any `XImage`/SHM image, and finally `XCloseDisplay`.

### Window selection and state changes

- Accept `--rviz-pid PID`; do not select by title alone.
- Poll `_NET_CLIENT_LIST` for a bounded startup interval and require `_NET_WM_PID == PID`. Optionally check `WM_CLASS`/title only as diagnostics.
- Verify Composite availability/version before naming the pixmap. Do **not** manually unredirect or redirect a GNOME-managed top-level; use the compositor's existing automatic redirection.
- Listen for `ConfigureNotify`, `MapNotify`, `UnmapNotify`, and `DestroyNotify`. Re-read geometry and recreate the named pixmap/image on every map or size/depth change.
- Keep RViz mapped. Occluding it with the browser should work through the redirected pixmap; minimizing typically unmaps it, so serve a clear “RViz window is minimized/unmapped” status rather than stale pixels indefinitely.
- A destroyed window is terminal if the PID has exited; otherwise allow a short reacquisition interval for a remap/recreation.

Because RViz's Ogre render widget may be a native X11 child, the actual-host acceptance test must verify that the named top-level pixmap includes the moving 3D render area as well as Qt panels. If this host exposes the render child separately, walk the target's X11 child tree and capture/composite the native child, or select that child for a scene-only pane. Do not silently substitute a root-window crop, because it would capture whatever overlaps RViz rather than RViz itself.

### Capture and encoding

Smallest reliable v1:

- `XGetImage` from the named pixmap at a capped 10-15 fps;
- generic visual-mask conversion (`red_mask`, `green_mask`, `blue_mask`) rather than assuming BGRA byte order;
- libjpeg's standard C API (`jpeglib.h`, `jpeg_mem_dest`) at quality about 80;
- one encoder/capture loop and one immutable latest-frame buffer shared by all clients;
- no unbounded per-client frame queue: a slow client skips frames and receives the next current frame after its write completes;
- capture only while at least one viewer is connected, with a low-rate status/heartbeat path when idle.

The host also has MIT-SHM headers/runtime. `XShmGetImage` can be added after the basic path passes, guarded by `XShmQueryExtension` with an automatic `XGetImage` fallback. It reduces local X copies but adds shared-memory lifetime and resize complexity. XDamage can similarly coalesce redraws and reduce unchanged captures, but a frame-rate cap is simpler and adequate for the first version. If XDamage is used, retain a heartbeat because an unchanged RViz scene may generate no damage.

PNG is installed but is a poor moving-frame default due to size/CPU. It is useful only for a diagnostic snapshot endpoint.

### HTTP transport and browser pane

Use one compiled HTTP service, preferably Boost.Beast/Asio already on the machine:

- `GET /` and immutable static portal assets;
- `GET /api/health` returning JSON with `rviz_pid`, window state, frame dimensions, last-frame age, viewer count, and capture errors;
- `GET /api/rviz.mjpeg` returning `multipart/x-mixed-replace; boundary=...`, each part having `Content-Type: image/jpeg`, `Content-Length`, and no-cache headers;
- optional `GET /api/rviz.jpg` for a one-frame diagnostic;
- strict request/header/body limits and a small maximum viewer count.

MJPEG is preferable to binary JPEG-over-WebSocket for display: `<img>` decodes it natively, it has no JavaScript blob/object-URL lifecycle, reconnect behavior is simple, and it avoids adding libwebsockets. A WebSocket becomes justified only for bidirectional pointer/tool events or multiplexed telemetry. It is not needed just to show RViz.

Default the parsed listen address to literal `127.0.0.1`, not `0.0.0.0`. A remote bind should require an explicit option and a separate authentication/TLS design. Loopback is not authentication: validate same-origin state-changing requests, reject unexpected `Origin`/`Host` values where applicable, send no permissive CORS headers, and do not expose an event-injection endpoint in display-only mode.

## Installed dependency audit

Read-only inspection on 2026-07-29 found:

| Facility | Installed evidence | Use |
|---|---|---|
| X11 | `libx11-dev` 1.8.13 | window discovery/events/base capture |
| XComposite | `libxcomposite-dev` 0.4.6 | off-screen top-level pixmap |
| XDamage/XFixes | `libxdamage-dev` 1.1.7, `libxfixes-dev` 6.0.0 | optional redraw coalescing |
| MIT-SHM | `libxext-dev` 1.3.4 and `XShm.h` | optional faster capture |
| JPEG | `libjpeg-turbo8-dev` 2.1.5, `jpeglib.h`, `libjpeg.so`, pkg-config `libjpeg` | MJPEG encoding through the standard API |
| PNG | `libpng-dev` 1.6.57 | optional snapshots only |
| HTTP | Boost 1.90 development set, Beast and Asio headers | compiled HTTP/MJPEG server |
| TLS primitives | OpenSSL 3.5.5 development files | not needed for loopback v1 |
| Optional input | `libxtst-dev` 1.2.5 | later, explicitly authorized browser interaction only |
| ROS/RViz | ROS 2 Lyrical; RViz packages 15.2.4; Qt 6.10.2 | existing actual visualization |

`libturbojpeg` is not a separate pkg-config target/header on this Ubuntu build, but `libjpeg-turbo8-dev` supplies the standard accelerated `jpeglib.h`/`libjpeg.so` API. Link `JPEG::JPEG`; do not add a needless TurboJPEG API dependency.

Not installed: `xpra`, Xvfb, x11vnc, TigerVNC server, noVNC, websockify, and libwebsockets. Ubuntu has candidates for most VNC pieces, but none are necessary for the recommendation. `xpra` had no candidate in the configured repositories.

Suggested CMake dependencies are `X11::X11`, `X11::Xcomposite`, optional `X11::Xdamage`/`X11::Xfixes`/`X11::Xext`, `JPEG::JPEG`, `Boost::headers` or the appropriate Beast/Asio target, and `Threads::Threads`. Confirm the exact imported X11 target names with this project's CMake version; `pkg_check_modules` is a reasonable fallback for Composite/Damage.

## Alternatives compared

| Approach | Is it actual RViz? | Advantages | Costs/risks | Verdict |
|---|---|---|---|---|
| X11 window capture + MJPEG | Yes: pixels come from the live `rviz2` top-level window | smallest; matches forced XWayland; all core libraries present; native browser decode; easy loopback bind | must handle resize/remap and validate Ogre child capture; display-only unless input is added | **Recommended** |
| X11 capture + JPEG WebSocket | Yes | can pair frames with input/status and explicit backpressure | more JS and protocol code, no benefit for a simple image pane | use only if interactivity is required |
| xpra HTML5 | Yes, and strong application remoting/input behavior | purpose-built individual-X-app remoting; built-in HTML5 client | not installed/no apt candidate; Python-based project/runtime; large feature/security surface; new nested display/session behavior; conflicts with “no Python runtime” | reject for this scope |
| Xvfb/VNC + noVNC/websockify | Yes, if RViz runs in that nested display | mature interactive remote desktop and browser client | every server/proxy component is absent; noVNC requires WebSockets and commonly websockify; Python runtime; more processes/ports; captures a desktop, not naturally one app; must revalidate GLX/Ogre | reject for this scope |
| Embed/extract `rviz_common::RenderPanel` and stream it | Uses RViz libraries/rendering, but is not the existing full RViz window | direct access to render widget; possible scene-only output | requires owning Qt/Ogre/ROS initialization, config/plugin loading, render-thread/context handling and readback; duplicates RViz application lifecycle; excludes normal RViz chrome/panels; high ABI/maintenance coupling | not the smallest and does not best honor “actual RViz window” |
| Browser-native URDF/ROS/WebGL reimplementation | No | efficient browser interaction and customizable UI | duplicates RViz displays, TF semantics, materials/meshes/camera/tools; cannot truthfully be called actual RViz | explicitly reject |

Official primary references supporting the comparison:

- X.Org Composite pixmap and resize/remap semantics: [XComposite manual](https://www.x.org/archive/X11R7.5/doc/man/man3/Xcomposite.3.html)
- Xlib image/pixmap interface: [Xlib C language specification](https://www.x.org/releases/X11R7.6/doc/libX11/specs/libX11/libX11.html)
- RViz's `RenderPanel` is a Qt widget tied to the RViz display context and forwards mouse/key events: the installed Lyrical header says this directly; an older generated API description is [here](https://docs.ros.org/en/indigo/api/rviz/html/c%2B%2B/classrviz_1_1RenderPanel.html). Current ROS guidance also treats RViz panels as Qt/pluginlib components: [ROS 2 custom RViz panel tutorial](https://docs.ros.org/en/rolling/Tutorials/Intermediate/RViz/RViz-Custom-Panel/RViz-Custom-Panel.html).
- xpra describes itself as X application remoting, includes an HTML5 client, and is built/implemented with Python: [official xpra repository](https://github.com/Xpra-org/xpra/).
- noVNC requires a WebSocket-capable VNC server or a WebSocket-to-TCP proxy such as websockify: [official noVNC repository](https://github.com/novnc/noVNC/).

## Current-repository concerns

1. **The installed workspace is stale.** Source and launcher contain `close_rviz_window.cpp`, and CMake installs it, but `ros2_ws/install/openarm_ik_ros/lib/openarm_ik_ros/` currently contains only `openarm_ik_ros_node`. The source/helper timestamps are newer than the installed node. The current launcher will stop with “Missing ... close_rviz_window” until the workspace is rebuilt.
2. **Actual display extensions could not be runtime-queried from this sandbox.** `xdpyinfo -display :0 -queryExtensions` returned “unable to open display :0” despite the X socket and environment being present. Package/header availability is confirmed, but Composite, MIT-SHM, and the exact RViz native-child behavior still require an in-session acceptance test.
3. **Clarify the Python boundary.** The proposed portal/backend is compiled and has no Python dependency. However, today's launcher invokes `ros2 launch ...openarm_ik_rviz.launch.py`; `/usr/bin/ros2` itself is Python. Therefore the current full process tree is not Python-free. If “no Python runtime” means absolutely no Python process, replace ROS launch with direct execution of compiled nodes (or a compiled C++ supervisor/composed process) and pass the URDF to `robot_state_publisher`; the official package documents the robot description parameter/topic contract: [robot_state_publisher repository](https://github.com/ros/robot_state_publisher). That is a lifecycle change independent of the window-capture choice.
4. **Do not promise minimized capture.** A mapped, occluded compositor window and a minimized/unmapped one are different states. Detect and report the latter.
5. **Software RViz rendering plus JPEG encoding consumes CPU.** Start at 10 fps/quality 80, measure, and make fps/quality bounded options. Avoid per-viewer encoding.
6. **Input injection expands authority.** `XTest` can affect the whole desktop if implemented incorrectly. Keep v1 display-only; if later added, require window existence/focus checks, clamp all coordinates, whitelist buttons/keys, rate-limit, validate origin/session, and disable it whenever the target window identity changes.

## Exact implementation/acceptance tests

No tests below were run in this reconnaissance because they start GUI/listener processes. They are the required implementation gates.

### Build and dependency tests

```bash
./scripts/build.sh
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
colcon --log-base ros2_ws/log test \
  --base-paths ros2_ws/src \
  --packages-select openarm_ik_ros \
  --build-base ros2_ws/build \
  --install-base ros2_ws/install
colcon test-result --test-result-base ros2_ws/build/openarm_ik_ros --verbose
test -x ros2_ws/install/openarm_ik_ros/lib/openarm_ik_ros/openarm_portal
! ldd ros2_ws/install/openarm_ik_ros/lib/openarm_ik_ros/openarm_portal | \
  rg -i 'python|websockify|vnc'
```

Add deterministic unit tests for: CLI/address parsing; rejection of invalid PID/port/address; Host/Origin policy; RGB extraction for common and synthetic visual masks; JPEG magic, decodability, and exact dimensions; multipart boundaries/lengths; a slow-client latest-frame policy that never grows a queue; resize/pixmap recreation using a fake X capture interface; and idempotent shutdown before/after a client connects.

### Live host test

After rebuilding the ROS workspace, from the logged-in GNOME session:

```bash
./scripts/launch_rviz.sh
```

With the eventual portal-integrated launcher running, verify:

```bash
portal_port=8765
ss -ltnp | rg "127\\.0\\.0\\.1:${portal_port}"
! ss -ltn | rg "(0\\.0\\.0\\.0|\\[::\\]):${portal_port}"
curl --fail --silent "http://127.0.0.1:${portal_port}/api/health"
curl --fail --max-time 3 \
  "http://127.0.0.1:${portal_port}/api/rviz.jpg" \
  -o /tmp/openarm-rviz.jpg
file /tmp/openarm-rviz.jpg
```

The stream acceptance checks are:

1. the browser right pane shows the complete actual RViz window, including the live Ogre 3D region and expected Qt panels;
2. moving the robot/camera in native RViz visibly changes the browser within 250 ms at the configured rate;
3. covering RViz completely with the browser does not replace its pixels with the covering window;
4. resizing RViz repeatedly changes reported/frame dimensions without corruption, disconnect, `BadDrawable`, or stale pixmap use;
5. minimizing RViz produces an explicit unmapped status; restoring reacquires and resumes;
6. two browser clients see current frames while a throttled/paused client does not increase resident memory continuously;
7. closing RViz or killing the portal causes deterministic launcher cleanup rather than orphaning ROS/RViz;
8. Ctrl+C closes HTTP connections and the listen socket, closes RViz without the known Ogre SIGINT teardown crash, reaps all children, and immediately permits a clean second launch;
9. `ps` shows no portal/RViz/openarm child and `ss` shows no PORT listener after shutdown;
10. a second host cannot connect because the listener is loopback-only.

Run once with the default software renderer and once with `OPENARM_RVIZ_RENDERER=integrated`; the default path is the release gate because it is the documented stable configuration on this laptop. Use X11 error trapping around Composite/pixmap operations so a resize race becomes a recoverable reacquire/status event rather than process termination.

## Decision

Proceed with a compiled C++ XComposite/libjpeg/Boost.Beast capture service integrated into the existing PID-aware launcher lifecycle. Start with MJPEG and display-only semantics. Do not install or introduce xpra/VNC/noVNC, do not extract/rebuild the RViz renderer, and do not represent a browser-native renderer as RViz.

The design decision is complete. The remaining concerns are empirical XComposite/Ogre-child validation in the user's GUI session, a required workspace rebuild for the current close helper, and an explicit product decision on whether “no Python runtime” applies only to the new portal/backend or to the pre-existing ROS launch process too.
