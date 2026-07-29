# Local RViz stream and control portal: independent design/security critique

Date: 2026-07-29 (America/Los_Angeles)  
Scope: read-only inspection plus this report; no build, GUI, listener, network,
CAN, or hardware operation was performed.

## Status and decision

**CONDITIONAL DESIGN APPROVAL; IMPLEMENTATION AND RELEASE NOT APPROVED.**

The compiled Boost.Beast + XComposite + libjpeg design is the preferred first
candidate for a **display-only, loopback-only stream of the already-running
stock RViz process**. It is not proven on this host. Full live acceptance of
the top-level redirected pixmap, especially the native Ogre render child under
XWayland/GLX, is a release blocker rather than a follow-up optimization.

No state-changing web control is approved against the repository as it exists.
The current ROS node has only `/openarm_ik/paired_xyz`, instantly substitutes
IK output for state, has no request identity or replay protection, and exposes
no stop, reset, enable, action, measured-state, operator-lease, or controller
interface. The proposed control portal depends on the unimplemented controller
adapter and ABI additions identified in `portal_control_recon.md`. Until those
exist, the portal may show RViz and read-only truth/status only.

The direct C++ Qt/RViz `RenderPanel` alternative is not preferred for v1. It
would be a new RViz-derived application, not a capture of the stock `rviz2`
window that the launcher currently owns. It removes XComposite/PID binding but
adds ownership of QApplication, RViz ROS integration, plugin/config loading,
Ogre/GL context lifetime, render-thread synchronization, and framebuffer
readback. `QWidget::grab()` is not sufficient evidence because the installed
`RenderPanel` contains a native `rviz_rendering::RenderWindow` (`QWindow`). A
reliable implementation would need Ogre render-target readback on the render
thread. That captures the 3-D scene, not automatically the RViz docks and
chrome. Consider it only as a separately specified scene-only product or if the
stock-window live gate conclusively fails.

## Repository facts that constrain the design

- `scripts/launch_rviz.sh` deliberately forces `QT_QPA_PLATFORM=xcb` and GLX,
  disables Qt render-target scaling, and defaults to llvmpipe in a Wayland
  session because live-resize flicker was observed with hardware GLX. It starts
  `rviz2` directly under `setsid` and retains its PID.
- The launcher keeps ROS and RViz in separate process groups and closes RViz by
  `WM_DELETE_WINDOW` before signaling the ROS launch group. This is an
  important workaround for a known RViz/Ogre signal-teardown failure and must
  be preserved.
- `close_rviz_window.cpp` finds the first `_NET_CLIENT_LIST` member whose
  `_NET_WM_PID` equals the supplied PID. This is adequate as a close helper,
  but it is not strong enough for long-lived frame authority: it does not bind
  PID start time, detect PID reuse, resolve multiple matching windows, or prove
  that the selected pixels contain the Ogre child.
- The installed package is stale: source builds and installs
  `close_rviz_window`, but `ros2_ws/install/openarm_ik_ros/lib/openarm_ik_ros/`
  currently contains only `openarm_ik_ros_node`. The current GUI launcher exits
  before starting RViz until the workspace is rebuilt.
- The current ROS package links only `openarm_model`, X11 for the close helper,
  and ROS message dependencies. There is no portal target, Beast, Composite,
  JPEG, controller, action, or generated portal-state interface.
- The current node republishes its last commanded IK posture at 10 Hz with a
  fresh stamp and fabricates two zero finger joints. That publication is not
  measured feedback and must not be presented as such by a portal.
- The RViz config is small and deterministic: one RobotModel display, collision
  display disabled, fixed frame `world`, and an Orbit view at 30 Hz.

## Approved v1 boundary

The only approved incremental v1 is:

1. The launcher starts the existing ROS graph and the stock `rviz2`, records a
   launch epoch plus the RViz PID identity, then starts one compiled portal.
2. The portal validates and captures the actual mapped RViz client window. It
   owns no model, joint-state publisher, TF broadcaster, controller, or input
   injector.
3. Beast serves compiled-in static assets, read-only status, a diagnostic JPEG,
   and a bounded MJPEG stream on literal `127.0.0.1` only.
4. All command routes are absent or return capability-unavailable until the
   measured controller/action adapter is implemented. Do not bridge HTTP to
   the present `PoseArray` topic.
5. The UI permanently says: virtual model only, model body frame/metres,
   position-only IK with free orientation, collision checking unavailable,
   physical motion unauthorized, gripper unmeasured/uncontrolled.

This gives useful live visualization without turning the weak current ROS
topic contract into a browser command API.

## Capture and XWayland findings

### Native child and GL content: release blocker

`XCompositeNameWindowPixmap(top_level)` cannot be assumed to contain the
pixels of RViz's native Ogre `QWindow` merely because Qt panels appear. The
installed `RenderPanel` embeds `rviz_rendering::RenderWindow`, a native
`QWindow`, and RViz uses GLX. Whether Mutter/XWayland's automatic redirection
produces a complete parent pixmap must be demonstrated in the logged-in
session under both configured renderers.

The portal must refuse to label a partial/black/stale 3-D region as live RViz.
It must not silently switch to a root-window crop: a root crop leaks other
windows and changes when the browser covers RViz. Walking and compositing child
windows is not approved merely by matching geometry; it requires a separate
prototype proving redirected GL child availability, clipping, stacking, visual
conversion, resize handling, and target ownership. Manually redirecting a
Mutter-managed hierarchy can interfere with the compositor and is not a casual
fallback.

### Target identity

Use all of the following, not title matching or `_NET_WM_PID` alone:

- PID supplied directly by the launcher, positive and already alive;
- launcher-recorded `/proc/PID/stat` start time, or preferably a retained
  `pidfd`, to reject PID reuse;
- expected executable identity for the direct `rviz2` child;
- `_NET_WM_PID == PID`, normal mapped top-level membership, and stable geometry;
- deterministic rejection/retry if zero or multiple plausible windows exist;
- optional XRes client-PID verification as corroboration, not the sole source;
- terminal invalidation on pidfd readiness/process exit, never reacquisition to
  a later process that happens to receive the same numeric PID.

`_NET_WM_PID` is client-supplied metadata. It is useful for correctness but is
not an authentication boundary. The launcher should not announce the browser
URL until the portal has selected the window and produced one valid frame.

### Resize, map, damage, and X errors

One capture thread should exclusively own its `Display *` and all X calls.
Avoid sharing a display across Beast, ROS, or JPEG workers. X errors are
asynchronous; install a non-terminating process-local handler and bracket
fallible Composite/pixmap operations with sequence-aware `XSync` checks so a
resize race becomes a status transition, not process termination.

Handle `ConfigureNotify`, `MapNotify`, `UnmapNotify`, `ReparentNotify`, and
`DestroyNotify`. Free and reacquire the named pixmap after every relevant map
or size/depth change. Validate queried width, height, depth, multiplication,
and allocation before capture. Set a maximum dimension and pixel budget; an
extreme user resize must not cause integer overflow or unbounded memory/CPU.
Debounce resize churn without displaying a corrupt old-size frame. Increment a
capture generation whenever the pixmap or dimensions change.

Occlusion and minimization are distinct. A completely covered, still-mapped
window must continue to produce its own pixels. A minimized/unmapped window
must become `RViz unmapped` promptly and must not serve an indefinitely frozen
last frame. Restoration must reacquire the pixmap before resuming.

XDamage and MIT-SHM are optional optimizations only after plain `XGetImage`
passes. Damage-driven capture still needs a heartbeat/status deadline because
an unchanged scene may legitimately produce no damage. MIT-SHM needs an
automatic non-SHM fallback and rigorous detach-on-resize/shutdown behavior.

### Frame conversion, JPEG, and pacing

- Convert using XImage masks, bits-per-pixel, byte order, and stride; do not
  assume BGRA or 32-bit packed pixels.
- Give libjpeg a `setjmp`-based error boundary; its default fatal error path
  must not terminate the portal. Bound dimensions, row bytes, encoded output,
  and quality before allocation.
- Start at 10 fps and quality 75-80 with a single encoder. Pace from
  `steady_clock`; after a missed deadline, schedule the next future deadline
  rather than running a catch-up burst.
- Capture once per generation and publish one immutable `shared_ptr` frame to
  all viewers. Never encode per client.
- Capture and HTTP work must be independent of ROS control and the urgent stop
  path. Capture failure is a visualization fault, not permission to delay or
  reject a stop request.

## MJPEG/Beast reliability requirements

Each MJPEG client may have exactly one asynchronous write in flight. When it
completes, select the newest frame generation; skipped generations are normal.
There is no per-client frame deque. Configure a bounded socket send buffer and
a per-write deadline that cancels a stalled socket. Kernel buffering means
"one application write" alone is not a complete memory/backpressure proof;
measure slow-client RSS and queued bytes in acceptance.

Write the HTTP header once, then well-formed multipart parts with a fixed safe
boundary, CRLF framing, `Content-Type: image/jpeg`, and exact per-part
`Content-Length`. Do not set a whole-response length. Keep every buffer alive
until its asynchronous completion. A disconnect, cancellation, timeout, and
normal close must converge on one idempotent client cleanup path and decrement
viewer count exactly once.

Cap viewers (four is sufficient for a local portal), connections, accept rate,
and idle header time. A client that sends a partial request must not hold a
connection forever. The browser should reconnect the `<img>` with bounded
exponential backoff and jitter; it must not create multiple concurrent streams.

The browser cannot inspect custom headers on individual multipart images shown
through `<img>`. Per-part capture generation/time/launch epoch are still useful
for diagnostic clients, but the UI needs a separate same-origin status channel
or poll and a DOM `STALE/RViz unavailable` overlay. The last decoded image may
remain painted after a socket dies, so image presence is not freshness. Do not
claim an atomic relation between an RViz pixel frame and ROS feedback. A sampled
state sequence can be recorded as metadata, but pixels remain visualization,
never control acknowledgement or measured state.

## HTTP parser and browser security requirements

Loopback reduces network exposure but is not authentication. A malicious web
page can target localhost, DNS rebinding can manipulate Host, and another local
process can act as an HTTP client.

### Listener and request envelope

- Compile v1 without a remote-bind option. Bind the numeric IPv4 address
  `127.0.0.1`; fail startup if the requested port cannot be bound. Do not also
  listen on `::`, `0.0.0.0`, a hostname, or an inherited wildcard socket.
- Print and use one canonical origin, for example
  `http://127.0.0.1:8765`. Require an exact `Host` of
  `127.0.0.1:<actual-port>` on every request before routing. Reject absolute
  form, authority form, userinfo, ambiguous ports, multiple Host headers, and
  non-origin-form targets.
- Use a fresh Beast parser and bounded flat buffer per request. Suggested
  maxima are 1 KiB target, 16 KiB aggregate headers, 8 KiB command body, a
  small header count, and short header/body deadlines.
- For state changes, accept only POST, a known `Content-Length`, and exact
  JSON media type policy. The simplest safe policy rejects Transfer-Encoding,
  Content-Length conflicts, trailing bytes, duplicate JSON keys, unknown
  fields, nesting outside the schema, strings where numbers are required,
  NaN/Infinity, and excessive numeric token length. Test request smuggling and
  pipelining; closing the connection after a mutation is acceptable.
- Static assets should be compiled into the binary under exact routes. This
  removes path traversal, symlink, MIME-sniff, and deployment drift risks.

### Origin, CSRF, sessions, and replay

Every mutation requires all of:

1. exact `Host` and exact `Origin` matching the canonical origin;
2. an operator session generated from at least 128 bits from `getrandom()`;
3. an `HttpOnly; SameSite=Strict; Path=/` session cookie;
4. a separate unpredictable CSRF value delivered only by same-origin content
   and returned in a custom header;
5. `Content-Type: application/json`; and
6. a server-issued, single-use command nonce bound to launch epoch, session,
   operator lease, current state sequence, exact canonical target, and short
   expiry.

Reject missing/`null` origins for mutations. Send no CORS allow headers. A
browser retry queries command status by its immutable ID; it never resubmits a
consumed motion nonce. Rotate launch/session epochs and invalidate all nonces
on portal or controller restart. Keep a bounded terminal command ledger long
enough for retry/status lookup. A loopback session does not defeat a malicious
same-user process; document that residual boundary instead of calling it user
authentication.

One explicit expiring operator lease may mutate; other clients are observers.
There is one server-authoritative command reservation and no motion queue or
last-writer-wins behavior. Concurrent valid requests yield one accepted command
and explicit busy/stale/replay results. Browser disconnect does not silently
resubmit, transfer ownership, or reinterpret completion. Lease expiry blocks
new work; future physical behavior additionally requires an independent
producer watchdog.

### Static response policy

Use at least:

```text
Content-Security-Policy: default-src 'none'; script-src 'self'; style-src 'self'; img-src 'self'; connect-src 'self'; base-uri 'none'; frame-ancestors 'none'; form-action 'self'; object-src 'none'
X-Content-Type-Options: nosniff
X-Frame-Options: DENY
Referrer-Policy: no-referrer
Cache-Control: no-store
```

No inline event handlers, `eval`, external scripts/fonts/images, service worker,
wildcard source, or permissive CORS. Use fixed MIME types and immutable asset
content. Add a restrictive Permissions-Policy as defense in depth.

Return stable error codes and a short request ID to the browser. Do not expose
exception text, filesystem paths, X resource IDs, environment variables,
cookies/nonces, ROS graph internals, or raw library messages. Bound and sanitize
server logs; never log command/session secrets or arbitrary headers/bodies.

## Control and stop-path critique

The control design in `portal_control_recon.md` is a future architecture, not a
description of current code. Its prerequisites remain blockers:

- `oa_controller_plan_tcp` for honest one-arm requests;
- a production standard virtual-manifest builder;
- a single-owner controller-backed ROS adapter;
- measured atomic portal state and action results;
- enable/disable, reset, and stop services;
- command arbitration and an independent watchdog.

Do not implement left/right browser buttons by reading the other arm from the
page and publishing the present paired `PoseArray`. Do not infer completion
from diagnostics, a fresh `/joint_states` timestamp, or RViz pixels. Do not
offer physical motion, gripper control, or calibration.

A button labelled `E-stop` is rejected. The eventual UI may say **Request stop
(not a safety E-stop)**. It still needs CSRF/session validation, but it bypasses
operator-lease and normal command queues once accepted. The web handler first
latches local reject-new state, then invokes an urgent ROS/controller path.

There is an important unresolved preemption issue: queueing urgent work to the
same single owner thread does not preempt that thread while it is inside a
long/blocking plan, controller, or middleware call. Acceptance must measure a
specified upper bound on stop-request latency while IK, normal ROS work, HTTP
streaming, JPEG encoding, and capture are deliberately stalled. If the control
ABI cannot guarantee short bounded calls, it needs a reviewed thread-safe
out-of-band stop/interlock primitive. For physical use, only the independent
hardwired E-stop/deadman is the safety mechanism; no process or browser timing
claim substitutes for it.

Reset never clears a physical interlock and never re-arms. Reconnect, refresh,
duplicate POST, controller restart, or portal restart cannot clear a latch.
Until the services actually exist, render no active stop/reset/enable control.

## Shutdown and failure ownership

The integrated launcher needs three retained PIDs/process groups and should
treat unexpected exit of ROS, RViz, or portal as failure of the whole virtual
session. Preserve the current RViz `WM_DELETE_WINDOW` path.

Orderly shutdown is:

1. atomically reject new web mutations and revoke the operator lease;
2. terminalize/retire queued work and request controller disable/stop while the
   ROS authority is still alive;
3. for a future physical backend, require a fresh coherent disabled
   acknowledgement, otherwise report `STOP STATE UNCONFIRMED—USE PHYSICAL
   E-STOP`;
4. stop accepting HTTP, cancel client timers/sockets, and drain only bounded
   in-flight responses;
5. stop/join capture, free XImage/SHM/damage/pixmap resources, and close Display;
6. close stock RViz by the existing helper, with bounded TERM/KILL fallback;
7. INT/TERM/KILL the ROS process group, reap all children, release the lock.

`boost::asio::signal_set` may initiate portal shutdown, but no Xlib, JPEG, heap,
or socket cleanup belongs in an asynchronous POSIX signal handler. All joins
need deadlines at the launcher boundary because an X server or socket write can
stall. A SIGKILL fallback is process cleanup, never a successful safety stop.

The current `wait -n` watches only ROS and RViz. Portal integration must also
watch the portal and must not orphan it. Terminal SIGINT reaches the launcher
while its `setsid` children are isolated, which is useful; retain explicit,
ordered forwarding instead of placing all children back into one signal group.

## XComposite versus embedded RViz

| Criterion | Stock RViz + XComposite | Custom Qt/RViz render process |
|---|---|---|
| Literal existing RViz app | Yes, if live gate passes | No; it is a new app using RViz libraries |
| Existing host workaround | Preserves launcher, xcb/GLX, llvmpipe, WM close | Must reproduce and requalify all of it |
| Full RViz panels/chrome | Potentially, but native-child inclusion unproven | `VisualizationFrame` can provide it; framebuffer capture still mixed Qt/native GL |
| Scene pixels | Empirical Composite/GLX blocker | Ogre target readback can be deterministic if done on render thread |
| Occlusion/minimize | Occlusion should work; minimize intentionally unavailable | Can keep a hidden/offscreen render target, but this changes application behavior |
| Resize | Pixmap invalidation/reacquisition and X races | Qt/Ogre render-target resize and readback races |
| ABI/maintenance | X11/Composite/JPEG/Beast only | RViz 15.2.4, Qt 6, Ogre 1.12, plugins, ROS integration |
| Failure isolation | Portal may die without crashing RViz; launcher owns both | Web/capture fault may take down visualization process |
| Verdict | **Preferred prototype, conditional on full live proof** | **Not v1 fallback without a new scene-only/full-frame specification and tests** |

If only the 3-D scene is actually required, a custom RViz visualization process
using the installed `RenderPanel` and
`RenderWindowOgreAdapter::getOgreViewport()->getTarget()->copyContentsToMemory`
could ultimately be more deterministic than desktop capture. It must execute
readback on the Qt/Ogre render thread, double-buffer safely, and load the same
RobotModel/config. It should be labelled an RViz-library render, not a stream of
the separately launched stock RViz window. If the complete stock application
is required, XComposite remains the smaller honest design.

## Required automated tests before any live run

### Capture/encoding units

- CLI rejection for invalid PID, PID reuse/start-time mismatch, port, fps,
  quality, dimensions, and non-loopback address.
- Fake-X state machine covering no window, multiple windows, map/unmap,
  reparent, resize/depth change, destroy/recreate, stale pixmap `BadDrawable`,
  and shutdown during each transition.
- Visual-mask conversion for common 16/24/30/32-bit layouts, both byte orders,
  padded strides, synthetic masks, and overflow dimensions.
- libjpeg fatal-error recovery, exact dimensions, decode validity, output cap,
  and deterministic memory ownership.
- steady-clock pacing with missed ticks and no catch-up burst.

### HTTP/security units and integration

- exact Host/Origin matrix including missing, duplicate, malformed, absolute
  URI, localhost, IPv6, rebinding host, wrong port, and `Origin: null`;
- cross-site form/fetch/image/WebSocket attempts, absent cookie/token, stale
  session, bad CSRF, wildcard/no-CORS verification, and framed-page denial;
- duplicate/replayed/expired command nonce, wrong lease/state/epoch/target,
  retry-by-status, controller restart, and concurrent-client barrier races;
- request-line/header/body limits, partial slow headers/body, chunked body,
  CL+TE, duplicate CL/Host, pipelining/smuggling, malformed multipart paths,
  invalid/duplicate/extra JSON fields, deep nesting, NaN/Infinity, and huge
  numbers;
- static-route exactness, traversal/encoding variants, fixed MIME/CSP/no-store,
  no third-party assets, and generic bounded error responses;
- multipart parser validation, immutable buffer lifetime, one write per client,
  timeout/disconnect/reset at every byte boundary, viewer-count idempotence,
  maximum viewers, and browser reconnect without duplicate streams;
- slow-client and disconnect soak under ASan/UBSan and TSan where practical,
  demonstrating bounded heap/RSS, descriptors, threads, and latest-frame
  behavior.

### Control tests once the controller adapter exists

- all action, freshness, atomicity, measured provenance, one-lease/one-command,
  replay, reset, cancellation, and no-physical-path gates already specified in
  `portal_control_recon.md` and `portal_safety_ranges.md`;
- stop request while capture, JPEG, HTTP normal command handling, ROS callbacks,
  IK planning, execution, and result delivery are each blocked; prove the
  declared latency and persistent latch;
- portal/browser loss cannot clear a latch or cause a command retry; action
  result correlates to exactly one immutable command ID;
- no state-changing endpoint is registered when the adapter reports missing or
  incompatible capabilities.

## Mandatory full live acceptance on this laptop

These tests cannot be replaced by mocks, screenshots, or headless CI. Run them
from the logged-in GNOME session after rebuilding. Repeat the stream suite with
the default software renderer and `OPENARM_RVIZ_RENDERER=integrated`; the
documented default software path is the release gate.

1. Verify the only listener is the selected `127.0.0.1:PORT`, a second machine
   cannot connect, and no wildcard/IPv6 listener exists.
2. Prove the selected X window belongs to the launcher-recorded RViz PID/start
   identity and that PID exit invalidates the stream immediately.
3. The browser shows the complete actual RViz client, including RobotModel
   motion in the Ogre region and the expected Qt Displays panel. Deliberately
   animate camera/model content so a static or black child cannot pass.
4. Cover RViz completely with the browser and another window. The stream must
   retain RViz pixels and must never show the covering window or desktop.
5. Resize continuously and repeatedly across small/large sizes and HiDPI screen
   transitions. No black holes, stale dimensions, torn old/new buffers,
   `BadDrawable`, crash, reconnect loop, unbounded allocation, or recurrence of
   the host's RViz resize flicker is accepted.
6. Minimize: the UI overlays `RViz unmapped` within its deadline and does not
   imply the frozen last frame is live. Restore: a new capture generation and
   correct dimensions resume without restarting the session.
7. Move the native RViz camera and issue the approved virtual regression target
   through the ROS test path, not the portal. Visible browser change must meet a
   declared end-to-end latency (250 ms is reasonable at 10 fps), while portal
   state remains explicitly virtual/collision-unchecked.
8. Connect four clients; throttle one to very low bandwidth, pause one tab,
   repeatedly disconnect/reconnect another, and leave one normal. The normal
   client stays current, slow clients skip frames, and RSS/fd/thread/socket
   counts remain bounded over a soak.
9. Kill the browser mid-part, close a tab, send TCP reset, suspend/resume the
   browser, and leave an incomplete HTTP request. Viewer counts and resources
   return to baseline and a new viewer connects immediately.
10. Exercise capture with unchanged pixels, rapid updates, live resize, and
    renderer stalls. Frame rate never exceeds the cap, no catch-up burst occurs,
    and status freshness—not the painted last image—drives `STALE`.
11. Send SIGINT during startup window search, first capture, JPEG encode, active
    MJPEG write, resize reacquire, no-viewer idle, and after RViz exit. Also kill
    each of ROS, RViz, and portal unexpectedly. Every run exits within its bound,
    reaps all children, closes the listener, releases X resources/lock, and
    permits immediate clean relaunch.
12. Confirm RViz still closes through `WM_DELETE_WINDOW` without its known Ogre
    teardown crash. Confirm no portal, RViz, ROS launch, or OpenArm child and no
    listen socket remains after shutdown.
13. Once control is implemented, run multi-browser lease/replay/CSRF races and
    stop-priority fault injection live. Until those pass, commands remain
    disabled even if the stream itself passes.

Store acceptance evidence: exact commit/build IDs, ROS/RViz/library versions,
renderer/environment selection, PID/start identity, X extension versions,
health snapshots, frame hashes/screenshots for native-child and occlusion
proof, latency/resource traces, test logs, and clean process/socket listings.

## Dependencies and packaging decision

Mandatory for the preferred stream target:

| Dependency | Current evidence | Purpose |
|---|---|---|
| X11 | `libx11-dev` 1.8.13 | window discovery/events/image access |
| XComposite | `libxcomposite-dev` 0.4.6 | redirected window pixmap |
| JPEG | `libjpeg-turbo8-dev` 2.1.5, standard `jpeglib.h` | one shared MJPEG encoder |
| Boost | Boost 1.90 Beast/Asio headers installed | HTTP server and async lifecycle |
| Threads | system C++ threads/pthreads | isolated X capture/encoding and I/O |

Optional after correctness:

| Dependency | Use |
|---|---|
| XDamage + XFixes | redraw coalescing, never sole freshness clock |
| Xext/MIT-SHM | faster local capture with tested fallback |
| XRes | corroborate X client/PID identity |

Use CMake imported targets where available (`X11::X11`, verified Composite
target or pkg-config fallback, `JPEG::JPEG`, Boost headers/system as required,
`Threads::Threads`). Add exact rosdep/package.xml dependencies instead of
assuming workstation headers. OpenSSL/TLS, WebSocket, VNC, xpra, websockify,
and Python web packages are unnecessary for loopback v1.

The embedded-RViz alternative additionally requires `rviz_common`,
`rviz_rendering`, Qt 6 Widgets/Gui, RViz's Ogre 1.12 vendor libraries, ROS node
integration, pluginlib, and associated runtime resources. Those dependencies
are installed transitively with RViz but materially increase ABI and lifecycle
surface; they are not approved additions merely as a workaround for a failed
capture test.

## Final approval statement

Approve implementation of a **read-only prototype** using stock RViz,
XComposite, one shared libjpeg encoder, and a strictly loopback Boost.Beast
server. Do not approve release until every mandatory live gate passes on the
actual XWayland/GLX host. Do not approve any browser mutation, independent arm
button, software stop, reset, enable, or calibration endpoint against the
current ROS node. The control portal becomes reviewable only after the native
controller/action prerequisites exist and the security, replay, arbitration,
stop-priority, and shutdown acceptance suites pass in full.
