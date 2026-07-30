# Portal black RViz capture investigation

Date: 2026-07-29 (America/Los_Angeles)  
Inspected revision: `ff38733`  
Scope: read-only production investigation; no GUI, RViz, portal, screenshot, or
desktop-capture process was started.

## FINDINGS

### Root cause: the capture converts every named pixmap to black

The observed valid JPEG containing a black square is explained deterministically
by `rviz_capture.cpp`; GLX failure is not needed to produce this symptom.

`XCompositeNameWindowPixmap()` returns a **Pixmap**. The code then calls
`XGetImage()` on that pixmap and derives RGB with:

```cpp
channel(pixel, image->red_mask)
channel(pixel, image->green_mask)
channel(pixel, image->blue_mask)
```

This cannot work for an X11 Pixmap. The locally installed core X11 protocol
specification (`/usr/share/doc/xproto/x11protocol.txt.gz`, `GetImage`) says:

- for a Window drawable, GetImage returns its visual;
- for a Pixmap drawable, GetImage returns `visual = None`.

Local libX11 1.8.13 implements that literally: `XGetImage` resolves the reply's
visual ID and calls `XCreateImage`; with `visual=None`, `XCreateImage` leaves
the `XImage` RGB masks zero. The authored `channel()` explicitly returns zero
for a zero mask. Thus raw pixmap pixels may be perfectly good and nonzero, but
all three output channels are forced to zero before JPEG encoding. The server
then truthfully returns HTTP 200 and a valid, uniformly black JPEG.

This also explains why `/api/health` reported `window_ready:true`: that route
only proves process identity and finds one mapped top-level window. It never
reads or validates a frame.

The smallest robust correction is to use the source Window's Visual masks,
which are already available in `XWindowAttributes attributes.visual`, while
reading pixel values from the named Pixmap's `XImage`:

```cpp
const Visual * visual = attributes.visual;
// Require TrueColor and nonzero, pairwise-disjoint masks.
rgb[offset]     = channel(pixel, visual->red_mask);
rgb[offset + 1] = channel(pixel, visual->green_mask);
rgb[offset + 2] = channel(pixel, visual->blue_mask);
```

The named hierarchy pixmap has the redirected Window's depth/storage. Using
that Window's TrueColor visual supplies the pixel layout that the Pixmap itself
does not carry. Do not substitute `DefaultVisual`: RViz may use a non-default
GLX visual.

### Local environment and why GLX is a second gate, not today's diagnosis

The stopped session's environment and installed objects show:

- GNOME Wayland session (`XDG_SESSION_TYPE=wayland`) with `DISPLAY=:0`;
- XWayland 24.1.10, Mutter 50.1, XComposite library 0.4.6, Xlib 1.8.13;
- Qt 6.10.2, Mesa 26.0.3, and stock ROS Lyrical RViz 15.2.4;
- launcher forces `QT_QPA_PLATFORM=xcb`, `QT_XCB_GL_INTEGRATION=xcb_glx`, and
  on Wayland chooses `LIBGL_ALWAYS_SOFTWARE=1` in auto mode.

The installed RViz binaries prove that the 3-D surface is a native child:
`RenderPanel` constructs `rviz_rendering::RenderWindow` (a `QWindow`) and embeds
it with `QWidget::createWindowContainer()`. `RenderWindowImpl::initialize()`
passes that `QWindow::winId()` to Ogre's render-window creation. Therefore the
Ogre/GLX drawable is a real descendant X window, not ordinary raster painting
into the Qt top-level.

That matters for live acceptance, but it is not evidence that the named pixmap
was black. The current conversion destroys all evidence before the JPEG is
produced. XComposite's installed 0.4 protocol documentation states that
per-hierarchy off-screen storage includes the window, borders, and all
descendants. The first test must therefore be the Visual-mask correction on the
existing top-level hierarchy capture. Only a corrected frame that shows Qt
chrome while the Ogre rectangle remains black/empty would establish a native
GLX-child capture problem.

`xdpyinfo` could not reconnect to `:0` after the application/session test was
stopped, so no live tree or drawable was inspected. This report does not claim
that XWayland/llvmpipe GLX inclusion has passed; it identifies the deterministic
bug that currently prevents that question from being measured.

### XComposite redirect and version semantics

- Requiring Composite protocol 0.2 or newer is correct for
  `NameWindowPixmap`. There is no reason to raise the minimum to 0.4 for this
  automatic hierarchy capture; 0.4 changes manual-child clipping semantics.
- `CompositeRedirectAutomatic` is the correct update mode for a visible RViz
  hierarchy. Do not use `CompositeRedirectManual`: it transfers presentation
  responsibility to the portal and can make or corrupt the visible RViz
  window. The protocol only makes Manual exclusive; multiple clients may hold
  Automatic redirection.
- The portal should name the pixmap only after its own Automatic redirect has
  succeeded, as it does now. Reacquiring a new named pixmap on every snapshot is
  inefficient but safely covers map/resize allocation changes.
- The current code never calls `XCompositeUnredirectWindow` for an Automatic
  redirect it requested. Connection close eventually cleans it up, but a
  changed/destroyed target leaves stale state until then. A cleanup helper
  should free the named pixmap first and unredirect the exact Window with the
  same Automatic mode when retargeting or destructing; trap `BadWindow` during
  destruction races. This is a lifecycle correction, not the black-frame fix.

### Window and root readback tradeoffs

Do not replace the named pixmap with a root-window crop. A root readback returns
desktop composition, leaks covering windows/notifications, and changes when the
browser covers RViz. It violates the launcher-owned-pixels boundary.

Direct `XGetImage(rviz_window, ...)` would conveniently return a Visual and can
be a useful one-off diagnostic, but core X11 defines obscured regions as
undefined without backing store. It therefore cannot be the production
occlusion-independent path. Reading a native child Window has the same
obscuration caveat. The production path should remain Automatic Composite ->
named Pixmap -> GetImage, with masks taken from the corresponding Window.

### Numerical black-frame detection

Capture must validate the uncompressed RGB before JPEG encoding. At minimum,
compute in one pass:

- channel minima and maxima;
- count of pixels whose `max(R,G,B) <= 4`;
- luminance mean and variance (64-bit/incremental arithmetic);
- a small quantized RGB histogram or count of distinct 5-bit RGB buckets;
- a stable frame hash for test diagnostics, not for control feedback.

Production should reject an exact conversion failure (`all channel maxima ==
0`) and an effectively uniform frame (for example RGB range <= 2 and luminance
standard deviation < 0.5) with HTTP 503, rather than label it actual RViz
pixels. Keep the production threshold conservative: a deliberately dark scene
can be valid. The repository's fixed RViz configuration uses background
`48;48;48` and a visible RobotModel, so an all-zero frame is never an expected
configured result.

The live acceptance test can be stricter and must test time as well as one
frame. Record metrics for 20 corrected frames, deliberately move the native
RViz camera or animate the robot, and require:

1. no frame is exact/effectively uniform;
2. at least a modest non-modal population (for example >0.5% of pixels outside
   the dominant quantized RGB bucket);
3. luminance range and variance consistent with visible chrome/model;
4. at least two distinct frame hashes during deliberate motion;
5. the changed pixels overlap the known 3-D-region rectangle, not only a Qt
   status label.

Thresholds 2 and 3 should be recorded from the first successful host run rather
than guessed into a permanent generic X11 API. Exact-zero/uniform detection is
the regression guard for this bug; motion/region change is the proof of live
Ogre pixels.

## Concrete patch plan

1. In `rviz_capture.cpp`, validate `attributes.visual` before naming/reading:
   require `TrueColor`; require nonzero, pairwise-disjoint red/green/blue masks;
   retain the existing dimension/depth bounds. Return a precise unsupported
   visual error rather than a black JPEG.
2. Change conversion to use `attributes.visual->{red,green,blue}_mask`, never
   `image->{red,green,blue}_mask` for a named Pixmap.
3. Extract the masked-channel conversion and frame-stat calculation into a
   small testable helper. Reject exact/effectively uniform RGB before JPEG and
   expose the reason through the existing 503 JSON path.
4. Add `release_redirect()` that frees `pixmap_`, calls
   `XCompositeUnredirectWindow(..., CompositeRedirectAutomatic)` only for the
   portal-owned `redirected_`, traps resize/destroy errors, and is used on
   target change and destruction. Do not unredirect a window merely observed
   as redirected by another client.
5. Rebuild and run non-GUI unit tests, then perform the logged-in host gate with
   software rendering first and integrated rendering second. Cover RViz fully,
   resize it, minimize/restore it, and deliberately move camera/model content.

## Automated regression tests

- Pure conversion: synthetic nonzero pixel values plus explicit 8:8:8 and
  10:10:10 Window masks produce expected RGB even when a stand-in XImage's own
  masks are all zero. This exact case would fail at `ff38733`.
- Mask validation: reject zero, overlapping, and unsupported visual masks;
  verify scaling for non-8-bit channels.
- Frame metrics: all-zero and nearly uniform buffers reject; the configured
  background plus synthetic grid/robot colors accepts; use checked arithmetic
  at maximum 4096x4096 dimensions.
- JPEG boundary: accepted synthetic RGB decodes to the same dimensions and is
  not numerically black; rejected frames never invoke the encoder.
- Fake capture lifecycle: redirect -> name -> free -> unredirect ordering;
  resize/remap reacquisition; target destroy/change; trapped `BadDrawable` and
  `BadWindow` recover without serving stale JPEG.

## Conditional native-Ogre-child fallback

Do not implement child heuristics in the first fix. If and only if the corrected
top-level frame visibly contains Qt panels but its Ogre region is black, add a
diagnostic prototype that recursively enumerates **only descendants of the
already PID/start-time-validated top-level**, records geometry/depth/Visual and
raw/RGB statistics, and automatically redirects/names the native child. Never
scan/crop the root.

The fallback production design is to capture the unique viewable native Ogre
descendant's named pixmap using that child's Visual masks. If the product still
requires the full RViz chrome, blit the child RGB into the corrected top-level
RGB at coordinates obtained with `XTranslateCoordinates`, clipped to both
geometries. Prove candidate uniqueness and motion in the 3-D rectangle before
shipping; do not select a child solely by title or a coincidental black/bright
histogram. If robust child identity cannot be established, return unavailable
rather than silently serving a partial or desktop-derived image.

This fallback remains local X11/XComposite readback of the launcher-owned stock
RViz process. It needs no XDG screenshot permission, remote desktop stack, input
injection, or browser renderer.
