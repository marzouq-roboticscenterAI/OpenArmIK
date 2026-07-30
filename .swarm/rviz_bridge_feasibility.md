# Out-of-tree RViz render-panel bridge feasibility

Date: 2026-07-30  
Scope: read-only investigation of the installed ROS 2 Lyrical stack and the current OpenArmIK tree. No GUI was launched and no production file was changed.

## Decision

**Yes: an out-of-tree plugin can capture only the live RViz Ogre render target and apply typed Orbit deltas without forking `rviz_common`.** The bridge must have a small **in-process RViz plugin** component. A separate companion process by itself cannot dereference RViz's C++ objects; it can only communicate with the in-process component.

The following parts are proven present in the installed public ABI:

- `rviz_common::Panel::getDisplayContext()`
- `rviz_common::DisplayContext::getViewManager()` and `queueRender()`
- `rviz_common::ViewManager::getRenderPanel()` and `getCurrent()`
- `rviz_common::RenderPanel::getRenderWindow()`
- `rviz_rendering::RenderWindowOgreAdapter::addListener()`, `removeListener()`, and `getOgreViewport()`
- Ogre 1.12.10 `RenderTargetListener::postRenderTargetUpdate()` and `RenderTarget::copyContentsToMemory()`
- exported `rviz_default_plugins::view_controllers::OrbitViewController::{yaw,pitch,zoom,move}`

The installed versions are `rviz_common`, `rviz_rendering`, `rviz_default_plugins`, and `rviz_ogre_vendor` **15.2.4**, Qt **6.10.2**, and Ogre **1.12.10**. `nm -D -C` confirms that all named RViz and Orbit functions are dynamic symbols in the installed libraries, not header-only or private implementation accidents.

There are two important qualifications:

1. **Exact 1280x720 native rendering is only obtained when the live Ogre render target is actually 1280x720.** `copyContentsToMemory()` copies pixels; it does not rescale them. A bridge can always produce a 1280x720 output by scaling the captured frame, but that is a resampled live view. Forcing the embedded `RenderPanel` to 1280x720 via inherited Qt sizing APIs would alter RViz's window/layout. Rendering a separate 1280x720 Ogre texture would be a second rendering, not a capture of only the live panel.
2. **30 FPS is not guaranteed by the public readback API.** The installed GL renderer implements `copyContentsToMemory()` with synchronous `glReadPixels` followed by a vertical flip on the calling render thread. It is the correct portable API, but it can stall RViz. Thirty FPS is credible only after an instrumented test on the declared renderer and scene. It is materially risky on this repository's default Wayland path, which intentionally uses Mesa software rendering (`LIBGL_ALWAYS_SOFTWARE=1`).

This still makes the plugin route the smallest exact-RViz design: it removes XComposite, top-level-window chrome, and generic input injection while retaining stock RViz rendering and stock Orbit semantics.

## Exact in-process API path

An `rviz_common::Panel` subclass receives its context in `onInitialize()`:

```text
Panel::getDisplayContext()
  -> DisplayContext::getViewManager()
  -> ViewManager::getRenderPanel()
  -> RenderPanel::getRenderWindow()
```

All four links are public installed headers. The main `RenderPanel`, rather than the bridge's dock widget, is returned by `ViewManager::getRenderPanel()`.

The capture setup is:

1. Derive a listener from public `Ogre::RenderTargetListener`.
2. Register it with
   `rviz_rendering::RenderWindowOgreAdapter::addListener(render_window, listener)`.
3. In `postRenderTargetUpdate(const Ogre::RenderTargetEvent & event)`, use `event.source`, or obtain the target through
   `RenderWindowOgreAdapter::getOgreViewport(render_window)->getTarget()`.
4. Read exactly the target dimensions from `RenderTarget::getWidth()/getHeight()` (or the viewport's actual dimensions).
5. Supply an equal-sized `Ogre::PixelBox` and call
   `event.source->copyContentsToMemory(source_box, pixel_box, Ogre::RenderTarget::FB_AUTO)`.
6. Remove the listener with `RenderWindowOgreAdapter::removeListener()` before the plugin/listener is destroyed.

This target contains the Ogre 3D view and Ogre overlays rendered into it. It does **not** contain Qt menus, toolbar, dock panels, dialogs, the desktop, or the mouse cursor. That is the required isolation improvement over the current `XComposite` capture, which reads the complete top-level RViz window.

`RenderWindow::captureScreenShot(std::string)` is also public, but it writes a file through Ogre. It is unsuitable for a 30 FPS memory stream. The listener plus `copyContentsToMemory()` is the relevant interface.

RViz 15.2.4's implementation explicitly forwards `RenderWindowOgreAdapter::addListener()` to its underlying Ogre render window; it also queues listeners added before Ogre window initialization. The corresponding tagged source is [RViz 15.2.4 `render_window.cpp`](https://raw.githubusercontent.com/ros2/rviz/15.2.4/rviz_rendering/src/rviz_rendering/render_window.cpp), and the installed/public interface is also listed in the [current official `RenderWindowOgreAdapter` documentation](https://docs.ros.org/en/ros2_packages/rolling/api/rviz_rendering/generated/classrviz__rendering_1_1RenderWindowOgreAdapter.html). Ogre documents the post-target callback as occurring just after all target viewports have rendered and documents the memory-copy API in its official [RenderTargetListener](https://ogrecave.github.io/ogre/api/1.12/class_ogre_1_1_render_target_listener.html) and [RenderTarget](https://ogrecave.github.io/ogre/api/1.12/class_ogre_1_1_render_target.html) references.

### 1280x720 handling

At 1280x720:

- RGBA/BGRA frame: `1280 * 720 * 4 = 3,686,400` bytes = **3.516 MiB**.
- RGB frame: `1280 * 720 * 3 = 2,764,800` bytes = **2.637 MiB**.

There are three distinct contracts; they should not be conflated:

| Contract | Feasible without an RViz fork? | Consequence |
|---|---:|---|
| Capture the live target at its actual size | Yes | Exact live Ogre pixels. |
| Always emit a 1280x720 image | Yes | Capture actual size, then CPU/GPU rescale in the worker; output is resampled if the panel differs. |
| Guarantee the live target itself renders natively at 1280x720 | Conditionally | RViz layout/HiDPI must be constrained so the central `RenderWindow` is exactly that size. There is no RViz config key for render-panel dimensions in the current 42-line config. |

The repository launchers disable Qt HiDPI scaling for RViz, so their intended XCB/GLX path has a device pixel ratio of one. Nevertheless, the bridge must assert the target's observed width and height and report whether it is native or resampled; it must not infer dimensions from the top-level window.

## Typed Orbit camera deltas

The active view is public:

```text
DisplayContext::getViewManager()->getCurrent()
```

The bridge should require both:

```text
current->getClassId() == "rviz_default_plugins/Orbit"
dynamic_cast<rviz_default_plugins::view_controllers::OrbitViewController *>(current) != nullptr
```

If either check fails, reject the command. Do not silently synthesize mouse events or mutate an unrelated view controller. The public slot
`ViewManager::setCurrentViewControllerType("rviz_default_plugins/Orbit")` can deliberately switch views, but that destroys/replaces the previous current controller; it should only be exposed as an explicit product operation, not as a side effect of a delta.

The installed Orbit header and shared-library symbols expose these direct, typed operations:

| Method | Exact installed 15.2.4 behavior | Appropriate command field |
|---|---|---|
| `yaw(float angle)` | Subtracts `angle` from yaw and wraps it into `[0, 2*pi)`. | Finite bounded yaw delta in radians. |
| `pitch(float angle)` | Adds `-angle` to pitch; the property clamps to approximately `[-pi/2, +pi/2]`. | Finite bounded pitch delta in radians. |
| `zoom(float amount)` | Adds `-amount` to camera distance; distance has minimum `0.001`. Positive amount moves toward the focal point. | Finite bounded distance delta in metres. |
| `move(float x, float y, float z)` | Rotates the `(x,y,z)` vector by the camera-parent orientation, then adds it to the focal point. | Finite bounded camera-local pan/dolly delta in metres. |
| inherited `reset()` | Restores the controller's stock initial view. | Explicit boolean/enum operation, not a synthetic key. |

These semantics are directly visible in the exact [RViz 15.2.4 Orbit implementation](https://raw.githubusercontent.com/ros2/rviz/15.2.4/rviz_default_plugins/src/rviz_default_plugins/view_controllers/orbit/orbit_view_controller.cpp), lines 327-367. They are deltas despite the terse header comment on `yaw()`.

After one or more accepted deltas, call `DisplayContext::queueRender()`. The property changes are applied to the Ogre camera by `OrbitViewController::updateCamera()` during the next RViz `ViewManager::update()`. `queueRender()` is public and explicitly documented as callable from any thread, although the controller mutation itself is not.

A narrow command type should carry only finite, range-limited fields such as yaw delta, pitch delta, zoom/distance delta, camera-local pan, reset, and a monotonic command sequence. It should have no generic Qt event, screen coordinate, key, mouse button, focus, menu, file, plugin, or arbitrary ROS operation. Whether this type is a ROS message/service or a same-process queue is an application choice; it is not necessary to alter RViz APIs.

## Threading and lifetime constraints

All RViz, Qt GUI, Orbit-property, and Ogre calls must occur on RViz's GUI/render thread.

The exact installed RViz flow supports this:

- `VisualizationManager` owns a Qt `QTimer`.
- Its `onUpdate()` calls `executor_->spin_some(10 ms)`, updates displays and the view controller, then takes its render mutex and invokes `Ogre::Root::renderOneFrame()`.
- The executor is a `rclcpp::executors::SingleThreadedExecutor` containing RViz's raw node.
- The Ogre render-target listener therefore runs synchronously inside `renderOneFrame()`, on the Qt update thread in stock RViz.

This is shown in the exact [RViz 15.2.4 `visualization_manager.cpp`](https://raw.githubusercontent.com/ros2/rviz/15.2.4/rviz_common/src/rviz_common/visualization_manager.cpp), especially lines 214-216 and 319-377.

Consequences:

- A subscription created on `getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node()` and serviced by RViz's existing executor will run during that GUI-thread `spin_some()`; it does not need a second executor.
- If any socket, codec, or separately spun ROS executor invokes commands from another thread, marshal the validated/coalesced delta to a `QObject` slot using a queued Qt connection. Qt states that `QWidget` subclasses are main-thread-only and that queued slots execute in the receiver's thread; see [Qt Threads and QObjects](https://doc.qt.io/qt-6/threads-qobject.html).
- Do not call Orbit methods, `ViewManager`, `RenderPanel`, `RenderWindow`, Ogre, or Qt widgets from a worker thread.
- In `postRenderTargetUpdate()`, do **not** call `DisplayContext::lockRender()`: `VisualizationManager` already holds its non-recursive render mutex around `renderOneFrame()`, so doing so would deadlock.
- Do not recursively request `renderNow()`/`renderOneFrame()` from the listener.
- Keep the listener callback bounded: perform the unavoidable readback into a preallocated available slot, timestamp/sequence it, and return. Scale/convert/JPEG-encode in a worker.
- Use two or three fixed slots and latest-wins backpressure. If no slot is available, drop the new capture (or replace the oldest unpublished slot); never block the render thread waiting for the encoder and never queue an unbounded history.
- Remove the listener on the GUI thread before destroying its storage. Track the `RenderWindow` with Qt lifetime notification or a guarded pointer so shutdown cannot call through a dead window.

The panel's `onInitialize()` is called after `VisualizationFrame::addPanelByName()` constructs and docks it. The exact tagged implementation also loads configured panels by class/name and calls `Panel::initialize(manager_)`; see [RViz 15.2.4 `visualization_frame.cpp`](https://raw.githubusercontent.com/ros2/rviz/15.2.4/rviz_common/src/rviz_common/visualization_frame.cpp), lines 850-878 and 1198-1224.

## Plugin build, discovery, and launch

The supported extension point is an ordinary pluginlib `rviz_common::Panel`:

```cpp
PLUGINLIB_EXPORT_CLASS(openarm_rviz_bridge::BridgePanel, rviz_common::Panel)
```

Its plugin XML has a library entry with:

```xml
<class
  name="openarm_rviz_bridge/Bridge"
  type="openarm_rviz_bridge::BridgePanel"
  base_class_type="rviz_common::Panel"/>
```

The CMake package must install the library and XML and invoke:

```cmake
pluginlib_export_plugin_description_file(rviz_common plugins.xml)
```

The official current ROS tutorial confirms the panel base class, export macro, XML, CMake registration, menu loading, and RViz config mechanism: [Building a Custom RViz Panel](https://docs.ros.org/en/rolling/Tutorials/Intermediate/RViz/RViz-Custom-Panel/RViz-Custom-Panel.html).

For Lyrical 15.2.4, direct build dependencies are:

- `ament_cmake`
- `pluginlib`
- `rviz_common`
- `rviz_rendering`
- `rviz_default_plugins`
- `rviz_ogre_vendor`
- Qt 6 Core/Widgets (`CMAKE_AUTOMOC ON` for `Q_OBJECT`)
- `rclcpp` and the chosen command/frame message package if ROS transport is used
- `libjpeg-dev`/`JPEG::JPEG` only if the plugin or companion encodes JPEG

The exact installed CMake targets are:

- `pluginlib::pluginlib`
- `rviz_common::rviz_common`
- `rviz_rendering::rviz_rendering`
- `rviz_default_plugins::rviz_default_plugins`
- `rviz_ogre_vendor::OgreMain`
- `Qt6::Widgets`

`rviz_common` and `rviz_default_plugins` transitively link the relevant Ogre/Qt targets, but code directly including Ogre and Orbit headers should declare its direct packages and targets rather than rely on accidental transitivity. `rviz2` is a runtime dependency, not a library link dependency.

There are two loading methods:

1. Source the overlay containing the installed plugin, launch stock `rviz2`, then choose **Panels -> Add New Panel**. This is the supported manual discovery test.
2. For normal startup, add the following record to the RViz YAML passed with `rviz2 -d`:

   ```yaml
   Panels:
     - Class: openarm_rviz_bridge/Bridge
       Name: Viewer Bridge
   ```

The current launchers can continue executing the installed stock `rviz2`; no replacement executable or `rviz_common` fork is required. They must source the overlay before launch so pluginlib's ament resource index sees the bridge. If a separate encoder/portal companion is used, launch it alongside RViz; it communicates with the loaded plugin through the deliberately narrow transport.

## Readback and 30 FPS assessment

At 30 FPS, raw traffic before scaling/encoding is:

- RGBA: `3,686,400 * 30 = 110,592,000` bytes/s = **105.47 MiB/s**.
- RGB: `2,764,800 * 30 = 82,944,000` bytes/s = **79.10 MiB/s**.

A three-slot RGBA ring consumes **10.55 MiB** for pixel payloads. These numbers are reasonable host-memory bandwidth, but they do not account for render completion stalls, format conversion, resize, codec work, ROS/IPC copies, browser decode, or paint.

The installed Ogre GL shared object imports `glReadPixels`; disassembly of
`Ogre::GLRenderSystem::_copyContentsToMemory()` shows a direct `glReadPixels` call and then `Ogre::PixelUtil::bulkPixelVerticalFlip()` before returning. The installed GL3Plus plugin likewise exposes `gl3wReadPixels` and implements `_copyContentsToMemory()`. Therefore the public path is synchronous CPU-visible readback, not an asynchronous PBO pipeline.

RViz's configured `Frame Rate: 30` causes its Qt update timer to target about 33.3 ms. The listener naturally sees completed rendered frames; it should throttle to at most 30 and use monotonic frame sequence/time. A separate 30 Hz capture timer would risk reading between renders and does not remove the synchronous stall.

Concrete performance conclusion:

- **Hardware GL, this small RobotModel scene:** 30 FPS is plausible, but not proven. The synchronous readback consumes part of the same 33.3 ms frame budget and can force the GPU/CPU to synchronize.
- **This repository's default Wayland/XWayland launcher:** 30 FPS is higher risk because it selects llvmpipe software rendering to avoid hardware-resize flicker. Readback avoids a discrete-GPU transfer but competes for the same CPU and memory bandwidth as rendering and JPEG.
- **Encoding on the render thread:** not acceptable. It will directly lower RViz's update rate.
- **Raw ROS image transport:** about 79-105 MiB/s before middleware copies, so JPEG/video or shared memory should be considered. Encode each source frame once, not once per HTTP request.
- **Async PBO optimization:** not exposed by the public RViz/Ogre capture surface. Raw renderer-specific OpenGL calls from the listener could be explored because this installation is GLX, but that couples the plugin to currently bound GL state/framebuffers and is not a backend-neutral or RViz-supported contract. It must not be assumed in the feasibility claim.

The current portal `<img>` polls every 350 ms (maximum 2.86 FPS), and `RvizCapture` reads the whole X window, calls `XGetPixel` for every pixel, converts to RGB, and encodes a fresh JPEG per request. The Ogre plugin removes those structural costs and captures the correct surface, but only measurement can establish that the new path sustains 30 source-to-painted frames per second.

## Testability and acceptance evidence

The design is testable without modifying RViz, but the render boundary needs a real Qt/Ogre/GL integration test.

### Build/discovery tests

- Compile and link the plugin only against the installed imported targets above.
- Verify the pluginlib ament resource and XML are installed and the declared base is exactly `rviz_common::Panel`.
- Source the install overlay and verify the class appears in RViz's panel factory/manual chooser.
- Load a minimal config containing the panel class and fail clearly if plugin creation returns a `FailedPanel`.

### Unit tests without a GUI

- Put schema validation, finite/range checks, sequencing, rate limiting, coalescing, and latest-frame ring policy behind authored interfaces and unit-test them independently.
- Test exact Orbit delta sign conventions against an adapter/fake: yaw subtraction/wrap, pitch clamp, positive zoom reducing distance, camera-local pan, reset, and rejection when the current class is not Orbit.
- Test bounded-memory behavior under a stalled encoder/consumer; frame sequence may skip but must never regress or produce a backlog.
- Test resize/native-resolution metadata and conversion buffer size/overflow checks.

### RViz/GL integration tests

- Run stock RViz with the plugin-configured YAML and the pinned Stage-A RobotModel on each supported renderer. This host does not currently have an installed RViz visual-test package or Xvfb, so such a test needs the real XWayland/XCB/GLX session or an explicitly provisioned GL-capable CI display.
- Assert the observed target is exactly 1280x720 when native mode is claimed; otherwise assert the output is marked resampled.
- Capture a scene with conspicuous Qt docks/dialogs around it and pixel-compare that none appear in the output.
- Apply typed commands and read back public view properties with `current->subProp("Yaw")`, `"Pitch"`, `"Distance"`, and `"Focal Point"`; verify the configured starting values, signs, clamp, reset, and non-Orbit rejection.
- Delete/reload the panel and exit RViz under ASan/TSan to validate listener lifetime and worker shutdown.

### 30 FPS release gate

For a 60-second warm run at fixed 1280x720, record timestamps/sequences at render completion, readback completion, encode completion, HTTP/IPC receipt, browser decode, and browser paint. Require a declared minimum renderer/host and at least 29.0 newly painted frames/s, bounded source-to-paint latency, no unbounded slots/requests, and no RViz update-rate drop below the declared threshold. Repeat during continuous Orbit commands, robot motion, slow/nonreading client, encoder overload, resize, and reconnect. Report p50/p95/p99 readback and encode times; a static screenshot is not evidence of 30 FPS.

## Contrast with vendored browser WebGL

| Property | RViz render-target plugin | Vendored browser WebGL |
|---|---|---|
| Pixels | Actual stock RViz RobotModel/Ogre rendering and overlays; no Qt chrome. | A separate renderer; cannot truthfully be called RViz pixels. |
| Camera | Stock `OrbitViewController` semantics through exported typed methods. | Browser `OrbitControls`; easy and responsive, but semantics/golden views must be aligned deliberately. |
| 1280x720 | Native only if the live panel is that size; otherwise scale. | Canvas/backing store can be explicitly 1280x720 independent of host RViz layout. |
| 30 FPS | Synchronous readback plus encode/transport/decode; must be proven, especially with llvmpipe. | No host pixel readback/encode; browser/GPU renders directly and receives only joint state, making 30 FPS substantially easier. |
| Fidelity | Automatically follows installed RViz displays, materials, TF, and plugin behavior. | Must parse the pinned derived URDF, resolve/package meshes, apply all 14 measured joints, reproduce materials/transparency/axes, and preserve stale-state truth. |
| Dependencies | Adds a Lyrical ABI-coupled in-process C++ plugin and codec/IPC path; keeps RViz/Ogre/XWayland in portal runtime. | Adds pinned/vendored JS renderer, controls, URDF loader, asset licenses/hashes, and browser GPU memory; can remove portal RViz/XComposite/JPEG runtime. |
| Resource profile | RViz render + 79-105 MiB/s raw readback + codec; ring about 10.55 MiB RGBA. | Tiny pose stream; rasterization stays in browser. High-detail DAE assets may have a much larger browser/GPU working set unless optimized. |
| Security boundary | Narrow typed camera commands are strong; no generic X/VNC input. In-process plugin raises RViz plugin ABI/lifetime risk. | Same-origin canvas controls need no host GUI input. Static assets and WebSocket/HTTP routes require CSP, allowlisting, provenance, and origin checks. |
| Test focus | Renderer/platform/readback/lifetime/performance. | URDF/mesh/joint fidelity, asset provenance, stale state, and browser performance. |

If **actual RViz pixels** are a hard product requirement, the in-process render-target plugin is feasible and preferable to XComposite/VNC/input injection, subject to the native-resolution qualification and measured 30 FPS gate.

If the real requirement is instead **a robot-only interactive 1280x720 view at sustained 30 FPS**, vendored same-origin WebGL is the lower-resource and more deterministic architecture. The repository's architecture reconnaissance measured about 2.4 MiB of unique collision-proxy STL assets versus about 69.2 MiB of unique high-detail DAE visuals, so a browser implementation should start with an explicitly labeled visual proxy or an audited optimized asset pipeline. It must receive the authoritative measured 14-joint snapshot and must not reconstruct state from TCP positions. It also must never be described as embedded RViz.

## Final feasibility statement

No fork of `rviz_common`, replacement RViz executable, generic GUI input, VNC, or XComposite is required. The supported implementation boundary is:

```text
stock rviz2
  + out-of-tree rviz_common::Panel plugin
      - public RenderWindow/Ogre target listener
      - bounded synchronous readback
      - GUI-thread public Orbit delta calls
      - latest-frame handoff
  + optional out-of-process codec/portal companion
```

That boundary is concretely buildable from the installed public Lyrical 15.2.4 headers and shared libraries. It satisfies capture isolation and typed camera control. It does **not**, without measurement, prove native 1280x720 under an unconstrained RViz layout or sustained 30 FPS under the current software-rendering default.
