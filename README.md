# OpenArmIK

OpenArmIK is a C-first, ROS 2 control and visualization stack for a bimanual
OpenArm v1.0. It provides:

- a double-precision C kinematics, inverse-kinematics, collision, routing, CAN,
  commissioning, control, and runtime API;
- a simulated measured-feedback backend for development without hardware;
- a deliberately passive-at-start SocketCAN backend for the physical arms;
- the pinned OpenArm v1.0 URDF and its visual/collision meshes in RViz;
- a loopback-only web portal with an interactive, panel-free RViz view;
- independent, centroid, mirrored, converging/contact, joint, and paired motion;
- live encoder feedback, dynamic route correction, software Stop, E-stop,
  Disconnect, and Return to Neutral; and
- Clap, Cross, Heart, and RViz-only Box pick/place demonstrations.

The two supported entry points are:

```bash
./run.sh       # virtual robot; never opens SocketCAN
./run-real.sh  # physical arms; starts passive with every motor disabled
```

## Safety warning

`run-real.sh` can enable and move real motors after the operator presses
**Connect and enable motors**. Do not use the physical mode until the motor map,
side assignment, joint directions, neutral offsets, CAN wiring, and mechanical
workspace have been checked for the exact robot in front of you.

This project implements conservative collision mitigation, not certified
collision avoidance. Its software E-stop is not a safety-rated or hardwired
E-stop. Keep the hardware power disconnect or hardware E-stop within reach.
Never put a person, cable, tool, table, box, or other unmodeled object inside
the arms' swept volume.

Important behavior:

- Launching `run-real.sh` does **not** enable a motor. The physical controller
  does not open its CAN sockets until Connect is explicitly pressed.
- Connect requires fresh replies from IDs 1 through 8 on both buses and writes
  and reads back a volatile 200 ms drive-side communications timeout on every
  motor before it enables anything. The timeout is refreshed by the 50 Hz
  command stream and is not saved to motor flash.
- Connect seeds every position command from the encoder pose that was just
  measured, sends a zero-gain seed, enables all 16 motors, and ramps gains over
  500 ms. It does not command the upstream driver's automatic return-to-zero.
- Disconnect, E-stop, Ctrl+C, process shutdown, a feedback-watchdog fault, or a
  controller fault repeatedly sends disable commands and polls all 16 status
  replies before closing CAN. Software reports "confirmed disabled" only when
  every motor reports disabled; otherwise it explicitly requires the hardware
  stop. The drive-side timeout is the fallback when an unreachable enabled
  motor cannot receive a host disable frame.
- The normal `run-real.sh` path always retains that 16/16 rule. A separate,
  explicit `active_side_mask=1` or `2` recovery launch may control one arm only
  when the other arm is electrically unpowered and physically outside the
  active arm's swept volume. It never opens the inactive CAN interface and
  rejects every command that would move the inactive side.
- Releasing E-stop never re-enables the arms. The operator must press Connect
  again.
- Stop cancels motion and holds the newest measured pose; it is less forceful
  than E-stop because the motors remain enabled.
- J8 is the gripper motor. The physical controller inventories and watches both
  J8 encoders, configures DaMiao position-force mode at each explicit Connect,
  and supports calibrated open, close, position, and torque-limited grasp
  commands through the ROS action and compiled CLI. The portal does not yet
  expose gripper buttons.
- No motion, including Return to Neutral, is allowed to override an E-stop,
  stale feedback, a motor fault, a joint-limit failure, or an unproved path.

## Supported robot and measured hardware map

The intended robot is OpenArm v1.0: seven revolute arm joints and one DaMiao
gripper motor per arm. On the currently commissioned pair:

- `can0` is robot-right;
- `can1` is robot-left;
- motor IDs 1 through 7 are J1 through J7; and
- motor ID 8 is the parallel gripper.

The official motor/profile map used by the physical session is:

| CAN send ID | Function | Motor | MIT position range | Velocity range | Torque range | Reply ID |
| ---: | --- | --- | ---: | ---: | ---: | ---: |
| 1 | J1 | DM8009 | ±12.5 rad | ±45 rad/s | ±54 Nm | `0x11` |
| 2 | J2 | DM8009 | ±12.5 rad | ±45 rad/s | ±54 Nm | `0x12` |
| 3 | J3 | DM4340 | ±12.5 rad | ±10 rad/s | ±28 Nm | `0x13` |
| 4 | J4 | DM4340 | ±12.5 rad | ±10 rad/s | ±28 Nm | `0x14` |
| 5 | J5 | DM4310 | ±12.5 rad | ±30 rad/s | ±10 Nm | `0x15` |
| 6 | J6 | DM4310 | ±12.5 rad | ±30 rad/s | ±10 Nm | `0x16` |
| 7 | J7 | DM4310 | ±12.5 rad | ±30 rad/s | ±10 Nm | `0x17` |
| 8 | gripper | DM4310 | ±12.5 rad | ±30 rad/s | ±10 Nm | `0x18` |

Every CAN payload ultimately uses the DaMiao protocol's fixed-width quantized
wire representation. Before that final codec boundary, coordinates, joint
angles, calibration transforms, IK/FK, trajectories, path sampling, and
collision distances remain IEEE-754 binary64 C/C++ `double` values. There is no
intentional `double` → `float` → `double` round trip in the coordinate path.

## Coordinate convention and units

Cartesian targets are expressed in `openarm_body_link0`:

- `+X`: forward from the mounting body;
- `+Y`: robot-left;
- `+Z`: upward.

The portal defaults to centimetres. Its unit selector supports centimetres,
inches, and metres. Changing the selector changes presentation, not the stored
target: the browser preserves a binary64 metre value and simply re-renders it.

The public unit-aware C calls take an explicit unit enum and normalize once to
metres. The ROS actions are metric. RViz and the RViz grid always remain metric.

Examples of the same point:

```text
[30.0, 22.0, 30.0] cm
[11.811023622, 8.661417323, 11.811023622] in
[0.30, 0.22, 0.30] m
```

## Repository layout

| Path | Purpose |
| --- | --- |
| `model/` | C11 FK, Jacobian, bounded XYZ IK, collision geometry, exact claw meshes, and paired route planning |
| `can/` | C11 DaMiao codec, profiles, discovery/probe helpers, Linux SocketCAN helpers, and fake transport |
| `transport/` | transport abstraction and query-only SocketCAN backend |
| `commission/` | calibration/commissioning state machines and C facade |
| `control/` | C API with an internal C++ OOP controller implementation |
| `runtime/` | installed runtime facade, units, events, motion contracts, E-stop, identity, and persistence |
| `ros2_ws/src/openarm_ik_ros/` | ROS actions/services, real and virtual sessions, RViz/portal integration |
| `ros2_ws/src/openarm_control_msgs/` | custom ROS actions and calibration service definitions |
| `upstream/openarm_description/` | pinned official OpenArm description checkout, including v1.0 xacro and meshes |
| `calibration/` | hand-range calibration outputs and notes |
| `dread/` | adapted third-party range-calibration workflow for this v1.0 motor map |
| `scripts/` | dependency, build, launch, CAN setup, and verification scripts |
| `OPENARM_V1_RESEARCH.md` | OpenArm v1.0 hardware/software research record |
| `UPSTREAM_SOURCES.md` | pinned upstream commits and licenses |
| `codex.md`, `CLAUDE.md` | detailed implementation handoff and repository traps |

The public implementation is C wherever ROS, browser capture, threading, or
object lifetime does not require C++. The ROS layer and the physical/virtual
session state machines use modular C++ OOP behind the C model/runtime/CAN APIs.
The authored portal, planner, controller, and `run-real.sh` runtime path do not
use Python; ROS build tools such as xacro and rosidl still use their normal
Python packages while generating artifacts.

## Dependencies

The target host is x86-64 Ubuntu with ROS 2 Lyrical installed under
`/opt/ros/lyrical`. RViz can use the laptop's accelerated graphics stack; the
launcher defaults to the integrated renderer because it has been more reliable
than forcing NVIDIA PRIME on this machine.

Review dependency installation without changing the system:

```bash
./scripts/install_all_dependencies.sh
```

Install the required apt packages:

```bash
./scripts/install_all_dependencies.sh --apply
```

That is the only dependency operation that may require sudo. Build and runtime
processes must remain unprivileged. `run-real.sh` refuses to run as root.

Fetch or refresh the pinned open-source upstreams when network access is
available:

```bash
./scripts/fetch_upstreams.sh
```

## Build and test

Build the native libraries and all ROS packages:

```bash
./scripts/build.sh
```

Run the full hardware-free build/test gate:

```bash
./scripts/build.sh --tests
```

Useful options:

```text
--incremental       reuse compatible compiler caches
--jobs N            bound build/test parallelism; default is 2
--build-type TYPE   CMake build type; default is Release
--output-root PATH  isolate build, log, install, and test output
```

The top-level test build covers the native C/C++ unit tests, C11/C++17 public
headers and installed consumers, ABI canaries (including the production real
controller's pure-C client), IK/FK/collision/routing,
commissioning, transport, virtual measured-feedback control, display
calibration, passive physical-session lifecycle and E-stop, portal guard logic,
URDF generation, ROS actions, Ctrl+C cleanup, RViz capture, and launcher
contracts. Hardware movement is deliberately not part of an unattended CTest.

To rerun the already-built ROS adapter tests directly:

```bash
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
ctest --test-dir ros2_ws/build/openarm_ik_ros --output-on-failure
```

## Virtual operation

From the repository root:

```bash
./run.sh
```

This incrementally builds when needed, starts the virtual measured-feedback
controller and `robot_state_publisher`, starts a panel-free RViz window,
captures the raw RViz render into the portal, and opens Firefox at
`http://127.0.0.1:8080/`.

The virtual mode never opens PF_CAN/SocketCAN and cannot move the physical
robot. It is the correct place to test targets, unit conversion, motion speed,
route behavior, demos, and the portal UI.

Common options:

```bash
./run.sh --no-browser
./run.sh --no-rviz
./run.sh --port 8081
./run.sh --jobs 4
./run.sh --no-build
```

`--no-build` is fail-closed: it launches only if the installed tree's source
fingerprint, pinned description, package prefix, and artifact hashes match the
current checkout.

Press Ctrl+C in the launch terminal to stop the portal, RViz, and ROS process
groups. The launcher first asks RViz to close through X11, then terminates the
portal and sends SIGINT to ROS, avoiding the known Ogre teardown crash path.

## Physical operation

### 1. Prepare the workspace

- Place both arms where an unexpected hold at the current pose cannot hit a
  person or object.
- Keep the hardware stop or power disconnect within reach.
- Confirm there is no virtual/real OpenArm launcher already running.
- Confirm the saved neutral calibration belongs to this exact pair.

The active calibration file is:

```text
~/.openarm_real_zero
```

It stores, independently for robot-left/right J1 through J7:

```text
model_angle = direction * (encoder_angle - reference_angle) + offset_angle
```

The inverse used for physical command targets is exact binary64:

```text
encoder_target = reference_angle + direction * (model_target - offset_angle)
```

### 2. Bring up CAN

The privileged helper configures the interfaces; it does not enable a motor:

```bash
bash scripts/setup_can_interfaces.sh
```

Status-only and shutdown forms:

```bash
bash scripts/setup_can_interfaces.sh --status
bash scripts/setup_can_interfaces.sh --down
```

Default link settings are 1 Mbit/s arbitration, 5 Mbit/s CAN-FD data, and a
1000-frame transmit queue. They can be overridden for differently commissioned
hardware with `OPENARM_CAN_BITRATE`, `OPENARM_CAN_DBITRATE`, and
`OPENARM_CAN_TXQUEUELEN`.

### 3. Launch passively

```bash
./run-real.sh
```

The page opens with a **PASSIVE** overlay. At this point the controller has not
opened CAN and all motors remain disabled. RViz may show the calibrated neutral
pose until an active encoder stream exists.

### 4. Connect and enable

Press **Connect and enable motors** only after the workspace is clear.

The service transition performs this sequence:

1. Reject if E-stop is latched or calibration is missing.
2. Open `can1` for robot-left and `can0` for robot-right.
3. Install exact receive filters for replies `0x11` through `0x18`.
4. Request fresh status from all 16 motors.
5. Decode each J1–J7 encoder through its saved direction/reference/offset.
6. Reject nonfinite values or values outside the pinned URDF limits.
7. Evaluate the measured bimanual pose with the collision model.
8. Seed all J1–J7 MIT position targets from those measured encoders.
9. Write and confirm the volatile 200 ms communications timeout on IDs 1--8 of
   both arms.
10. Configure and confirm J8 position-force mode on both arms, then seed each J8
   target from its measured gripper encoder.
11. Send the measured targets at zero gain, enable all motors, and ramp gains
    over 50 control cycles.
12. Enter the 50 Hz measured-feedback hold loop.

The button then becomes **Disconnect and disable motors**. Pressing it cancels
any motion, repeatedly disables IDs 1 through 8 on both buses, polls their
reported states for up to three seconds, and closes both sockets. An
unconfirmed result is a fault requiring the hardware stop, not a successful
disconnect claim.

### Explicit single-arm recovery mode

Single-arm recovery is not the normal portal mode and must not be used merely
to hide a powered or mechanically dangerous fault. First remove motor power
from the inactive arm and place it outside the active arm's swept volume. Then
launch the controller directly with one active-side bit:

```bash
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash

# 1 = robot-left/can1 only; 2 = robot-right/can0 only; 3 = normal bimanual
ros2 launch openarm_ik_ros openarm_real.launch.xml \
  rviz:=false active_side_mask:=1
```

In left-only mode the controller opens only `can1`, inventories/configures/
enables only left IDs 1 through 8, and closes only that socket. Right-arm,
paired, centroid, mirrored, converge, and Return-to-Neutral commands are
rejected with `command_involves_inactive_arm`. A single left TCP request remains
collision routed with the inactive right model fixed at its neutral placeholder;
that placeholder is not live right-arm encoder data.

The normal `./run-real.sh` launch always uses mask `3` and keeps the strict
16-motor preflight.

### 5. Stop and E-stop

- **Stop** cancels the active ROS action or Return-to-Neutral command. The
  controller updates the target to the current encoder pose and continues an
  enabled hold.
- **EMERGENCY STOP** atomically latches before waiting for any controller lock,
  cancels motion, sends repeated disable frames to all 16 motors, and attempts
  to confirm every disabled status before closing both CAN sockets.
- **Release E-stop** only clears the latch. Motors remain disabled until a new
  explicit Connect.
- Ctrl+C and process shutdown disable before exit.
- A 100 ms complete-feedback watchdog enters the same confirmed-disable path.
  Each motor also receives a volatile 200 ms firmware communications timeout
  at Connect so loss of the process or an unreachable drive fails toward
  disabled without depending on another host frame.

If the UI or process is unresponsive, use the hardware stop. A browser button
cannot be made equivalent to a safety-rated electrical disconnect.

## Portal controls

The portal contains:

- live left/right claw XYZ readouts;
- left and right target fields;
- individual **Move left** and **Move right** buttons;
- **Move both** for one atomic paired request;
- centimetre/inch/metre display and input;
- a 50–100% movement-limit slider, defaulting to 100%;
- Stop, Verify, Connect/Disconnect, Return to Neutral, E-stop, and E-stop
  release;
- field-fill presets and complete demo-sequence buttons; and
- a real RViz pixel stream that supports mouse orbit, zoom, and pan.

Preset buttons only fill fields; they do not submit movement. Review the values
and press the appropriate Move button. Demo-sequence buttons do submit every
step in sequence and wait for measured completion before continuing.

The movement-limit slider scales the velocity, acceleration, and jerk limits
together. It is not a direct duration percentage. The physical trajectory uses
a synchronized seventh-order position curve, so velocity, acceleration, and
jerk start and end smoothly instead of commanding a step.

## Useful virtual XYZ targets

These targets come from the pinned OpenArm v1.0 model and are useful for
simulation. They are not a certification that the same target is safe on a
physical robot.

| Preset | X (cm) | Left Y (cm) | Right Y (cm) | Z (cm) | Purpose |
| --- | ---: | ---: | ---: | ---: | --- |
| Near low | 15 | 22 | -22 | 15 | compact low pose |
| Outer low | 15 | 40 | -40 | 15 | low lateral sweep |
| Near mid | 15 | 22 | -22 | 30 | compact mid-height |
| Outer mid | 15 | 40 | -40 | 30 | separated mid-height |
| Forward mid | 30 | 22 | -22 | 30 | ordinary forward motion |
| Forward outer | 30 | 50 | -50 | 30 | forward/lateral reach |
| Near-max forward | 48 | 17 | -17 | 35 | forward reach test |
| Outer high | 25 | 58 | -58 | 45 | high lateral reach |
| High far | 28 | 67 | -67 | 52 | audited near-full reach |

`High far` is intentionally a large movement. The pinned URDF's approximate
shoulder-to-TCP centreline reach is 74.7–74.8 cm; this pose exercises more than
89% of that geometric upper bound from the shoulder while retaining the
sampled planning clearance in the virtual model.

Virtual guard regression inputs, in centimetres:

| Test | Left XYZ | Right XYZ | Expected behavior |
| --- | --- | --- | --- |
| impossible reach | `[5000, 5000, 5000]` | `[5000, -5000, 5000]` | best reachable guarded prefix, never a 50 m command |
| pole keepout | `[40, 5, 40]` | `[40, -5, 40]` | stop/project before the central pole planning gate |
| inter-arm | first move both to Near-max forward, then request left `[48, -17, 35]` | hold right | shorten before the arms violate clearance |

Single-arm impossible XYZ requests use a sampled farthest-proved prefix on the
requested ray. A paired target remains exact and all-or-nothing: the planner
does not silently distort one side to make the pair succeed.

## Demos

### Clap

Moves through a guarded entry pose, converges the two claws using the scoped
terminal claw corridor, stops from measured mesh/contact evidence, and retreats
to an open pose. The rails/hand housings may enter the special terminal
envelope, but no STL intersection is authorized.

### Cross

Changes arm heights before crossing, moves through the swapped-claw waypoint,
then reverses the route. Dynamic routing prevents the direct locked/colliding
transition that previously trapped the arms after Cross.

### Heart

Traces the upper lobes, shoulders, lower point, and two contact cusps. Contact
poses use the same scoped terminal policy as Clap and are followed by a
monotonic, validated retreat.

### Box pick/place

The box is a visualization marker in RViz only. There is no physical box. The
real arms mimic the approach, lift, carry, place, and retreat trajectory in
empty space. There is no automatic assumption that an unmodeled physical box
is present; use an explicit torque-limited gripper command when commissioning a
real grasp. The marker is deleted in a `finally` path after success, rejection,
cancellation, timeout, or E-stop, so it does not remain in later scenes.

## Return to Neutral

**Return both arms to Neutral** targets the saved calibrated model-zero pose,
not a guessed visual angle. It:

1. computes the neutral TCP from the pinned model;
2. finds a collision-aware paired route from current encoder-derived joints;
3. validates every route edge;
4. validates the final joint-space edge at no more than 0.02 rad between
   collision samples; and
5. appends the exact all-zero J1–J7 pose, avoiding an IK-equivalent but
   non-neutral redundant posture.

Each executed route segment starts from the newest measured encoder pose.
During execution, the controller recomputes target interpolation continuously,
checks measured geometry every 20 ms, and stops a worsening approach. It never
blindly assumes that the arm reached the previously calculated waypoint.

No planner can honestly guarantee a route under every physical circumstance.
If the current state is inside the 25 mm planning envelope but outside the
10 mm live intervention envelope, only a monotonically improving recovery edge
is accepted. If no safe route is proved, Neutral is rejected and the arms hold
their measured pose.

## Collision and path mitigation

There are two distinct thresholds:

- a 25 mm planning clearance for nominal path acceptance; and
- a 10 mm live intervention floor evaluated from measured feedback.

The C route planner `oa_route_plan_paired()` searches a graph of audited
bimanual connectivity anchors. Anchor membership never authorizes motion:
every candidate edge is independently sampled through position IK, FK, URDF
limits, conservative arm/tool geometry, the central mounting pole, and the
paired clearance evaluator.

The portal replans the remaining route from fresh measured state after each ROS
action leg. The physical session also reseeds every internal trajectory leg
from the encoder state that was actually reached. During each 50 Hz physical
cycle, measured FK is collision checked again. If an arm is already between
the planning and intervention thresholds, it may move only while clearance is
monotonically improving; once it recovers the planning threshold, it may not
re-enter it.

The gripper rails are not approximated solely as points at the claw TCP. The
model embeds the pinned OpenArm v1.0 hand and finger triangle meshes and checks
cross-claw mesh pairs. Clap and Heart use a narrowly scoped terminal-contact
policy for the intended approaching pair; all other mesh pairs, arm links, and
the pole retain ordinary clearance. No STL overlap is permitted, and additional
clearance is retained for encoder/solver and physical-model mismatch.

Limitations:

- no camera/environment obstacle map;
- no safety certification;
- no guarantee that printed/machined parts exactly match the pinned mesh;
- no proof against loose cables or an object introduced after planning;
- no force/torque sensor at the claw; convergence and gripper contact use mesh
  evidence and persistent motor-reported torque rather than a calibrated
  fingertip force sensor; and
- gripper swept-volume collision is represented by the conservative fully-open
  hand/finger envelope; the gripper action itself does not plan around external
  objects.

## C interfaces

Primary headers:

```text
model/include/openarm_model.h
model/include/openarm_collision.h
model/include/openarm_route.h
can/include/openarm_can.h
commission/include/openarm_commission.h
control/include/openarm_control.h
runtime/include/openarm_runtime.h
runtime/include/openarm_runtime_units.h
runtime/include/openarm_runtime_motion.h
ros2_ws/src/openarm_ik_ros/include/openarm_real.h
```

Important model calls:

- `oa_fk()`
- `oa_geometric_jacobian()`
- `oa_ik_position()` / `oa_ik_position_v2()`
- `oa_ik_position_with_units()`
- `oa_model_limits()`
- `oa_collision_evaluate_scoped_fk_with_threshold()`
- `oa_route_plan_paired()`

Important high-level control calls:

- `oa_controller_plan_joint()`
- `oa_controller_plan_paired_tcp()`
- `oa_controller_plan_paired_tcp_with_units()`
- `oa_controller_plan_centroid_tcp_with_units()`
- `oa_controller_plan_mirrored_tcp_with_units()`
- `oa_controller_plan_converge_tcp_with_units()`
- `oa_controller_execute()` / `oa_controller_advance()`
- `oa_controller_stop()` / `oa_controller_disarm()`
- `oa_estop_assert()` / `oa_estop_clear()`

Important C11 gripper/CAN calls:

- `oa_can_gripper_motor_position()` / `oa_can_gripper_opening()`
- `oa_can_encode_gripper_move()`
- `oa_can_encode_gripper_open()` / `oa_can_encode_gripper_close()`
- `oa_can_encode_gripper_grasp()`

The unit-aware calls accept metres, centimetres, or inches while keeping the
calculation path in `double`. See [control/README.md](control/README.md) and
[model/README.md](model/README.md) for struct initialization, ABI versioning,
ownership, and complete examples.

### Production physical C ABI

`openarm_real.h` is the public C11 boundary to the controller started by
`run-real.sh` or the explicit single-arm launch. `libopenarm_real.so` does not
open SocketCAN, run sudo, or become a second motor authority; it sends lifecycle
services and actions to the sole ROS physical controller, so inventory,
calibration, drive timeout, encoder watchdog, E-stop, collision checks, and
confirmed disable remain in force.

Core calls are:

- `oa_real_client_create()` / `oa_real_client_destroy()`;
- `oa_real_client_wait_ready()` / `oa_real_client_read()`;
- `oa_real_client_connect()` / `oa_real_client_disconnect()`;
- `oa_real_client_stop()` / `oa_real_client_estop()`;
- `oa_real_client_move_joint()`;
- `oa_real_client_move_tcp()` / `oa_real_client_move_paired_tcp()`; and
- `oa_real_client_move_gripper()`.

All coordinate, joint, speed, torque, FK, and IK values at this boundary use C
`double`. Cartesian calls require an explicit `oa_length_unit`; conversion to
metres occurs once. `oa_real_snapshot.active_side_mask` reports which arms are
live (`1` left, `2` right, `3` both). An inactive side's snapshot values are a
fixed planning placeholder, not encoder measurements. Destroying a client does
not disconnect a shared controller, so an application must explicitly stop or
disconnect when that is its lifecycle responsibility.

Minimal left-arm Cartesian example:

```c
#include <openarm_real.h>

int main(void) {
    oa_real_client *client = NULL;
    oa_real_result result;
    const oa_vec3d target_cm = {28.0, 67.0, 52.0};

    oa_real_result_init(&result);
    if (oa_real_client_create(&client) != OA_REAL_OK) return 1;
    if (oa_real_client_wait_ready(client, 5000U, &result) != OA_REAL_OK) return 2;
    if (oa_real_client_connect(client, 15000U, &result) != OA_REAL_OK) return 3;
    if (oa_real_client_move_tcp(client, OA_REAL_SIDE_LEFT, &target_cm,
          OA_LENGTH_UNIT_CENTIMETRES, 0.5, 120000U, &result) != OA_REAL_OK) {
        (void)oa_real_client_disconnect(client, 10000U, &result);
        oa_real_client_destroy(client);
        return 4;
    }
    (void)oa_real_client_disconnect(client, 10000U, &result);
    oa_real_client_destroy(client);
    return 0;
}
```

CMake consumers use `find_package(openarm_ik_ros REQUIRED)` and link
`openarm_ik_ros::openarm_real_c`. The installed pure-C diagnostic client exposes
the same calls without Python:

```bash
ros2 run openarm_ik_ros openarm_real_cli ready
ros2 run openarm_ik_ros openarm_real_cli read
ros2 run openarm_ik_ros openarm_real_cli connect
ros2 run openarm_ik_ros openarm_real_cli tcp left cm 28 67 52 0.5
ros2 run openarm_ik_ros openarm_real_cli disconnect
```

On 2026-08-07, the C ABI passed a physical left-J4 measured-feedback move and
return while every reported right joint remained unchanged. With the faulty
right J2 electrically isolated, left-only mode then moved the left TCP from
approximately `[2.22, 12.96, 7.81] cm` to `[28.47, 61.79, 40.75] cm`. The two
near-limit `[28, 67, 52] cm` requests ended with measured-feedback settle
timeouts (`~0.030 rad` remaining), so that measured pose—not the requested
endpoint—is the accepted hardware result. The controller remained collision
checked and held the measured endpoint; this test is evidence for this robot,
not a safety certification or a reach guarantee for another unit.

## ROS interfaces

Motion actions:

```text
/openarm_ik/move_joint
/openarm_ik/move_paired_tcp
/openarm_ik/move_paired_tcp_scaled
/openarm_ik/move_bimanual
/openarm_ik/move_gripper
```

Physical lifecycle services:

```text
/openarm_real/connect
/openarm_real/disconnect
/openarm_real/stop
/openarm_real/estop
/openarm_real/estop_clear
/openarm_real/neutral
/openarm_real/capture_grippers_closed
```

State and diagnostics:

```text
/joint_states
/openarm_ik/diagnostics
/openarm_real/status
/openarm_ik/scene_box_enabled
```

The same action servers are used in virtual and physical mode. In physical
mode they dispatch to the SocketCAN session; in virtual mode they dispatch to
the simulated measured-feedback runtime. One reject-new arbiter prevents
concurrent action ownership.

Example CLI use from another terminal:

```bash
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash

ros2 run openarm_ik_ros openarm_control_cli status
ros2 run openarm_ik_ros openarm_control_cli move-joint openarm_left_joint4 0.2
ros2 run openarm_ik_ros openarm_control_cli move-paired-tcp \
  openarm_body_link0 0.30 0.22 0.30 0.30 -0.22 0.30
ros2 run openarm_ik_ros openarm_control_cli gripper open both 0.5 0.0044
ros2 run openarm_ik_ros openarm_control_cli gripper close both 1.0 0.0044
ros2 run openarm_ik_ros openarm_control_cli gripper grasp both 0.35 0.003
```

The CLI XYZ arguments are metres. Action completion is based on encoder-derived
measured position and velocity settling, not elapsed-time estimation.

Gripper opening is also metres and remains binary64 until the final DaMiao wire
codec. The calibrated URDF range is 0 m closed to 0.044 m per source finger.
The optional torque and speed arguments are motor torque in N·m and finger
opening speed in m/s. `grasp` closes with the supplied torque cap and stops only
after persistent motor-reported torque contact or the measured closed endpoint;
this is not a calibrated fingertip-force measurement. The commissioned no-load
full-stroke tests needed 0.5 N·m to open and 1.0 N·m to close robot-right.

The one-time closed reference must be captured only with both empty claws
physically closed, a clear workspace, and the controller connected but idle:

```bash
ros2 service call /openarm_real/capture_grippers_closed std_srvs/srv/Trigger '{}'
```

It atomically writes `~/.openarm_real_gripper`; reconnecting loads that exact
binary64 reference and does not silently recalibrate it.

## Portal HTTP interfaces

The server binds only to loopback and requires its per-page CSRF token for
mutating requests. Core endpoints include:

```text
GET  /api/health
GET  /api/state
GET  /api/view-state
GET  /api/real/status
GET  /api/rviz/stream
POST /api/rviz/input
POST /api/v3/move
POST /api/v3/move-both
POST /api/v3/converge
POST /api/v3/retreat
POST /api/stop
POST /api/estop
POST /api/estop/release
POST /api/real/connect
POST /api/real/disconnect
POST /api/real/neutral
POST /api/scene-box
```

`/api/v3/move` carries an explicit `unit` plus binary64 numeric XYZ and
`motion_limit_scale`. `/api/state` always reports canonical metres. The portal
does not use browser-rendered robot geometry for feedback or collision
decisions; it uses ROS encoder-derived state, while the RViz stream remains a
visual/operator surface.

## URDF and mesh locations

Authoritative upstream v1.0 xacro:

```text
upstream/openarm_description/assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro
```

Generated bimanual model URDF after build:

```text
ros2_ws/install/share/openarm_model/openarm_v10_bimanual.urdf
ros2_ws/install/openarm_ik_ros/share/openarm_ik_ros/urdf/openarm_v10_bimanual.urdf
```

Both real and virtual launches use the full URDF with movable v1.0 finger
joints. Real mode derives them from calibrated J8 encoders; virtual mode derives
them from the simulated measured-gripper state. The generated fixed-finger
Stage-A URDF remains as an explicitly unmeasured compatibility artifact, not the
normal RViz launch description.

Upstream v1.0 meshes are under:

```text
upstream/openarm_description/assets/robot/openarm_v1.0/mesh/
```

The portal's pinned/allowlisted installed viewer mesh closure is under:

```text
ros2_ws/install/openarm_ik_ros/share/openarm_ik_ros/viewer/mesh/
```

The URDF, mesh manifest, copied Apache-2.0 license, and file hashes are checked
by the build and launch-integrity tests.

## Calibration workflows

The saved per-joint display/control calibration is the authoritative mapping
for `run-real.sh`. Adjustments are binary64 and atomically persisted. Historical
backups are retained when recalibrating.

`dread.sh` is a separate powered, hand-guided hard-stop/range workflow. It uses
`dread/config/motor_map.openarm_v10.yaml`, not the old M1/Ranger map. It can
compliant-enable a motor at zero stiffness/damping/torque so its encoder updates
while the joint is back-driven. That means DREAD **does energize motors** even
though it does not command a pose.

Do not run `dread.sh` and `run-real.sh` together. Do not use `set_zero.sh` unless
you intend to write a persistent mechanical zero into motor firmware.

See:

- [calibration/README.md](calibration/README.md)
- [dread/README.md](dread/README.md)
- [dread/docs/CALIBRATION.md](dread/docs/CALIBRATION.md)

## RViz behavior

The web portal shows raw, unfiltered RViz pixels. It does not apply a blue
filter. The panel-free `openarm_bare.rviz` layout removes the Displays and Views
menus from the embedded/captured window. Use `scripts/launch_rviz.sh` for the
full engineering layout.

Browser mouse input is replayed into the real RViz render widget:

- left drag: orbit;
- mouse wheel: zoom;
- middle/right drag: RViz pan/orbit behavior, depending on active view control.

The capture loop targets interactive frame rates; actual FPS depends on X11,
GPU/GLX, RViz rendering cost, browser decode, and window visibility. If RViz is
black, verify that an RViz window exists, it is not minimized, and the renderer
selected by `scripts/lib/rviz_env.sh` can create a GLX context.

## Troubleshooting

### `colcon` reports stderr

Warnings are not automatically failures. Find the first actual `error:` line
in the package log, or run:

```bash
./scripts/build.sh --tests --incremental --jobs 2
```

The top-level wrapper preserves the failing command and exit code. Do not run a
second build against the same output tree while a launcher holds its lease.

### CAN interfaces are missing or down

```bash
ip -br link show
bash scripts/setup_can_interfaces.sh --status
bash scripts/setup_can_interfaces.sh
```

If an interface enters bus-off or reports zero replies, stop, remove power from
the arms, and inspect CAN-H/CAN-L/ground/termination before retrying.

### Connect says feedback is incomplete

Connect requires all reply IDs `0x11`–`0x18` on both buses. A red LED only
proves power, not valid communication. Do not bypass the normal 16/16 preflight
while both arms are powered. The explicit single-arm recovery mode is permitted
only after the inactive arm is electrically isolated and physically clear.
Inspect traffic without enabling movement using the status/refresh tooling in
`can/` or the passive observer utilities.

### The arms jump when Connect is pressed

Press the hardware stop immediately. The intended implementation seeds the
measured raw encoder position before enable. A jump means the calibration,
direction, side map, motor profile, firmware mode, or received encoder value is
wrong; do not reconnect until diagnosed.

### RViz pose is offset

Do not estimate commanded motion from the screen. RViz is driven from live
encoder feedback through `~/.openarm_real_zero`. Verify robot-left/right, then
direction and offset one joint at a time with the arms disabled and relaxed.

### RViz flickers during resize

The captured portal RViz window is not meant to be manually resized. The
launcher uses a fixed panel-free capture window. If a separate engineering RViz
window flickers only while resizing, stop resizing before evaluating pose; the
render should remain stable at rest.

### Portal shows a black RViz square

Check that RViz is running and visible:

```bash
pgrep -a rviz2
```

Then relaunch with the default integrated renderer. Avoid forcing NVIDIA PRIME
when the NVIDIA kernel/GLX driver is not loaded.

### The box remains after a failed demo

The current sequence deletes it in `finally`. If an old installed portal is
still running, Ctrl+C it and rebuild/relaunch. A stale transient-local marker
can also be cleared by toggling the scene box endpoint off or restarting the
single ROS stack.

### Ctrl+C does not exit

Run only one launcher. The scripts hold a per-user GUI lock and use separate
process groups. If a previous extension/window crashed, inspect only processes
whose command line points inside this repository before terminating them.

## Source provenance and licenses

OpenArm upstreams remain full git clones under ignored `upstream/`; this
repository records the exact audited commits and license files in
[UPSTREAM_SOURCES.md](UPSTREAM_SOURCES.md). The generated URDF and allowlisted
viewer assets are derived from the pinned official `openarm_description` source.

The repository's own code uses SPDX license identifiers. Preserve upstream
notices when redistributing URDFs, meshes, or copied source.

## Current verification boundary

Automated tests can prove deterministic math, state-machine behavior, message
contracts, passive startup, E-stop logic, route sampling, and teardown. They
cannot prove that a physical arm was assembled to the mesh, that an encoder was
zeroed correctly, that a cable will stay clear, or that a human will not enter
the workspace. Physical validation must therefore proceed incrementally:

1. passive 16/16 inventory;
2. Connect while already at the measured pose;
3. Stop and E-stop response;
4. tiny single-joint motion;
5. small symmetric paired XYZ motion;
6. Return to Neutral;
7. ordinary target presets; and only then
8. Cross, Box, Clap, and Heart with the workspace empty.

Record any physical discrepancy in `codex.md` before changing calibration or
collision geometry so a future session can distinguish a software regression
from a different robot state.

As of 2026-08-07, robot-right J2 (`can0`, command ID 2, feedback `0x12`)
reproducibly loses feedback during the Cross stack transition while both CAN
interfaces remain ERROR-ACTIVE with zero bus errors. After one such dropout it
later replied with enabled status, proving that transmitting disable frames was
not itself confirmation. It was subsequently disabled and verified by status.
Do not run further powered demo or range acceptance until that motor/controller
and its CAN/power harness are repaired or replaced. At the end of this
checkpoint the operator hand-placed both disabled arms in neutral; no
calibration reference was captured or changed.
