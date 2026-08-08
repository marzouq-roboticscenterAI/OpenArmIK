# OpenArm virtual and physical control ROS adapter

`openarm_ik_ros_node` selects either the virtual measured-feedback session or
the physical SocketCAN session with the `physical_hardware` parameter. It is
the sole `/joint_states` authority in either launch and publishes fourteen
J1--J7 arm joints plus the two measured source finger joints from one coherent
state. It publishes no TF;
`robot_state_publisher` remains the only TF authority.

Both RViz launches use the canonical dynamic
`openarm_v10_bimanual.urdf` with movable source finger joints and mimic joints.
The real values come from calibrated motor-8 encoders; the virtual values come
from the simulated measured-gripper state. `robot_state_publisher` derives the
two mimic transforms. The fixed-finger Stage-A description remains installed
only as an explicitly unmeasured compatibility artifact and must not be used as
a dynamics, calibrated-gripper, MoveIt, or collision-safety model.

The production command interfaces include `/openarm_ik/move_joint`,
`/openarm_ik/move_paired_tcp`, `/openarm_ik/move_paired_tcp_scaled`, and
`/openarm_ik/move_gripper` from `openarm_control_msgs`. Paired goals
name `left_tcp_m` and `right_tcp_m` and are transformed transactionally from
their stamped source frame into `openarm_body_link0`. The scaled variant also
requires a binary64 movement-limit scale in `[0.5, 1.0]`; the original paired
action remains fixed at `0.5`. One reject-new arbiter spans all actions and the
deprecated `/openarm_ik/paired_xyz` compatibility topic. Cancel stops the
active command from measured feedback and returns ownership to the idle state.

The virtual lower session consumes installed `OpenArm::Runtime` as its sole state,
motion, model/TCP identity, clock, event, and plan authority. Runtime owns the
virtual progression cadence; ROS only polls new measured generations and
heartbeats active commands. The backend is fixed to the canonical OpenArm v1.0
bimanual virtual manifest. The physical lower session uses the C DaMiao codec
over exact-ID SocketCAN sockets. Construction is passive;
`/openarm_real/connect` requires fresh IDs 1--8 on both arms, seeds all targets
from encoder feedback, writes and confirms a volatile 200 ms drive-side
communications timeout on all 16 motors, enables, and ramps gains. Stop holds
measured pose; Disconnect, E-stop, shutdown, and the feedback watchdog send
repeated disable frames and keep CAN open long enough to confirm all 16
disabled status replies. An unreachable motor is reported as unconfirmed and
requires the hardware stop; the motor timeout is the independent fallback.
Healthy operation remains WARN in both modes because the collision mitigation
is not safety certification. Virtual results remain `collision_checked=false`;
physical results record the sampled and live checks that were actually run.
Diagnostics report the exact runtime capabilities,
coordinate/model/TCP/scene digests, immutable-manifest authentication and
persistence state, exact virtual inventory, and the fact that runtime
calibration capabilities have no ROS endpoint.

Use the compiled client after sourcing the ROS workspace:

```bash
ros2 run openarm_ik_ros openarm_control_cli status
ros2 run openarm_ik_ros openarm_control_cli move-joint openarm_left_joint4 0.2
ros2 run openarm_ik_ros openarm_control_cli move-paired-tcp \
  openarm_body_link0 0.20 0.30 0.85 0.20 -0.30 0.85
ros2 run openarm_ik_ros openarm_control_cli gripper open both 0.5 0.0044
ros2 run openarm_ik_ros openarm_control_cli gripper close both 1.0 0.0044
ros2 run openarm_ik_ros openarm_control_cli gripper grasp both 0.35 0.003
```

All six `move-paired-tcp` XYZ arguments are metres; the legacy ROS CLI does not
perform portal display-unit conversion.

The installed `openarm_real.h` / `libopenarm_real.so` pair is the public C11
client ABI for the production physical controller. It never opens SocketCAN;
the ROS session remains the sole motor authority. It exposes readiness/state,
Connect/Disconnect/Stop/E-stop/Neutral, absolute joints, unit-aware single and
paired TCP motion, and torque/speed-limited gripper motion. All numerical motion
values are binary64 `double`. CMake consumers link
`openarm_ik_ros::openarm_real_c`; `openarm_real_cli` is a pure-C reference
consumer.

`oa_real_snapshot.active_side_mask` identifies live encoder sides (`1` left,
`2` right, `3` both). The normal `run-real.sh` mode is always `3`. When a failed
arm is electrically unpowered and physically clear, an explicit recovery launch
may select only the other bus:

```bash
ros2 launch openarm_ik_ros openarm_real.launch.xml \
  rviz:=false active_side_mask:=1
ros2 run openarm_ik_ros openarm_real_cli connect
ros2 run openarm_ik_ros openarm_real_cli tcp left cm 28 67 52 0.5
```

Single-arm mode rejects all commands involving the inactive arm and never opens
its CAN interface. Its inactive-side joint/TCP values are fixed collision-model
placeholders, not encoder feedback. See the repository-root `README.md` for the
full ABI example, physical acceptance record, and safety boundary.

Gripper positions and speeds are metres and metres/second; torque is N·m at
the J8 motor. Real mode requires the binary64 closed reference in
`~/.openarm_real_gripper`. Capture it only when both empty claws are physically
closed and the connected controller is idle:

```bash
ros2 service call /openarm_real/capture_grippers_closed std_srvs/srv/Trigger '{}'
```

`grasp` uses DaMiao position-force mode and stops after persistent
motor-reported torque contact or the measured closed endpoint. This is a torque
cap, not a calibrated fingertip-force guarantee.

The CLI returns success only after the matching action reaches measured
completion. `/use_sim_time=true`, nonfinite targets, unknown names, invalid or
stale stamps, missing transforms, unreachable targets, and concurrent commands
are rejected.

The physical session and legacy passive observer share a persistent binary64
per-joint calibration mapping:

```text
displayed = direction * (encoder - relaxed_reference) + display_offset
```

`direction` and `display_offset` are independent for robot-left/right J1--J7.
In the passive observer it affects only `/joint_states`. In the physical
controller its exact inverse also generates encoder targets. Calibration
utilities must be used only while the motors are disabled. The legacy observer
client supports:

```bash
ros2 run openarm_ik_ros openarm_display_calibration_cli capture-relaxed
ros2 run openarm_ik_ros openarm_display_calibration_cli query robot-left 1
ros2 run openarm_ik_ros openarm_display_calibration_cli flip robot-left 1
ros2 run openarm_ik_ros openarm_display_calibration_cli offset robot-left 1 -5.0
ros2 run openarm_ik_ros openarm_display_calibration_cli set-current robot-left 1 0.0
```

`offset` adds a signed number of degrees. `set-current` assigns an exact
displayed angle to the latest encoder reading. Calibration is saved atomically
and reconnecting does not silently replace the relaxed reference.

Run the local compiled portal from the repository root with `./run.sh`. The
launcher builds the workspace, starts the virtual controller and a bare RViz
window, and opens the loopback-only `127.0.0.1` portal. The page embeds cropped
RViz render pixels as an MJPEG stream. Drag, right-drag, middle-drag, and wheel
events are replayed into the RViz render widget for orbit, zoom, and pan. This
view is not collision checking and is never used as control feedback. The
portal always shows the unfiltered RViz output. Use `scripts/launch_rviz.sh` for the
separate full engineering view.

Use `./run-real.sh` for hardware. It starts the same portal/RViz surface with
`physical_hardware=true`, but remains passive until **Connect and enable
motors** is pressed. Connect validates all 16 encoders, confirms the motor
communications timeouts and J8 position-force mode, and seeds measured targets
before enable. The same button
becomes Disconnect; Return to Neutral
routes to the exact calibrated zero pose; E-stop disables and closes both CAN
sockets. Motor 8 can be positioned or torque-limited through the MoveGripper
action and compiled CLI, though it is not yet exposed as a portal button. See
the repository-root `README.md` for the complete operator procedure and safety
boundary.

Portal motion controls are shared by virtual and physical mode. Left/right requests use the freshest
encoder-derived state as the opposite TCP target in a paired action. The public
C11/binary64 route planner samples every candidate edge through public IK/FK
and the shared conservative arm/tool and finite-pole collision evaluator. Dual
targets stay exact and all-or-nothing; an unreachable single-arm request may
still use the separately sampled farthest-proven ray projection. For an
accepted route, the portal recomputes the entire remaining path from measured
joints after every completed leg, re-proves the selected next edge, and checks
that the encoder/diagnostic generation is still equivalent at handoff. It waits
asynchronously for fresh idle diagnostics between legs. The virtual controller
checks measured keepout geometry every 5 ms and the physical controller every
20 ms during each leg, failing closed on a worsening approach. The virtual
result remains `collision_checked=false`; the physical result records the
nominal/live check without claiming certification. “Auto Calibrate” performs
only a nonmoving model/state verification. The software stop uses a
separately admitted request lane and invalidates an in-progress guard before it
can submit, but it is not a hardwired or safety-rated E-stop.

The portal's movement-limits slider spans 50–100% and defaults to 100%; the
former fixed behavior was 50%. The strict `/api/v3/move` request carries an
explicit binary64 `motion_limit_scale` together with the explicit coordinate
unit. That value equally scales the configured virtual velocity, acceleration,
and jerk limits, so the percentage is not a linear travel-time promise. The
existing `/openarm_ik/move_paired_tcp` ROS action remains unchanged at 50% for
compatibility; the portal uses the additive
`/openarm_ik/move_paired_tcp_scaled` action. Both retain the synchronized
seventh-order smooth trajectory.

`/api/v3/converge` is reserved for the nominal facing hand-rail envelope. From the
freshest measured joint state it searches a 15--70 mm radial stop corridor,
using exact binary64 FCL distance on the pinned `hand.stl`/`finger.stl` meshes
at each candidate rather than assuming a fixed TCP offset. Only the approaching
hand-housing pair targets 24 mm separation and may never be planned below
23 mm; the other eight cross-claw mesh pairs, every arm pair, and the pole
retain the normal thresholds. The feedback monitor stops virtual and physical
backends on entry into the expanded 25 mm rail envelope, and success still
requires the corresponding reported stop cause. `/api/v3/retreat` accepts only
a path that begins in that scoped corridor, never moves deeper, clears it, and
keeps every protected pair clear. Ordinary paired moves never receive the
exception. No STL contact is permitted; collision checking uses the
conservative fully-open finger/hand envelope regardless of measured opening.

Motion eligibility rechecks producer timestamps, local receipt ages, and
unchanged joint/diagnostic generations immediately before action submission.

The portal offers Current plus nine audited field-fill presets per arm: Near
low, Outer low, Near mid, Outer mid, Forward mid, Forward outer, Near-max
forward, Outer high, and High far. They only fill target fields. The virtual
targets deliberately span 15–48 cm forward, 17–67 cm outward, and 15–52 cm
high, instead of the former roughly one-centimetre increments. High far is the
farthest symmetric target found by the audited 1 cm high/far search while
retaining at least 2.6 cm sampled neutral-path clearance; it moves either TCP
about 73.6 cm from neutral. The pinned upstream URDF measurements sum to a
74.7–74.8 cm shoulder-to-TCP centreline reach (including the 10.25 cm hand and
8.35 cm TCP offsets); High far places the TCP over 89% of that geometric upper
bound from the shoulder. Across both arms, all 1,800 quantized
endpoint/cross-state transitions retain at least 2.65 cm sampled
nominal clearance against the 2.5 cm gate.

| Preset | X (cm) | Left Y (cm) | Right Y (cm) | Z (cm) |
| --- | ---: | ---: | ---: | ---: |
| Near low | 15 | 22 | -22 | 15 |
| Outer low | 15 | 40 | -40 | 15 |
| Near mid | 15 | 22 | -22 | 30 |
| Outer mid | 15 | 40 | -40 | 30 |
| Forward mid | 30 | 22 | -22 | 30 |
| Forward outer | 30 | 50 | -50 | 30 |
| Near-max forward | 48 | 17 | -17 | 35 |
| Outer high | 25 | 58 | -58 | 45 |
| High far | 28 | 67 | -67 | 52 |

Additional manual virtual regression inputs, in the default centimetres:

| Purpose | Left XYZ (cm) | Right XYZ (cm) | Expected virtual result from neutral |
| --- | --- | --- | --- |
| Impossible reach | `[5000, 5000, 5000]` | `[5000, -5000, 5000]` | Large visible move to the farthest sampled guarded reachable prefix |
| Pole keepout | `[40, 5, 40]` | `[40, -5, 40]` | Shortened at the 2.5 cm sampled nominal pole gate |

For an inter-arm mitigation test, first move each arm to its own Near-max
forward preset (`[48, 17, 35]` left and `[48, -17, 35]` right). Then request
the left arm at `[48, -17, 35]`. The audited virtual path is shortened before
the sampled left/right capsule clearance falls below 2.5 cm. These deliberate
guard tests are not physical-arm test instructions.

XYZ values default to centimetres and can be displayed and entered in metres,
centimetres, or inches; the page keeps canonical metre values,
and the versioned portal endpoint normalizes explicit `m`, `cm`, or `in` input
to metres before the unchanged guard and ROS action path. `/api/move` remains
the compatibility metre-only endpoint, while `/api/state` explicitly reports
`coordinate_unit: "m"`. The frame is `openarm_body_link0` (+X forward, +Y
left, +Z up). These presets remain virtual-model, sampled-nominal-guard test
values—not physically safe poses. Stock RViz and ROS remain metric, and the
viewer proxy has no portal-switchable coordinate grid. `/api/view-state` is a
read-only, 30 Hz latest-snapshot contract with fixed joint order, sequence,
producer timestamp, receipt age, and freshness. A stale viewer cannot change
motion eligibility; guard handoff remains authoritative.
