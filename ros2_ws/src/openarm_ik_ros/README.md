# OpenArm measured virtual control ROS adapter

`openarm_ik_ros_node` is the sole `/joint_states` authority for the Stage-A
virtual controller. It publishes exactly the fourteen actuated arm joints from
coherent `oa_runtime_snapshot` encoder feedback, including measured position,
velocity, effort, and conservative runtime-steady measurement timestamps. It
publishes no TF and no finger state. `robot_state_publisher` is the launch's
only TF authority.

The RViz launch gives that publisher the derived
`openarm_v10_bimanual_stage_a_visualization.urdf`. Because Stage A has no
motor-8 feedback, this description fixes all four finger joints at their
canonical `q=0 m` closed transforms as an **unmeasured visualization
convention**. It does not report a gripper measurement. The four invalid
upstream finger inertials are omitted so RViz does not invent equivalent
inertia geometry. The canonical dynamic `openarm_v10_bimanual.urdf`, its model
digest, arm kinematics, hand/TCP frames, meshes, and collision geometry remain
unchanged.

The derived description defines visualization-launch TF and must not be used as
a dynamics, calibrated gripper, MoveIt, or collision-safety model. When a
future version adds authoritative motor-8 state to the existing sole
`/joint_states` publisher, the launch must return to movable/mimic finger joints
and publish only the two measured source joints; `robot_state_publisher` should
continue deriving the mimic transforms.

The production command interfaces are the reliable actions
`/openarm_ik/move_joint` and `/openarm_ik/move_paired_tcp` from
`openarm_control_msgs`. Paired goals name `left_tcp_m` and `right_tcp_m` and are
transformed transactionally from their stamped source frame into
`openarm_body_link0`. One reject-new arbiter spans both actions and the
deprecated `/openarm_ik/paired_xyz` compatibility topic. Cancel uses a disable
stop and requires a process restart before another command.

The lower session consumes installed `OpenArm::Runtime` as its sole state,
motion, model/TCP identity, clock, event, and plan authority. Runtime owns the
virtual progression cadence; ROS only polls new measured generations and
heartbeats active commands. The backend is fixed to the canonical OpenArm v1.0
bimanual virtual manifest.
Physical control, transport selection, CAN configuration, calibration,
commissioning, simulator injection, and persistence endpoints are absent.
Collision checking is unavailable, so healthy operation remains WARN with
`collision_checked=false`. Diagnostics report the exact runtime capabilities,
coordinate/model/TCP/scene digests, immutable-manifest authentication and
persistence state, exact virtual inventory, and the fact that runtime
calibration capabilities have no ROS endpoint.

Use the compiled client after sourcing the ROS workspace:

```bash
ros2 run openarm_ik_ros openarm_control_cli status
ros2 run openarm_ik_ros openarm_control_cli move-joint openarm_left_joint4 0.2
ros2 run openarm_ik_ros openarm_control_cli move-paired-tcp \
  openarm_body_link0 0.20 0.30 0.85 0.20 -0.30 0.85
```

All six `move-paired-tcp` XYZ arguments are metres; the legacy ROS CLI does not
perform portal display-unit conversion.

The CLI returns success only after the matching action reaches measured
completion. `/use_sim_time=true`, nonfinite targets, unknown names, invalid or
stale stamps, missing transforms, unreachable targets, and concurrent commands
are rejected.

Run the local compiled portal from the repository root with `./run.sh`. The
launcher builds the workspace, starts the virtual controller, and opens the
loopback-only `127.0.0.1` portal. Its canvas is a browser-native WebGL2
measured-pose viewer: it applies the current authoritative 14-joint sample to
the pinned Stage-A URDF and allowlisted collision STL geometry. It is a
**visual proxy — not collision checking**, not RViz pixels or a safety-rated
feedback channel. Camera drag, wheel zoom, touch pinch, reset, resize, and
stale overlays are browser-local and cannot issue a control request. Use
`scripts/launch_rviz.sh` separately for stock RViz engineering views.
The redistributed collision meshes carry the exact pinned upstream Apache-2.0
license at `share/openarm_ik_ros/viewer/openarm_description-LICENSE.txt`.

Portal controls remain virtual-only. Left/right requests use the freshest
encoder-derived state as the opposite TCP target in a paired action. A sampled
public-IK/FK capsule and central-keepout guard rejects unproven nominal paths,
but the controller still reports `collision_checked=false`; the guard is not
physical collision certification. “Auto Calibrate” performs only a nonmoving
simulation verification, and the software stop button is not a hardwired or
safety-rated E-stop.

Motion eligibility rechecks producer timestamps, local receipt ages, and
unchanged joint/diagnostic generations immediately before action submission.

The portal offers Current plus nine audited field-fill presets per arm: Small
forward/up, Medium forward/up, Large forward/up, Low reach, Mid reach, Far
reach, High, High near, and High far. They only fill target fields. XYZ values default to centimetres and
can be displayed and entered in inches; the page keeps canonical metre values,
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
