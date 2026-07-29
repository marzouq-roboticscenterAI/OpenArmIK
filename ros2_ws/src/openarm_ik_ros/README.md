# OpenArm measured virtual control ROS adapter

`openarm_ik_ros_node` is the sole `/joint_states` authority for the Stage-A
virtual controller. It publishes exactly the fourteen actuated arm joints from
coherent `oa_snapshot` encoder feedback, including measured position, velocity,
effort, and conservative measurement timestamps. It publishes no TF and no
finger state. `robot_state_publisher` is the launch's only TF authority.

The production command interfaces are the reliable actions
`/openarm_ik/move_joint` and `/openarm_ik/move_paired_tcp` from
`openarm_control_msgs`. Paired goals name `left_tcp_m` and `right_tcp_m` and are
transformed transactionally from their stamped source frame into
`openarm_body_link0`. One reject-new arbiter spans both actions and the
deprecated `/openarm_ik/paired_xyz` compatibility topic. Cancel uses a disable
stop and requires a process restart before another command.

The backend is fixed to the canonical OpenArm v1.0 bimanual virtual manifest.
Physical control, transport selection, CAN configuration, calibration,
commissioning, simulator injection, and persistence endpoints are absent.
Collision checking is unavailable, so healthy operation remains WARN with
`collision_checked=false`.

Use the compiled client after sourcing the ROS workspace:

```bash
ros2 run openarm_ik_ros openarm_control_cli status
ros2 run openarm_ik_ros openarm_control_cli move-joint openarm_left_joint4 0.2
ros2 run openarm_ik_ros openarm_control_cli move-paired-tcp \
  openarm_body_link0 0.20 0.30 0.85 0.20 -0.30 0.85
```

The CLI returns success only after the matching action reaches measured
completion. `/use_sim_time=true`, nonfinite targets, unknown names, invalid or
stale stamps, missing transforms, unreachable targets, and concurrent commands
are rejected.

Run the local compiled portal from the repository root with `./run.sh`. The
launcher builds the workspace, starts the virtual controller and stock RViz,
then opens Firefox at a service bound only to `127.0.0.1`. The right pane is a
JPEG snapshot stream of the launcher-owned RViz X11 window; it is not a browser
renderer. The launcher resolves the stock `rviz2` executable to an absolute
path, and the portal requires that exact `/proc/PID/exe` identity plus
XComposite protocol 0.2 or newer.

Portal controls remain virtual-only. Left/right requests use the freshest
encoder-derived state as the opposite TCP target in a paired action. A sampled
public-IK/FK capsule and central-keepout guard rejects unproven nominal paths,
but the controller still reports `collision_checked=false`; the guard is not
physical collision certification. “Auto Calibrate” performs only a nonmoving
simulation verification, and the software stop button is not a hardwired or
safety-rated E-stop.

Motion eligibility rechecks producer timestamps, local receipt ages, and
unchanged joint/diagnostic generations immediately before action submission.
