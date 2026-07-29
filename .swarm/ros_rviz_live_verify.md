# ROS/RViz live independent verification

Date: 2026-07-29 (America/Los_Angeles)  
Repository: `/home/signalprocessing-dev/OpenArmIK`  
Commit: `ea706280f53c15a072fda55f80dc5c21502fbc9d`  
Verdict: **FINDINGS**

## Scope and safety

- Read-only verification of implementation/source. No implementation file or commit was changed.
- The pre-existing dirty `transport/tests/test_transport.cpp` was preserved.
- Only the Stage-A virtual backend was launched. Diagnostics continually reported
  `backend=virtual`, `virtual_execution_enabled=true`, and
  `physical_motion_authorized=false`.
- No CAN device, SocketCAN command, network interface, or physical motor was accessed or
  manipulated.
- Permitted outputs are under `ros2_ws/{build,install,log,native_build}` and `.swarm/`.
- The temporary directory `/var/tmp/openarmik-live-verify` was removed after testing.

## Findings

### MEDIUM: active stack shutdown leaves the compiled action CLI waiting about 48 seconds

An active action was proven at `measured_progress=0.02050602185560413`, command 1,
with coherent left/right feedback sequence 1597. Ctrl+C was then sent to the real
`scripts/launch_rviz.sh` PTY.

- The wrapper returned 130 in 0.809 s after Ctrl+C.
- `robot_state_publisher` PID 2108695 and `openarm_ik_ros_node` PID 2108696 both
  reported clean termination; the launch log records their completion at
  1785366154.6905570 and 1785366154.6926608.
- RViz, the launcher, and both ROS children were gone after wrapper exit.
- The separately invoked compiled client remained as PIDs 2111715/2112021 after the
  server was gone (observed elapsed time 27 s), then eventually exited failure 5 with:
  `terminal result timeout; cancel response timeout`.

This is bounded by the CLI's 45-second result wait plus cancel wait, but is not a clean,
prompt client exit and is a transient lingering process during active stack shutdown.

### LOW: RViz reports four nonfatal finger-link inertia visualization errors

RViz remained alive and responsive and rendered the bimanual robot, but logged
`The link ... has unrealistic inertia, so the equivalent inertia box will not be shown`
for:

- `openarm_left_left_finger`
- `openarm_left_right_finger`
- `openarm_right_left_finger`
- `openarm_right_right_finger`

The RobotModel row is red in the screenshot, although the robot is visibly rendered and
the status bar says `RViz is ready.` This did not crash or destabilize RViz and did not
affect virtual commands.

## Build and tests

Clean test-enabled build command:

```bash
mkdir -p /var/tmp/openarmik-live-verify .swarm
set -o pipefail
TMPDIR=/var/tmp/openarmik-live-verify scripts/build.sh --tests 2>&1 \
  | tee .swarm/ros_rviz_live_verify_build.log
```

Result: exit 0. All native CTests passed:

- can: 1/1
- model: 4/4
- commission: 2/2
- transport: 3/3
- control: 4/4

ROS packages built successfully in 23.2 s, and the expected 9 ROS tests were registered.

Explicit ROS test command in isolated domain:

```bash
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=181 TMPDIR=/var/tmp/openarmik-live-verify
ctest --test-dir ros2_ws/build/openarm_ik_ros --output-on-failure 2>&1 \
  | tee .swarm/ros_rviz_live_verify_ros_tests.log
```

Result: exit 0, 9/9 passed, 69.96 s. This includes `test_ros_contract`,
`test_active_sigint`, the virtual-control session test, no-CAN-linkage test, URDF test,
parameter test, and RViz close-helper tests.

## Idle Ctrl+C through the repository RViz wrapper

Command:

```bash
export ROS_DOMAIN_ID=182 TMPDIR=/var/tmp/openarmik-live-verify
unset OPENARM_RVIZ_RENDERER
scripts/launch_rviz.sh
```

The stable Wayland default selected
`OpenArm RViz renderer: Mesa software rasterizer (XWayland/GLX)`.

Observed process tree:

- wrapper: PID 2040887
- ROS launch/core process group: PID/PGID 2041053
- RViz isolated process group: PID/PGID 2041054
- `robot_state_publisher`: PID 2041140
- `openarm_ik_ros_node`: PID 2041141

Ctrl+C result: wrapper exit 130 in 0.643 s. Both ROS children reported clean termination.
All five PIDs were absent afterward, and `ROS_DOMAIN_ID=182 ros2 node list` was empty.

## Main live RViz/control session

Command:

```bash
export ROS_DOMAIN_ID=183 TMPDIR=/var/tmp/openarmik-live-verify
unset OPENARM_RVIZ_RENDERER
scripts/launch_rviz.sh
```

Observed PIDs:

- wrapper: 2041627
- ROS launch/core PID/PGID: 2041684
- RViz PID/PGID: 2041685
- `robot_state_publisher`: 2041777
- `openarm_ik_ros_node`: 2041778

The renderer again selected Mesa software. RViz stayed alive across both completed
commands and rendered the updated robot for roughly four minutes. Its OpenGL report was
4.5 / GLSL 4.5. No RViz/Ogre crash or process restart occurred.

### ROS ownership and interfaces

With `ROS2CLI_NO_DAEMON=1` in domain 183:

- `/joint_states`: exactly one publisher, node `openarm_ik_ros`.
- `/tf`: exactly one publisher, node `robot_state_publisher`.
- `/tf_static`: exactly one publisher, node `robot_state_publisher`.
- Actions present:
  - `/openarm_ik/move_joint [openarm_control_msgs/action/MoveJoint]`
  - `/openarm_ik/move_paired_tcp [openarm_control_msgs/action/MovePairedTcp]`
- No second JointState authority or TF authority was present.

Representative commands:

```bash
ros2 action list -t
ros2 topic info /joint_states -v
ros2 topic info /tf -v
ros2 topic info /tf_static -v
ros2 run openarm_ik_ros openarm_control_cli status
```

Initial status was WARN (`level: 1`) with `virtual backend; collision unchecked`,
`collision_checked=false`, no fault masks, and zero left/right timestamp skew.

### Individual joint goal

Compiled client command:

```bash
ros2 run openarm_ik_ros openarm_control_cli \
  move-joint openarm_left_joint4 0.2
```

Result: exit 0, `completed command_id=1`.

Live measured evidence:

- Baseline JointState position: 0.0000667582207984907 rad.
- Intermediate measured position: 0.00502594033722481 rad while the command target was
  0.2 rad; measured velocity was 0.07081807081807057 rad/s. This directly shows the
  published measured state lagging the command target rather than jumping to it.
- Intermediate action feedback: command 1, progress 0.011448087692351616,
  left/right feedback sequence 13945/13945.
- Executing diagnostics: WARN, `adapter_state=executing`, `committed=false`, seed sequence
  13899/13899, current sequence 13901/13901, timestamps
  69500261941/69500261941 ns, pair skew 0.
- Terminal JointState position: 0.1999599450675209 rad.
- Terminal diagnostics: seed 13899/13899, terminal 14202/14202,
  `reason=completed_measured_feedback`, `outcome=completed`, `committed=true`,
  `result_collision_checked=false`.
- Later health sample: sequence 14651/14651 and timestamp
  73250261297/73250261297 ns, still zero skew. Thus sequences and measurement timestamps
  increased coherently on both sides.

The terminal sequence advancing beyond the plan seed and the explicit
`completed_measured_feedback` terminal reason are live evidence that success was not
returned on command submission; it followed measured execution and settling/dwell.

Capture commands used explicit message types, `--no-daemon`, `--once`, and filters to
record a moving JointState, feedback with progress strictly between zero and one, and a
diagnostic message whose `executing` value was true. Logs:

- `.swarm/live_joint_baseline.log`
- `.swarm/live_joint_intermediate.log`
- `.swarm/live_joint_action_feedback.log`
- `.swarm/live_status_during_joint.log`
- `.swarm/live_joint_terminal.log`
- `.swarm/live_status_after_joint.log`
- `.swarm/live_joint_cli.log`

### Named paired left/right TCP XYZ goal

Compiled client command:

```bash
ros2 run openarm_ik_ros openarm_control_cli move-paired-tcp \
  openarm_body_link0 0.20 0.30 0.85 0.20 -0.30 0.85
```

This is the repository-documented safe virtual goal. The generated action schema names
the fields `left_tcp_m` and `right_tcp_m`; the source frame was
`openarm_body_link0`.

Result: exit 0, `completed command_id=2`.

Live evidence:

- Intermediate action feedback: progress 0.010222081348961298, left/right sequence
  17935/17935.
- Executing diagnostics: WARN, virtual and physical false, collision false,
  seed 17825/17825, current sequence 17851/17851, timestamps
  89250258946/89250258946 ns, pair skew 0, pending/not committed.
- Terminal diagnostics: plan duration 20525467196 ns, terminal sequence 21969/21969,
  later health sequence 22201/22201 and timestamp 111000261149/111000261149 ns,
  pair skew 0, `reason=completed_measured_feedback`, `outcome=completed`,
  `committed=true`, and `result_collision_checked=false`.

Logs:

- `.swarm/live_paired_action_feedback.log`
- `.swarm/live_status_during_paired.log`
- `.swarm/live_status_after_paired.log`
- `.swarm/live_paired_cli.log`

## Screenshot

The GNOME screenshot D-Bus interface rejected direct capture with AccessDenied, so the
already-present `ffmpeg` X11-grab input was used against RViz's XWayland window only:

```bash
xwininfo -root -tree
ffmpeg -hide_banner -loglevel warning -f x11grab \
  -window_id 0x1400056 -framerate 1 -i :0 -frames:v 1 -y \
  .swarm/ros_rviz_live_verify.png
```

Result: 900x646 PNG, 64,054 bytes, visually inspected. It shows the rendered bimanual
OpenArm model and `RViz is ready.`

Path: `/home/signalprocessing-dev/OpenArmIK/.swarm/ros_rviz_live_verify.png`

## Active Ctrl+C details

A separate fresh stack was started in domain 184 with the same wrapper and automatic
Mesa renderer. Captured PIDs were:

- ROS launch/core PID/PGID 2108550
- `robot_state_publisher` PID 2108695
- `openarm_ik_ros_node` PID 2108696

The short-lived RViz PID was not separately recorded before shutdown; its isolated
process group was nevertheless reaped by the wrapper and no matching RViz process
remained afterward.

The compiled CLI goal was:

```bash
ros2 run openarm_ik_ros openarm_control_cli \
  move-joint openarm_right_joint1 0.8
```

Before Ctrl+C, direct action feedback proved lifecycle 5, event 3, command 1,
progress 0.02050602185560413, and equal sequence 1597/1597. Ctrl+C then caused the
wrapper to return 130 in 0.809 s. The two ROS nodes finished cleanly and all captured
stack PIDs were absent afterward. The CLI timeout behavior is the MEDIUM finding above.

## Cleanup and final process/source check

- Removed the four `launch_params_*` files created under
  `/var/tmp/openarmik-live-verify`, then removed that directory.
- Confirmed no repository RViz, launch wrapper, `openarm_ik_ros_node`,
  `robot_state_publisher`, or test CLI process remained.
- Final Git status retained the pre-existing
  `M transport/tests/test_transport.cpp`; the only new untracked entries are this
  verification's permitted `.swarm` report, logs, and screenshot.
