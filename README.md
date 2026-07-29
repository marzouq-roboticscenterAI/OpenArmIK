# OpenArmIK

`openarm_ik_ros` is a hardware-free ROS 2 Lyrical visualization adapter for the checked-in OpenArm v1.0 bimanual C model.

**It is position-only IK: orientation is free. It performs no self, body, inter-arm, or environment collision checking. It is not motion authorization and has no CAN, ros2_control, MoveIt, or hardware backend.**

## Run

```bash
./scripts/fetch_upstreams.sh
./scripts/build.sh
./scripts/launch_rviz.sh
```

The launch starts only `robot_state_publisher`, the virtual adapter, and RViz. `robot_state_publisher` is the only TF authority. The pinned `openarm_description` package supplies mesh URI resolution; the installed robot description is copied from [model/generated/openarm_v10_bimanual.urdf](model/generated/openarm_v10_bimanual.urdf).

In another sourced terminal, send one atomic paired request:

```bash
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
./scripts/send_paired_xyz.py 0.20 0.30 0.85 0.20 -0.30 0.85
```

The only input is `/openarm_ik/paired_xyz` (`geometry_msgs/PoseArray`): its header must have frame `world`, a fresh nonzero stamp (one-second default expiry), and exactly two poses ordered **left, right**. Only `position.x/y/z` is used. If either position-IK solve fails, the adapter keeps the previous complete two-arm state. Results are emitted as structured `diagnostic_msgs/DiagnosticArray` messages on `/openarm_ik/diagnostics`, including sequence, both statuses/residuals/achieved TCP XYZ, `backend=virtual`, and `collision_checked=false`.

`scripts/install_ros_dependencies.sh` is review-only unless explicitly passed `--apply`; it is never run automatically. Build and test with `colcon test --test-result-base ros2_ws/build` after `scripts/build.sh`.
