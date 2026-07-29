# ROS 2 Lyrical compatibility audit

Audit date: 2026-07-28  
Host: Ubuntu 26.04 (`resolute`), ROS 2 Lyrical in `/opt/ros/lyrical`, GCC 15.2, CMake 4.2.3  
Scope: read-only inspection and hardware-free builds of the pinned upstream sources. No `sudo`, CAN interface, robot hardware, system-package installation, or source modification was used. All generated build/log/package-extraction artifacts are under `.deps/ros_audit`.

## Audited revisions

| Repository | Commit |
|---|---|
| `upstream/openarm_description` | `6c7b720f1ba48e8bafa3a3dc752c45f397b42221` |
| `upstream/openarm_can` | `c32ecd31da267967f0c913c2118c843177d88b91` |
| `upstream/openarm_ros2` | `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed` |

## Result summary

| Package | Result on Lyrical | Evidence / qualification |
|---|---|---|
| `openarm_can` | **Compiles** | Library, demo, motor-sampling check, and CLI built and installed. Only an unused-parameter warning occurred. No executable was run because that could access CAN/hardware. Requires system `libcli11-dev`. |
| `openarm_description` | **Compiles** | Isolated build passed after explicitly selecting `/usr/bin/python3`. The first failure was a host PATH issue: CMake selected an STEdgeAI Python lacking `catkin_pkg`; it was not a ROS incompatibility. |
| `openarm_bimanual_moveit_config` | **Compiles as an asset package** | CMake/install passed, but its declared MoveIt runtime is unavailable/incomplete on this Lyrical apt source, so the configuration is not runnable here. |
| `openarm_bringup` | **Compiles as an asset/launch package** | CMake/install passed. Runtime still needs ros2_control/controllers, MoveIt, and the hardware plugin. |
| `openarm` | **Package source compiles; full colcon graph is blocked** | Direct isolated CMake configure/build/install passed. Colcon did not process it after `openarm_hardware` failed because the expected dependency environment was absent. |
| `openarm_hardware` | **Not compiled: missing dependency** | Configure stopped at `find_package(hardware_interface REQUIRED)`. No source/API incompatibility was reached or observed. Compatibility behind that dependency remains untested. |

No Lyrical source/API incompatibility was observed in code that reached compilation. The failures divide into:

- **Host environment issue:** the first description build selected a third-party `python` from the PATH; `-DPython3_EXECUTABLE=/usr/bin/python3` resolves it.
- **Missing build dependency:** `openarm_hardware` needs ROS 2 control's `hardware_interface` (and subsequently its dependency closure).
- **Missing runtime dependencies:** the bringup and MoveIt configuration install, but cannot run their intended stack with the packages currently available/installed.

## Exact build commands and evidence

### Entire source set

```bash
source /opt/ros/lyrical/setup.bash
colcon --log-base .deps/ros_audit/log_all build \
  --base-paths upstream/openarm_description upstream/openarm_can upstream/openarm_ros2 \
  --build-base .deps/ros_audit/build_all \
  --install-base .deps/ros_audit/install_all \
  --continue-on-error --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=OFF
```

Observed results:

- `openarm_can`: success.
- `openarm_description`: failed before source configuration because CMake selected `/home/signalprocessing-dev/STMicroelectronics/STEdgeAI/10.2.0/Utilities/linux/python`, whose environment lacks `catkin_pkg`.
- `openarm_hardware`: failed at missing `hardware_interface`.
- Three dependent packages were not processed.

### Description with the system Python selected

```bash
source /opt/ros/lyrical/setup.bash
colcon --log-base .deps/ros_audit/log_desc_py build \
  --base-paths upstream/openarm_description \
  --build-base .deps/ros_audit/build_desc_py \
  --install-base .deps/ros_audit/install_desc_py \
  --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=OFF \
               -DPython3_EXECUTABLE=/usr/bin/python3 \
               -DPYTHON_EXECUTABLE=/usr/bin/python3
```

Result: success. `Python3_EXECUTABLE` is the relevant override; CMake reported `PYTHON_EXECUTABLE` unused.

### Non-hardware ROS packages

```bash
source /opt/ros/lyrical/setup.bash
colcon --log-base .deps/ros_audit/log_ros2_nonhw build \
  --base-paths upstream/openarm_ros2 \
  --packages-select openarm openarm_bringup openarm_bimanual_moveit_config \
  --build-base .deps/ros_audit/build_ros2_nonhw \
  --install-base .deps/ros_audit/install_ros2_nonhw \
  --continue-on-error --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3
```

Result: `openarm_bringup` and `openarm_bimanual_moveit_config` passed. Colcon blocked the `openarm` metapackage because `openarm_hardware/package.sh` was absent after the hardware dependency failure. The metapackage itself was verified independently:

```bash
source /opt/ros/lyrical/setup.bash
cmake -S upstream/openarm_ros2/openarm \
  -B .deps/ros_audit/build_openarm_direct \
  -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_INSTALL_PREFIX="$PWD/.deps/ros_audit/install_openarm_direct"
cmake --build .deps/ros_audit/build_openarm_direct
cmake --install .deps/ros_audit/build_openarm_direct
```

Result: success.

## Manifest dependency audit

`openarm_description` declares runtime dependencies on:

- `joint_state_publisher_gui`
- `joint_state_publisher`
- `robot_state_publisher`
- `rviz2`
- `xacro`
- `ros_gz`
- `zed_description`

The current v1.0 robot xacro and display launch do not reference `ros_gz`. ZED content exists only in separate sensor/v2 assets; it is not included by the current `openarm_v10.urdf.xacro`. Thus `ros_gz` and `zed_description` are unconditional/over-broad manifest dependencies for this display path, not functional requirements for it.

The ROS 2 source manifests additionally declare:

- `openarm_hardware`: `rclcpp`, `hardware_interface`, `openarm_can`, and `pluginlib`; CMake also requires `rclcpp_lifecycle` and `OpenArmCAN`.
- `openarm_bringup`: `ros2_controllers`, `controller_manager`, the MoveIt config, description, and RViz.
- `openarm_bimanual_moveit_config`: MoveIt move-group, kinematics, planners, controller manager, visualization, and setup-assistant runtime packages.

Installed and available for the visualization adapter on this host include `ament_cmake`, `rclcpp`, `sensor_msgs`, `robot_state_publisher`, `rviz2`, `tf2_ros`, and the RViz plugin packages. `xacro` was not installed system-wide.

Missing from the installed ROS environment include `xacro`, both joint-state-publisher packages, `hardware_interface`, `controller_manager`, `ros2_controllers`, `ros_gz`, `zed_description`, and the MoveIt runtime packages.

The Lyrical apt metadata has candidates for xacro, joint-state publisher, ros-gz, ZED description, ros2-control, ros2-controllers, controller manager, and hardware interface. It has `moveit-common`, `moveit-configs-utils`, messages/resources/tests, and task-constructor packages, but no candidates for the required `moveit_ros_move_group`, `moveit_kinematics`, `moveit_planners`, `moveit_simple_controller_manager`, visualization, or setup-assistant packages. Therefore the pinned MoveIt runtime closure cannot currently be satisfied from this host's Lyrical apt sources.

## Current generated xacro and mesh validation

The audit downloaded and extracted the Lyrical xacro `.deb` beneath `.deps/ros_audit` without installing it. The current pinned xacro—not the checked-in example URDF—was generated with:

```bash
source /opt/ros/lyrical/setup.bash
source .deps/ros_audit/install_desc_py/setup.bash
xacro_prefix="$PWD/.deps/ros_audit/xacro_root/opt/ros/lyrical"
export PATH="$xacro_prefix/bin:$PATH"
export PYTHONPATH="$xacro_prefix/lib/python3.14/site-packages:${PYTHONPATH:-}"
export AMENT_PREFIX_PATH="$xacro_prefix:$AMENT_PREFIX_PATH"

xacro upstream/openarm_description/assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro \
  arm_type:=v10 bimanual:=true ros2_control:=false \
  > .deps/ros_audit/generated/openarm_v10_bimanual.urdf

xacro upstream/openarm_description/assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro \
  arm_type:=v10 bimanual:=false ros2_control:=false \
  > .deps/ros_audit/generated/openarm_v10_single.urdf
```

Generated-file evidence:

| Model | Size | SHA-256 | Structure |
|---|---:|---|---|
| Bimanual | 32,692 bytes | `632ad533893830a15d15b78f0207ec88216cea0a88e5f1da525ec4e011550aab` | 26 links, 25 joints, 18 moving joints, 46 mesh uses |
| Single | 15,668 bytes | `cabfd5a25be6bda8e2fcf100f52950e0fbd830d5f67051bf8a60335341b5d401` | 12 links, 11 joints, 9 moving joints, 22 mesh uses |

The bimanual URDF contains 23 DAE and 23 STL mesh references (22 unique package paths). Every `package://openarm_description/...` reference resolves to an existing file in the pinned description package. The single-arm model likewise has no missing mesh files. This proves package URI/file resolution; actual RViz rendering was not possible in the headless audit session.

## TF and joint naming contract

The generated bimanual model has root frame `world` and these actuated arm joints:

```text
openarm_left_joint1 ... openarm_left_joint7
openarm_right_joint1 ... openarm_right_joint7
```

The grippers add two independent prismatic joints:

```text
openarm_left_finger_joint1
openarm_right_finger_joint1
```

Each side's `finger_joint2` mimics its corresponding `finger_joint1`, so it must not be separately published. Fixed joints establish:

```text
world -> openarm_body_link0
openarm_body_link0 -> openarm_left_link0
openarm_body_link0 -> openarm_right_link0
openarm_{left,right}_link7 -> openarm_{left,right}_hand
openarm_{left,right}_hand -> openarm_{left,right}_hand_tcp
```

`robot_state_publisher` was run on Lyrical with the generated XML supplied through its `robot_description` parameter. It initialized successfully and published the static tree. A current-stamped synthetic `sensor_msgs/msg/JointState` containing the 14 arm joints plus the two independent gripper joints produced the dynamic chain; `tf2_echo world openarm_left_hand_tcp` received the expected zero-state transform. Lyrical's robot-state-publisher did **not** accept the URDF merely as a positional CLI argument in this test; it reported an empty `robot_description`, while the parameter form used by the upstream launch worked.

## Upstream display launch without ros-gz or ZED

The display launch directly invokes xacro and starts only:

- `robot_state_publisher`
- `joint_state_publisher_gui`
- RViz 2

With isolated extracted xacro but no joint-state GUI, launch failed specifically with:

```text
package 'joint_state_publisher_gui' not found
```

No `ros_gz` or `zed_description` lookup occurred. After downloading and extracting the Lyrical joint-state-publisher and GUI `.deb` files into `.deps/ros_audit` (still without system installation), this command was used:

```bash
source /opt/ros/lyrical/setup.bash
source .deps/ros_audit/install_desc_py/setup.bash
xacro_prefix="$PWD/.deps/ros_audit/xacro_root/opt/ros/lyrical"
display_prefix="$PWD/.deps/ros_audit/display_root/opt/ros/lyrical"
export PATH="$xacro_prefix/bin:$PATH"
export PYTHONPATH="$display_prefix/lib/python3.14/site-packages:$xacro_prefix/lib/python3.14/site-packages:${PYTHONPATH:-}"
export AMENT_PREFIX_PATH="$display_prefix:$xacro_prefix:$AMENT_PREFIX_PATH"
export ROS_LOG_DIR="$PWD/.deps/ros_audit/ros_logs_display"
export QT_QPA_PLATFORM=offscreen
unset DISPLAY
timeout 8s ros2 launch openarm_description display_openarm.launch.py arm_type:=v10
```

`robot_state_publisher`, `joint_state_publisher_gui`, and RViz all started. Robot-state-publisher logged `Robot initialized`; the GUI received `robot_description` and configured. RViz then exited because Ogre/GLX could not open an X display, which is the expected headless-host limitation. The launch was allowed to run until the eight-second timeout (exit 124). This demonstrates that the current v1 display launch dependency closure does not require optional `ros_gz` or ZED packages.

## Minimal C-model-to-RViz adapter

The smallest robust architecture is one new `ament_cmake` C++ package, for example `openarm_ik_ros`, with dependencies on `rclcpp`, `sensor_msgs`, and the existing pure-C paired IK/state library. It should:

1. Receive or compute a paired left/right state using the C model.
2. Validate both seven-joint results as one transaction. If either solve fails, retain the previous coherent pair and report the failure; never publish a half-updated robot.
3. Map explicitly—not by accidental array order—to `openarm_left_joint1..7`, then `openarm_right_joint1..7`.
4. Publish one current-stamped `sensor_msgs/msg/JointState` containing those 14 positions and the two independent `finger_joint1` positions. Grippers may default to zero or be parameters/explicit state. Omit both mimic `finger_joint2` joints because robot-state-publisher derives them.
5. Publish no TF and open no CAN interface. `robot_state_publisher` must be the sole TF authority.

A minimal Python launch file should:

- process the pinned current xacro with `arm_type:=v10`, `bimanual:=true`, and `ros2_control:=false`;
- pass the generated string as the `robot_description` parameter to `robot_state_publisher`;
- start the paired-state adapter;
- start RViz using the upstream bimanual RViz configuration, with fixed frame `world`.

Do not start `joint_state_publisher_gui` in the normal adapter launch because it would be a competing publisher on `/joint_states`. Keep the upstream GUI launch as an independent manual/debug path. This visualization path needs neither OpenArm CAN nor ros2-control, MoveIt, ros-gz, or ZED.

## Minimal package installer lists

On this audited host, all packages needed by the proposed adapter/display path are installed except xacro:

```bash
sudo apt install ros-lyrical-xacro
```

For a clean Lyrical development machine, the reproducible minimal set is:

```bash
sudo apt install \
  build-essential \
  ros-dev-tools \
  ros-lyrical-ament-cmake \
  ros-lyrical-rclcpp \
  ros-lyrical-sensor-msgs \
  ros-lyrical-robot-state-publisher \
  ros-lyrical-rviz2 \
  ros-lyrical-xacro
```

Only for the upstream interactive slider display, additionally install:

```bash
sudo apt install ros-lyrical-joint-state-publisher-gui
```

That GUI package pulls the base joint-state publisher dependency. `ros-lyrical-ros-gz` and `ros-lyrical-zed-description` are not needed for the current v1 display. `libcli11-dev` is needed only to build `openarm_can`. A future hardware-plugin experiment would minimally begin with `ros-lyrical-ros2-control` and `ros-lyrical-ros2-controllers`, but hardware-plugin compatibility has not been proven and those packages are not part of the visualization adapter. The unavailable MoveIt runtime closure must not be included in the minimal adapter installer.
