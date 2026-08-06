"""RViz side of the live hardstop calibration wizard.

Runs robot_state_publisher + RViz ONLY. The `m1_calibrate_live` wizard
(`ros2 run m1_can_tools m1_calibrate_live <side>`, in its own terminal so it has
a TTY for the guided prompts) owns the CAN bus, compliant-enables the arm at zero
torque, and publishes the live `/joint_states` this RViz renders. Deliberately no
CAN node and no joint_state_publisher here -- exactly one publisher owns the bus.

  ros2 launch m1_bringup live_calibration.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Adapted for OpenArmIK. Upstream defaulted to ranger_air_description and
    # m1_bringup, neither of which exists here; both were only ever defaults
    # behind an overridable argument, so only the defaults change.
    #
    # The FULL bimanual URDF is used, not the Stage-A visualization one: Stage-A
    # rewrites the prismatic finger joints to fixed, and a fixed joint cannot
    # move whatever is published, so the gripper calibration would render as a
    # dead claw.
    desc_share = get_package_share_directory("openarm_ik_ros")
    default_urdf = os.path.join(
        desc_share, "urdf", "openarm_v10_bimanual.urdf")
    default_rviz = os.path.join(desc_share, "rviz", "openarm_ik.rviz")
    urdf_path = LaunchConfiguration("urdf_path")
    rviz_config = LaunchConfiguration("rviz_config")
    robot_description = ParameterValue(Command(["cat ", urdf_path]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("urdf_path", default_value=default_urdf),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="live_cal_rsp",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": False,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="live_cal_rviz",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
