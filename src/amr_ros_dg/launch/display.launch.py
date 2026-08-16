"""
Load amr_ros_dg.urdf.xacro, publish TF via robot_state_publisher, expose the wheel/caster
joints via joint_state_publisher_gui, and open RViz2 pointed at the saved config.

Usage:
    ros2 launch amr_ros_dg display.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("amr_ros_dg")

    default_xacro_path = os.path.join(pkg_share, "urdf", "amr_ros_dg.urdf.xacro")
    default_rviz_config = os.path.join(pkg_share, "rviz", "amr_ros_dg.rviz")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=default_xacro_path,
        description="Absolute path to the robot xacro/URDF file",
    )

    rviz_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=default_rviz_config,
        description="Absolute path to the RViz2 config file",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    return LaunchDescription(
        [
            model_arg,
            rviz_arg,
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
