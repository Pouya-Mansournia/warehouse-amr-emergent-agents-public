"""
Spawn amr_ros_dg in Gazebo Sim (gz sim, "Harmonic" - the Gazebo that ships with ROS 2 Jazzy),
bring up robot_state_publisher, and start the ros2_control controllers
(joint_state_broadcaster + diff_drive_controller) so the robot can be driven.

Usage:
    ros2 launch amr_ros_dg gazebo.launch.py

Then, in another terminal, drive it with WASD (see scripts/wasd_teleop.py):
    ros2 run amr_ros_dg wasd_teleop.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("amr_ros_dg")
    default_xacro_path = os.path.join(pkg_share, "urdf", "amr_ros_dg.urdf.xacro")
    default_world_path = os.path.join(pkg_share, "worlds", "amr_test_world.sdf")

    # When gz sim converts our URDF to SDF for spawning, every "package://amr_ros_dg/..."
    # mesh URI becomes "model://amr_ros_dg/...". Gazebo has no idea what a ROS package is -
    # "model://X/..." is resolved by searching each directory in GZ_SIM_RESOURCE_PATH for a
    # subfolder literally named "X". pkg_share is ".../install/amr_ros_dg/share/amr_ros_dg",
    # so its PARENT (".../share") is exactly the directory gz needs: share/amr_ros_dg/meshes/*.STL
    # then matches model://amr_ros_dg/meshes/*.STL. Without this, gz sim can't find any mesh and
    # the robot silently fails to render (that's the "Unable to find file with URI [model://...]"
    # error) - it still gets created as an entity, but with dropped/invisible geometry.
    gz_resource_path = os.path.dirname(pkg_share)
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if existing_resource_path:
        gz_resource_path = gz_resource_path + os.pathsep + existing_resource_path
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=gz_resource_path
    )

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=default_xacro_path,
        description="Absolute path to the robot xacro/URDF file",
    )

    world_arg = DeclareLaunchArgument(
        name="world",
        default_value=default_world_path,
        # Not ros_gz_sim's bundled "empty.sdf" - the d435i/floor cameras and ultrasonic ring
        # need the world to load gz-sim's Sensors system plugin, which empty.sdf doesn't
        # include (see worlds/amr_test_world.sdf for the full reasoning).
        description="Absolute path to the Gazebo Sim world to load",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": [LaunchConfiguration("world"), " -r -v 4"]}.items(),
    )

    # Wheel joint z = -0.16929 m (in base_link frame), wheel radius = 0.075 m.
    # For wheels to touch the ground (z=0), robot origin must be spawned at:
    # z = 0.16929 + 0.075 = 0.2443 m
    # Using 0.245 m to ensure wheels make solid ground contact for traction.
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "amr_ros_dg",
            "-z", "0.245",
        ],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Bridges gz sim's native sensor topics (gz.msgs.*) to the matching ROS 2 message types.
    # Camera sensors publish under "<topic>" (image) and "<topic>/camera_info"; each
    # ultrasonic is a 1-sample gpu_lidar publishing a LaserScan with a single range reading
    # (see urdf/amr_ros_dg.urdf.xacro - gz sim has no dedicated ultrasonic sensor type).
    # All sensor <topic> elements in the xacro are prefixed with "${robot_name}/" (default
    # "amr_ros_dg", matching the "-name amr_ros_dg" spawn below) - added so the same xacro
    # can be instantiated multiple times under fleet.launch.py without every robot's sensors
    # colliding on one gz-transport topic name (see the xacro's "multi-robot fleet support"
    # header comment). Single-robot gazebo.launch.py picks up that same prefix here.
    sensor_bridge_topics = [
        "/amr_ros_dg/camera/d435i@sensor_msgs/msg/Image[gz.msgs.Image",
        "/amr_ros_dg/camera/d435i/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "/amr_ros_dg/camera/floor@sensor_msgs/msg/Image[gz.msgs.Image",
        "/amr_ros_dg/camera/floor/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "/amr_ros_dg/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    ] + [
        f"/amr_ros_dg/ultrasonic_{i}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        for i in range(1, 13)
    ]

    sensor_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=sensor_bridge_topics,
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        output="screen",
    )

    # `ros_gz_sim create` needs the Gazebo Sim server to be fully up (world/entity-creation
    # service registered) before it can spawn anything - starting it in the same instant as
    # `gz sim` is a known race condition (it just fails silently / times out, which looks like
    # "nothing showed up"). Give the server a few seconds' head start.
    delayed_spawn_robot = TimerAction(period=5.0, actions=[spawn_robot])

    # Controllers can only be spawned once the robot entity (and its ros2_control
    # controller_manager, brought up by the gz_ros2_control plugin) exist in the sim -
    # so chain them off the spawn_robot process exiting successfully, with a short extra
    # delay for the controller_manager service to finish coming up.
    delayed_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[joint_state_broadcaster_spawner, diff_drive_spawner],
                )
            ],
        )
    )

    return LaunchDescription(
        [
            set_gz_resource_path,
            model_arg,
            world_arg,
            gz_sim,
            clock_bridge,
            sensor_bridge,
            robot_state_publisher_node,
            delayed_spawn_robot,
            delayed_controllers,
        ]
    )
