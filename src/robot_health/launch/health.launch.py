"""Launches health_monitor (always) and fault_injector (only if schedule_file is non-empty
or manual-only injection is desired - fault_injector runs either way, since the topic it
publishes on is also the manual-trigger API; passing no schedule just means it never fires
anything on its own).

    ros2 launch robot_health health.launch.py num_robots:=3
    ros2 launch robot_health health.launch.py num_robots:=3 \
        schedule_file:=$(ros2 pkg prefix robot_health)/share/robot_health/config/faults/example_schedule.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _robot_namespaces(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    namespaces = [f"robot{i}" for i in range(1, num_robots + 1)]

    health_monitor_node = Node(
        package="robot_health",
        executable="health_monitor",
        name="health_monitor",
        output="screen",
        parameters=[{"robot_namespaces": namespaces, "use_sim_time": True}],
    )

    fault_injector_node = Node(
        package="robot_health",
        executable="fault_injector",
        name="fault_injector",
        output="screen",
        parameters=[
            {
                "schedule_file": LaunchConfiguration("schedule_file"),
                "use_sim_time": True,
            }
        ],
    )

    return [health_monitor_node, fault_injector_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="num_robots",
                default_value="3",
                description="Must match the fleet.launch.py num_robots this is monitoring.",
            ),
            DeclareLaunchArgument(
                name="schedule_file",
                default_value="",
                description="Path to a fault schedule YAML (see config/faults/example_schedule.yaml). "
                "Empty = manual-trigger-only (still able to receive FaultCommand on "
                "/fleet/faults/command from any external publisher).",
            ),
            OpaqueFunction(function=_robot_namespaces),
        ]
    )
