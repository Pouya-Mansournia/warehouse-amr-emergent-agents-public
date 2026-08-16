"""
Multi-robot warehouse fleet simulation: N robots (robot1..robotN), each running its OWN
SLAM Toolbox + Nav2 stack, spawned into the 50x50m warehouse world, plus a single
fleet_manager.py node that keeps assigning idle robots random station-to-station tasks.

Usage:
    ros2 launch amr_ros_dg fleet.launch.py num_robots:=5

Built by extending launch/gazebo.launch.py's pattern (GZ_SIM_RESOURCE_PATH env var trick,
xacro Command substitution, ros_gz_sim create spawn, ros_gz_bridge parameter_bridge,
TimerAction delays, controller spawners) rather than inventing a new one - read that file
first if this one is confusing, most of the "why" comments here only cover what's NEW for
the multi-robot case.

============================== TF / namespacing design (read this first) ==============================
This is the single biggest design decision in this file, so it's spelled out fully here
rather than scattered across per-node comments:

  - Gazebo-side uniqueness (model/link/sensor ENTITY names, gz-transport TOPIC names) is
    handled by (a) passing a distinct `-name robotN` to each `ros_gz_sim create` spawn call
    below, and (b) the urdf xacro's `robot_name` arg, which prefixes only its gz-sim
    sensor <topic> overrides (camera/d435i, ultrasonic_N/scan, scan) - see the big comment
    block at the top of urdf/amr_ros_dg.urdf.xacro for the full reasoning on why link/joint
    names themselves are NOT prefixed (gz-sim scopes those per spawned model already, no
    prefixing needed there).

  - ROS-side uniqueness (topics, TF) is handled by ROS 2 NAMESPACING, not by string-
    prefixing frame_ids. Every node for robot i (robot_state_publisher, the diff-drive
    controller stack, the sensor bridge, slam_toolbox, nav2) is launched inside the
    `robotN` namespace via `PushRosNamespace`/`namespace=`. Namespacing makes tf2 topics
    resolve to `/robotN/tf` and `/robotN/tf_static`, which is what actually isolates each
    robot's TF tree - NOT unique frame_id strings. frame_ids inside config/nav2_params.yaml
    are left as plain "map"/"odom"/"base_link" for every robot, matching what robot_state_
    publisher publishes (it mirrors the URDF's unprefixed link names verbatim) and what
    Nav2/SLAM Toolbox expect by default inside a namespace - this is the standard,
    documented multi-robot Nav2 pattern (see the turtlebot3 multi-robot demo for the same
    approach). Each robot therefore has its OWN independent `map` -> `odom` -> `base_link`
    chain, with no shared global frame across robots - correct here because each robot runs
    its own SLAM Toolbox instance building its own map, per spec (not a shared map).
============================================================================================================
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.actions import EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node, PushRosNamespace
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def _robot_actions(context, *args, **kwargs):
    """
    OpaqueFunction body: num_robots is only known at launch time (it's a LaunchConfiguration),
    so the per-robot action list has to be built here rather than at module-import time.
    """
    pkg_share = get_package_share_directory("amr_ros_dg")
    xacro_path = os.path.join(pkg_share, "urdf", "amr_ros_dg.urdf.xacro")
    controllers_yaml = os.path.join(pkg_share, "config", "amr_ros_dg_controllers.yaml")
    nav2_params_yaml = os.path.join(pkg_share, "config", "nav2_params.yaml")

    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    enable_cameras = LaunchConfiguration("enable_cameras").perform(context) == "true"

    # Spread starting poses along the center aisle (x=0, clear of the station rows at
    # x=+/-6.9 and the east/west walls at +/-7.5), spaced out in y instead - the world was
    # resized from the original 50x50m spec down to a lighter 15x20m footprint for fleet
    # testing, so the old "spread along x" layout no longer fits (only +/-7.5m of x room
    # vs the +/-25m it was designed for). y spacing of 1.0m keeps up to ~18 robots inside
    # +/-9m before hitting the north/south walls at +/-10m; num_robots is capped at 20 per
    # spec, so a caller pushing past ~18 gets tighter (but still non-overlapping) spacing
    # rather than a crash.
    start_x = 0.0
    start_y_spacing = 1.0

    all_actions = []

    for i in range(1, num_robots + 1):
        ns = f"robot{i}"
        start_y = (i - (num_robots + 1) / 2.0) * start_y_spacing

        robot_description = ParameterValue(
            Command([
                "xacro ", xacro_path, " robot_name:=", ns,
                " namespace_controller_manager:=true",
                " enable_cameras:=", "true" if enable_cameras else "false",
            ]),
            value_type=str,
        )

        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=ns,
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
            # robot_state_publisher publishes tf/tf_static to the hardcoded ABSOLUTE topics
            # "/tf"/"/tf_static" (leading slash - this bypasses ROS namespacing entirely,
            # unlike an ordinary relative topic such as "robot_description" above, which
            # namespace= alone already correctly scopes to /robotN/robot_description).
            # Remapping to a bare RELATIVE "tf" (relying on __ns:=/robotN to then namespace
            # it) did NOT work - confirmed live in WSL: `ps aux` showed the process actually
            # running with `-r /tf:=tf -r /tf_static:=tf_static` exactly as intended, yet
            # `ros2 node info /robot1/robot_state_publisher` still listed /tf (unnamespaced)
            # as its publisher. Remapping straight to the fully-qualified absolute topic
            # (f"/{ns}/tf") removes any dependency on how/when namespace substitution is
            # applied relative to remap rules, and is what actually worked.
            remappings=[("/tf", f"/{ns}/tf"), ("/tf_static", f"/{ns}/tf_static")],
        )

        # ros_gz_sim create reads the URDF from this robot's OWN namespaced
        # /robotN/robot_description topic (published by the robot_state_publisher node
        # above, which is namespaced) - each robot needs -name robotN too so Gazebo gives
        # every spawned model a distinct entity name (gazebo.launch.py's single-robot
        # version hardcodes "-name amr_ros_dg" since there's only ever one there).
        spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-topic", f"/{ns}/robot_description",
                "-name", ns,
                "-x", str(start_x),
                "-y", str(start_y),
                "-z", "0.245",  # see gazebo.launch.py for how this height is derived
            ],
            output="screen",
        )

        # Per-robot sensor + cmd_vel/odom bridge, gz-side topics matching the robot_name
        # prefix baked into the xacro's sensor <topic> elements, ROS-side topics namespaced
        # so they land under /robotN/... automatically (no explicit remap needed - the
        # bridge node itself is launched inside the PushRosNamespace group below).
        # Camera bridge topics skipped when enable_cameras is false - the xacro doesn't
        # generate those <sensor> elements in that case (see the urdf's enable_cameras arg
        # comment), so bridging them here would just be dead entries.
        camera_bridge_topics = (
            [
                f"/{ns}/camera/d435i@sensor_msgs/msg/Image[gz.msgs.Image",
                f"/{ns}/camera/d435i/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                f"/{ns}/camera/floor@sensor_msgs/msg/Image[gz.msgs.Image",
                f"/{ns}/camera/floor/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ]
            if enable_cameras
            else []
        )
        sensor_bridge_topics = [
            f"/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ] + camera_bridge_topics + [
            f"/{ns}/ultrasonic_{k}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
            for k in range(1, 13)
        ]
        sensor_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="sensor_bridge",
            namespace=ns,
            arguments=sensor_bridge_topics,
            output="screen",
        )

        # --controller-manager-timeout raised from the spawner's 10s default to 60s: at
        # fleet sizes around 10 robots, WSL2's CPU/IO gets saturated by N simultaneous
        # controller_manager + Nav2 + SLAM Toolbox stacks bringing up at once, and the
        # spawner's service calls to its OWN robot's controller_manager (a per-robot
        # resource, not shared across robots) started timing out purely from system load -
        # confirmed live via "Failed to acquire lock after multiple attempts" followed by
        # "process has died [exit code 1]" for several robots' spawners at num_robots:=10.
        # Waiting longer instead of dying fixes it without touching any actual logic.
        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            namespace=ns,
            arguments=[
                "joint_state_broadcaster", "--controller-manager", "controller_manager",
                "--controller-manager-timeout", "60",
            ],
            output="screen",
        )

        # Reuses config/amr_ros_dg_controllers.yaml UNCHANGED - its joint list
        # (left_wheel_joint, right_wheel_joint, ...) already matches the URDF, because the
        # urdf xacro's robot_name arg deliberately does NOT prefix joint names (see the
        # design comment at the top of this file / in the xacro). Running the spawner
        # inside the robotN namespace is what scopes it to that robot's own
        # controller_manager - no per-robot YAML variant needed.
        diff_drive_spawner = Node(
            package="controller_manager",
            executable="spawner",
            namespace=ns,
            arguments=[
                "diff_drive_controller", "--controller-manager", "controller_manager",
                "--controller-manager-timeout", "60",
            ],
            output="screen",
        )

        # SLAM Toolbox, async/online mode - each robot builds its OWN map independently
        # (no shared map, per spec). odom_frame/base_frame/map_frame left unprefixed
        # ("odom"/"base_link"/"map") - see the TF design comment at the top of this file.
        #
        # CORRECTNESS NOTE (found the hard way - map->odom TF and /map never appeared,
        # confirmed live in WSL with `ros2 topic list | grep robot1` showing no /robot1/map
        # at all, and slam_toolbox's own log showing nothing past "process started"):
        # async_slam_toolbox_node is a ROS 2 LIFECYCLE node - unlike a plain Node(), it does
        # NOT self-activate. slam_toolbox's own bundled launch/online_async_launch.py
        # (found via `find /opt/ros -iname "*online_async*"`) uses launch_ros.LifecycleNode
        # plus an explicit EmitEvent(ChangeState(...CONFIGURE)) followed by a
        # RegisterEventHandler(OnStateTransition) that fires ACTIVATE once configuring
        # finishes - a plain Node() (what this used to be) never emits either transition,
        # so the node sat in "unconfigured" forever, silently. Replicating that exact
        # pattern here, namespaced, instead of nav2's lifecycle_manager (SLAM Toolbox isn't
        # one of nav2_bringup's managed nodes).
        slam_toolbox_node = LifecycleNode(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            namespace=ns,
            output="screen",
            # Base params = slam_toolbox's OWN default config (found via
            # `find /opt/ros -iname "*online_async*"`), not a hand-rolled 5-key dict -
            # slam_toolbox reached "Registering sensor" with just those 5 keys but then
            # never published /map at all (confirmed live in WSL: process kept running, no
            # crash, no further log output, /robotN/map never appeared) - likely missing
            # some param (mode, resolution, update intervals, etc.) that the full default
            # file provides and this trimmed-down version silently left at an unusable
            # default. Only the per-robot frame/topic names are overridden on top.
            parameters=[
                os.path.join(
                    get_package_share_directory("slam_toolbox"),
                    "config",
                    "mapper_params_online_async.yaml",
                ),
                {
                    "use_sim_time": True,
                    "odom_frame": "odom",
                    "base_frame": "base_link",
                    "map_frame": "map",
                    "scan_topic": "scan",
                    # slam_toolbox's default max_laser_range (25m) exceeds this robot's
                    # lidar (12m, see the sensor block in the xacro) - harmless (just a
                    # WARN observed live in WSL), but matched here to silence it.
                    "max_laser_range": 12.0,
                },
            ],
            # Same /tf, /tf_static absolute-topic remap as robot_state_publisher_node above,
            # and same fix (remap target must be the fully-qualified /{ns}/tf, not a bare
            # relative "tf" - see that Node's comment for why).
            remappings=[("/tf", f"/{ns}/tf"), ("/tf_static", f"/{ns}/tf_static")],
        )
        slam_toolbox_configure = EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(slam_toolbox_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            )
        )
        slam_toolbox_activate = RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=slam_toolbox_node,
                start_state="configuring",
                goal_state="inactive",
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(slam_toolbox_node),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )
                    )
                ],
            )
        )

        # nav2_bringup's navigation_launch.py (not bringup_launch.py) - deliberately, since
        # SLAM Toolbox above already supplies "map" via TF + the /map topic, so the
        # map_server + amcl stack that bringup_launch.py would otherwise start is not needed
        # (and would fight with SLAM Toolbox for map ownership).
        #
        # CORRECTNESS NOTE (found the hard way, via a live SIGABRT/"No critics defined for
        # FollowPath" crash loop): navigation_launch.py's own Node() definitions do NOT set
        # namespace= on themselves - it relies entirely on being wrapped in an external
        # PushRosNamespace. Its `configured_params` ARE rewritten with `root_key=namespace`
        # (nav2_common's RewrittenYaml), which nests the whole params file one level deeper
        # under a "robot1:" key - matching a node whose fully-qualified name is
        # "/robot1/controller_server". If this IncludeLaunchDescription is NOT also wrapped
        # in PushRosNamespace(ns), the controller_server node actually comes up as plain
        # "/controller_server" (no namespace), which no longer matches the rewritten
        # params file's "robot1: {controller_server: {...}}" structure - so NONE of its
        # params apply (defaults used instead, including an empty critics list -> DWB
        # aborts at configure()). Confirmed by testing controller_server standalone with
        # this same config/nav2_params.yaml (unnamespaced, un-rewritten): critics loaded
        # fine - the bug was purely this missing namespace wrapper, not the params file.
        nav2_navigation = GroupAction(
            actions=[
                PushRosNamespace(ns),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory("nav2_bringup"),
                            "launch",
                            "navigation_launch.py",
                        )
                    ),
                    launch_arguments={
                        "namespace": ns,
                        "use_sim_time": "true",
                        "params_file": nav2_params_yaml,
                        "autostart": "true",
                    }.items(),
                ),
            ]
        )

        # robot_state_publisher_node and sensor_bridge set namespace=ns directly on
        # themselves instead of using PushRosNamespace - unlike navigation_launch.py above,
        # these Node() objects DO already accept namespace= directly, so wrapping them in
        # PushRosNamespace too would double-apply it (e.g. "/robot1/robot1/..."), which is
        # exactly the bug that made ros_gz_sim create wait forever on a topic that never
        # existed (robot_state_publisher was actually publishing one level deeper).
        robot_group = GroupAction(
            actions=[
                robot_state_publisher_node,
                sensor_bridge,
            ]
        )

        # Controllers can only be spawned once this robot's entity (and the
        # gz_ros2_control controller_manager it brings up) exists - chain off spawn_robot
        # exiting, same pattern as gazebo.launch.py.
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

        # SLAM/Nav2 need the controllers (and therefore odometry/TF) up first. This was
        # originally a fixed TimerAction(period=8.0) - broke down under real fleet load
        # (confirmed live in WSL with 3 robots): diff_drive_controller sometimes took 20+
        # seconds to actually activate (controller_manager/spawner contention across
        # multiple robots), so Nav2's local_costmap started trying to activate against a
        # transform that didn't exist yet, HUNG waiting for it, and stalled the entire
        # lifecycle_manager bringup sequence forever (bt_navigator etc. never left
        # "inactive" - `ros2 lifecycle get` confirmed this directly, and NavigateToPose
        # goals were rejected in an infinite loop by fleet_manager.py as a result). Chaining
        # off diff_drive_spawner's actual exit (spawners exit only once the controller is
        # genuinely configured+activated) removes the race entirely, regardless of how long
        # that takes on a loaded system.
        delayed_autonomy = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=diff_drive_spawner,
                on_exit=[
                    slam_toolbox_node,
                    slam_toolbox_configure,
                    slam_toolbox_activate,
                    nav2_navigation,
                ],
            )
        )

        # Stagger spawn_robot itself across robots (0.5s apart) on top of the shared 5s
        # head-start below - spawning N robots in the exact same instant is a heavier hit
        # on the "wait for world/entity-creation service" race gazebo.launch.py's comment
        # already describes, worth spreading out even at N=5, more so approaching N=20.
        delayed_spawn = TimerAction(period=5.0 + 0.5 * (i - 1), actions=[spawn_robot])

        all_actions += [robot_group, delayed_spawn, delayed_controllers, delayed_autonomy]

    return all_actions


def generate_launch_description():
    pkg_share = get_package_share_directory("amr_ros_dg")
    default_world_path = os.path.join(pkg_share, "worlds", "warehouse_fleet_world.sdf")

    # Same GZ_SIM_RESOURCE_PATH trick as gazebo.launch.py - gz sim resolves
    # "model://amr_ros_dg/meshes/*.STL" (what URDF's package:// URIs become after SDF
    # conversion) by searching this path for a folder literally named "amr_ros_dg".
    gz_resource_path = os.path.dirname(pkg_share)
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if existing_resource_path:
        gz_resource_path = gz_resource_path + os.pathsep + existing_resource_path
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=gz_resource_path
    )

    num_robots_arg = DeclareLaunchArgument(
        name="num_robots",
        default_value="5",
        description="Number of robots to spawn (robot1..robotN). Architecture scales to 20; "
        "not yet performance-tested that high (see README known-limitations).",
    )

    world_arg = DeclareLaunchArgument(
        name="world",
        default_value=default_world_path,
        description="Absolute path to the Gazebo Sim warehouse world to load",
    )

    seed_arg = DeclareLaunchArgument(
        name="seed",
        default_value="0",
        description="Seeds fleet_manager's task-generation RNG (station selection, initial "
        "input/output side per robot) for reproducibility. Same seed "
        "+ same code -> same sequence of station assignments.",
    )

    coordination_arg = DeclareLaunchArgument(
        name="coordination",
        default_value="centralized",
        description="'centralized' (Mode A, default): single fleet_manager node assigns "
        "every robot's tasks. 'decentralized' (Mode C): one independent "
        "fleet_coordination agent process per robot, coordinating peer-to-peer over "
        "/fleet/agent/claim, no central broker. 'centralized_then_failover' "
        "runs BOTH fleet_manager AND per-robot decentralized_agent processes "
        "from the start - agents begin DORMANT (touching nothing) and watch "
        "fleet_manager's /fleet/manager/heartbeat; once it's been missing for "
        "manager_timeout_sec, they activate their normal coordination cycle "
        "(agent_backend selects rule/llm/hybrid, same as pure decentralized mode) - "
        "the fair recovery comparison against Mode B's no-recovery baseline.",
    )

    manager_timeout_sec_arg = DeclareLaunchArgument(
        name="manager_timeout_sec",
        default_value="5.0",
        description="Only used when coordination:=centralized_then_failover - seconds "
        "of missing /fleet/manager/heartbeat before agents conclude fleet_manager is "
        "gone and activate decentralized failover.",
    )

    agent_backend_arg = DeclareLaunchArgument(
        name="agent_backend",
        default_value="rule",
        description="Only used when coordination:=decentralized. 'rule' (default, "
        "RuleAgent, deterministic cost formula), 'llm' (LLMAgent "
        "backed by a local Ollama server at 127.0.0.1:11434, falling back to RuleAgent "
        "on malformed/unsafe/unreachable-server responses), or 'hybrid' (Mode "
        "E: RuleAgent by default, LLMAgent only for ambiguous near-tied decisions).",
    )

    memory_enabled_arg = DeclareLaunchArgument(
        name="memory_enabled",
        default_value="false",
        description="Enables episodic peer memory. Only used when "
        "coordination is decentralized or centralized_then_failover.",
    )

    # enable_cameras: default true. Set false to skip the d435i/floor RGB camera <sensor>
    # elements on every robot (lidar + ultrasonics still run - Nav2/SLAM only need the
    # lidar). Real, measured GPU render cost cut - see the urdf's enable_cameras comment.
    enable_cameras_arg = DeclareLaunchArgument(
        name="enable_cameras",
        default_value="true",
        description="Spawn the d435i/floor RGB cameras on each robot. Set false to cut GPU "
        "render load (Nav2/SLAM don't need them, only the lidar) when running headless:=false "
        "with several robots.",
    )

    # headless: default true. `top` in a live WSL session showed "gz sim gui" alone using
    # ~127% CPU on top of "gz sim server"'s ~236% at num_robots:=3 - with load average
    # (21+) far past the 12 available cores, this GUI render cost was a direct contributor
    # to robots crawling. The web dashboard (scripts/web_dashboard.py) already shows live
    # camera + lidar views per robot, so the Gazebo GUI window is redundant for fleet
    # testing - pass headless:=false to get it back (e.g. for visually placing a sensor).
    headless_arg = DeclareLaunchArgument(
        name="headless",
        default_value="true",
        description="Run Gazebo Sim without its GUI window (-s) to save CPU. The web "
        "dashboard's camera/lidar views work either way. Set to 'false' to see the 3D view.",
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [
                LaunchConfiguration("world"),
                " -r -v 4",
                PythonExpression(["' -s' if '", LaunchConfiguration("headless"), "' == 'true' else ''"]),
            ]
        }.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # coordination:=centralized (default, Mode A) launches fleet_manager.py: a single
    # node, NOT per-robot/namespaced - it talks to every robot's namespaced
    # NavigateToPose action server itself (/robotN/navigate_to_pose), and owns the one
    # global in-process station-reservation dict, so it must not be duplicated per
    # robot. coordination:=decentralized (Mode C, Phase 5) launches
    # fleet_coordination's decentralized_agents.launch.py instead: one independent OS
    # process PER ROBOT, no shared memory, no central broker - see that package's
    # decentralized_agent.py module docstring. Exactly one of the two runs; IfCondition
    # is used (not a plain if/else at launch-description-build time) because
    # `coordination` is a LaunchConfiguration, only resolved at launch time.
    # There is also a third value, 'centralized_then_failover', which needs BOTH
    # fleet_manager_node (identical to plain 'centralized') AND decentralized_agents
    # (identical to plain 'decentralized' except start_dormant:=true) running at once -
    # so fleet_manager launches whenever coordination is EITHER 'centralized' or
    # 'centralized_then_failover', and decentralized_agents launches whenever it's
    # EITHER 'decentralized' or 'centralized_then_failover' (with start_dormant set
    # accordingly below). Exactly one of {centralized, centralized_then_failover,
    # decentralized} is ever chosen, so 'decentralized' and 'centralized_then_failover'
    # never both launch decentralized_agents at once for the same run.
    centralized_condition = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration("coordination"), "' == 'centralized' or '",
            LaunchConfiguration("coordination"), "' == 'centralized_then_failover'",
        ])
    )
    decentralized_condition = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration("coordination"), "' == 'decentralized' or '",
            LaunchConfiguration("coordination"), "' == 'centralized_then_failover'",
        ])
    )
    start_dormant_expr = PythonExpression([
        "'true' if '", LaunchConfiguration("coordination"),
        "' == 'centralized_then_failover' else 'false'",
    ])

    fleet_manager_node = Node(
        package="amr_ros_dg",
        executable="fleet_manager.py",
        name="fleet_manager",
        output="screen",
        condition=centralized_condition,
        parameters=[
            {
                "num_robots": LaunchConfiguration("num_robots"),
                "seed": LaunchConfiguration("seed"),
                "use_sim_time": True,
            }
        ],
    )
    # Give every robot's Nav2 stack time to finish bringing up before the fleet manager
    # starts sending goals - crude but matches the delay style already used throughout this
    # file/gazebo.launch.py (real dependency would be watching each Nav2 lifecycle reach
    # ACTIVE, which none of the launch files in this package do yet).
    delayed_fleet_manager = TimerAction(period=20.0, actions=[fleet_manager_node])

    decentralized_agents = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fleet_coordination"),
                "launch",
                "decentralized_agents.launch.py",
            )
        ),
        launch_arguments={
            "num_robots": LaunchConfiguration("num_robots"),
            "seed": LaunchConfiguration("seed"),
            "agent_backend": LaunchConfiguration("agent_backend"),
            "start_dormant": start_dormant_expr,
            "manager_timeout_sec": LaunchConfiguration("manager_timeout_sec"),
            "memory_enabled": LaunchConfiguration("memory_enabled"),
        }.items(),
        condition=decentralized_condition,
    )

    # NOTE (out of scope, not implemented): a `disable_extra_sensors` launch arg that skips
    # spawning the d435i/floor camera/12x ultrasonic gz-sim sensors would meaningfully cut
    # per-robot GPU render + bridge overhead at fleet sizes approaching 20 (see README
    # scaling section) - left in for now since they're harmless at 4-6 robots and adding
    # the arg means threading a condition through the xacro's macro instantiations too.

    return LaunchDescription(
        [
            set_gz_resource_path,
            num_robots_arg,
            world_arg,
            seed_arg,
            coordination_arg,
            manager_timeout_sec_arg,
            agent_backend_arg,
            memory_enabled_arg,
            headless_arg,
            enable_cameras_arg,
            gz_sim,
            clock_bridge,
            OpaqueFunction(function=_robot_actions),
            delayed_fleet_manager,
            decentralized_agents,
        ]
    )
