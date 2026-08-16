"""Launches one decentralized_agent process per robot (Mode C / Phase 5) - the
decentralized counterpart to amr_ros_dg/fleet_manager.py's single Mode A node.
Included from amr_ros_dg/fleet.launch.py when coordination:=decentralized.

    ros2 launch fleet_coordination decentralized_agents.launch.py num_robots:=3 seed:=42
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _agent_actions(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    seed = LaunchConfiguration("seed").perform(context)
    agent_backend = LaunchConfiguration("agent_backend").perform(context)
    start_dormant = LaunchConfiguration("start_dormant").perform(context) == "true"
    manager_timeout_sec = float(LaunchConfiguration("manager_timeout_sec").perform(context))
    memory_enabled = LaunchConfiguration("memory_enabled").perform(context) == "true"

    actions = []
    for i in range(1, num_robots + 1):
        ns = f"robot{i}"
        agent_node = Node(
            package="fleet_coordination",
            executable="decentralized_agent",
            name="decentralized_agent",
            namespace=ns,
            output="screen",
            parameters=[
                {
                    "robot_id": ns,
                    "seed": int(seed),
                    "use_sim_time": True,
                    "agent_backend": agent_backend,
                    "start_dormant": start_dormant,
                    "manager_timeout_sec": manager_timeout_sec,
                    "memory_enabled": memory_enabled,
                }
            ],
        )
        # Same 20s bringup-wait rationale as fleet.launch.py's delayed_fleet_manager -
        # every robot's Nav2 stack needs time to come up before an agent starts sending
        # goals. Staggered slightly per robot (matching fleet.launch.py's spawn stagger)
        # so N agents don't all hit /fleet/agent/claim in the exact same instant.
        actions.append(TimerAction(period=20.0 + 0.3 * (i - 1), actions=[agent_node]))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="num_robots",
                default_value="3",
                description="Must match the fleet.launch.py num_robots this is coordinating.",
            ),
            DeclareLaunchArgument(name="seed", default_value="0"),
            DeclareLaunchArgument(
                name="agent_backend",
                default_value="rule",
                description="'rule' (default, Phase 6 RuleAgent), 'llm' (Phase 7 "
                "LLMAgent backed by a local Ollama server, falling back to RuleAgent "
                "on malformed/unsafe/unreachable-server responses), or 'hybrid' "
                "(Phase 9 Mode E: RuleAgent by default, LLMAgent only when the top "
                "two candidate stations are within HybridAgent's ambiguity_margin).",
            ),
            DeclareLaunchArgument(
                name="start_dormant",
                default_value="false",
                description="Centralized -> decentralized failover support. "
                "'true': every agent starts DORMANT (touches nothing) until the "
                "central fleet_manager's /fleet/manager/heartbeat has been missing for "
                "manager_timeout_sec, then activates its normal cycle - used by "
                "--coordination centralized_then_failover. 'false' (default): "
                "agent is active from the start.",
            ),
            DeclareLaunchArgument(
                name="manager_timeout_sec",
                default_value="5.0",
                description="Only used when start_dormant:=true.",
            ),
            DeclareLaunchArgument(
                name="memory_enabled",
                default_value="false",
                description="Episodic peer memory. 'true': each agent "
                "tracks real ACCEPT/REJECT/TIMEOUT outcomes of task-transfer "
                "negotiations it initiated, per peer, and nudges "
                "Conversation.select_winner toward historically-reliable peers. "
                "'false' (default): unchanged prior behavior.",
            ),
            OpaqueFunction(function=_agent_actions),
        ]
    )
