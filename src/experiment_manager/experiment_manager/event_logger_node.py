"""Passive event/telemetry logger for a fleet experiment run.

Subscribes only to topics amr_ros_dg already publishes (Nav2 action
status, per-robot odometry), plus - if amr_interfaces/robot_health were
built (Phase 2) - each robot's /health/state and the fleet-wide
/fleet/faults/command. Never publishes into the fleet, never touches
fleet_manager.py or robot_health, so baseline behavior is unchanged
(Phase 1 requirement, still holds under Phase 2's own "publish, don't act"
scope). Writes:

  <run_dir>/events.jsonl      one JSON object per line, append-only
  <run_dir>/robot_<ns>.csv    odometry samples per robot
  <run_dir>/health_<ns>.csv   RobotHealth samples per robot (Phase 2)
  <run_dir>/faults.csv        every FaultCommand seen (Phase 2)
  <run_dir>/claims.csv        every StationClaim seen (Phase 5, decentralized mode only)
  <run_dir>/negotiations.csv  every TaskTransfer seen (Phase 8, decentralized mode only)
  <run_dir>/agent_decisions.jsonl  every AgentBackend.decide() call (Phase 12, decentralized mode only)
  <run_dir>/safety_events.csv  every nav2_collision_monitor CollisionMonitorState seen (real,
                                already-computed Nav2 safety-polygon triggers - STOP/SLOWDOWN/
                                APPROACH/LIMIT - the honestly-named proxy this repo uses for
                                "collisions/near-collisions", since no contact-based collision
                                detection exists in this simulation)
"""
from __future__ import annotations

import csv
import json
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatusArray
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

try:
    from amr_interfaces.msg import (
        AgentDecision,
        CoordinationModeChanged,
        FaultCommand,
        RobotHealth,
        StationClaim,
        TaskTransfer,
    )
except ImportError:  # amr_interfaces/robot_health not built - event_logger still works
    AgentDecision = None
    CoordinationModeChanged = None
    FaultCommand = None
    RobotHealth = None
    StationClaim = None
    TaskTransfer = None

_HEALTH_SAMPLE_PERIOD_SEC = 1.0

# action_msgs/msg/GoalStatus status codes
_STATUS_NAMES = {
    0: "UNKNOWN",
    1: "ACCEPTED",
    2: "EXECUTING",
    3: "CANCELING",
    4: "SUCCEEDED",
    5: "CANCELED",
    6: "ABORTED",
}

_ODOM_SAMPLE_PERIOD_SEC = 0.5

# nav2_msgs/msg/CollisionMonitorState action_type codes
_COLLISION_ACTION_NAMES = {
    0: "DO_NOTHING",
    1: "STOP",
    2: "SLOWDOWN",
    3: "APPROACH",
    4: "LIMIT",
}


class EventLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("event_logger")

        self.declare_parameter("run_dir", "")
        self.declare_parameter(
            "robot_namespaces", ["robot1", "robot2", "robot3"]
        )
        # Controls which clock is used for the recorded timestamps. "wall" (default) preserves
        # the original behavior - only the existing wall-clock `timestamp`
        # column is written, `simulation_time` is left blank. "simulation" additionally
        # writes a `simulation_time` column (self.get_clock().now(), which only returns
        # real simulation time once this node is ALSO launched with the ROS-standard
        # use_sim_time:=true parameter - run_experiment.py sets both together, see its
        # _SimTimeSource). Kept as an explicit separate parameter (not inferred from
        # use_sim_time alone) because a node can have use_sim_time:=true without this
        # logger caring, and because "did /clock ever actually arrive" isn't something
        # this node can safely assume just from a launch flag.
        self.declare_parameter("time_source", "wall")
        self._time_source = self.get_parameter("time_source").get_parameter_value().string_value

        run_dir = self.get_parameter("run_dir").get_parameter_value().string_value
        if not run_dir:
            raise RuntimeError("event_logger requires the 'run_dir' parameter")
        self._run_dir = Path(run_dir)
        self._events_path = self._run_dir / "events.jsonl"
        self._events_path.touch(exist_ok=True)

        self._last_goal_status: dict[str, int] = {}
        self._last_odom_write: dict[str, float] = {}
        self._csv_writers: dict[str, csv.writer] = {}
        self._csv_files = []

        robots = list(
            self.get_parameter("robot_namespaces")
            .get_parameter_value()
            .string_array_value
        )
        self._last_health_write: dict[str, float] = {}
        self._health_csv_writers: dict[str, csv.writer] = {}
        self._faults_csv_writer = None
        self._claims_csv_writer = None
        self._negotiations_csv_writer = None
        self._agent_decisions_path = None
        self._safety_csv_writer = None
        self._setup_safety_logging()

        for ns in robots:
            self._setup_robot(ns)

        if FaultCommand is not None:
            self._setup_fault_logging()
        if StationClaim is not None:
            self._setup_claim_logging()
        if TaskTransfer is not None:
            self._setup_negotiation_logging()
        if AgentDecision is not None:
            self._setup_decision_logging()
        if CoordinationModeChanged is not None:
            self.create_subscription(
                CoordinationModeChanged,
                "/fleet/agent/coordination_mode",
                self._on_coordination_mode_changed,
                10,
            )

        self._log_event("EXPERIMENT_LOGGER_STARTED", robot_id=None, robots=robots)
        self.get_logger().info(f"event_logger watching {robots}, writing to {run_dir}")

    def _sim_time(self):
        """Returns the current simulation time (float seconds) if this run is in
        --time-source simulation mode AND a /clock message has actually been received
        by this node's clock (ROS's use_sim_time machinery blocks self.get_clock().now()
        at zero until then) - None otherwise, so callers can distinguish "not tracked
        this run" from a real, very-early simulated timestamp."""
        if self._time_source != "simulation":
            return None
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return None
        return now.nanoseconds / 1e9

    def _setup_safety_logging(self) -> None:
        safety_csv_path = self._run_dir / "safety_events.csv"
        sf = open(safety_csv_path, "w", newline="", buffering=1)
        self._csv_files.append(sf)
        self._safety_csv_writer = csv.writer(sf)
        self._safety_csv_writer.writerow(
            ["timestamp", "simulation_time", "robot_id", "action_type", "polygon_name"]
        )

    def _setup_robot(self, ns: str) -> None:
        self.create_subscription(
            GoalStatusArray,
            f"/{ns}/navigate_to_pose/_action/status",
            lambda msg, ns=ns: self._on_goal_status(ns, msg),
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )
        self.create_subscription(
            Odometry,
            f"/{ns}/diff_drive_controller/odom",
            lambda msg, ns=ns: self._on_odom(ns, msg),
            10,
        )
        self.create_subscription(
            CollisionMonitorState,
            f"/{ns}/collision_monitor_state",
            lambda msg, ns=ns: self._on_collision_monitor_state(ns, msg),
            10,
        )

        csv_path = self._run_dir / f"robot_{ns}.csv"
        f = open(csv_path, "w", newline="", buffering=1)
        self._csv_files.append(f)
        writer = csv.writer(f)
        writer.writerow(
            ["timestamp", "simulation_time", "x", "y", "yaw_quat_z", "yaw_quat_w", "v", "w"]
        )
        self._csv_writers[ns] = writer

        if RobotHealth is not None:
            self.create_subscription(
                RobotHealth,
                f"/{ns}/health/state",
                lambda msg, ns=ns: self._on_health(ns, msg),
                10,
            )
            health_csv_path = self._run_dir / f"health_{ns}.csv"
            hf = open(health_csv_path, "w", newline="", buffering=1)
            self._csv_files.append(hf)
            health_writer = csv.writer(hf)
            health_writer.writerow(
                [
                    "timestamp", "simulation_time", "battery_soc", "battery_health", "discharge_rate",
                    "drive_left_health", "drive_right_health", "caster_health",
                    "lidar_confidence", "camera_confidence", "imu_confidence",
                    "ultrasonic_confidence", "localization_confidence", "path_blocked",
                    "nav_failures_recent", "packet_loss", "central_manager_alive",
                    "completed_tasks_recent", "distance_recent_m", "active_fault_types",
                ]
            )
            self._health_csv_writers[ns] = health_writer

    def _setup_fault_logging(self) -> None:
        self.create_subscription(
            FaultCommand, "/fleet/faults/command", self._on_fault_command, 10
        )
        faults_csv_path = self._run_dir / "faults.csv"
        ff = open(faults_csv_path, "w", newline="", buffering=1)
        self._csv_files.append(ff)
        self._faults_csv_writer = csv.writer(ff)
        self._faults_csv_writer.writerow(
            [
                "timestamp", "simulation_time", "fault_id", "robot_id", "fault_type", "severity",
                "duration_sec", "trigger_source", "clear",
            ]
        )

    def _setup_claim_logging(self) -> None:
        self.create_subscription(StationClaim, "/fleet/agent/claim", self._on_claim, 10)
        claims_csv_path = self._run_dir / "claims.csv"
        cf = open(claims_csv_path, "w", newline="", buffering=1)
        self._csv_files.append(cf)
        self._claims_csv_writer = csv.writer(cf)
        self._claims_csv_writer.writerow(
            ["timestamp", "simulation_time", "robot_id", "station_name", "side", "x", "y", "cost", "release"]
        )

    def _on_claim(self, msg) -> None:
        self._claims_csv_writer.writerow(
            [
                time.time(), self._sim_time(), msg.robot_id, msg.station_name, msg.side, msg.x, msg.y,
                msg.cost, msg.release,
            ]
        )

    def _setup_negotiation_logging(self) -> None:
        self.create_subscription(
            TaskTransfer, "/fleet/agent/task_transfer", self._on_task_transfer, 10
        )
        negotiations_csv_path = self._run_dir / "negotiations.csv"
        nf = open(negotiations_csv_path, "w", newline="", buffering=1)
        self._csv_files.append(nf)
        self._negotiations_csv_writer = csv.writer(nf)
        self._negotiations_csv_writer.writerow(
            [
                "timestamp", "simulation_time", "conversation_id", "performative", "from_robot_id",
                "to_robot_id", "station_name", "side", "x", "y", "cost", "reason_code",
            ]
        )

    def _on_task_transfer(self, msg) -> None:
        self._negotiations_csv_writer.writerow(
            [
                time.time(), self._sim_time(), msg.conversation_id, msg.performative, msg.from_robot_id,
                msg.to_robot_id, msg.station_name, msg.side, msg.x, msg.y, msg.cost,
                msg.reason_code,
            ]
        )
        self._log_event(
            "TASK_TRANSFER",
            robot_id=msg.from_robot_id,
            conversation_id=msg.conversation_id,
            performative=msg.performative,
            to_robot_id=msg.to_robot_id,
            station_name=msg.station_name,
            cost=msg.cost,
            reason_code=msg.reason_code,
        )

    def _setup_decision_logging(self) -> None:
        self.create_subscription(
            AgentDecision, "/fleet/agent/decision", self._on_agent_decision, 10
        )
        self._agent_decisions_path = self._run_dir / "agent_decisions.jsonl"
        self._agent_decisions_path.touch(exist_ok=True)

    def _on_agent_decision(self, msg) -> None:
        record = {
            "timestamp": time.time(),
            "simulation_time": self._sim_time(),
            "robot_id": msg.robot_id,
            "backend": msg.backend,
            "action": msg.action,
            "decision_latency_sec": msg.decision_latency_sec,
            # LLM-observability fields - None/null throughout when
            # has_llm_meta is false (backend=="rule", or a "hybrid" decision answered
            # deterministically), never a fabricated 0/false standing in for "not
            # applicable".
            "has_llm_meta": msg.has_llm_meta,
            "provider": msg.provider or None,
            "model": msg.model or None,
            "prompt_tokens": msg.prompt_tokens if msg.prompt_tokens >= 0 else None,
            "completion_tokens": (
                msg.completion_tokens if msg.completion_tokens >= 0 else None
            ),
            "schema_valid": msg.schema_valid if msg.has_llm_meta else None,
            "safety_valid": msg.safety_valid if msg.has_llm_meta else None,
            "fallback_used": msg.fallback_used if msg.has_llm_meta else None,
            "retry_count": msg.retry_count if msg.has_llm_meta else None,
        }
        with open(self._agent_decisions_path, "a") as f:
            f.write(json.dumps(record) + "\n")

    def _on_coordination_mode_changed(self, msg) -> None:
        # Logs a centralized-to-decentralized coordination-mode failover event, using
        # a fixed field shape (event/robot/from/to/reason) for consistent downstream parsing.
        self._log_event(
            "COORDINATION_MODE_CHANGED",
            robot_id=msg.robot_id,
            **{"from": msg.from_mode, "to": msg.to_mode},
            reason=msg.reason,
        )

    def _on_health(self, ns: str, msg) -> None:
        now = time.time()
        if now - self._last_health_write.get(ns, 0.0) < _HEALTH_SAMPLE_PERIOD_SEC:
            return
        self._last_health_write[ns] = now
        self._health_csv_writers[ns].writerow(
            [
                now, self._sim_time(), msg.battery_soc, msg.battery_health, msg.discharge_rate,
                msg.drive_left_health, msg.drive_right_health, msg.caster_health,
                msg.lidar_confidence, msg.camera_confidence, msg.imu_confidence,
                msg.ultrasonic_confidence, msg.localization_confidence, msg.path_blocked,
                msg.nav_failures_recent, msg.packet_loss, msg.central_manager_alive,
                msg.completed_tasks_recent, msg.distance_recent_m,
                "|".join(msg.active_fault_types),
            ]
        )

    def _on_fault_command(self, msg) -> None:
        self._faults_csv_writer.writerow(
            [
                time.time(), self._sim_time(), msg.fault_id, msg.robot_id, msg.fault_type, msg.severity,
                msg.duration_sec, msg.trigger_source, msg.clear,
            ]
        )
        self._log_event(
            "FAULT_CLEARED" if msg.clear else "FAULT_INJECTED",
            robot_id=msg.robot_id,
            fault_type=msg.fault_type,
            severity=msg.severity,
            duration_sec=msg.duration_sec,
            trigger_source=msg.trigger_source,
            fault_id=msg.fault_id,
        )

    def _log_event(self, event: str, *, robot_id: str | None, **fields) -> None:
        record = {
            "event": event,
            "robot_id": robot_id,
            "timestamp": time.time(),
            "simulation_time": self._sim_time(),
            **fields,
        }
        with open(self._events_path, "a") as f:
            f.write(json.dumps(record) + "\n")

    def _on_goal_status(self, ns: str, msg: GoalStatusArray) -> None:
        if not msg.status_list:
            return
        latest = msg.status_list[-1]
        status = latest.status
        if self._last_goal_status.get(ns) == status:
            return
        self._last_goal_status[ns] = status
        self._log_event(
            "NAV_GOAL_STATUS_CHANGED",
            robot_id=ns,
            status=_STATUS_NAMES.get(status, str(status)),
            goal_id=latest.goal_info.goal_id.uuid.hex()
            if hasattr(latest.goal_info.goal_id.uuid, "hex")
            else str(list(latest.goal_info.goal_id.uuid)),
        )

    def _on_odom(self, ns: str, msg: Odometry) -> None:
        now = time.time()
        if now - self._last_odom_write.get(ns, 0.0) < _ODOM_SAMPLE_PERIOD_SEC:
            return
        self._last_odom_write[ns] = now
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular
        self._csv_writers[ns].writerow(
            [now, self._sim_time(), pos.x, pos.y, ori.z, ori.w, lin.x, ang.z]
        )

    def _on_collision_monitor_state(self, ns: str, msg: CollisionMonitorState) -> None:
        self._safety_csv_writer.writerow(
            [
                time.time(), self._sim_time(), ns,
                _COLLISION_ACTION_NAMES.get(msg.action_type, str(msg.action_type)),
                msg.polygon_name,
            ]
        )
        self._log_event(
            "SAFETY_INTERVENTION",
            robot_id=ns,
            action_type=_COLLISION_ACTION_NAMES.get(msg.action_type, str(msg.action_type)),
            polygon_name=msg.polygon_name,
        )

    def destroy_node(self) -> bool:
        for f in self._csv_files:
            f.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = EventLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._log_event("EXPERIMENT_LOGGER_STOPPED", robot_id=None)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
