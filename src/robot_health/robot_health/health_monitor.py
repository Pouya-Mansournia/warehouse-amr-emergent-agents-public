#!/usr/bin/env python3
"""Per-robot operational-health publisher.

One node manages every robot in the fleet (same pattern as amr_ros_dg's fleet_manager.py:
a single multi-threaded node, not one process per robot). For each robot it:

  - derives a baseline health state from real telemetry it already has access to
    (distance travelled from odometry, task success/failure counts from NavigateToPose
    goal-status transitions, elapsed time for baseline battery drain);
  - applies any currently-active injected faults (see faults.py) on top of that baseline,
    each tick;
  - publishes the result as amr_interfaces/RobotHealth on /<ns>/health/state.

It does not act on this state (no behavior change to any robot) - that is the
coordination layer's responsibility. It only subscribes to
/fleet/faults/command to learn which faults are currently active; it never publishes
into the fleet_manager/Nav2/control stack.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatusArray, GoalStatus
from amr_interfaces.msg import FaultCommand, RobotHealth
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from robot_health.faults import apply_fault

_TICK_PERIOD_SEC = 0.5
_BASELINE_DISCHARGE_PER_SEC = 1.0 / (2.0 * 3600.0)  # ~2h to empty while idle
_DISCHARGE_PER_METER = 1.0 / 2000.0  # ~2km of driving to empty, on top of idle drain


@dataclass
class RobotHealthState:
    """Mutable per-tick working state for one robot. Faults in faults.py mutate this
    directly; health_monitor resets the *_health/*_confidence fields to their undegraded
    baseline each tick before re-applying faults, except for permanent-degradation fields
    (battery_health, battery_soc) which persist across ticks by design."""

    robot_id: str
    battery_soc: float = 1.0
    battery_health: float = 1.0
    discharge_rate: float = 0.0
    discharge_rate_multiplier: float = 1.0
    drive_left_health: float = 1.0
    drive_right_health: float = 1.0
    caster_health: float = 1.0
    lidar_confidence: float = 1.0
    camera_confidence: float = 1.0
    imu_confidence: float = 1.0
    ultrasonic_confidence: float = 1.0
    localization_confidence: float = 1.0
    path_blocked: bool = False
    packet_loss: float = 0.0
    central_manager_alive: bool = True
    distance_recent_m: float = 0.0
    completed_tasks_recent: int = 0
    nav_failures_recent: int = 0
    _last_odom_xy: Optional[Tuple[float, float]] = field(default=None, repr=False)
    _last_distance_for_discharge: float = field(default=0.0, repr=False)


@dataclass
class _ActiveFault:
    fault_id: str
    fault_type: str
    severity: float
    expires_at: Optional[float]  # monotonic seconds, None = until explicitly cleared


class HealthMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("health_monitor")

        self.declare_parameter("robot_namespaces", ["robot1", "robot2", "robot3"])
        robots = list(
            self.get_parameter("robot_namespaces").get_parameter_value().string_array_value
        )

        self._states: Dict[str, RobotHealthState] = {ns: RobotHealthState(robot_id=ns) for ns in robots}
        self._active_faults: Dict[str, List[_ActiveFault]] = {ns: [] for ns in robots}
        self._health_publishers: Dict[str, "rclpy.publisher.Publisher"] = {}

        cb_group = MutuallyExclusiveCallbackGroup()
        for ns in robots:
            self._health_publishers[ns] = self.create_publisher(RobotHealth, f"/{ns}/health/state", 10)
            self.create_subscription(
                Odometry,
                f"/{ns}/diff_drive_controller/odom",
                lambda msg, ns=ns: self._on_odom(ns, msg),
                10,
                callback_group=cb_group,
            )
            self.create_subscription(
                GoalStatusArray,
                f"/{ns}/navigate_to_pose/_action/status",
                lambda msg, ns=ns: self._on_goal_status(ns, msg),
                QoSPresetProfiles.SYSTEM_DEFAULT.value,
                callback_group=cb_group,
            )
        self._last_goal_status: Dict[str, int] = {}

        self.create_subscription(
            FaultCommand,
            "/fleet/faults/command",
            self._on_fault_command,
            10,
            callback_group=cb_group,
        )

        self._start_time = time.monotonic()
        self.create_timer(_TICK_PERIOD_SEC, self._tick, callback_group=cb_group)
        self.get_logger().info(f"health_monitor watching {robots}")

    # ------------------------------------------------------------------ telemetry inputs

    def _on_odom(self, ns: str, msg: Odometry) -> None:
        state = self._states[ns]
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if state._last_odom_xy is not None:
            dx = x - state._last_odom_xy[0]
            dy = y - state._last_odom_xy[1]
            state.distance_recent_m += (dx * dx + dy * dy) ** 0.5
        state._last_odom_xy = (x, y)

    def _on_goal_status(self, ns: str, msg: GoalStatusArray) -> None:
        if not msg.status_list:
            return
        status = msg.status_list[-1].status
        if self._last_goal_status.get(ns) == status:
            return
        self._last_goal_status[ns] = status
        state = self._states[ns]
        if status == GoalStatus.STATUS_SUCCEEDED:
            state.completed_tasks_recent += 1
        elif status == GoalStatus.STATUS_ABORTED:
            state.nav_failures_recent += 1

    # ------------------------------------------------------------------ fault commands

    def _targets(self, robot_id: str) -> List[str]:
        if robot_id in ("", "*"):
            return list(self._states.keys())
        return [robot_id] if robot_id in self._states else []

    def _on_fault_command(self, msg: FaultCommand) -> None:
        targets = self._targets(msg.robot_id)
        if not targets:
            self.get_logger().warn(f"fault command for unknown robot_id '{msg.robot_id}', dropped")
            return
        for ns in targets:
            active = self._active_faults[ns]
            if msg.clear:
                before = len(active)
                self._active_faults[ns] = [f for f in active if f.fault_type != msg.fault_type]
                if len(self._active_faults[ns]) < before:
                    self.get_logger().info(f"{ns}: cleared fault {msg.fault_type}")
                continue

            expires_at = (
                time.monotonic() + msg.duration_sec if msg.duration_sec > 0.0 else None
            )
            active.append(
                _ActiveFault(
                    fault_id=msg.fault_id,
                    fault_type=msg.fault_type,
                    severity=msg.severity,
                    expires_at=expires_at,
                )
            )
            self.get_logger().info(
                f"{ns}: fault {msg.fault_type} active (severity={msg.severity:.2f}, "
                f"duration={msg.duration_sec:.0f}s, source={msg.trigger_source})"
            )

    # ------------------------------------------------------------------ tick

    def _tick(self) -> None:
        now = time.monotonic()
        for ns, state in self._states.items():
            self._expire_faults(ns, now)
            self._advance_baseline(state)
            self._reset_degradable_fields(state)
            for fault in self._active_faults[ns]:
                if not apply_fault(state, fault.fault_type, fault.severity):
                    self.get_logger().warn(f"{ns}: unknown fault_type '{fault.fault_type}', ignored")
            self._publish(ns, state)

    def _expire_faults(self, ns: str, now: float) -> None:
        still_active = [
            f for f in self._active_faults[ns] if f.expires_at is None or f.expires_at > now
        ]
        expired = [f for f in self._active_faults[ns] if f not in still_active]
        for f in expired:
            self.get_logger().info(f"{ns}: fault {f.fault_type} expired (recovered)")
        self._active_faults[ns] = still_active

    def _advance_baseline(self, state: RobotHealthState) -> None:
        distance_delta = state.distance_recent_m - state._last_distance_for_discharge
        state._last_distance_for_discharge = state.distance_recent_m
        drain = (
            _BASELINE_DISCHARGE_PER_SEC * _TICK_PERIOD_SEC
            + _DISCHARGE_PER_METER * max(distance_delta, 0.0)
        ) * state.discharge_rate_multiplier / max(state.battery_health, 0.05)
        state.battery_soc = max(0.0, state.battery_soc - drain)
        state.discharge_rate = (
            _BASELINE_DISCHARGE_PER_SEC * state.discharge_rate_multiplier
        )
        state.discharge_rate_multiplier = 1.0  # faults re-apply their multiplier every tick

    def _reset_degradable_fields(self, state: RobotHealthState) -> None:
        # battery_soc / battery_health are NOT reset - they persist/accumulate. Everything
        # else reflects only currently-active faults, so it must return to baseline before
        # this tick's active faults are re-applied (otherwise a cleared fault's effect
        # would never go away).
        state.drive_left_health = 1.0
        state.drive_right_health = 1.0
        state.caster_health = 1.0
        state.lidar_confidence = 1.0
        state.camera_confidence = 1.0
        state.imu_confidence = 1.0
        state.ultrasonic_confidence = 1.0
        state.localization_confidence = 1.0
        state.path_blocked = False
        state.packet_loss = 0.0
        state.central_manager_alive = True

    def _publish(self, ns: str, state: RobotHealthState) -> None:
        # localization_confidence is derived (not independently faulted): a composite of
        # the sensors Nav2/SLAM actually rely on for pose estimation, further reduced by
        # any direct LOCALIZATION_DEGRADATION fault already applied above.
        state.localization_confidence = min(
            state.localization_confidence,
            0.6 * state.lidar_confidence + 0.4 * state.imu_confidence,
        )

        msg = RobotHealth()
        msg.robot_id = ns
        msg.stamp = self.get_clock().now().to_msg()
        msg.battery_soc = float(state.battery_soc)
        msg.battery_health = float(state.battery_health)
        msg.discharge_rate = float(state.discharge_rate)
        msg.drive_left_health = float(state.drive_left_health)
        msg.drive_right_health = float(state.drive_right_health)
        msg.caster_health = float(state.caster_health)
        msg.lidar_confidence = float(state.lidar_confidence)
        msg.camera_confidence = float(state.camera_confidence)
        msg.imu_confidence = float(state.imu_confidence)
        msg.ultrasonic_confidence = float(state.ultrasonic_confidence)
        msg.localization_confidence = float(state.localization_confidence)
        msg.path_blocked = bool(state.path_blocked)
        msg.nav_failures_recent = int(state.nav_failures_recent)
        msg.packet_loss = float(state.packet_loss)
        msg.central_manager_alive = bool(state.central_manager_alive)
        msg.completed_tasks_recent = int(state.completed_tasks_recent)
        msg.distance_recent_m = float(state.distance_recent_m)
        msg.active_fault_types = [f.fault_type for f in self._active_faults[ns]]
        self._health_publishers[ns].publish(msg)


def main() -> None:
    rclpy.init()
    node = HealthMonitorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
