"""Fault-effect catalogue for the robot_health package.

Scope is explicitly declarative: fault effects modify the *published health
signal* (robot_health/health_monitor.py), not actual robot motion/sensing - the design
intent is to publish the degraded signal without directly acting on it. Wiring these
into real behavior (e.g. an agent reacting to LOW_BATTERY, or a physically stalled
motor) is handled separately by the coordination layer.

Each effect function takes the running `RobotHealthState` (see health_monitor.py) and a
`severity` in [0, 1], and mutates the state in place. Effects are applied every tick for
as long as the fault is active (health_monitor re-derives baseline health each tick, then
applies all active faults on top), so they compose additively/multiplicatively as written
here rather than needing separate "apply once" / "undo" logic.
"""
from __future__ import annotations

from typing import Callable, Dict

# Robot faults (physical/onboard)
LOW_BATTERY = "LOW_BATTERY"
ACCELERATED_BATTERY_DISCHARGE = "ACCELERATED_BATTERY_DISCHARGE"
# The only fault type in this catalogue that INCREASES battery_soc rather than
# degrading anything, used for the charging-scarcity experiment - reused as
# the charging mechanism itself (see fleet_coordination/decentralized_agent.py's
# charging state machine) rather than adding a new message type or Gazebo charger
# entity. A robot occupying a charger slot publishes this as a FaultCommand targeting
# itself directly on /fleet/faults/command (the same topic fault_injector.py already
# uses - no new plumbing needed), and clears it once charged.
CHARGING_BOOST = "CHARGING_BOOST"
BATTERY_CAPACITY_DEGRADATION = "BATTERY_CAPACITY_DEGRADATION"
WHEEL_EFFICIENCY_REDUCTION = "WHEEL_EFFICIENCY_REDUCTION"
MOTOR_DEGRADED = "MOTOR_DEGRADED"
MOTOR_FAILURE = "MOTOR_FAILURE"
LOCALIZATION_DEGRADATION = "LOCALIZATION_DEGRADATION"
LIDAR_NOISE = "LIDAR_NOISE"
LIDAR_DROPOUT = "LIDAR_DROPOUT"
CAMERA_DROPOUT = "CAMERA_DROPOUT"
IMU_DRIFT = "IMU_DRIFT"
ULTRASONIC_FALSE_POSITIVES = "ULTRASONIC_FALSE_POSITIVES"
NAV_ACTION_FAILURE = "NAV_ACTION_FAILURE"
ROBOT_IMMOBILIZED = "ROBOT_IMMOBILIZED"

# Environment faults - robot_id is typically "*" or a specific robot standing in for
# "the robot(s) affected by this environmental condition". No shared-world state exists
# yet (Phase 5+), so these only affect the health signal of whichever robot_id the
# FaultCommand names.
BLOCKED_AISLE = "BLOCKED_AISLE"
STATION_UNAVAILABLE = "STATION_UNAVAILABLE"
DYNAMIC_OBSTACLE_INCREASE = "DYNAMIC_OBSTACLE_INCREASE"
STATION_CONGESTION = "STATION_CONGESTION"
TASK_BURST = "TASK_BURST"

# Communication faults
CENTRAL_LINK_LOSS = "CENTRAL_LINK_LOSS"
PEER_PACKET_LOSS = "PEER_PACKET_LOSS"
ROBOT_ISOLATED = "ROBOT_ISOLATED"
NETWORK_PARTITION = "NETWORK_PARTITION"
COMM_DELAY = "COMM_DELAY"
STALE_STATE = "STALE_STATE"


def _clamp01(v: float) -> float:
    return max(0.0, min(1.0, v))


def _low_battery(state, severity: float) -> None:
    state.battery_soc = min(state.battery_soc, 0.15 * (1.0 - 0.5 * severity) + 0.02)


def _accelerated_battery_discharge(state, severity: float) -> None:
    state.discharge_rate_multiplier *= 1.0 + 4.0 * severity


def _charging_boost(state, severity: float) -> None:
    # Reapplied every health_monitor tick (0.5s, sim-time) for as long as this fault
    # stays active - 0.02/tick at severity=1.0 is ~0.04/sim-second, so recovering from
    # a critical ~0.1 SOC to a "charged enough" ~0.9 threshold takes ~20 simulated
    # seconds - long enough to be a real, observable state in a short experiment,
    # short enough that a pilot-scale run can see a full charge cycle complete.
    state.battery_soc = min(1.0, state.battery_soc + 0.02 * severity)


def _battery_capacity_degradation(state, severity: float) -> None:
    state.battery_health = _clamp01(state.battery_health - 0.5 * severity)


def _wheel_efficiency_reduction(state, severity: float) -> None:
    state.drive_left_health = _clamp01(state.drive_left_health - 0.5 * severity)
    state.drive_right_health = _clamp01(state.drive_right_health - 0.5 * severity)


def _motor_degraded(state, severity: float) -> None:
    state.drive_left_health = _clamp01(state.drive_left_health - 0.7 * severity)


def _motor_failure(state, severity: float) -> None:
    state.drive_left_health = 0.0


def _localization_degradation(state, severity: float) -> None:
    state.localization_confidence = _clamp01(state.localization_confidence - severity)


def _lidar_noise(state, severity: float) -> None:
    state.lidar_confidence = _clamp01(state.lidar_confidence - 0.4 * severity)


def _lidar_dropout(state, severity: float) -> None:
    state.lidar_confidence = 0.0


def _camera_dropout(state, severity: float) -> None:
    state.camera_confidence = 0.0


def _imu_drift(state, severity: float) -> None:
    state.imu_confidence = _clamp01(state.imu_confidence - 0.5 * severity)


def _ultrasonic_false_positives(state, severity: float) -> None:
    state.ultrasonic_confidence = _clamp01(state.ultrasonic_confidence - 0.5 * severity)


def _robot_immobilized(state, severity: float) -> None:
    state.path_blocked = True


def _environment_blocks_path(state, severity: float) -> None:
    state.path_blocked = True


def _no_op(state, severity: float) -> None:
    # TASK_BURST is fleet_manager-/task-generation-level, not a per-robot health effect -
    # accepted and logged (see fault_injector.py / event_logger_node.py) but has no health
    # signal until Phase 5+ makes task generation itself fault-aware.
    pass


def _central_link_loss(state, severity: float) -> None:
    state.central_manager_alive = False


def _peer_comm_degradation(state, severity: float) -> None:
    state.packet_loss = _clamp01(state.packet_loss + severity)


EFFECTS: Dict[str, Callable] = {
    LOW_BATTERY: _low_battery,
    ACCELERATED_BATTERY_DISCHARGE: _accelerated_battery_discharge,
    CHARGING_BOOST: _charging_boost,
    BATTERY_CAPACITY_DEGRADATION: _battery_capacity_degradation,
    WHEEL_EFFICIENCY_REDUCTION: _wheel_efficiency_reduction,
    MOTOR_DEGRADED: _motor_degraded,
    MOTOR_FAILURE: _motor_failure,
    LOCALIZATION_DEGRADATION: _localization_degradation,
    LIDAR_NOISE: _lidar_noise,
    LIDAR_DROPOUT: _lidar_dropout,
    CAMERA_DROPOUT: _camera_dropout,
    IMU_DRIFT: _imu_drift,
    ULTRASONIC_FALSE_POSITIVES: _ultrasonic_false_positives,
    NAV_ACTION_FAILURE: _no_op,  # counted directly from goal-status transitions, not here
    ROBOT_IMMOBILIZED: _robot_immobilized,
    BLOCKED_AISLE: _environment_blocks_path,
    STATION_UNAVAILABLE: _no_op,
    DYNAMIC_OBSTACLE_INCREASE: _no_op,
    STATION_CONGESTION: _no_op,
    TASK_BURST: _no_op,
    CENTRAL_LINK_LOSS: _central_link_loss,
    PEER_PACKET_LOSS: _peer_comm_degradation,
    ROBOT_ISOLATED: _central_link_loss,
    NETWORK_PARTITION: _central_link_loss,
    COMM_DELAY: _peer_comm_degradation,
    STALE_STATE: _peer_comm_degradation,
}

ALL_FAULT_TYPES = tuple(EFFECTS.keys())


def apply_fault(state, fault_type: str, severity: float) -> bool:
    """Apply one active fault's effect to `state`. Returns False for an unknown fault_type
    (caller logs + drops it - see health_monitor.py's schema-validation note)."""
    effect = EFFECTS.get(fault_type)
    if effect is None:
        return False
    effect(state, _clamp01(severity))
    return True
