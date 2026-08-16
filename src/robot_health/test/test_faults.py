"""Unit tests for robot_health.faults - pure Python, no rclpy/ROS graph needed."""
from dataclasses import dataclass

import pytest

from robot_health.faults import (
    ALL_FAULT_TYPES,
    CAMERA_DROPOUT,
    CHARGING_BOOST,
    LIDAR_DROPOUT,
    LOW_BATTERY,
    MOTOR_FAILURE,
    apply_fault,
)


@dataclass
class _FakeState:
    battery_soc: float = 1.0
    battery_health: float = 1.0
    discharge_rate_multiplier: float = 1.0
    drive_left_health: float = 1.0
    drive_right_health: float = 1.0
    lidar_confidence: float = 1.0
    camera_confidence: float = 1.0
    imu_confidence: float = 1.0
    ultrasonic_confidence: float = 1.0
    localization_confidence: float = 1.0
    packet_loss: float = 0.0
    central_manager_alive: bool = True
    path_blocked: bool = False


def test_unknown_fault_type_returns_false_and_does_not_raise():
    state = _FakeState()
    assert apply_fault(state, "NOT_A_REAL_FAULT", 0.5) is False


def test_low_battery_forces_soc_down():
    state = _FakeState(battery_soc=0.9)
    apply_fault(state, LOW_BATTERY, severity=1.0)
    assert state.battery_soc < 0.2


def test_charging_boost_increases_soc():
    state = _FakeState(battery_soc=0.1)
    apply_fault(state, CHARGING_BOOST, severity=1.0)
    assert state.battery_soc > 0.1


def test_charging_boost_never_exceeds_full_charge():
    state = _FakeState(battery_soc=0.99)
    for _ in range(10):
        apply_fault(state, CHARGING_BOOST, severity=1.0)
    assert state.battery_soc == 1.0


def test_lidar_dropout_zeroes_confidence_regardless_of_severity():
    state = _FakeState()
    apply_fault(state, LIDAR_DROPOUT, severity=0.1)
    assert state.lidar_confidence == 0.0


def test_camera_dropout_zeroes_confidence():
    state = _FakeState()
    apply_fault(state, CAMERA_DROPOUT, severity=0.01)
    assert state.camera_confidence == 0.0


def test_motor_failure_zeroes_left_drive_health():
    state = _FakeState()
    apply_fault(state, MOTOR_FAILURE, severity=0.01)
    assert state.drive_left_health == 0.0


def test_severity_is_clamped_to_unit_interval():
    state = _FakeState()
    apply_fault(state, LOW_BATTERY, severity=5.0)  # out-of-range severity must not crash
    assert 0.0 <= state.battery_soc <= 1.0


@pytest.mark.parametrize("fault_type", ALL_FAULT_TYPES)
def test_every_catalogued_fault_type_is_applicable_without_raising(fault_type):
    state = _FakeState()
    result = apply_fault(state, fault_type, severity=0.7)
    assert result is True
