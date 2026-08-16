#!/usr/bin/env python3
"""Fault injector: publishes amr_interfaces/FaultCommand
on /fleet/faults/command. health_monitor.py is the only subscriber that acts on these;
this node owns no robot state itself, only the injection schedule/RNG.

Three trigger mechanisms, all logged (every publish is itself observable on the bagged
/fleet/faults/command topic, and event_logger_node.py additionally writes each one to
faults.csv/events.jsonl):

  - manual:        `ros2 topic pub /fleet/faults/command ...` (or any other publisher) -
                    nothing to implement here, the topic itself is the manual-trigger API.
  - scheduled:      a YAML file's `schedule:` list, each entry fired once at t = t_offset_sec
                    seconds after this node starts.
  - seeded_random:  a YAML file's `random:` block - a deterministic `random.Random(seed)`
                    draws inter-arrival times and fault parameters, so the same seed always
                    produces the same sequence of injected faults (reproducibility, master
                    prompt section 15).

Config format (config/faults/example_schedule.yaml):

    schedule:
      - t_offset_sec: 30.0
        robot_id: robot1
        fault_type: LOW_BATTERY
        severity: 0.8
        duration_sec: 60.0

    random:
      seed: 42
      rate_per_min: 2.0
      robots: [robot1, robot2, robot3]
      fault_types: [LIDAR_NOISE, WHEEL_EFFICIENCY_REDUCTION]
      severity_range: [0.3, 0.9]
      duration_range_sec: [20.0, 90.0]
"""
from __future__ import annotations

import random
import time
import uuid
from dataclasses import dataclass
from typing import List, Optional

import rclpy
import yaml
from amr_interfaces.msg import FaultCommand
from rclpy.node import Node


@dataclass
class _ScheduledFault:
    t_offset_sec: float
    robot_id: str
    fault_type: str
    severity: float
    duration_sec: float
    fired: bool = False


class FaultInjectorNode(Node):
    def __init__(self) -> None:
        super().__init__("fault_injector")

        self.declare_parameter("schedule_file", "")
        schedule_path = self.get_parameter("schedule_file").get_parameter_value().string_value

        self._publisher = self.create_publisher(FaultCommand, "/fleet/faults/command", 10)
        self._start_time = time.monotonic()

        self._scheduled: List[_ScheduledFault] = []
        self._rng: Optional[random.Random] = None
        self._random_cfg = None
        self._next_random_fire_at: Optional[float] = None

        if schedule_path:
            self._load_schedule(schedule_path)
        else:
            self.get_logger().info(
                "fault_injector started with no schedule_file - manual triggers "
                "(publish FaultCommand on /fleet/faults/command) only"
            )

        self.create_timer(1.0, self._tick)

    def _load_schedule(self, path: str) -> None:
        with open(path) as f:
            cfg = yaml.safe_load(f) or {}

        for entry in cfg.get("schedule", []):
            self._scheduled.append(
                _ScheduledFault(
                    t_offset_sec=float(entry["t_offset_sec"]),
                    robot_id=str(entry["robot_id"]),
                    fault_type=str(entry["fault_type"]),
                    severity=float(entry.get("severity", 1.0)),
                    duration_sec=float(entry.get("duration_sec", 0.0)),
                )
            )
        self.get_logger().info(f"loaded {len(self._scheduled)} scheduled fault(s) from {path}")

        random_cfg = cfg.get("random")
        if random_cfg:
            self._random_cfg = random_cfg
            self._rng = random.Random(int(random_cfg["seed"]))
            self._schedule_next_random()
            self.get_logger().info(
                f"seeded random fault injection enabled, seed={random_cfg['seed']}"
            )

    def _schedule_next_random(self) -> None:
        rate_per_min = float(self._random_cfg.get("rate_per_min", 1.0))
        # Exponential inter-arrival time for a Poisson process at this rate - deterministic
        # given the seeded RNG, so identical seeds reproduce identical fault timing.
        mean_interval_sec = 60.0 / max(rate_per_min, 1e-6)
        interval = self._rng.expovariate(1.0 / mean_interval_sec)
        self._next_random_fire_at = (time.monotonic() - self._start_time) + interval

    def _fire_random_fault(self) -> None:
        cfg = self._random_cfg
        robot_id = self._rng.choice(cfg["robots"])
        fault_type = self._rng.choice(cfg["fault_types"])
        lo, hi = cfg.get("severity_range", [0.3, 0.9])
        severity = self._rng.uniform(lo, hi)
        dlo, dhi = cfg.get("duration_range_sec", [20.0, 90.0])
        duration = self._rng.uniform(dlo, dhi)
        self._publish(
            robot_id=robot_id,
            fault_type=fault_type,
            severity=severity,
            duration_sec=duration,
            trigger_source="seeded_random",
        )
        self._schedule_next_random()

    def _tick(self) -> None:
        elapsed = time.monotonic() - self._start_time

        for sf in self._scheduled:
            if not sf.fired and elapsed >= sf.t_offset_sec:
                sf.fired = True
                self._publish(
                    robot_id=sf.robot_id,
                    fault_type=sf.fault_type,
                    severity=sf.severity,
                    duration_sec=sf.duration_sec,
                    trigger_source="scheduled",
                )

        if self._next_random_fire_at is not None and elapsed >= self._next_random_fire_at:
            self._fire_random_fault()

    def _publish(
        self,
        *,
        robot_id: str,
        fault_type: str,
        severity: float,
        duration_sec: float,
        trigger_source: str,
        clear: bool = False,
    ) -> None:
        msg = FaultCommand()
        msg.robot_id = robot_id
        msg.fault_type = fault_type
        msg.severity = severity
        msg.duration_sec = duration_sec
        msg.trigger_source = trigger_source
        msg.fault_id = str(uuid.uuid4())
        msg.clear = clear
        msg.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(msg)
        self.get_logger().info(
            f"injected {fault_type} -> {robot_id} (severity={severity:.2f}, "
            f"duration={duration_sec:.0f}s, source={trigger_source})"
        )


def main() -> None:
    rclpy.init()
    node = FaultInjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
