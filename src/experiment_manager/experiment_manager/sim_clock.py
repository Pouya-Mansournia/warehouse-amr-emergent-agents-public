"""Simulation-time tracking.

Every prior phase's experiment control (--duration/--t-fail in run_experiment.py,
every node's own deferred-work scheduling) runs on host wall-clock time
(time.time()/time.monotonic()), even though fleet.launch.py already bridges Gazebo's
simulation clock to /clock (rosgraph_msgs/msg/Clock, via clock_bridge) and already sets
use_sim_time: True on Nav2/SLAM/robot_state_publisher. On this project's WSL2 host, the
realtime factor is ~0.07-0.1x, so a "300s" wall-clock experiment is only ~21-30 simulated
seconds - nowhere near enough for a full pickup/dropoff task cycle, which is exactly why
prior throughput measurements read 0.0. --t-fail=100 meaning "100 wall-clock seconds"
also makes cross-run comparisons meaningless once realtime factor varies session to
session.

Split into two pieces, matching this repo's existing convention of separating ROS-free
logic (claim_book.py, negotiation.py) from the ROS node that drives it:

  ClockTracker  - plain Python, unit-testable without rclpy or a running /clock
                  publisher: just tracks the most recent Clock message it's given.
  SimClockNode  - a thin rclpy wrapper: subscribes an existing Node to /clock and
                  forwards every message into a ClockTracker. Takes an existing Node
                  rather than being one itself, because run_experiment.py (this
                  module's caller) is a plain subprocess-orchestration script with no
                  pre-existing rclpy context of its own - see run_experiment.py's
                  _SimTimeSource for how it constructs exactly one throwaway Node for
                  this single purpose, only when --time-source simulation is requested.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class ClockTracker:
    """Tracks the most recently observed /clock time. `None` until the first message
    ever arrives - callers must handle "no clock yet" explicitly (Gazebo doesn't
    publish /clock until the world is actually running, and bring-up takes real time),
    never silently treat it as 0.0 (which would look like a valid, very-early
    timestamp rather than "not started yet")."""

    _sim_time_sec: Optional[float] = None

    def on_clock(self, sec: int, nanosec: int) -> None:
        self._sim_time_sec = sec + nanosec * 1e-9

    @property
    def sim_time_sec(self) -> Optional[float]:
        return self._sim_time_sec

    @property
    def has_clock(self) -> bool:
        return self._sim_time_sec is not None


class SimClockNode:
    """Subscribes `node` to /clock and forwards every message into a ClockTracker.

    Does not own `node`'s lifecycle (creation, spinning, destruction, rclpy.init/
    shutdown) - the caller is responsible for all of that, same as any other
    subscription a node might add. This keeps SimClockNode usable both from a
    dedicated single-purpose probe node (run_experiment.py's _SimTimeSource) and,
    later, from within an existing node like event_logger_node.py if that ever needs
    its own direct access to sim time rather than relying on use_sim_time's built-in
    clock-source switching.
    """

    def __init__(self, node) -> None:
        self.tracker = ClockTracker()
        from rosgraph_msgs.msg import Clock

        node.create_subscription(Clock, "/clock", self._on_clock_msg, 10)

    def _on_clock_msg(self, msg) -> None:
        self.tracker.on_clock(msg.clock.sec, msg.clock.nanosec)

    def sim_now(self) -> Optional[float]:
        return self.tracker.sim_time_sec

    @property
    def has_clock(self) -> bool:
        return self.tracker.has_clock
