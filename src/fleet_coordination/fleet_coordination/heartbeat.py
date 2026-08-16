"""Pure, ROS-free central-manager-heartbeat tracking, for centralized ->
decentralized failover. Kept separate from decentralized_agent.py so it can be
unit-tested without rclpy - same convention as claim_book.py/negotiation.py.

Design: fleet_manager.py publishes FleetManagerHeartbeat on /fleet/manager/heartbeat at
a fixed period for as long as it's alive (a single persistent timer, same safe pattern as
every other periodic/deferred work in this repo - see fleet_manager.py's own docstring
for why a fresh create_timer() per retry was found unsafe under load; a timer created
once in __init__ and never touched again has no such problem). A DecentralizedAgent
started in --coordination centralized_then_failover mode begins DORMANT (touches nothing
- no claims, no Nav2 goals) and watches this heartbeat; once more than
manager_timeout_sec has passed since the last one was seen, it concludes the central
manager is gone and activates its normal Mode C/D/E cycle.

This only ever looks at ARRIVAL TIME, never message content - a heartbeat's only job is
"did one show up recently enough."
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class HeartbeatMonitor:
    manager_timeout_sec: float

    _last_seen_monotonic: Optional[float] = None

    def on_heartbeat(self, now_monotonic: float) -> None:
        self._last_seen_monotonic = now_monotonic

    def is_alive(self, now_monotonic: float) -> bool:
        """True until manager_timeout_sec has elapsed since the last heartbeat OR no
        heartbeat has ever been seen yet. The "never seen yet" case is deliberately
        treated as alive (not timed-out) - a robot that starts up a fraction of a
        second before fleet_manager's own first heartbeat tick must not immediately
        conclude the manager is dead; it should behave exactly like a fresh
        --coordination centralized run until a REAL timeout is observed."""
        if self._last_seen_monotonic is None:
            return True
        return (now_monotonic - self._last_seen_monotonic) < self.manager_timeout_sec

    @property
    def has_ever_seen_heartbeat(self) -> bool:
        return self._last_seen_monotonic is not None
