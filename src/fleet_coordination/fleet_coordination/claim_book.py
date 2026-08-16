"""Pure, ROS-free conflict-resolution logic for decentralized station claiming.
Kept separate from decentralized_agent.py so it can be
unit-tested without rclpy - see test/test_claim_book.py.

Design: there is no central broker. Every robot's agent broadcasts a StationClaim on the
shared /fleet/agent/claim topic whenever it wants a station, and every agent (including
the sender) feeds every claim it observes - its own and everyone else's - into its own
ClaimBook instance. Because every agent sees the same set of broadcasts (ROS pub/sub
delivers to all subscribers) and resolves conflicts with the same deterministic rule
(lowest cost wins, robot_id string comparison as a tiebreak), every agent's ClaimBook
converges to the SAME belief about who holds each station without ever needing to talk to
a coordinator or to each other directly about who "really" won - each just needs to see
the same claims.

This is a deliberately simple "eventual consistency within a settling window" model, not
a Byzantine-safe or network-partition-safe consensus protocol - correct only if all
agents are on the same reliable network and observe the same claims before the window
each agent waits before acting on a claim closes (see decentralized_agent.py's
CONTENTION_WINDOW_SEC). That is an appropriate and explicit limitation for a
"deterministic decentralized BASELINE" (Phase 5) - real fault tolerance under partition
is Phase 5+/Phase 7 territory (communication faults).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional


@dataclass(frozen=True)
class _Holder:
    robot_id: str
    cost: float


class ClaimBook:
    def __init__(self) -> None:
        self._holders: Dict[str, _Holder] = {}

    def observe(self, station_name: str, robot_id: str, cost: float, release: bool) -> None:
        if release:
            current = self._holders.get(station_name)
            if current is not None and current.robot_id == robot_id:
                del self._holders[station_name]
            return

        candidate = _Holder(robot_id=robot_id, cost=cost)
        current = self._holders.get(station_name)
        if current is None or _beats(candidate, current):
            self._holders[station_name] = candidate

    def winner_of(self, station_name: str) -> Optional[str]:
        holder = self._holders.get(station_name)
        return holder.robot_id if holder is not None else None

    def is_free(self, station_name: str) -> bool:
        return station_name not in self._holders

    def free_stations(self, candidates) -> list:
        """`candidates`: iterable of (name, side, x, y) tuples (see stations.py).
        Returns the subset currently believed free."""
        return [c for c in candidates if self.is_free(c[0])]


def _beats(candidate: _Holder, current: _Holder) -> bool:
    """Deterministic tiebreak: lower cost wins; exact cost ties broken by robot_id
    string comparison so every agent computes the identical outcome from the identical
    pair of claims, regardless of the order the two messages happened to arrive in."""
    if candidate.cost != current.cost:
        return candidate.cost < current.cost
    return candidate.robot_id < current.robot_id
