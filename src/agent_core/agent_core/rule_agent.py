"""Deterministic cost-based AgentBackend.

Formalizes the exact weighted-sum formula used for station-bidding decisions:

    task_cost = w_distance * normalized_distance
              + w_energy   * energy_risk
              + w_health   * degradation_risk
              + w_workload * utilization

`w_health`/`w_workload` default to 0.0: `degradation_risk`/`utilization`
have no live data source yet (no mechanical-health or workload model exists
in this repo - only battery telemetry), so their weight is
zero rather than guessing at meaningless values. This is a formula
formalization, not a behavior change from the original inline `_cost()` - with
these defaults it is numerically identical (0.7 distance + 0.3 energy),
now expressed through the provider-independent `AgentBackend` interface
instead of being embedded directly in `decentralized_agent.py`.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet, List, Tuple

from agent_core.interfaces import Action, AgentBackend, Observation, StationCandidate

_MAX_EXPECTED_DISTANCE_M = 20.0  # world is ~15x20m - normalizes distance into [0, 1]


def _distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


@dataclass(frozen=True)
class RuleAgent(AgentBackend):
    w_distance: float = 0.7
    w_energy: float = 0.3
    w_health: float = 0.0
    w_workload: float = 0.0

    def _cost(self, observation: Observation, x: float, y: float) -> float:
        normalized_distance = min(
            _distance(observation.x, observation.y, x, y) / _MAX_EXPECTED_DISTANCE_M, 1.0
        )
        energy_risk = 1.0 - observation.battery_soc
        return (
            self.w_distance * normalized_distance
            + self.w_energy * energy_risk
            + self.w_health * observation.degradation_risk
            + self.w_workload * observation.utilization
        )

    def rank(self, observation: Observation) -> List[Tuple[StationCandidate, float]]:
        """All candidates sorted by cost ascending (ties broken by station name) - the
        same ordering `decide()` uses internally, exposed separately for callers (Phase
        9's `HybridAgent`) that need to reason about the full ranking - specifically,
        how close the top two candidates are - not just the single winner."""
        return sorted(
            ((c, self._cost(observation, c.x, c.y)) for c in observation.candidate_stations),
            key=lambda pair: (pair[1], pair[0].name),
        )

    def decide(self, observation: Observation, allowed_actions: FrozenSet[str]) -> Action:
        ranked = self.rank(observation)
        if not ranked:
            return Action(action="WAIT")

        best, best_cost = ranked[0]
        return Action(action="BID_FOR_TASK", station_name=best.name, cost=best_cost)
