"""Provider-independent agent decision interface.

`AgentBackend.decide()` is the single seam through which ANY decision-making
strategy - a hand-written rule, a scripted replay, or an LLM - picks
the next high-level action for one robot. Nothing downstream of `decide()`
(claim publishing, contention resolution, Nav2 goal dispatch - all still in
fleet_coordination/decentralized_agent.py) needs to know or care which
concrete `AgentBackend` produced the action, which is exactly the point: the
platform must not be hard-coded to one commercial model or even to
rule-based logic.

This module is intentionally ROS-free and dependency-free, like
fleet_coordination/claim_book.py, so it can be unit tested without rclpy and
reused by any future coordination package.

`ALLOWED_ACTIONS` here is deliberately a small subset of the full action
vocabulary this interface could eventually support (`ACCEPT_TASK`,
`REQUEST_HELP`, `PROPOSE_TASK_TRANSFER`, etc.) - only the two actions that
have real, already-wired meaning today (bidding for a station via the claim
protocol, or waiting). The rest of the vocabulary becomes meaningful once
peer-negotiation messages/topics are built to carry it; defining it here now
would be unvalidated surface area.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import FrozenSet, Optional, Tuple

ALLOWED_ACTIONS: FrozenSet[str] = frozenset({"BID_FOR_TASK", "WAIT"})


@dataclass(frozen=True)
class StationCandidate:
    """One free station this robot could bid for, as seen in its local ClaimBook belief."""

    name: str
    side: str  # "input" | "output"
    x: float
    y: float


@dataclass(frozen=True)
class Observation:
    """Everything an AgentBackend is allowed to see for one decision.

    Mirrors the relevant subset of the platform's `robot_state`
    schema - only the fields actually available today (health
    telemetry, odometry/ClaimBook). Fields with no live data source
    yet (`degradation_risk`, `utilization`) default to 0.0 (neutral / no
    risk), matching how `energy_risk` already defaulted to 0 before any
    `/health/state` sample arrived - so a RuleAgent's behavior is unchanged
    whether or not robot_health happens to be running.
    """

    robot_id: str
    x: float
    y: float
    battery_soc: float
    degradation_risk: float
    utilization: float
    candidate_stations: Tuple[StationCandidate, ...]


@dataclass(frozen=True)
class Action:
    """A validated, structured decision - never raw text or code."""

    action: str
    station_name: Optional[str] = None
    cost: Optional[float] = None

    def __post_init__(self) -> None:
        if self.action not in ALLOWED_ACTIONS:
            raise ValueError(f"unknown action {self.action!r}, allowed: {sorted(ALLOWED_ACTIONS)}")
        if self.action == "BID_FOR_TASK" and (self.station_name is None or self.cost is None):
            raise ValueError("BID_FOR_TASK requires station_name and cost")


class AgentBackend(ABC):
    """Provider-independent decision-making strategy for one robot.

    Concrete implementations: `RuleAgent` (deterministic cost formula
    algorithm), `ReplayAgent` (scripted fixed sequence, for deterministic
    tests), and `LLMAgent` (LLM-backed reasoning), all validated against
    this same interface so the coordination stack is agnostic to which one
    is in use.
    """

    @abstractmethod
    def decide(self, observation: Observation, allowed_actions: FrozenSet[str]) -> Action:
        """Return exactly one validated Action given the current observation."""
        raise NotImplementedError
