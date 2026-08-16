"""Read-only reuse of Phase I's already-ROS-free decision/coordination logic.

Phase I's `agent_core` and `fleet_coordination` packages contain pure-Python,
zero-rclpy-dependency modules (confirmed by direct inspection before writing
this file - none of the modules imported below import `rclpy` or any ROS
package, directly or transitively): `RuleAgent`/`LLMAgent`/`HybridAgent`
(agent_core), `ClaimBook`/negotiation's `Conversation` (fleet_coordination).
Phase II imports them exactly as any script outside the ROS workspace would -
via `sys.path`, no `colcon build`/`source install/setup.bash` needed - and
never modifies them. This module is the single place that path-wiring
happens, so it's obvious at a glance that Phase II depends on Phase I code
only by reading it, never by writing to it.

Why reuse instead of reimplementing: Phase II's whole point is studying what
changes when the SAME decision logic (deterministic/LLM/hybrid bidding, the
same claim-conflict tie-break, the same peer-memory nudge) runs long-horizon
with persistent memory, not a different logic re-implemented from scratch for
a toy world - that would answer a different, weaker question.
"""
from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
for _pkg in ("agent_core", "fleet_coordination"):
    _path = str(_REPO_ROOT / "src" / _pkg)
    if _path not in sys.path:
        sys.path.insert(0, _path)

from agent_core.interfaces import (  # noqa: E402
    Action,
    AgentBackend,
    ALLOWED_ACTIONS,
    Observation,
    StationCandidate,
)
from agent_core.rule_agent import RuleAgent  # noqa: E402
from agent_core.hybrid_agent import HybridAgent  # noqa: E402
from agent_core.llm_agent import LLMAgent  # noqa: E402
from agent_core.ollama_client import OllamaClient  # noqa: E402
from agent_core.replay_agent import ReplayAgent  # noqa: E402
from agent_core.peer_memory import (  # noqa: E402
    PeerMemory,
    REJECTED,
    SUCCESSFUL_HELP,
    TIMED_OUT,
)
from fleet_coordination.claim_book import ClaimBook  # noqa: E402
from fleet_coordination.negotiation import Conversation  # noqa: E402
from fleet_coordination.stations import STATIONS, STATIONS_BY_SIDE, OPPOSITE_SIDE  # noqa: E402

__all__ = [
    "Action",
    "AgentBackend",
    "ALLOWED_ACTIONS",
    "Observation",
    "StationCandidate",
    "RuleAgent",
    "HybridAgent",
    "LLMAgent",
    "OllamaClient",
    "ReplayAgent",
    "PeerMemory",
    "SUCCESSFUL_HELP",
    "REJECTED",
    "TIMED_OUT",
    "ClaimBook",
    "Conversation",
    "STATIONS",
    "STATIONS_BY_SIDE",
    "OPPOSITE_SIDE",
]
