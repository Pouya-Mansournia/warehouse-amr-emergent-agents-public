"""Episodic peer memory.

A ROS-free, pure-Python class - same convention as claim_book.py/negotiation.py -
tracking, per peer robot_id, real measured outcomes of past task-transfer
negotiations this robot participated in. Deliberately narrow: only counts,
response-time samples, and a derived success rate from actual ACCEPT/REJECT/
TIMEOUT resolutions this robot observed - never a symbolic/emotional score;
peer trust is grounded in measurable interaction history rather than invented
sentiment.

Entirely optional and additive: nothing in fleet_coordination/negotiation.py or
decentralized_agent.py requires a PeerMemory instance to function. When present
(memory_enabled:=true), `Conversation.select_winner()` accepts an optional
reliability map derived from this class to nudge - not override - the
deterministic lowest-cost-wins rule, and `agent_core.interfaces.Observation`
carries a bounded `peer_history` summary an LLM prompt can read.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

SUCCESSFUL_HELP = "successful_help"
REJECTED = "rejected"
TIMED_OUT = "timed_out"

_OUTCOMES = frozenset({SUCCESSFUL_HELP, REJECTED, TIMED_OUT})


@dataclass
class PeerRecord:
    successful_help: int = 0
    rejected_requests: int = 0
    timed_out_requests: int = 0
    response_times_sec: List[float] = field(default_factory=list)

    @property
    def total_requests(self) -> int:
        return self.successful_help + self.rejected_requests + self.timed_out_requests

    @property
    def task_transfer_success_rate(self) -> Optional[float]:
        total = self.total_requests
        return round(self.successful_help / total, 4) if total else None

    @property
    def average_response_time_sec(self) -> Optional[float]:
        if not self.response_times_sec:
            return None
        return round(sum(self.response_times_sec) / len(self.response_times_sec), 4)

    def as_dict(self) -> dict:
        return {
            "successful_help": self.successful_help,
            "rejected_requests": self.rejected_requests,
            "timed_out_requests": self.timed_out_requests,
            "average_response_time_sec": self.average_response_time_sec,
            "task_transfer_success_rate": self.task_transfer_success_rate,
        }


class PeerMemory:
    """Owned by one robot. `record_outcome` is called once per peer per resolved
    negotiation this robot initiated (won bidder -> successful_help, every other
    bidder -> rejected; nobody bid before the deadline -> no peer outcome, it's not
    about a specific peer). `response_time_sec` is the wall/sim-clock delay between
    this robot broadcasting OFFER and that peer's BID arriving, when known."""

    def __init__(self) -> None:
        self._peers: Dict[str, PeerRecord] = {}

    def _get(self, peer_id: str) -> PeerRecord:
        return self._peers.setdefault(peer_id, PeerRecord())

    def record_outcome(
        self, peer_id: str, outcome: str, response_time_sec: Optional[float] = None
    ) -> None:
        if outcome not in _OUTCOMES:
            raise ValueError(f"unknown peer-memory outcome: {outcome!r}")
        record = self._get(peer_id)
        if outcome == SUCCESSFUL_HELP:
            record.successful_help += 1
        elif outcome == REJECTED:
            record.rejected_requests += 1
        else:
            record.timed_out_requests += 1
        if response_time_sec is not None:
            record.response_times_sec.append(response_time_sec)

    def reliability(self, peer_id: str) -> float:
        """0.0 (no history, or all-negative history) to 1.0 (always helped when
        asked) - the input `Conversation.select_winner()`'s optional reliability
        map uses to nudge tie-breaking. Unknown peers default to 0.0 (no bonus,
        not a penalty), so memory can only ever help a peer with a genuinely good
        track record, never punish one with no history yet."""
        record = self._peers.get(peer_id)
        rate = record.task_transfer_success_rate if record else None
        return rate if rate is not None else 0.0

    def summary(self, max_peers: int = 5) -> Dict[str, dict]:
        """Bounded snapshot for an LLM prompt (section 21: "limit memory size").
        Sorted by total interaction count, descending, then truncated - the peers
        this robot has the most history with are the most informative context."""
        ranked = sorted(
            self._peers.items(), key=lambda kv: kv[1].total_requests, reverse=True
        )
        return {peer_id: record.as_dict() for peer_id, record in ranked[:max_peers]}
