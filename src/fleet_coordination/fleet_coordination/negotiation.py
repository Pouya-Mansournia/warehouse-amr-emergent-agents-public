"""Peer-to-peer task-transfer negotiation protocol.

Pure, ROS-free state machine - the negotiation counterpart to claim_book.py's
`ClaimBook`. One `Conversation` tracks exactly one attempted task transfer,
identified by a `conversation_id` the initiator generates; every message belonging to
that transfer (`OFFER`, `BID`/`COUNTER_PROPOSAL`, `ACCEPT`, `REJECT`, `COMMIT`,
`TIMEOUT` - amr_interfaces/TaskTransfer's `performative` field) carries the same id, so
the full history of any transfer - who proposed it, who bid, who won, when - is
reconstructable purely from the ordered sequence of messages sharing that id
(negotiations.csv via event_logger_node.py). This is what makes the protocol
auditable from proposal to execution.

Selection rule mirrors claim_book.py's and rule_agent.py's: lowest cost wins, exact
ties broken by robot_id - deterministic and coordinator-free, same reasoning as
`ClaimBook`'s. `BID` and `COUNTER_PROPOSAL` are treated identically for
selection purposes - the protocol vocabulary distinguishes "an initial offer response"
from "a renegotiated one," but this project's simple negotiation policy (one round,
bounded deadline - see "prevent infinite negotiation" below) doesn't yet drive a
distinct COUNTER_PROPOSAL round; the message type exists so a more elaborate policy can
use it later without a protocol change.

Preventing infinite negotiation is
enforced by `deadline`: `resolve()` is only ever meant to be called once, at or after
`is_expired()` becomes true, and always returns a definite outcome (a winner, or None
meaning "no bids - the initiator keeps its own task") - there is no retry-forever path.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional

OFFER = "OFFER"
BID = "BID"
COUNTER_PROPOSAL = "COUNTER_PROPOSAL"
ACCEPT = "ACCEPT"
REJECT = "REJECT"
COMMIT = "COMMIT"
TIMEOUT = "TIMEOUT"

BID_LIKE = frozenset({BID, COUNTER_PROPOSAL})
ALL_PERFORMATIVES = frozenset({OFFER, BID, COUNTER_PROPOSAL, ACCEPT, REJECT, COMMIT, TIMEOUT})


@dataclass
class Conversation:
    """Initiator-side record of one in-flight (or resolved) task-transfer negotiation."""

    conversation_id: str
    initiator: str
    station_name: str
    side: str
    x: float
    y: float
    reason_code: str
    deadline: float  # time.monotonic() timestamp; caller owns the clock
    bids: Dict[str, float] = field(default_factory=dict)  # robot_id -> best cost offered

    def record_bid(self, robot_id: str, performative: str, cost: float) -> None:
        """Records a BID or COUNTER_PROPOSAL from a peer - ignored from the initiator
        itself and from any performative outside the bid-like set (defensive; the ROS
        layer should never route anything else here)."""
        if performative not in BID_LIKE or robot_id == self.initiator:
            return
        current = self.bids.get(robot_id)
        if current is None or cost < current:
            self.bids[robot_id] = cost

    def is_expired(self, now: float) -> bool:
        return now >= self.deadline

    def select_winner(
        self, reliability: Optional[Dict[str, float]] = None, reliability_weight: float = 0.05
    ) -> Optional[str]:
        """Deterministic lowest-cost-wins, tie-broken by robot_id - same rule as
        claim_book.py's ClaimBook and rule_agent.py's RuleAgent, so a fleet of
        independent agents observing the same bids always agrees on the same winner
        without a coordinator. Returns None if nobody bid.

        `reliability` (episodic peer memory - agent_core.peer_memory)
        is an optional peer_id -> [0, 1] score. When provided, each bidder's cost is
        nudged down by `reliability_weight * reliability[peer_id]` (0.0 for an
        unknown/never-interacted peer, so memory can only ever help a peer with a
        genuinely good track record, never penalize one with no history) before
        ranking - a real, bounded, still-deterministic influence, not a rule
        override. Selection remains coordinator-free: any peer observing the same
        bids AND the same reliability map reaches the same winner."""
        if not self.bids:
            return None
        if not reliability:
            return min(self.bids.items(), key=lambda item: (item[1], item[0]))[0]
        adjusted = {
            robot_id: cost - reliability_weight * reliability.get(robot_id, 0.0)
            for robot_id, cost in self.bids.items()
        }
        return min(adjusted.items(), key=lambda item: (item[1], item[0]))[0]
