"""Hybrid deterministic + LLM AgentBackend (Mode E).

Implements Mode E's coordination policy: deterministic safety, deterministic
execution, algorithmic bidding where sufficient, and LLM reasoning only for
ambiguous/high-level situations. Concretely, decisions are routed as follows:

    normal task allocation -> deterministic auction   (RuleAgent, the common case)
    ambiguous disruption   -> LLM reasoning            (LLMAgent, only when needed)
    safety                 -> deterministic            (unchanged - see below)
    navigation              -> Nav2                    (unchanged)
    low-level control       -> ros2_control             (unchanged)

The last three rows are not something this class does anything special for - they were
already true of every prior `AgentBackend`: `decide()` only ever returns a
`BID_FOR_TASK`/`WAIT` `Action`, never a motion command, so there has never been a code
path from any backend's decision to `cmd_vel`. `HybridAgent` only changes WHICH backend
answers WHICH decisions.

"Ambiguous" is defined concretely (not vibes): using `RuleAgent.rank()`'s full ordering,
if the best and second-best candidate's costs are within `ambiguity_margin` of each
other, the auction can't confidently distinguish them - that is treated as ambiguous and
escalated to the LLM. Everything else (a clear winner, zero or one candidate) is handled
by the fast, free, deterministic path with no LLM call at all - directly minimizing
the "cost of intelligence" (LLM_requests, decision_latency) by only paying the LLM's
latency and request cost when the deterministic auction is genuinely undecided.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import FrozenSet

from agent_core.interfaces import Action, AgentBackend, Observation
from agent_core.rule_agent import RuleAgent


@dataclass
class HybridDecisionStats:
    """Tracks how often the hybrid controller actually needed the LLM vs. handled
    things deterministically. Persisting these counters to experiments/<run>/ is the
    responsibility of the experiment logging pipeline, not this module."""

    deterministic_decisions: int = 0
    llm_escalations: int = 0


@dataclass
class HybridAgent(AgentBackend):
    deterministic: RuleAgent
    llm: AgentBackend
    ambiguity_margin: float = 0.05
    stats: HybridDecisionStats = field(default_factory=HybridDecisionStats)
    # Mirrors LLMAgent.last_decision_meta - None whenever this decision
    # was handled deterministically (no LLM call happened at all), otherwise the
    # underlying LLMAgent's own last_decision_meta, so decentralized_agent.py can read
    # this uniformly regardless of which backend (llm vs hybrid) is configured.
    last_decision_meta: object = field(default=None, init=False, repr=False)

    def decide(self, observation: Observation, allowed_actions: FrozenSet[str]) -> Action:
        ranked = self.deterministic.rank(observation)

        if len(ranked) < 2:
            # 0 candidates (WAIT) or exactly 1 (no second option to be ambiguous
            # against) - nothing for an LLM to usefully adjudicate.
            self.stats.deterministic_decisions += 1
            self.last_decision_meta = None
            return self.deterministic.decide(observation, allowed_actions)

        (_, best_cost), (_, second_cost) = ranked[0], ranked[1]
        if second_cost - best_cost >= self.ambiguity_margin:
            self.stats.deterministic_decisions += 1
            self.last_decision_meta = None
            return self.deterministic.decide(observation, allowed_actions)

        self.stats.llm_escalations += 1
        action = self.llm.decide(observation, allowed_actions)
        self.last_decision_meta = getattr(self.llm, "last_decision_meta", None)
        return action
