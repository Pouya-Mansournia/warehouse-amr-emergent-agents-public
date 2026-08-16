"""Scripted AgentBackend that replays a fixed action sequence, ignoring observations.

Exists to prove `AgentBackend` is genuinely pluggable - the research platform is not
hard-coded to one commercial model - and to give integration tests a fully
deterministic decision source with zero simulation
variance - useful for regression-testing everything downstream of `decide()`
(claim publishing, contention resolution) without also depending on
`RuleAgent`'s cost math being correct.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import FrozenSet, List

from agent_core.interfaces import Action, AgentBackend, Observation


@dataclass
class ReplayAgent(AgentBackend):
    """Returns `script[i]` on the i-th call to `decide()`; `WAIT` once exhausted."""

    script: List[Action] = field(default_factory=list)
    _calls: int = field(default=0, init=False, repr=False)

    def decide(self, observation: Observation, allowed_actions: FrozenSet[str]) -> Action:
        if self._calls < len(self.script):
            action = self.script[self._calls]
        else:
            action = Action(action="WAIT")
        self._calls += 1
        return action
