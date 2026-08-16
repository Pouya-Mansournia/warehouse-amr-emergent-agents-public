from agent_core.hybrid_agent import HybridAgent
from agent_core.interfaces import ALLOWED_ACTIONS, Action, Observation, StationCandidate
from agent_core.replay_agent import ReplayAgent
from agent_core.rule_agent import RuleAgent


def _obs(**overrides):
    defaults = dict(
        robot_id="robot1",
        x=0.0,
        y=0.0,
        battery_soc=1.0,
        degradation_risk=0.0,
        utilization=0.0,
        candidate_stations=(),
    )
    defaults.update(overrides)
    return Observation(**defaults)


class _CountingLLM:
    """A minimal AgentBackend stub that records whether it was ever called - used to
    prove the deterministic path genuinely never invokes the LLM when it shouldn't."""

    def __init__(self):
        self.calls = 0

    def decide(self, observation, allowed_actions):
        self.calls += 1
        return Action(action="WAIT")


def test_clear_winner_uses_deterministic_path_without_calling_llm():
    llm = _CountingLLM()
    agent = HybridAgent(deterministic=RuleAgent(), llm=llm, ambiguity_margin=0.05)
    near = StationCandidate(name="near", side="output", x=1.0, y=0.0)
    far = StationCandidate(name="far", side="output", x=15.0, y=0.0)
    action = agent.decide(_obs(candidate_stations=(near, far)), ALLOWED_ACTIONS)

    assert action.action == "BID_FOR_TASK"
    assert action.station_name == "near"
    assert llm.calls == 0
    assert agent.stats.deterministic_decisions == 1
    assert agent.stats.llm_escalations == 0


def test_ambiguous_tie_escalates_to_llm():
    llm_response = Action(action="BID_FOR_TASK", station_name="b", cost=0.5)
    llm = ReplayAgent(script=[llm_response])
    agent = HybridAgent(deterministic=RuleAgent(), llm=llm, ambiguity_margin=0.05)
    a = StationCandidate(name="a", side="output", x=1.0, y=0.0)
    b = StationCandidate(name="b", side="output", x=1.01, y=0.0)  # near-identical cost
    action = agent.decide(_obs(candidate_stations=(a, b)), ALLOWED_ACTIONS)

    assert action == llm_response
    assert agent.stats.llm_escalations == 1
    assert agent.stats.deterministic_decisions == 0


def test_last_decision_meta_none_on_deterministic_path():
    llm = _CountingLLM()
    agent = HybridAgent(deterministic=RuleAgent(), llm=llm, ambiguity_margin=0.05)
    near = StationCandidate(name="near", side="output", x=1.0, y=0.0)
    far = StationCandidate(name="far", side="output", x=15.0, y=0.0)
    agent.decide(_obs(candidate_stations=(near, far)), ALLOWED_ACTIONS)

    assert agent.last_decision_meta is None


def test_last_decision_meta_mirrors_llm_on_escalation():
    class _MetaLLM:
        def __init__(self):
            self.last_decision_meta = "real-llm-meta"

        def decide(self, observation, allowed_actions):
            return Action(action="WAIT")

    agent = HybridAgent(deterministic=RuleAgent(), llm=_MetaLLM(), ambiguity_margin=0.05)
    a = StationCandidate(name="a", side="output", x=1.0, y=0.0)
    b = StationCandidate(name="b", side="output", x=1.01, y=0.0)  # near-identical cost
    agent.decide(_obs(candidate_stations=(a, b)), ALLOWED_ACTIONS)

    assert agent.last_decision_meta == "real-llm-meta"


def test_zero_candidates_never_calls_llm():
    llm = _CountingLLM()
    agent = HybridAgent(deterministic=RuleAgent(), llm=llm)
    action = agent.decide(_obs(candidate_stations=()), ALLOWED_ACTIONS)

    assert action == Action(action="WAIT")
    assert llm.calls == 0
    assert agent.stats.deterministic_decisions == 1


def test_single_candidate_never_calls_llm():
    llm = _CountingLLM()
    agent = HybridAgent(deterministic=RuleAgent(), llm=llm)
    only = StationCandidate(name="only", side="output", x=5.0, y=0.0)
    action = agent.decide(_obs(candidate_stations=(only,)), ALLOWED_ACTIONS)

    assert action.station_name == "only"
    assert llm.calls == 0
    assert agent.stats.deterministic_decisions == 1


def test_gap_clearly_above_margin_is_not_ambiguous():
    # second_cost - best_cost well above the margin - unambiguous, deterministic path.
    # (Deliberately not testing the exact boundary: cost involves a sqrt, so exact
    # float equality there is inherently fragile - ">= margin" is still exercised by
    # this and the tie test above, just not at a hairline-precision boundary.)
    llm = _CountingLLM()
    agent = HybridAgent(deterministic=RuleAgent(w_distance=1.0, w_energy=0.0), llm=llm, ambiguity_margin=0.1)
    a = StationCandidate(name="a", side="output", x=1.0, y=0.0)
    b = StationCandidate(name="b", side="output", x=1.0 + 0.2 * 20.0, y=0.0)  # cost delta == 0.2
    agent.decide(_obs(candidate_stations=(a, b)), ALLOWED_ACTIONS)
    assert llm.calls == 0


def test_stats_accumulate_across_multiple_decisions():
    llm = ReplayAgent(script=[Action(action="WAIT")])
    agent = HybridAgent(deterministic=RuleAgent(), llm=llm, ambiguity_margin=0.05)
    near = StationCandidate(name="near", side="output", x=1.0, y=0.0)
    far = StationCandidate(name="far", side="output", x=15.0, y=0.0)
    tied_a = StationCandidate(name="a", side="output", x=1.0, y=0.0)
    tied_b = StationCandidate(name="b", side="output", x=1.001, y=0.0)

    agent.decide(_obs(candidate_stations=(near, far)), ALLOWED_ACTIONS)
    agent.decide(_obs(candidate_stations=(tied_a, tied_b)), ALLOWED_ACTIONS)

    assert agent.stats.deterministic_decisions == 1
    assert agent.stats.llm_escalations == 1
