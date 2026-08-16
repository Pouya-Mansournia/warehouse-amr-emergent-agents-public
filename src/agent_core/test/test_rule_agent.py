from agent_core.interfaces import ALLOWED_ACTIONS, Action, Observation, StationCandidate
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


def test_no_candidates_waits():
    agent = RuleAgent()
    action = agent.decide(_obs(), ALLOWED_ACTIONS)
    assert action == Action(action="WAIT")


def test_picks_lowest_cost_candidate():
    agent = RuleAgent()
    near = StationCandidate(name="near", side="output", x=1.0, y=0.0)
    far = StationCandidate(name="far", side="output", x=10.0, y=0.0)
    action = agent.decide(_obs(candidate_stations=(far, near)), ALLOWED_ACTIONS)
    assert action.action == "BID_FOR_TASK"
    assert action.station_name == "near"


def test_tie_broken_by_station_name():
    agent = RuleAgent()
    a = StationCandidate(name="station_b", side="output", x=1.0, y=0.0)
    b = StationCandidate(name="station_a", side="output", x=1.0, y=0.0)
    action = agent.decide(_obs(candidate_stations=(a, b)), ALLOWED_ACTIONS)
    assert action.station_name == "station_a"


def test_low_battery_raises_cost_for_equidistant_choice():
    agent = RuleAgent()
    only = StationCandidate(name="only", side="output", x=5.0, y=0.0)
    low_battery = agent.decide(_obs(battery_soc=0.1, candidate_stations=(only,)), ALLOWED_ACTIONS)
    full_battery = agent.decide(_obs(battery_soc=1.0, candidate_stations=(only,)), ALLOWED_ACTIONS)
    assert low_battery.cost > full_battery.cost


def test_zero_weight_dimensions_do_not_affect_default_agent():
    agent = RuleAgent()
    only = StationCandidate(name="only", side="output", x=5.0, y=0.0)
    low_risk = agent.decide(
        _obs(degradation_risk=0.0, utilization=0.0, candidate_stations=(only,)), ALLOWED_ACTIONS
    )
    high_risk = agent.decide(
        _obs(degradation_risk=1.0, utilization=1.0, candidate_stations=(only,)), ALLOWED_ACTIONS
    )
    assert low_risk.cost == high_risk.cost  # w_health=w_workload=0.0 by default


def test_custom_weights_use_health_and_workload():
    agent = RuleAgent(w_distance=0.0, w_energy=0.0, w_health=1.0, w_workload=0.0)
    only = StationCandidate(name="only", side="output", x=5.0, y=0.0)
    action = agent.decide(_obs(degradation_risk=0.42, candidate_stations=(only,)), ALLOWED_ACTIONS)
    assert action.cost == 0.42
