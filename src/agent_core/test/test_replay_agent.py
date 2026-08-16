import pytest

from agent_core.interfaces import ALLOWED_ACTIONS, Action, Observation
from agent_core.replay_agent import ReplayAgent


def _obs():
    return Observation(
        robot_id="robot1",
        x=0.0,
        y=0.0,
        battery_soc=1.0,
        degradation_risk=0.0,
        utilization=0.0,
        candidate_stations=(),
    )


def test_replays_script_in_order():
    script = [
        Action(action="BID_FOR_TASK", station_name="s1", cost=0.1),
        Action(action="WAIT"),
    ]
    agent = ReplayAgent(script=script)
    assert agent.decide(_obs(), ALLOWED_ACTIONS) == script[0]
    assert agent.decide(_obs(), ALLOWED_ACTIONS) == script[1]


def test_waits_forever_once_script_exhausted():
    agent = ReplayAgent(script=[Action(action="WAIT")])
    agent.decide(_obs(), ALLOWED_ACTIONS)
    assert agent.decide(_obs(), ALLOWED_ACTIONS) == Action(action="WAIT")
    assert agent.decide(_obs(), ALLOWED_ACTIONS) == Action(action="WAIT")


def test_ignores_observation_contents():
    agent = ReplayAgent(script=[Action(action="BID_FOR_TASK", station_name="fixed", cost=0.5)])
    action = agent.decide(_obs(), ALLOWED_ACTIONS)
    assert action.station_name == "fixed"


def test_action_rejects_unknown_type():
    with pytest.raises(ValueError):
        Action(action="LAUNCH_MISSILES")


def test_action_rejects_bid_without_station():
    with pytest.raises(ValueError):
        Action(action="BID_FOR_TASK")
