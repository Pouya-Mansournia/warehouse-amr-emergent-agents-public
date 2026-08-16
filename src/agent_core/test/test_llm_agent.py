import pytest

from agent_core.interfaces import ALLOWED_ACTIONS, Observation, StationCandidate
from agent_core.llm_agent import LLMAgent, build_prompt
from agent_core.llm_client import FakeLLMClient
from agent_core.replay_agent import ReplayAgent
from agent_core.interfaces import Action


def _obs(**overrides):
    defaults = dict(
        robot_id="robot1",
        x=0.0,
        y=0.0,
        battery_soc=1.0,
        degradation_risk=0.0,
        utilization=0.0,
        candidate_stations=(StationCandidate(name="output_station_5", side="output", x=6.9, y=-1.0),),
    )
    defaults.update(overrides)
    return Observation(**defaults)


def test_valid_bid_response_is_accepted():
    client = FakeLLMClient(responses=['{"action": "BID_FOR_TASK", "station_name": "output_station_5", "cost": 0.3}'])
    agent = LLMAgent(client=client)
    action = agent.decide(_obs(), ALLOWED_ACTIONS)
    assert action == Action(action="BID_FOR_TASK", station_name="output_station_5", cost=0.3)
    assert agent.stats.requests == 1
    assert agent.stats.schema_failures == 0


def test_valid_wait_response_is_accepted():
    client = FakeLLMClient(responses=['{"action": "WAIT"}'])
    agent = LLMAgent(client=client)
    action = agent.decide(_obs(), ALLOWED_ACTIONS)
    assert action == Action(action="WAIT")


def test_malformed_json_falls_back_to_wait_after_retries():
    client = FakeLLMClient(responses=["not json at all"])
    agent = LLMAgent(client=client, max_retries=1)
    action = agent.decide(_obs(), ALLOWED_ACTIONS)
    assert action == Action(action="WAIT")
    assert agent.stats.requests == 2  # 1 initial + 1 retry
    assert agent.stats.schema_failures == 2
    assert agent.stats.fallbacks_used == 1


def test_hallucinated_station_is_rejected_as_unsafe():
    client = FakeLLMClient(
        responses=['{"action": "BID_FOR_TASK", "station_name": "made_up_station", "cost": 0.1}']
    )
    agent = LLMAgent(client=client, max_retries=0)
    action = agent.decide(_obs(), ALLOWED_ACTIONS)
    assert action == Action(action="WAIT")
    assert agent.stats.schema_failures == 1


def test_disallowed_action_type_is_rejected():
    client = FakeLLMClient(responses=['{"action": "LAUNCH_MISSILES"}'])
    agent = LLMAgent(client=client, max_retries=0)
    action = agent.decide(_obs(), ALLOWED_ACTIONS)
    assert action == Action(action="WAIT")


def test_retry_succeeds_on_second_attempt():
    client = FakeLLMClient(
        responses=[
            "garbage",
            '{"action": "BID_FOR_TASK", "station_name": "output_station_5", "cost": 0.2}',
        ]
    )
    agent = LLMAgent(client=client, max_retries=1)
    action = agent.decide(_obs(), ALLOWED_ACTIONS)
    assert action.action == "BID_FOR_TASK"
    assert agent.stats.schema_failures == 1
    assert agent.stats.fallbacks_used == 0


def test_fallback_backend_used_when_all_retries_exhausted():
    client = FakeLLMClient(responses=["garbage"])
    fallback = ReplayAgent(script=[Action(action="BID_FOR_TASK", station_name="output_station_5", cost=0.9)])
    agent = LLMAgent(client=client, max_retries=0, fallback=fallback)
    action = agent.decide(_obs(), ALLOWED_ACTIONS)
    assert action.station_name == "output_station_5"
    assert action.cost == 0.9
    assert agent.stats.fallbacks_used == 1


def test_prompt_contains_only_observation_fields_and_candidates():
    obs = _obs()
    prompt = build_prompt(obs)
    assert "robot1" in prompt
    assert "output_station_5" in prompt
    assert "JSON" in prompt


def test_prompt_lists_no_candidates_when_none_free():
    prompt = build_prompt(_obs(candidate_stations=()))
    assert "none currently free" in prompt


def test_missing_cost_field_is_rejected():
    client = FakeLLMClient(responses=['{"action": "BID_FOR_TASK", "station_name": "output_station_5"}'])
    agent = LLMAgent(client=client, max_retries=0)
    action = agent.decide(_obs(), ALLOWED_ACTIONS)
    assert action == Action(action="WAIT")


def test_non_object_json_is_rejected():
    client = FakeLLMClient(responses=["42"])
    agent = LLMAgent(client=client, max_retries=0)
    action = agent.decide(_obs(), ALLOWED_ACTIONS)
    assert action == Action(action="WAIT")


def test_decision_meta_on_success_reports_schema_and_safety_valid():
    client = FakeLLMClient(responses=['{"action": "BID_FOR_TASK", "station_name": "output_station_5", "cost": 0.3}'])
    agent = LLMAgent(client=client)
    agent.decide(_obs(), ALLOWED_ACTIONS)

    meta = agent.last_decision_meta
    assert meta.schema_valid is True
    assert meta.safety_valid is True
    assert meta.fallback_used is False
    assert meta.retry_count == 0


def test_decision_meta_distinguishes_schema_failure_from_safety_failure():
    client = FakeLLMClient(responses=["not json"])
    agent = LLMAgent(client=client, max_retries=0)
    agent.decide(_obs(), ALLOWED_ACTIONS)
    assert agent.last_decision_meta.schema_valid is False
    assert agent.last_decision_meta.safety_valid is None

    client2 = FakeLLMClient(responses=['{"action": "BID_FOR_TASK", "station_name": "made_up_station", "cost": 0.1}'])
    agent2 = LLMAgent(client=client2, max_retries=0)
    agent2.decide(_obs(), ALLOWED_ACTIONS)
    assert agent2.last_decision_meta.schema_valid is True
    assert agent2.last_decision_meta.safety_valid is False


def test_decision_meta_retry_count_reflects_which_attempt_succeeded():
    client = FakeLLMClient(
        responses=["garbage", '{"action": "BID_FOR_TASK", "station_name": "output_station_5", "cost": 0.2}']
    )
    agent = LLMAgent(client=client, max_retries=1)
    agent.decide(_obs(), ALLOWED_ACTIONS)
    assert agent.last_decision_meta.retry_count == 1


def test_decision_meta_reports_fallback_used_on_exhaustion():
    client = FakeLLMClient(responses=["garbage"])
    agent = LLMAgent(client=client, max_retries=0)
    agent.decide(_obs(), ALLOWED_ACTIONS)
    assert agent.last_decision_meta.fallback_used is True
    assert agent.last_decision_meta.retry_count == 0


def test_decision_meta_provider_and_model_from_client_when_present():
    class _FakeProviderClient(FakeLLMClient):
        provider = "fake-provider"
        model = "fake-model-1b"

    client = _FakeProviderClient(responses=['{"action": "WAIT"}'])
    agent = LLMAgent(client=client)
    agent.decide(_obs(), ALLOWED_ACTIONS)
    assert agent.last_decision_meta.provider == "fake-provider"
    assert agent.last_decision_meta.model == "fake-model-1b"


def test_decision_meta_provider_and_model_none_when_client_does_not_report_them():
    client = FakeLLMClient(responses=['{"action": "WAIT"}'])
    agent = LLMAgent(client=client)
    agent.decide(_obs(), ALLOWED_ACTIONS)
    assert agent.last_decision_meta.provider is None
    assert agent.last_decision_meta.model is None
