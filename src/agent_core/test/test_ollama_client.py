"""Live integration test against a real local Ollama server, validating the
"an LLM can propose high-level fleet actions" behavior with an actual model rather
than only FakeLLMClient.

Skips automatically if Ollama isn't reachable, so the rest of the suite (and any
environment without Ollama installed) is unaffected - this test exists to be run where
Ollama IS available, not to gate CI on it.
"""
import socket

import pytest

from agent_core.interfaces import ALLOWED_ACTIONS, Observation, StationCandidate
from agent_core.llm_agent import LLMAgent
from agent_core.ollama_client import OllamaClient
from agent_core.rule_agent import RuleAgent


def _ollama_reachable(host: str = "127.0.0.1", port: int = 11434) -> bool:
    try:
        with socket.create_connection((host, port), timeout=1.0):
            return True
    except OSError:
        return False


pytestmark = pytest.mark.skipif(
    not _ollama_reachable(), reason="no local Ollama server reachable on 127.0.0.1:11434"
)


def _obs():
    return Observation(
        robot_id="robot1",
        x=0.0,
        y=0.0,
        battery_soc=0.9,
        degradation_risk=0.0,
        utilization=0.0,
        candidate_stations=(
            StationCandidate(name="output_station_5", side="output", x=6.9, y=-1.0),
            StationCandidate(name="output_station_6", side="output", x=6.9, y=1.0),
        ),
    )


def test_real_model_produces_a_valid_action_or_falls_back_safely():
    client = OllamaClient(model="llama3.2:1b")
    agent = LLMAgent(client=client, max_retries=2, fallback=RuleAgent())
    action = agent.decide(_obs(), ALLOWED_ACTIONS)

    # Whatever the model actually said, decide() must always return a schema-valid
    # Action - either the model's own valid proposal, or (after retries) the RuleAgent
    # fallback. There is no code path here that could return raw model text or crash.
    assert action.action in ALLOWED_ACTIONS
    if action.action == "BID_FOR_TASK":
        assert action.station_name in {"output_station_5", "output_station_6"}
        assert isinstance(action.cost, float)

    assert agent.stats.requests >= 1


def test_real_model_never_proposes_a_hallucinated_station():
    """Runs several times since a real model's output varies - every single response
    must still pass the safety check (or be rejected and fall back), never silently
    let through a station outside what was offered."""
    client = OllamaClient(model="llama3.2:1b")
    for _ in range(3):
        agent = LLMAgent(client=client, max_retries=2, fallback=RuleAgent())
        action = agent.decide(_obs(), ALLOWED_ACTIONS)
        if action.action == "BID_FOR_TASK":
            assert action.station_name in {"output_station_5", "output_station_6"}
