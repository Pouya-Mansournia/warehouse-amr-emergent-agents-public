from agent_core.llm_client import FakeLLMClient


def test_returns_responses_in_order():
    client = FakeLLMClient(responses=["a", "b"])
    assert client.complete("prompt") == "a"
    assert client.complete("prompt") == "b"


def test_repeats_last_response_once_exhausted():
    client = FakeLLMClient(responses=["only"])
    client.complete("prompt")
    assert client.complete("prompt") == "only"
    assert client.complete("prompt") == "only"


def test_empty_responses_returns_empty_string():
    client = FakeLLMClient()
    assert client.complete("prompt") == ""
