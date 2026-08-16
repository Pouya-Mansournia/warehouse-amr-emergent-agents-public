"""Provider-independent LLM transport - deliberately not hard-coded to one
commercial model.

`LLMClient` is deliberately the thinnest possible seam: one method, raw string in, raw
string out. `LLMAgent` (llm_agent.py) owns everything provider-agnostic - prompt
construction, JSON-schema validation, safety rejection, retry policy. A concrete client
for a real API (Anthropic, OpenAI, ...) only ever has to implement `complete()`; it is
never trusted to have gotten the response format right, which is what makes swapping
providers safe.

`FakeLLMClient` (below) is deterministic and network-free, for tests. `OllamaClient`
(ollama_client.py) is the real provider - a local, free model via Ollama, chosen
so the platform can be run and validated without a paid API. Wiring a
different/paid provider later is a drop-in addition - implement `LLMClient.complete()`
against that provider's SDK, nothing else in this package needs to change.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List


class LLMClient(ABC):
    """One round-trip: a prompt string in, a raw completion string out. No parsing,
    no retries, no schema knowledge - that all belongs to the caller (`LLMAgent`)."""

    @abstractmethod
    def complete(self, prompt: str) -> str:
        raise NotImplementedError


@dataclass
class FakeLLMClient(LLMClient):
    """Returns `responses[i]` on the i-th call; repeats the last response once exhausted
    (or returns "" if never given any) - deterministic, no network, for tests and for
    exercising LLMAgent's validation/retry/fallback pipeline without a real provider."""

    responses: List[str] = field(default_factory=list)
    _calls: int = field(default=0, init=False, repr=False)

    def complete(self, prompt: str) -> str:
        if not self.responses:
            response = ""
        elif self._calls < len(self.responses):
            response = self.responses[self._calls]
        else:
            response = self.responses[-1]
        self._calls += 1
        return response
