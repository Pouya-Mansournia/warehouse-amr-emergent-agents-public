"""Real, free, local LLM provider: an `LLMClient` backed by Ollama (https://ollama.com).

Chosen over a hosted API for this project specifically because it needs no account, no
API key, and no per-token cost (the user has none of those to spend) - it runs entirely
on the same machine already running the WSL2 simulation. Uses only the Python standard
library (`urllib`), matching `agent_core`'s dependency-free design (no `requests`, no
Ollama SDK) - the point of `LLMClient` being a one-method seam (llm_client.py's
docstring) is that swapping to a hosted provider later needs nothing more than a second
small class like this one.

Requests `format: "json"` from Ollama's `/api/generate` endpoint, which constrains the
model's own decoding to emit syntactically valid JSON - this catches most malformed
output before it ever reaches `LLMAgent`'s own schema/safety validation, though that
validation (and the retry/fallback it drives) is still what actually enforces the
*content* being correct (e.g. real vs. hallucinated station names) - `format: "json"`
only guarantees the response parses, not that it's a schema-valid or safe Action.

Network/timeout failures return `""` rather than raising, so `LLMAgent`'s existing
retry/fallback logic (built for "the model said something we can't use") handles "the
server wasn't reachable" the same way, without `LLMAgent` needing to know the
difference or `LLMClient`'s single-method contract needing an exception clause.
"""
from __future__ import annotations

import json
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Optional

from agent_core.llm_client import LLMClient

# Identifies this client's provider in persisted decision records
# (agent_decisions.jsonl's "provider" field) - a plain string constant, not something
# that needs its own config, since this class only ever talks to Ollama.
PROVIDER_NAME = "ollama"


@dataclass
class OllamaClient(LLMClient):
    model: str = "llama3.2:1b"
    host: str = "http://127.0.0.1:11434"
    timeout_sec: float = 30.0
    provider: str = field(default=PROVIDER_NAME, init=False)
    # Token counts from the MOST RECENT complete() call, duck-typed
    # attributes LLMAgent reads after calling complete() (see llm_agent.py's decide())
    # - not part of LLMClient's own minimal interface (llm_client.py's docstring: "one
    # method, raw string in, raw string out"), so a client that doesn't set these
    # (FakeLLMClient, or a future provider) just leaves LLMAgent reading None, which it
    # already handles as "not available" and stores as null.
    last_prompt_tokens: Optional[int] = field(default=None, init=False, repr=False)
    last_completion_tokens: Optional[int] = field(default=None, init=False, repr=False)

    def complete(self, prompt: str) -> str:
        self.last_prompt_tokens = None
        self.last_completion_tokens = None
        body = json.dumps(
            {"model": self.model, "prompt": prompt, "stream": False, "format": "json"}
        ).encode("utf-8")
        request = urllib.request.Request(
            f"{self.host}/api/generate",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                payload = json.loads(response.read())
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError):
            return ""
        # Ollama's non-streaming /api/generate response includes these when the model
        # finishes normally - real, provider-reported counts, never estimated (section
        # 24). Absent (e.g. an error response that still parsed as JSON) -> None.
        self.last_prompt_tokens = payload.get("prompt_eval_count")
        self.last_completion_tokens = payload.get("eval_count")
        return payload.get("response", "")
