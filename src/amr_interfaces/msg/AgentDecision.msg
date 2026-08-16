# One record per AgentBackend.decide() call, capturing "Agent Performance" metrics
# such as decision_latency for the agent_decisions.jsonl log. Published on
# /fleet/agent/decision by every decentralized_agent, regardless of which backend
# (rule/llm/hybrid) is configured - this is what lets cross-mode comparison tables
# report a real, measured "Decision Latency" column instead of an invented one.

string robot_id
string backend         # "rule" | "llm" | "hybrid" - decentralized_agent.py's agent_backend param
string action           # "BID_FOR_TASK" | "WAIT" - the Action.action this decide() call returned
float32 decision_latency_sec
builtin_interfaces/Time stamp

# --- LLM observability ---
# has_llm_meta=false for every field below (backend=="rule", or "hybrid" answered
# deterministically without escalating) - these fields are meaningless/unset in that
# case, NOT "false"/"0" as a measured value. Only meaningful when has_llm_meta=true
# (backend=="llm", or "hybrid" DID escalate this decision to the LLM).
bool has_llm_meta
string provider          # e.g. "ollama" - "" if not reported
string model              # e.g. "llama3.2:1b" - "" if not reported
int32 prompt_tokens        # -1 = not reported by the provider (never estimated)
int32 completion_tokens    # -1 = not reported by the provider (never estimated)
bool schema_valid           # only meaningful if has_llm_meta
bool safety_valid            # only meaningful if has_llm_meta AND schema_valid
bool fallback_used
int32 retry_count
