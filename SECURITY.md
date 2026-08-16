# Security Policy

## Reporting a vulnerability

This is a research/simulation platform, not production infrastructure
— it has no network-facing service beyond the optional local web
dashboard (`src/amr_ros_dg`, intended for localhost use during
simulation) and a local Ollama LLM server. If you find a security
issue (e.g. an unsafe deserialization path, an injection vector in
experiment configuration parsing, or a credential-handling problem),
please report it privately to the maintainer
(p.mansournia@gmail.com) rather than opening a public issue, so a fix
can be prepared first.

## Scope notes

- No cloud credentials or API keys are used by default — the LLM
  backend is a local Ollama server. If you configure a cloud LLM
  provider, keep its API key in an environment variable (see
  `.env.example`), never committed to the repository.
- The LLM never has a direct path to robot motor commands; all
  LLM-backed decisions pass through a schema/safety validator with a
  deterministic fallback (see `README.md`'s "Safety"
  section). Vulnerabilities in that validation path are considered
  high priority.
