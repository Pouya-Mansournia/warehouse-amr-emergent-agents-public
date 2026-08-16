# Phase II — Experiment Family II (Long-Horizon Collective Behavior)

A separate, isolated codebase from Phase I (`src/`, `experiments/`, `analysis/`,
`docs/` at the repo root). This directory exists specifically so Phase II can be
built and run without ever modifying Phase I's data or code.

## Isolation rules (non-negotiable, per explicit author direction, 2026-08-13)

1. **Nothing under `phase2/` ever edits a file outside `phase2/`**, except
   `paper/main.tex`'s already-existing Family II placeholder section (which
   Phase I's own Future Work section designated for exactly this).
2. **Phase I's `agent_core`/`fleet_coordination` modules are imported
   read-only** (`world/reuse.py`) - never copied, never modified. If Phase II
   needs different behavior, it wraps or subclasses locally in `phase2/`,
   it does not edit the Phase I source.
3. **If Phase II's data ever suggests a Phase I result was wrong, incomplete,
   or improvable, that is documented here as a new, additive Phase II
   finding - Phase I's own docs/data are never retroactively "corrected."**
   Phase I is frozen (see the root README's Future Work section).
4. All Phase II work happens on the `phase2-emergence` git branch, kept
   separate from `main` until/unless explicitly merged.

## Why a separate lightweight world, not Gazebo

Phase I's embodied ROS 2 + Gazebo + Nav2/SLAM platform validated real
navigation and physics, but costs 15-40+ minutes of wall-clock time per run
on this hardware (documented, measured realtime factor ~0.03-0.1x) - making
the long-horizon, many-seed runs Experiment Family II needs infeasible here.
Phase II trades embodiment fidelity for speed: a pure-Python, tick-based
world (`world/world.py`) with no physics engine, running the SAME
decision-making code (`RuleAgent`/`LLMAgent`/`HybridAgent`, `ClaimBook`,
`Conversation`, `PeerMemory` - imported from Phase I, not reimplemented) at
effectively unlimited speed: thousands of simulated seconds in well under a
second of wall-clock time for the deterministic backend, and real Ollama
latency (unavoidably ~0.5s wall-clock per simulated second) only for
LLM/hybrid conditions.

This is an explicit, documented tradeoff, not a hidden one - see
`world/world.py`'s own module docstring for exactly what's simplified
(no physics/collision, synchronous single-process claim resolution instead
of a real distributed race) and what's kept faithful (the actual decision
logic, the actual claim tie-break rule, the actual negotiation protocol).

## Layout

- `world/` - the simulation itself (`world.py`), the interaction-logging
  schema (`interactions.py`), and the read-only Phase I import shim (`reuse.py`).
- `sim/` - experiment runner (`run_experiment.py`, one run) and pilot
  campaign orchestrator (`run_pilot_campaign.py`, N seeds x 4 conditions).
- `analysis/` - role formation, peer preference, resource conventions,
  interaction network, and emergence-criteria evaluation, plus the report/
  figure generators that tie them together.
- `test/` - pytest suite for everything above (`pytest phase2/test -q` from
  the repo root, or `pytest test -q` from inside `phase2/`).
- `experiments/` - gitignored (`phase2/.gitignore`), immutable per-run output,
  same convention as the root `experiments/` directory.

## Conditions (this pilot's explicit scope)

| Condition | Backend | Memory |
|---|---|---|
| `deterministic_no_memory` | RuleAgent | off |
| `llm_no_memory` | LLMAgent (Ollama) | off |
| `llm_memory` | LLMAgent (Ollama) | on |
| `hybrid_memory` | HybridAgent (Ollama-backed) | on |

The clean memory ablation (same backend, memory on vs. off) is
`llm_no_memory` vs. `llm_memory` specifically - the only pair in this set
that holds the backend constant.

## Status

Pilot campaign: 4 conditions x 5 paired seeds x 3000 simulated seconds
(`phase2/experiments/pilot_001/`). See `docs/notes.md` and
`phase2/analysis/results/` for the analyzed results.
