#!/usr/bin/env python3
"""Phase II experiment runner - the lightweight-world counterpart to Phase I's
`experiment_manager/run_experiment.py`, same "one command, immutable output
directory" discipline, none of the ROS/Gazebo machinery.

    python3 phase2/sim/run_experiment.py \
        --condition llm_memory --seed 1 --num-robots 4 --duration-sec 3000 \
        --out-dir phase2/experiments/llm_memory_seed1

Four conditions, matched to this pilot's explicit scope (a fifth condition,
"local information only", is not implemented this pilot):
  deterministic_no_memory  - RuleAgent, memory OFF
  llm_no_memory             - LLMAgent (Ollama), memory OFF
  llm_memory                 - LLMAgent (Ollama), memory ON
  hybrid_memory               - HybridAgent (Ollama-backed), memory ON

Real per-robot PeerMemory state IS persisted to disk this time
(`peer_memory.json`), closing a gap from Phase I where nothing was written
to disk about what was learned.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from world.interactions import InteractionLog  # noqa: E402
from world.reuse import HybridAgent, LLMAgent, OllamaClient, RuleAgent  # noqa: E402
from world.world import CHARGER_STATION, World  # noqa: E402

CONDITIONS = {
    "deterministic_no_memory": {"backend": "rule", "memory": False},
    "llm_no_memory": {"backend": "llm", "memory": False},
    "llm_memory": {"backend": "llm", "memory": True},
    "hybrid_memory": {"backend": "hybrid", "memory": True},
}

STATE_SAMPLE_INTERVAL_SEC = 10.0


def _make_backend(kind: str):
    if kind == "rule":
        return lambda robot_id: RuleAgent()
    if kind == "llm":
        return lambda robot_id: LLMAgent(client=OllamaClient(), fallback=RuleAgent())
    if kind == "hybrid":
        return lambda robot_id: HybridAgent(
            deterministic=RuleAgent(),
            llm=LLMAgent(client=OllamaClient(), fallback=RuleAgent()),
        )
    raise ValueError(f"unknown backend kind: {kind!r}")


def run(
    *,
    condition: str,
    seed: int,
    num_robots: int,
    duration_sec: int,
    stations_per_side: int,
    out_dir: Path,
) -> dict:
    spec = CONDITIONS[condition]
    out_dir.mkdir(parents=True, exist_ok=True)
    interactions = InteractionLog(out_dir / "interactions.jsonl")
    world = World(
        num_robots=num_robots, seed=seed, backend_factory=_make_backend(spec["backend"]),
        memory_enabled=spec["memory"], interaction_log=interactions,
        stations_per_side=stations_per_side,
    )

    state_samples = []
    wall_start = time.time()
    for tick in range(duration_sec):
        world.step()
        if tick % STATE_SAMPLE_INTERVAL_SEC == 0:
            for r in world.robots.values():
                state_samples.append({
                    "simulation_time": world.sim_time,
                    "robot_id": r.robot_id,
                    "x": round(r.x, 3), "y": round(r.y, 3),
                    "battery_soc": round(r.battery_soc, 4),
                    "state": r.state,
                    "current_side": r.current_side,
                    "target": r.target[0] if r.target else None,
                    "tasks_completed": r.tasks_completed,
                    "distance_traveled_m": round(r.distance_traveled_m, 2),
                    "utilization": r.utilization(),
                })
    wall_clock_sec = time.time() - wall_start
    interactions.close()

    (out_dir / "robot_states.jsonl").write_text(
        "\n".join(json.dumps(s) for s in state_samples) + "\n", encoding="utf-8"
    )

    peer_memory_summary = {}
    if spec["memory"]:
        for r in world.robots.values():
            if r.memory is not None:
                peer_memory_summary[r.robot_id] = r.memory.summary(max_peers=num_robots)
    (out_dir / "peer_memory.json").write_text(
        json.dumps(peer_memory_summary, indent=2), encoding="utf-8"
    )

    metadata = {
        "condition": condition,
        "backend": spec["backend"],
        "memory_enabled": spec["memory"],
        "seed": seed,
        "num_robots": num_robots,
        "duration_sec": duration_sec,
        "stations_per_side": stations_per_side,
        "wall_clock_sec": round(wall_clock_sec, 3),
        "started_at": datetime.now(timezone.utc).isoformat(),
    }
    (out_dir / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")

    from collections import Counter
    type_result_counts = Counter(
        (e["interaction_type"], e["result"]) for e in interactions.events
    )
    summary = {
        **metadata,
        "tasks_completed_total": sum(r.tasks_completed for r in world.robots.values()),
        "tasks_completed_per_robot": {r.robot_id: r.tasks_completed for r in world.robots.values()},
        "final_battery_soc": {r.robot_id: round(r.battery_soc, 4) for r in world.robots.values()},
        "distance_traveled_m": {r.robot_id: round(r.distance_traveled_m, 2) for r in world.robots.values()},
        "utilization": {r.robot_id: r.utilization() for r in world.robots.values()},
        "interaction_counts": {f"{t}|{r}": c for (t, r), c in type_result_counts.items()},
        "total_interactions": len(interactions.events),
        "negotiations_initiated": sum(
            1 for e in interactions.events
            if e["interaction_type"] == "HELP_REQUEST" and e["result"] is None
        ),
        "negotiations_succeeded": type_result_counts.get(("TASK_TRANSFER", "SUCCESS"), 0),
        "charging_cycles_completed": type_result_counts.get(("CHARGER_YIELD", "RELEASED"), 0),
        "resource_conflicts_blocked": (
            type_result_counts.get(("RESOURCE_REQUEST", "BLOCKED_NO_CANDIDATES"), 0)
            + type_result_counts.get(("CHARGER_REQUEST", "BLOCKED_NO_CANDIDATES"), 0)
            + type_result_counts.get(("RESOURCE_YIELD", "LOST_CONTENTION"), 0)
            + type_result_counts.get(("CHARGER_YIELD", "LOST_CONTENTION"), 0)
        ),
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--condition", required=True, choices=list(CONDITIONS))
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--num-robots", type=int, default=4)
    parser.add_argument("--duration-sec", type=int, default=3000)
    parser.add_argument("--stations-per-side", type=int, default=3)
    parser.add_argument("--out-dir", required=True)
    args = parser.parse_args()

    summary = run(
        condition=args.condition, seed=args.seed, num_robots=args.num_robots,
        duration_sec=args.duration_sec, stations_per_side=args.stations_per_side,
        out_dir=Path(args.out_dir),
    )
    print(
        f"[phase2 run_experiment] condition={args.condition} seed={args.seed} "
        f"tasks={summary['tasks_completed_total']} "
        f"negotiations={summary['negotiations_initiated']} "
        f"charging_cycles={summary['charging_cycles_completed']} "
        f"resource_conflicts={summary['resource_conflicts_blocked']} "
        f"wall_clock_sec={summary['wall_clock_sec']}"
    )
    print(f"[phase2 run_experiment] wrote {args.out_dir}")


if __name__ == "__main__":
    main()
