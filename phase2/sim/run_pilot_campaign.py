#!/usr/bin/env python3
"""Phase II pilot campaign: 4 conditions x N paired seeds, one manifest.

    python3 phase2/sim/run_pilot_campaign.py --seeds 1,2,3,4,5 \
        --num-robots 4 --duration-sec 3000 --out-dir phase2/experiments/pilot_001

Same seeds are used across all four conditions (deterministic_no_memory,
llm_no_memory, llm_memory, hybrid_memory) so environment/task-sequence
randomness is comparable across conditions, per the pilot's explicit design
requirement. A failing run does not abort the campaign (same "log failures,
keep going" discipline as Phase I's run_batch.py).
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_experiment import CONDITIONS, run  # noqa: E402


def parse_seeds(spec: str):
    spec = spec.strip()
    if ":" in spec:
        a, b = spec.split(":", 1)
        return list(range(int(a), int(b) + 1))
    if "," in spec:
        return [int(s) for s in spec.split(",") if s.strip()]
    return [int(spec)]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", required=True)
    parser.add_argument("--num-robots", type=int, default=4)
    parser.add_argument("--duration-sec", type=int, default=3000)
    parser.add_argument("--stations-per-side", type=int, default=3)
    parser.add_argument("--out-dir", required=True)
    args = parser.parse_args()

    seeds = parse_seeds(args.seeds)
    campaign_dir = Path(args.out_dir)
    campaign_dir.mkdir(parents=True, exist_ok=True)

    runs = []
    total = len(CONDITIONS) * len(seeds)
    i = 0
    campaign_start = time.time()
    for condition in CONDITIONS:
        for seed in seeds:
            i += 1
            run_dir = campaign_dir / f"{condition}_seed{seed}"
            print(
                f"[phase2 campaign] ({i}/{total}) condition={condition} seed={seed} "
                f"-> {run_dir}",
                file=sys.stderr,
            )
            start = time.time()
            try:
                summary = run(
                    condition=condition, seed=seed, num_robots=args.num_robots,
                    duration_sec=args.duration_sec, stations_per_side=args.stations_per_side,
                    out_dir=run_dir,
                )
                record = {
                    "condition": condition, "seed": seed, "run_dir": str(run_dir),
                    "success": True, "wall_clock_sec": round(time.time() - start, 2),
                    "tasks_completed_total": summary["tasks_completed_total"],
                    "negotiations_initiated": summary["negotiations_initiated"],
                    "charging_cycles_completed": summary["charging_cycles_completed"],
                    "resource_conflicts_blocked": summary["resource_conflicts_blocked"],
                }
                print(
                    f"[phase2 campaign]   -> OK tasks={record['tasks_completed_total']} "
                    f"negotiations={record['negotiations_initiated']} "
                    f"charging={record['charging_cycles_completed']} "
                    f"conflicts={record['resource_conflicts_blocked']} "
                    f"({record['wall_clock_sec']}s)",
                    file=sys.stderr,
                )
            except Exception as exc:  # noqa: BLE001 - log and keep going, same as Phase I's run_batch.py
                record = {
                    "condition": condition, "seed": seed, "run_dir": str(run_dir),
                    "success": False, "wall_clock_sec": round(time.time() - start, 2),
                    "error": f"{exc.__class__.__name__}: {exc}",
                }
                print(f"[phase2 campaign]   -> FAILED: {record['error']}", file=sys.stderr)
                traceback.print_exc(file=sys.stderr)
            runs.append(record)

            manifest = {
                "seeds": seeds, "num_robots": args.num_robots,
                "duration_sec": args.duration_sec, "stations_per_side": args.stations_per_side,
                "started_at": datetime.now(timezone.utc).isoformat(),
                "campaign_wall_clock_sec_so_far": round(time.time() - campaign_start, 2),
                "runs": runs,
                "succeeded": sum(1 for r in runs if r["success"]),
                "failed": sum(1 for r in runs if not r["success"]),
            }
            (campaign_dir / "campaign_manifest.json").write_text(
                json.dumps(manifest, indent=2), encoding="utf-8"
            )

    print(
        f"[phase2 campaign] done: {manifest['succeeded']}/{len(runs)} succeeded, "
        f"{manifest['campaign_wall_clock_sec_so_far']}s total, "
        f"manifest at {campaign_dir / 'campaign_manifest.json'}"
    )


if __name__ == "__main__":
    main()
