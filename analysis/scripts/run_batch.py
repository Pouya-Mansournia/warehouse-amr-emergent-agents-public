#!/usr/bin/env python3
"""Batch experiment runner.

    python3 analysis/scripts/run_batch.py \
        --experiment central_failure \
        --coordination-modes centralized decentralized \
        --seeds 1:30

"Never manually launch dozens of experiments" - this loops
`ros2 run experiment_manager run_experiment` over every (coordination mode, seed)
combination, one at a time. Deliberately sequential, not parallel: this repo's own
documented WSL2 capacity limits (TROUBLESHOOTING.md #21/#23 - even a single 2-3 robot
Gazebo+Nav2+SLAM stack saturates this host) mean running two experiments concurrently
would not speed anything up, it would just make both fail. A batch of independent runs
is exactly the right place to accept "slower but reliable" over "fast but interferes
with itself."

A failing run does NOT abort the batch: failures are logged, not hidden, and the
batch keeps going - every run's outcome (success/failure, exit code, experiment
directory, wall-clock duration) is recorded in `batch_manifest.json`, so a batch can be
inspected and individual failures investigated after the fact, exactly like any single
run's own `events.jsonl` is meant to be read after the fact.
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional

REPO_ROOT = Path(__file__).resolve().parents[2]
_RUN_DIR_RE = re.compile(r"^\[run_experiment\] experiment directory: (.+)$")


def parse_seeds(spec: str) -> List[int]:
    """Accepts a comma-separated list ("1,5,9"), an inclusive range ("1:30"), or a
    single value ("42")."""
    spec = spec.strip()
    if ":" in spec:
        start_str, end_str = spec.split(":", 1)
        start, end = int(start_str), int(end_str)
        if end < start:
            raise ValueError(f"invalid seed range {spec!r}: end < start")
        return list(range(start, end + 1))
    if "," in spec:
        return [int(s) for s in spec.split(",") if s.strip()]
    return [int(spec)]


def build_run_command(
    *,
    experiment: str,
    coordination: str,
    seed: int,
    num_robots: int,
    duration: float,
    agent_backend: str,
    headless: bool,
    t_fail: Optional[float] = None,
    time_source: str = "wall",
    manager_timeout_sec: Optional[float] = None,
) -> List[str]:
    cmd = [
        "ros2",
        "run",
        "experiment_manager",
        "run_experiment",
        "--mode",
        experiment,
        "--num-robots",
        str(num_robots),
        "--seed",
        str(seed),
        "--coordination",
        coordination,
        "--duration",
        str(duration),
        "--time-source",
        time_source,
    ]
    # agent_backend only means anything once per-robot agents exist - both pure
    # 'decentralized' (Mode C/D/E, no central node ever) and 'centralized_then_failover'
    # (Mode C/D/E as failover targets) run the same decentralized_agent
    # loop once active; plain 'centralized' never touches AgentBackend at all.
    if coordination in ("decentralized", "centralized_then_failover"):
        cmd += ["--agent-backend", agent_backend]
    # Mode B (Central Manager Failure) and Modes C/D/E's
    # failover comparison both need a real fault to fail over from - run_experiment.py
    # itself rejects --t-fail with pure decentralized (there is no single central node
    # to kill), so mirror that restriction here rather than passing a flag that would
    # just make the run fail.
    if t_fail is not None and coordination in ("centralized", "centralized_then_failover"):
        cmd += ["--t-fail", str(t_fail)]
    if manager_timeout_sec is not None and coordination == "centralized_then_failover":
        cmd += ["--manager-timeout-sec", str(manager_timeout_sec)]
    if headless:
        cmd.append("--headless")
    return cmd


def _extract_run_dir(output: str) -> Optional[str]:
    for line in output.splitlines():
        m = _RUN_DIR_RE.match(line.strip())
        if m:
            return m.group(1)
    return None


def run_one(
    *,
    experiment: str,
    coordination: str,
    seed: int,
    num_robots: int,
    duration: float,
    agent_backend: str,
    headless: bool,
    log_path: Path,
    t_fail: Optional[float] = None,
    time_source: str = "wall",
    manager_timeout_sec: Optional[float] = None,
) -> dict:
    cmd = build_run_command(
        experiment=experiment,
        coordination=coordination,
        seed=seed,
        num_robots=num_robots,
        duration=duration,
        agent_backend=agent_backend,
        headless=headless,
        t_fail=t_fail,
        time_source=time_source,
        manager_timeout_sec=manager_timeout_sec,
    )
    start = time.time()
    with open(log_path, "w") as log_file:
        result = subprocess.run(
            cmd, cwd=REPO_ROOT, stdout=log_file, stderr=subprocess.STDOUT
        )
    wall_clock_sec = time.time() - start
    output = log_path.read_text(errors="replace")
    return {
        "coordination": coordination,
        "seed": seed,
        "exit_code": result.returncode,
        "success": result.returncode == 0,
        "wall_clock_sec": round(wall_clock_sec, 1),
        "run_dir": _extract_run_dir(output),
        "log": str(log_path),
    }


def run_batch(
    *,
    experiment: str,
    coordination_modes: List[str],
    seeds: List[int],
    num_robots: int,
    duration: float,
    agent_backend: str,
    headless: bool,
    batch_dir: Path,
    t_fail: Optional[float] = None,
    time_source: str = "wall",
    manager_timeout_sec: Optional[float] = None,
    append: bool = False,
) -> dict:
    """`append=True` merges this call's runs into an existing `batch_manifest.json`
    in `batch_dir` instead of overwriting it - needed because a single invocation
    only takes one `agent_backend` value, but a real B/C/D/E resilience comparison
    needs `centralized_then_failover` runs at `rule`/`llm`/
    `hybrid` all landing in one manifest for `generate_report.py` to read together.
    Each run's own `coordination`/`agent_backend` already lives in its
    `summary.json` (read independently by `generate_report.py`), so merging run
    records from different invocations is safe - nothing here mixes data between
    runs, it only accumulates the list. Default `False` preserves every prior
    caller's overwrite behavior exactly."""
    batch_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = batch_dir / "batch_manifest.json"

    prior_runs: List[dict] = []
    prior_invocations: List[dict] = []
    first_started_at = datetime.now(timezone.utc).isoformat()
    if append and manifest_path.exists():
        prior = json.loads(manifest_path.read_text())
        prior_runs = prior.get("runs", [])
        prior_invocations = prior.get("invocations", [])
        if prior_invocations:
            first_started_at = prior_invocations[0].get("started_at", first_started_at)
        elif prior.get("started_at"):
            first_started_at = prior["started_at"]

    runs = []
    total = len(coordination_modes) * len(seeds)
    i = 0
    for coordination in coordination_modes:
        for seed in seeds:
            i += 1
            print(
                f"[run_batch] ({i}/{total}) experiment={experiment} "
                f"coordination={coordination} seed={seed}",
                file=sys.stderr,
            )
            # agent_backend-qualified filename: without this, re-running the same
            # (coordination, seed) with a different --agent-backend (e.g. building up
            # a merged Mode C/D/E manifest via --append) would silently overwrite the
            # previous invocation's log file.
            backend_tag = (
                f"_{agent_backend}"
                if coordination in ("decentralized", "centralized_then_failover")
                else ""
            )
            log_path = batch_dir / f"{coordination}{backend_tag}_seed{seed}.log"
            record = run_one(
                experiment=experiment,
                coordination=coordination,
                seed=seed,
                num_robots=num_robots,
                duration=duration,
                agent_backend=agent_backend,
                headless=headless,
                log_path=log_path,
                t_fail=t_fail,
                time_source=time_source,
                manager_timeout_sec=manager_timeout_sec,
            )
            runs.append(record)
            status = "OK" if record["success"] else f"FAILED (exit {record['exit_code']})"
            print(f"[run_batch]   -> {status}, {record['run_dir']}", file=sys.stderr)

    invocation_record = {
        "coordination_modes": coordination_modes,
        "seeds": seeds,
        "agent_backend": agent_backend,
        "t_fail_sec": t_fail,
        "manager_timeout_sec": manager_timeout_sec,
        "time_source": time_source,
        "started_at": datetime.now(timezone.utc).isoformat(),
    }
    all_runs = prior_runs + runs

    manifest = {
        "experiment": experiment,
        "coordination_modes": coordination_modes,
        "seeds": seeds,
        "num_robots": num_robots,
        "duration_sec": duration,
        "agent_backend": agent_backend,
        "time_source": time_source,
        "t_fail_sec": t_fail,
        "manager_timeout_sec": manager_timeout_sec,
        "started_at": first_started_at,
        "invocations": prior_invocations + [invocation_record],
        "runs": all_runs,
        "succeeded": sum(1 for r in all_runs if r["success"]),
        "failed": sum(1 for r in all_runs if not r["success"]),
    }
    manifest_path.write_text(json.dumps(manifest, indent=2))
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment", required=True, help="run_experiment.py --mode value")
    parser.add_argument(
        "--coordination-modes",
        nargs="+",
        default=["centralized"],
        choices=["centralized", "decentralized", "centralized_then_failover"],
        help="coordination modes to sweep. 'centralized_then_failover' "
        "pairs with --t-fail/--manager-timeout-sec for a fair Mode B/C/D/E resilience "
        "comparison - Mode D/E's llm/hybrid live under --agent-backend within "
        "'centralized_then_failover' (or plain 'decentralized'), not as separate "
        "coordination modes.",
    )
    parser.add_argument(
        "--agent-backend",
        default="rule",
        choices=["rule", "llm", "hybrid"],
        help="only used for --coordination-modes decentralized runs.",
    )
    parser.add_argument("--seeds", required=True, help="e.g. '1:30', '1,5,9', or '42'")
    parser.add_argument(
        "--num-robots",
        type=int,
        default=2,
        help="default 2 - this WSL2 host's documented reliable ceiling "
        "(TROUBLESHOOTING.md #21/#23), not an arbitrary choice.",
    )
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument(
        "--t-fail",
        type=float,
        default=None,
        help="Central Manager Failure experiment: seconds into each "
        "centralized/centralized_then_failover run at which to kill fleet_manager. "
        "Ignored for pure decentralized runs within the same batch (no central node "
        "to kill).",
    )
    parser.add_argument(
        "--manager-timeout-sec",
        type=float,
        default=5.0,
        help="Only applied to --coordination-modes centralized_then_failover runs: "
        "seconds of missing /fleet/manager/heartbeat before agents activate their "
        "--agent-backend failover cycle. Matches run_experiment.py's "
        "own default.",
    )
    parser.add_argument(
        "--time-source",
        choices=["wall", "simulation"],
        default="wall",
        help="simulation-time experiment control, threaded through to "
        "every run_experiment.py invocation in the batch. 'wall' (default, unchanged "
        "prior behavior). 'simulation': --duration is simulated seconds - needed for "
        "any batch campaign to produce meaningful (non-near-zero) throughput on this "
        "host's ~0.03-0.1x realtime factor, same reasoning as a single run.",
    )
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument(
        "--batch-dir",
        default=None,
        help="defaults to experiments/<date>_batch_<experiment>/",
    )
    parser.add_argument(
        "--append",
        action="store_true",
        help="merge this invocation's runs into an existing batch_manifest.json in "
        "--batch-dir instead of overwriting it. Use this to build up a single Mode "
        "B/C/D/E manifest across separate --agent-backend invocations (one call "
        "per backend, all pointed at the same --batch-dir).",
    )
    args = parser.parse_args()

    seeds = parse_seeds(args.seeds)
    batch_dir = Path(args.batch_dir) if args.batch_dir else (
        REPO_ROOT
        / "experiments"
        / f"{datetime.now(timezone.utc).strftime('%Y-%m-%d')}_batch_{args.experiment}"
    )

    manifest = run_batch(
        experiment=args.experiment,
        coordination_modes=args.coordination_modes,
        seeds=seeds,
        num_robots=args.num_robots,
        duration=args.duration,
        agent_backend=args.agent_backend,
        headless=args.headless,
        t_fail=args.t_fail,
        time_source=args.time_source,
        manager_timeout_sec=args.manager_timeout_sec,
        batch_dir=batch_dir,
        append=args.append,
    )
    print(
        f"[run_batch] done: {manifest['succeeded']}/{len(manifest['runs'])} succeeded, "
        f"manifest at {batch_dir / 'batch_manifest.json'}"
    )


if __name__ == "__main__":
    main()
