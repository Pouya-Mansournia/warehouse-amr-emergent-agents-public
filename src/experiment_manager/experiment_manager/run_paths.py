"""Experiment directory layout and metadata helpers.

Creates one immutable directory per run under experiments/, as required
by docs/architecture.md's Phase 1 acceptance criterion: reconstruct what
happened purely from saved data. Never overwrites an existing run.
"""
from __future__ import annotations

import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path

def _find_repo_root(start: Path) -> Path:
    """Walk up from `start` looking for the repo's `.git` directory.

    `Path(__file__).resolve().parents[3]` (the old approach) only landed on the repo
    root when colcon used `--symlink-install`, which makes the installed file a symlink
    that `.resolve()` follows back into `src/`. A plain (non-symlink) colcon install
    copies the file instead, so `__file__` resolves to somewhere under `install/`, and a
    fixed parent-count silently pointed `experiments/` at the wrong directory - found
    live when `experiment_manager` was rebuilt without `--symlink-install` for the first
    time this session and every subsequent run wrote its data under
    `install/experiment_manager/lib/.../experiments/` instead of the repo's own
    `experiments/` (Phase 1's core acceptance criterion silently broken). Searching for
    `.git` works identically regardless of install method.
    """
    for candidate in (start, *start.parents):
        if (candidate / ".git").exists():
            return candidate
    raise RuntimeError(
        f"could not find repo root (no .git found walking up from {start})"
    )


REPO_ROOT = _find_repo_root(Path(__file__).resolve())
EXPERIMENTS_ROOT = REPO_ROOT / "experiments"


def _git_commit(cwd: Path) -> str:
    try:
        out = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=cwd,
            capture_output=True,
            text=True,
            check=True,
        )
        return out.stdout.strip()
    except Exception:
        return "unknown"


def make_run_dir(mode: str) -> Path:
    """Create experiments/<date>_<mode>_<NNN>/ and return its path.

    NNN increments per day+mode so repeated runs never collide or
    overwrite each other.
    """
    date = datetime.now(timezone.utc).strftime("%Y-%m-%d")
    EXPERIMENTS_ROOT.mkdir(parents=True, exist_ok=True)
    existing = sorted(EXPERIMENTS_ROOT.glob(f"{date}_{mode}_*"))
    next_n = len(existing) + 1
    while True:
        run_dir = EXPERIMENTS_ROOT / f"{date}_{mode}_{next_n:03d}"
        if not run_dir.exists():
            break
        next_n += 1
    run_dir.mkdir(parents=True)
    (run_dir / "rosbag").mkdir()
    (run_dir / "plots").mkdir()
    return run_dir


def write_metadata(
    run_dir: Path,
    *,
    mode: str,
    num_robots: int,
    seed: int,
    ros_distro: str,
    gazebo_version: str,
    t_fail_sec: float | None = None,
    coordination: str = "centralized",
    agent_backend: str = "rule",
    time_source: str = "wall",
    llm_provider: str | None = None,
    llm_model: str | None = None,
    memory_enabled: bool = False,
) -> None:
    metadata = {
        "git_commit": _git_commit(REPO_ROOT),
        "ros_distribution": ros_distro,
        "gazebo_version": gazebo_version,
        "experiment_mode": mode,
        "coordination": coordination,
        "agent_backend": agent_backend,
        "robot_count": num_robots,
        "random_seed": seed,
        "t_fail_sec": t_fail_sec,
        # Populated by run_experiment.py from the same OllamaClient() defaults
        # decentralized_agent.py actually constructs, when agent_backend is llm/hybrid
        # (left None otherwise).
        "llm_provider": llm_provider,
        "llm_model": llm_model,
        # Whether episodic peer memory is enabled - records which of a paired
        # memory-ablation comparison this run belongs to.
        "memory_enabled": memory_enabled,
        "model_parameters": None,
        "prompt_version": None,
        "start_time": datetime.now(timezone.utc).isoformat(),
        "duration_sec": None,
        # Simulation-time experiment control: records which clock was used for
        # this run. "wall" (default) means
        # duration_sec/t_fail_sec above are host wall-clock seconds. "simulation" means
        # they're simulated seconds
        # (tracked via /clock - see experiment_manager/sim_clock.py) - required for a
        # meaningful throughput/recovery-time comparison on hosts with a realtime
        # factor far from 1.0 (this project's WSL2 host measures ~0.07-0.1x). Populated
        # by finalize_duration() once the run actually ends, since the realtime factor
        # can only be computed after both wall and simulated elapsed time are known.
        "time_source": time_source,
        "simulation_duration_sec": None,
        "realtime_factor": None,
    }
    (run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2))
    (run_dir / "seed.txt").write_text(str(seed))


def finalize_duration(
    run_dir: Path,
    duration_sec: float,
    *,
    simulation_duration_sec: float | None = None,
) -> None:
    meta_path = run_dir / "metadata.json"
    metadata = json.loads(meta_path.read_text())
    metadata["duration_sec"] = duration_sec
    if simulation_duration_sec is not None:
        metadata["simulation_duration_sec"] = simulation_duration_sec
        # realtime_factor = simulated seconds elapsed per wall-clock second - e.g. 0.1
        # means the simulation ran at 1/10th real speed, matching how this project's
        # docs have described its own measured WSL2 slowdown throughout.
        metadata["realtime_factor"] = (
            round(simulation_duration_sec / duration_sec, 4) if duration_sec else None
        )
    meta_path.write_text(json.dumps(metadata, indent=2))
