"""Post-run summary + plot generation.

Reads a completed run's data files (`events.jsonl`, `robot_<ns>.csv`,
`health_<ns>.csv`, `negotiations.csv`, `faults.csv`, `metadata.json` - all written by
`event_logger_node.py` during the run) and produces:

  <run_dir>/summary.json   Productivity/Resilience/Safety/Energy metrics,
                            computed only from what this repo actually instruments -
                            no fabricated metrics for anything unmeasured. "Collisions"
                            has no contact-based sensor in this simulation, so
                            `safety` reports nav2_collision_monitor's own real
                            STOP/SLOWDOWN/APPROACH/LIMIT interventions instead, honestly
                            named `safety_interventions` rather than claimed as literal
                            collisions. `resilience` (recovery time) is only populated
                            for runs with `t_fail_sec` set (Mode B) - meaningless for
                            decentralized modes, which have no central point of failure.
  <run_dir>/plots/*.png     a set of summary figures, reproducible
                            from raw data alone (matplotlib, `Agg` backend - headless,
                            no display needed).

Runs automatically at the end of every `run_experiment.py` invocation (at
termination: save all logs, calculate summary, generate plots, print experiment
directory), and is also directly runnable to re-summarize a past run:

    ros2 run experiment_manager summarize_run <run_dir>
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple


def _read_csv_rows(path: Path) -> List[dict]:
    if not path.exists():
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def _read_events(run_dir: Path) -> List[dict]:
    path = run_dir / "events.jsonl"
    if not path.exists():
        return []
    events = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                events.append(json.loads(line))
    return events


def _read_jsonl(path: Path) -> List[dict]:
    if not path.exists():
        return []
    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def _robot_namespaces(metadata: dict) -> List[str]:
    n = metadata.get("robot_count", 0) or 0
    return [f"robot{i}" for i in range(1, n + 1)]


def _completed_task_cycles(
    events: List[dict], clock_field: str = "timestamp"
) -> List[Tuple[float, float]]:
    """Pairs each EXECUTING -> SUCCEEDED nav-goal transition, across every robot, into
    (assigned_ts, completed_ts) on the given clock (`timestamp` = wall-clock, always
    present; `simulation_time` = simulated seconds, only present on --time-source
    simulation runs). Separate from _completed_task_durations_sec (per-robot durations
    only, always wall-clock) because resilience/pre-post-failure throughput need to
    know WHEN each task was assigned relative to t_fail_sec, not just how long it took.
    A pair is dropped (not appended) if either endpoint's clock value is missing
    (e.g. simulation_time is None before this node's own clock synced its first
    /clock message) - never fabricate a cycle from a partial timestamp."""
    cycles: List[Tuple[float, float]] = []
    executing_since: Dict[str, Optional[float]] = {}
    for e in events:
        if e.get("event") != "NAV_GOAL_STATUS_CHANGED":
            continue
        robot_id = e.get("robot_id")
        status = e.get("status")
        ts = e.get(clock_field)
        if status == "EXECUTING":
            executing_since[robot_id] = ts
        elif status == "SUCCEEDED" and robot_id in executing_since:
            assigned_ts = executing_since.pop(robot_id)
            if assigned_ts is not None and ts is not None:
                cycles.append((assigned_ts, ts))
        elif status in ("ABORTED", "CANCELED"):
            executing_since.pop(robot_id, None)
    return cycles


def _completed_task_durations_sec(
    events: List[dict], robot_id: str, clock_field: str = "timestamp"
) -> List[float]:
    """Pairs each EXECUTING -> SUCCEEDED nav-goal status transition for one robot into
    a duration, feeding `mean/median/p95_task_completion_time`, from the only
    per-goal timing this repo currently logs (`NAV_GOAL_STATUS_CHANGED` events).
    Deliberately SUCCEEDED-only: an ABORTED attempt didn't complete a task, and folding
    its (shorter, failure-truncated) duration into a "completion time" average would
    silently understate it - a real distinction, not a rounding choice.

    `clock_field` defaults to wall-clock `timestamp` for backward compatibility, but
    `build_summary` passes `simulation_time` on `--time-source simulation` runs - a
    real bug was found and fixed here, where every sim-time run's task-duration
    figures were silently measured in wall-clock seconds (~10-14x too large at this
    host's ~0.07-0.1x realtime factor) instead of the simulated seconds every other
    sim-time metric in this file already uses."""
    durations = []
    executing_since: Optional[float] = None
    for e in events:
        if e.get("event") != "NAV_GOAL_STATUS_CHANGED" or e.get("robot_id") != robot_id:
            continue
        status = e.get("status")
        ts = e.get(clock_field)
        if status == "EXECUTING":
            executing_since = ts
        elif status == "SUCCEEDED" and executing_since is not None and ts is not None:
            durations.append(ts - executing_since)
            executing_since = None
        elif status in ("ABORTED", "CANCELED"):
            executing_since = None
    return durations


def _distance_traveled_m(rows: List[dict]) -> float:
    total = 0.0
    prev = None
    for row in rows:
        x, y = float(row["x"]), float(row["y"])
        if prev is not None:
            total += ((x - prev[0]) ** 2 + (y - prev[1]) ** 2) ** 0.5
        prev = (x, y)
    return total


def _percentile(values: List[float], p: float) -> Optional[float]:
    if not values:
        return None
    s = sorted(values)
    k = (len(s) - 1) * p
    f, c = int(k), min(int(k) + 1, len(s) - 1)
    if f == c:
        return s[f]
    return s[f] + (s[c] - s[f]) * (k - f)


def build_summary(run_dir: Path) -> dict:
    metadata = json.loads((run_dir / "metadata.json").read_text())
    events = _read_events(run_dir)
    robots = _robot_namespaces(metadata)
    duration_sec = metadata.get("duration_sec") or 0.0
    # Same clock-basis rule as the resilience block below: on --time-source simulation
    # runs, every duration this function computes must use simulated seconds, not
    # wall-clock ones - see _completed_task_durations_sec's docstring for the bug this
    # fixes.
    duration_clock_field = (
        "simulation_time" if metadata.get("time_source", "wall") == "simulation" else "timestamp"
    )

    per_robot: Dict[str, dict] = {}
    all_durations: List[float] = []
    total_tasks = 0
    for ns in robots:
        succeeded = sum(
            1
            for e in events
            if e.get("event") == "NAV_GOAL_STATUS_CHANGED"
            and e.get("robot_id") == ns
            and e.get("status") == "SUCCEEDED"
        )
        durations = _completed_task_durations_sec(events, ns, duration_clock_field)
        all_durations.extend(durations)
        total_tasks += succeeded

        odom_rows = _read_csv_rows(run_dir / f"robot_{ns}.csv")
        distance_m = _distance_traveled_m(odom_rows)

        health_rows = _read_csv_rows(run_dir / f"health_{ns}.csv")
        min_battery_soc = (
            min(float(r["battery_soc"]) for r in health_rows) if health_rows else None
        )
        # health_<ns>.csv rows are written in arrival order (event_logger_node.py's
        # buffering=1 append-only writer), so first/last rows are chronological start/end.
        battery_soc_start = float(health_rows[0]["battery_soc"]) if health_rows else None
        battery_soc_end = float(health_rows[-1]["battery_soc"]) if health_rows else None
        energy_per_task = (
            round((battery_soc_start - battery_soc_end) / succeeded, 4)
            if health_rows and succeeded > 0
            else None
        )

        per_robot[ns] = {
            "tasks_completed": succeeded,
            "distance_m": round(distance_m, 3),
            "min_battery_soc": min_battery_soc,
            "battery_soc_start": battery_soc_start,
            "battery_soc_end": battery_soc_end,
            "energy_per_task": energy_per_task,
        }

    negotiations = _read_csv_rows(run_dir / "negotiations.csv")
    negotiation_summary = None
    if negotiations:
        negotiation_summary = {
            "conversations": len({r["conversation_id"] for r in negotiations}),
            "offers": sum(1 for r in negotiations if r["performative"] == "OFFER"),
            "commits": sum(1 for r in negotiations if r["performative"] == "COMMIT"),
            "timeouts": sum(1 for r in negotiations if r["performative"] == "TIMEOUT"),
        }

    faults = _read_csv_rows(run_dir / "faults.csv")
    fault_summary = None
    if faults:
        fault_summary = {
            "faults_injected": sum(1 for r in faults if r["clear"].lower() == "false")
        }

    # Battery-degradation / workload redistribution summary. Only populated
    # when a degradation-class fault
    # (ACCELERATED_BATTERY_DISCHARGE or BATTERY_CAPACITY_DEGRADATION - the two
    # fault types that actually reduce a robot's usable battery over time, per
    # robot_health/faults.py) was injected this run. Compares each robot's share
    # of total completed tasks before vs. after the fault's onset time, on the
    # same clock basis as the resilience block above - "did coordination actually
    # shift work away from the degraded robot," not merely "was the fault applied."
    degradation_summary = None
    degradation_fault_types = {"ACCELERATED_BATTERY_DISCHARGE", "BATTERY_CAPACITY_DEGRADATION"}
    degradation_events = [
        r for r in faults
        if r["fault_type"] in degradation_fault_types and r["clear"].lower() == "false"
    ]
    if degradation_events:
        time_source = metadata.get("time_source", "wall")
        onset_field = "simulation_time" if time_source == "simulation" else "timestamp"
        clock_field = onset_field
        onset = min(float(r[onset_field]) for r in degradation_events if r.get(onset_field))
        degraded_robots = sorted({r["robot_id"] for r in degradation_events})
        cycles_by_robot: Dict[str, List[Tuple[float, float]]] = {ns: [] for ns in robots}
        for ns in robots:
            ns_events = [e for e in events if e.get("robot_id") == ns]
            for assigned_ts, completed_ts in _completed_task_cycles(ns_events, clock_field):
                cycles_by_robot[ns].append((assigned_ts, completed_ts))
        per_robot_split = {}
        pre_total = 0
        post_total = 0
        for ns in robots:
            pre = sum(1 for _, c in cycles_by_robot[ns] if c <= onset)
            post = sum(1 for _, c in cycles_by_robot[ns] if c > onset)
            pre_total += pre
            post_total += post
            per_robot_split[ns] = {"tasks_completed_pre_onset": pre, "tasks_completed_post_onset": post}
        for ns in robots:
            pre = per_robot_split[ns]["tasks_completed_pre_onset"]
            post = per_robot_split[ns]["tasks_completed_post_onset"]
            per_robot_split[ns]["workload_share_pre_onset"] = (
                round(pre / pre_total, 4) if pre_total else None
            )
            per_robot_split[ns]["workload_share_post_onset"] = (
                round(post / post_total, 4) if post_total else None
            )
        degradation_summary = {
            "degraded_robots": degraded_robots,
            "onset_sec": round(onset, 3),
            "clock_basis": onset_field,
            "per_robot": per_robot_split,
        }

    decisions = _read_jsonl(run_dir / "agent_decisions.jsonl")
    decision_summary = None
    if decisions:
        latencies = [float(d["decision_latency_sec"]) for d in decisions]
        decision_summary = {
            "decisions": len(decisions),
            "backends": sorted({d["backend"] for d in decisions}),
            "mean_decision_latency_sec": round(sum(latencies) / len(latencies), 6),
            "median_decision_latency_sec": round(_percentile(latencies, 0.5), 6),
            "p95_decision_latency_sec": round(_percentile(latencies, 0.95), 6),
        }

        # LLM observability summary. Only over
        # decisions that actually involved the LLM (has_llm_meta - i.e. backend=="llm",
        # or "hybrid" decisions that escalated) - a "rule"-only run correctly gets no
        # llm block at all (None, not zeros standing in for "not applicable").
        llm_decisions = [d for d in decisions if d.get("has_llm_meta")]
        if llm_decisions:
            llm_latencies = [float(d["decision_latency_sec"]) for d in llm_decisions]
            prompt_tokens = [d["prompt_tokens"] for d in llm_decisions if d.get("prompt_tokens") is not None]
            completion_tokens = [
                d["completion_tokens"] for d in llm_decisions if d.get("completion_tokens") is not None
            ]
            decision_summary["llm"] = {
                "llm_requests": len(llm_decisions),
                # "success" = a real, validated, non-fallback LLM answer was used.
                "llm_successes": sum(
                    1 for d in llm_decisions
                    if d.get("schema_valid") and d.get("safety_valid") and not d.get("fallback_used")
                ),
                "llm_schema_failures": sum(1 for d in llm_decisions if d.get("schema_valid") is False),
                "llm_safety_rejections": sum(
                    1 for d in llm_decisions if d.get("schema_valid") and d.get("safety_valid") is False
                ),
                "llm_fallbacks": sum(1 for d in llm_decisions if d.get("fallback_used")),
                # Sum of retry_count across every LLM-involved decision - each
                # decide() call's OWN retry count (how many prior attempts it took
                # within that single decision), not a cross-decision total.
                "llm_retries": sum(d.get("retry_count") or 0 for d in llm_decisions),
                "mean_llm_latency_ms": round(sum(llm_latencies) / len(llm_latencies) * 1000.0, 3),
                "median_llm_latency_ms": round(_percentile(llm_latencies, 0.5) * 1000.0, 3),
                "p95_llm_latency_ms": round(_percentile(llm_latencies, 0.95) * 1000.0, 3),
                # None (not 0) when the provider never reported token counts this run
                # (section 24: never estimate) - distinguishes "measured zero" from
                # "not reported at all."
                "total_prompt_tokens": sum(prompt_tokens) if prompt_tokens else None,
                "total_completion_tokens": sum(completion_tokens) if completion_tokens else None,
            }

        # Hybrid-specific: how often the deterministic auction alone was sufficient
        # vs. needed the LLM - the "cost of intelligence" tradeoff.
        if "hybrid" in decision_summary["backends"]:
            hybrid_decisions = [d for d in decisions if d.get("backend") == "hybrid"]
            deterministic_count = sum(1 for d in hybrid_decisions if not d.get("has_llm_meta"))
            escalation_count = sum(1 for d in hybrid_decisions if d.get("has_llm_meta"))
            total = deterministic_count + escalation_count
            decision_summary["hybrid"] = {
                "deterministic_decisions": deterministic_count,
                "llm_escalations": escalation_count,
                "llm_escalation_rate": round(escalation_count / total, 4) if total else None,
            }

    # Resilience / Recovery Time / pre-post-failure throughput (section 13, section 14 -
    # Mode B / centralized_then_failover analysis only - meaningless for pure
    # decentralized modes, which have no single central point of failure to recover
    # from). None entirely means "no fault was injected this run" (not applicable).
    #
    # Clock basis: --time-source simulation runs must compare t_fail_sec (itself in
    # SIMULATED seconds under that mode - see run_experiment.py) against
    # simulation_time, not the wall-clock timestamp - a real bug (unit mismatch, same
    # class as the heartbeat-timeout bug found and fixed in the failover
    # implementation's live validation) that would otherwise measure "how many wall-clock seconds after a
    # simulated fault time" as if they were comparable.
    #
    # Recovery definition (docs/notes.md): a task counts as "recovery"
    # only if it was BOTH assigned (EXECUTING) AND completed (SUCCEEDED) after
    # t_fail_sec - not merely completed after it. A task assigned before the fault that
    # simply finishes late (its goal was already in flight; Nav2 doesn't need the
    # central node to finish an in-progress goal) is NOT recovery, it's the tail of
    # pre-existing work. This was a real, live-confirmed false positive (the same
    # class of mistake also surfaced in the failover heartbeat-timeout logic; see
    # docs/notes.md) - fixed here by requiring BOTH endpoints of the
    # EXECUTING->SUCCEEDED pair to fall after the fault, via _completed_task_cycles.
    resilience_summary = None
    t_fail_sec = metadata.get("t_fail_sec")
    time_source = metadata.get("time_source", "wall")
    clock_field = "simulation_time" if time_source == "simulation" else "timestamp"
    if t_fail_sec is not None:
        start_ts = next(
            (e[clock_field] for e in events if e.get("event") == "EXPERIMENT_LOGGER_STARTED"),
            None,
        )
        # On --time-source simulation runs, EXPERIMENT_LOGGER_STARTED's own
        # simulation_time is frequently null (its /clock subscription hasn't received
        # a first message yet at that instant - already documented as "harmless" in
        # the simulation-time control validation). It's NOT harmless here: `next(...)`
        # finds the event and returns its None value, which is not the same as "no
        # such event" - a real bug found live (this exact run's resilience block
        # silently stayed None despite a confirmed CENTRAL_MANAGER_KILLED/recovery in
        # events.jsonl). simulation_time is zero-based from experiment start by
        # construction (every observed CENTRAL_MANAGER_KILLED.simulation_time has
        # landed within ~0.1s of t_fail_sec with no offset needed), so 0.0 is the correct
        # fallback - never a guess, just the definition of this clock.
        if start_ts is None and clock_field == "simulation_time":
            start_ts = 0.0
        if start_ts is not None:
            fail_absolute_ts = start_ts + t_fail_sec
            cycles = _completed_task_cycles(events, clock_field)
            post_fail_recoveries = sorted(
                completed_ts
                for assigned_ts, completed_ts in cycles
                if assigned_ts > fail_absolute_ts
            )
            resilience_summary = {
                "t_fail_sec": t_fail_sec,
                "recovered": bool(post_fail_recoveries),
                "recovery_time_sec": (
                    round(post_fail_recoveries[0] - fail_absolute_ts, 2)
                    if post_fail_recoveries
                    else None
                ),
            }

            # Pre/post-failure throughput split
            # (pre_failure_throughput/post_failure_throughput/throughput_retention_ratio).
            # Window durations come from the same clock basis as everything above -
            # simulation_duration_sec when time_source=simulation, else duration_sec.
            total_duration = (
                metadata.get("simulation_duration_sec")
                if time_source == "simulation"
                else metadata.get("duration_sec")
            )
            if total_duration and total_duration > t_fail_sec:
                pre_count = sum(1 for _, c in cycles if c <= fail_absolute_ts)
                post_count = sum(1 for _, c in cycles if c > fail_absolute_ts)
                pre_window_hr = t_fail_sec / 3600.0
                post_window_hr = (total_duration - t_fail_sec) / 3600.0
                pre_tph = round(pre_count / pre_window_hr, 2) if pre_window_hr > 0 else None
                post_tph = round(post_count / post_window_hr, 2) if post_window_hr > 0 else None
                resilience_summary["pre_failure_tasks_completed"] = pre_count
                resilience_summary["post_failure_tasks_completed"] = post_count
                resilience_summary["pre_failure_throughput_per_hour"] = pre_tph
                resilience_summary["post_failure_throughput_per_hour"] = post_tph
                # Only meaningful when there was real pre-failure activity to compare
                # against - a zero (or missing) pre-failure throughput makes the ratio
                # undefined, not zero or infinite.
                resilience_summary["throughput_retention_ratio"] = (
                    round(post_tph / pre_tph, 4) if pre_tph else None
                )

    # Safety interventions (collisions/near_collisions/emergency_stops).
    # This simulation has no contact-based collision detection, so - rather than
    # fabricate a "collisions" number - this reports real, already-computed data from
    # Nav2's own collision_monitor safety layer: every time it actually intervened
    # (STOP/SLOWDOWN/APPROACH/LIMIT) because something entered a robot's safety polygon.
    # Named "safety_interventions", not "collisions", because that's honestly what it is.
    safety_events = _read_csv_rows(run_dir / "safety_events.csv")
    safety_summary = None
    if safety_events:
        by_action: Dict[str, int] = {}
        for row in safety_events:
            by_action[row["action_type"]] = by_action.get(row["action_type"], 0) + 1
        safety_summary = {
            "total_interventions": len(safety_events),
            "by_action_type": by_action,
        }

    return {
        "experiment_mode": metadata.get("experiment_mode"),
        "coordination": metadata.get("coordination"),
        "agent_backend": metadata.get("agent_backend"),
        "robot_count": metadata.get("robot_count"),
        "random_seed": metadata.get("random_seed"),
        "duration_sec": duration_sec,
        "productivity": {
            "tasks_completed": total_tasks,
            "fleet_throughput_per_hour": (
                round(total_tasks / duration_sec * 3600.0, 2) if duration_sec else None
            ),
            "mean_task_duration_sec": (
                round(sum(all_durations) / len(all_durations), 2) if all_durations else None
            ),
            "median_task_duration_sec": (
                round(_percentile(all_durations, 0.5), 2) if all_durations else None
            ),
            "p95_task_duration_sec": (
                round(_percentile(all_durations, 0.95), 2) if all_durations else None
            ),
        },
        "per_robot": per_robot,
        "negotiation": negotiation_summary,
        "faults": fault_summary,
        "agent_decisions": decision_summary,
        "resilience": resilience_summary,
        "safety": safety_summary,
        "degradation": degradation_summary,
    }


def write_summary(run_dir: Path) -> Path:
    summary = build_summary(run_dir)
    out_path = run_dir / "summary.json"
    out_path.write_text(json.dumps(summary, indent=2))
    return out_path


def generate_plots(run_dir: Path) -> List[Path]:
    """Only generates a figure when the underlying data actually exists this run
    (e.g. no battery_soc plot for a --no-health run) - an empty/misleading plot is
    worse than no plot."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    metadata = json.loads((run_dir / "metadata.json").read_text())
    robots = _robot_namespaces(metadata)
    plots_dir = run_dir / "plots"
    plots_dir.mkdir(exist_ok=True)
    generated: List[Path] = []

    fig, ax = plt.subplots()
    any_traj = False
    for ns in robots:
        rows = _read_csv_rows(run_dir / f"robot_{ns}.csv")
        if not rows:
            continue
        any_traj = True
        xs = [float(r["x"]) for r in rows]
        ys = [float(r["y"]) for r in rows]
        ax.plot(xs, ys, label=ns, marker=".", markersize=2, linewidth=0.8)
    if any_traj:
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title("Robot trajectories")
        ax.legend()
        ax.set_aspect("equal", adjustable="datalim")
        path = plots_dir / "trajectories.png"
        fig.savefig(path, dpi=120)
        generated.append(path)
    plt.close(fig)

    fig, ax = plt.subplots()
    any_health = False
    t0 = None
    for ns in robots:
        rows = _read_csv_rows(run_dir / f"health_{ns}.csv")
        if not rows:
            # File-exists-but-empty happens whenever a run ends before
            # health_monitor's ~1Hz sample period produces a single row (e.g. a short
            # run that spends most of its wall time on Nav2/SLAM bringup) - checking
            # file existence alone (an earlier version of this function did) would
            # still draw an empty, misleading figure. Actual row data is what matters.
            continue
        any_health = True
        ts = [float(r["timestamp"]) for r in rows]
        if t0 is None:
            t0 = ts[0]
        soc = [float(r["battery_soc"]) for r in rows]
        ax.plot([t - t0 for t in ts], soc, label=ns)
    if any_health:
        ax.set_xlabel("time (s)")
        ax.set_ylabel("battery_soc")
        ax.set_title("Battery state of charge vs time")
        ax.legend()
        path = plots_dir / "battery_soc.png"
        fig.savefig(path, dpi=120)
        generated.append(path)
    plt.close(fig)

    events = _read_events(run_dir)
    succeeded_ts = sorted(
        e["timestamp"]
        for e in events
        if e.get("event") == "NAV_GOAL_STATUS_CHANGED" and e.get("status") == "SUCCEEDED"
    )
    if succeeded_ts:
        start_ts = next(
            (e["timestamp"] for e in events if e.get("event") == "EXPERIMENT_LOGGER_STARTED"),
            succeeded_ts[0],
        )
        xs = [0.0] + [t - start_ts for t in succeeded_ts]
        ys = list(range(len(succeeded_ts) + 1))
        fig, ax = plt.subplots()
        ax.step(xs, ys, where="post")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("cumulative tasks completed")
        ax.set_title("Fleet throughput over time")
        path = plots_dir / "throughput.png"
        fig.savefig(path, dpi=120)
        plt.close(fig)
        generated.append(path)

    return generated


def main() -> None:
    if len(sys.argv) != 2:
        print("usage: summarize_run <run_dir>", file=sys.stderr)
        sys.exit(1)
    run_dir = Path(sys.argv[1])
    summary_path = write_summary(run_dir)
    plot_paths = generate_plots(run_dir)
    print(f"wrote {summary_path}")
    for p in plot_paths:
        print(f"wrote {p}")


if __name__ == "__main__":
    main()
