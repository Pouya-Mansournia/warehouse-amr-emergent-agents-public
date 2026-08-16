#!/usr/bin/env python3
"""Research-analysis comparison table generator: the final report-generation stage of
the experiment pipeline.

    python3 analysis/scripts/generate_report.py <batch_dir>

Reads a batch's `batch_manifest.json` (produced by `run_batch.py`), reads each
successful run's `summary.json` (produced by `summarize_run.py`, re-generating it first
if a run somehow doesn't have one yet), groups runs into "Mode" rows by
(coordination, agent_backend) (Centralized / Decentralized / LLM / Hybrid),
aggregates the numeric metrics across every seed within
a mode (mean, and standard deviation where more than one sample exists), and renders a
Markdown table.

The comparison table has five columns: Throughput, Recovery Time,
Energy/Task, Collisions, Decision Latency. All five are now populated from real,
measured data where it exists, and rendered as `not measured` (never fabricated)
where it genuinely doesn't:

  - Throughput, Decision Latency: as before (Phase 10/12).
  - Recovery Time: only populated for runs with `t_fail_sec` set (Mode B) -
    `summarize_run.py`'s `resilience` block measures time from the fault to the next
    successful task completion, or reports an honest "no recovery" if none ever came.
    Meaningless (and left `not measured`) for decentralized modes, which have no
    single central point of failure to recover from in the first place.
  - Energy/Task: `battery_soc` drop per robot divided by that robot's completed task
    count, only when both a health sample and at least one completed task exist.
  - "Collisions (safety events)": this simulation has no contact-based collision
    sensor, so rather than invent a number, this column reports real data from Nav2's
    own `collision_monitor` safety layer - every STOP/SLOWDOWN/APPROACH/LIMIT
    intervention it actually triggered. The column header says "safety events", not
    bare "Collisions", so the table itself is honest about what it measures.
"""
from __future__ import annotations

import argparse
import csv
import json
import random
import statistics
import sys
from pathlib import Path
from typing import Callable, Dict, List, Optional

REPO_ROOT = Path(__file__).resolve().parents[2]

NOT_MEASURED = "not measured"


def _mode_label(coordination: str, agent_backend: Optional[str]) -> str:
    if coordination == "centralized":
        return "Centralized"
    backend = (agent_backend or "rule").lower()
    if backend == "rule":
        return "Decentralized"
    if backend == "llm":
        return "LLM"
    if backend == "hybrid":
        return "Hybrid"
    return f"Decentralized ({backend})"


def _resilience_mode_label(coordination: str, agent_backend: Optional[str]) -> Optional[str]:
    """Labels for the Mode B/C/D/E resilience-comparison table - distinct from
    `_mode_label`'s productivity-table labels, since Mode C/D/E only means
    "recovery strategy after central failure" under `--coordination
    centralized_then_failover`, not plain `decentralized` (which never
    had a central coordinator to fail over from in the first place, so it doesn't
    belong in this table at all - callers filter it out via the None return here).
    """
    if coordination == "centralized":
        return "No Recovery"
    if coordination == "centralized_then_failover":
        backend = (agent_backend or "rule").lower()
        return {"rule": "Deterministic", "llm": "LLM", "hybrid": "Hybrid"}.get(
            backend, f"Failover ({backend})"
        )
    return None


def _load_summaries(manifest: dict) -> List[dict]:
    summaries = []
    for run in manifest["runs"]:
        if not run["success"] or not run["run_dir"]:
            continue
        run_dir = Path(run["run_dir"])
        summary_path = run_dir / "summary.json"
        if not summary_path.exists():
            # Should already exist (run_experiment.py writes it at termination) - only
            # a fallback for summarizing an older/partial run. Imported lazily (not at
            # module level) so this script's own pure logic (build_comparison_table,
            # render_markdown, _mode_label) stays unit-testable without a sourced ROS
            # environment - only this fallback path needs experiment_manager on the path.
            from experiment_manager.summarize_run import write_summary

            write_summary(run_dir)
        summaries.append(json.loads(summary_path.read_text()))
    return summaries


def _percentile(values: List[float], p: float) -> float:
    s = sorted(values)
    k = (len(s) - 1) * p
    f, c = int(k), min(int(k) + 1, len(s) - 1)
    if f == c:
        return s[f]
    return s[f] + (s[c] - s[f]) * (k - f)


def compute_metric_statistics(
    values: List[float], *, n_bootstrap: int = 2000, seed: int = 0
) -> Optional[dict]:
    """Minimum statistical output for one metric's values
    across seeds within a mode: N, mean, median, standard deviation, min, max, p95,
    and a bootstrap 95% confidence interval on the mean (percentile bootstrap - makes
    no assumption of normality, so significance tests are not applied automatically
    without checking assumptions; bootstrap confidence intervals
    are used instead). Returns None for an empty list - a metric no run measured gets
    no row, never a fabricated zero. Deterministic given the same input (a seeded
    local Random instance, not the shared global `random` module state), so the
    report itself is reproducible run to run."""
    if not values:
        return None
    n = len(values)
    result = {
        "n": n,
        "mean": round(statistics.mean(values), 4),
        "median": round(statistics.median(values), 4),
        "stdev": round(statistics.stdev(values), 4) if n > 1 else None,
        "min": round(min(values), 4),
        "max": round(max(values), 4),
        "p95": round(_percentile(values, 0.95), 4),
        "ci95_low": None,
        "ci95_high": None,
    }
    if n > 1:
        rng = random.Random(seed)
        boot_means = sorted(
            statistics.mean(values[rng.randrange(n)] for _ in range(n))
            for _ in range(n_bootstrap)
        )
        lo_idx = int(0.025 * n_bootstrap)
        hi_idx = max(int(0.975 * n_bootstrap) - 1, 0)
        result["ci95_low"] = round(boot_means[lo_idx], 4)
        result["ci95_high"] = round(boot_means[hi_idx], 4)
    return result


def compute_paired_differences(
    summaries_a: List[dict], summaries_b: List[dict], metric_fn: Callable[[dict], Optional[float]]
) -> Optional[dict]:
    """Where paired seeds exist, calculates paired differences - pairs
    runs from two mode-groups by matching `random_seed`, computes per-pair
    differences (b - a) for whatever metric_fn extracts from a summary.json dict, and
    returns the same compute_metric_statistics() shape over those differences. None
    if the two groups share no seeds (nothing to pair) or metric_fn returns None for
    every shared seed (metric wasn't measured in one side)."""
    by_seed_a = {s.get("random_seed"): s for s in summaries_a}
    by_seed_b = {s.get("random_seed"): s for s in summaries_b}
    shared_seeds = sorted(set(by_seed_a) & set(by_seed_b), key=str)
    diffs = []
    for seed in shared_seeds:
        va = metric_fn(by_seed_a[seed])
        vb = metric_fn(by_seed_b[seed])
        if va is not None and vb is not None:
            diffs.append(vb - va)
    if not diffs:
        return None
    return compute_metric_statistics(diffs)


def _throughput(s: dict) -> Optional[float]:
    return s.get("productivity", {}).get("fleet_throughput_per_hour")


def _fmt_mean_std(values: List[float], unit: str = "", digits: int = 2) -> str:
    if not values:
        return NOT_MEASURED
    mean = round(statistics.mean(values), digits)
    if len(values) > 1:
        std = round(statistics.stdev(values), digits)
        return f"{mean}{unit} (±{std})"
    return f"{mean}{unit}"


def build_comparison_table(summaries: List[dict]) -> List[Dict[str, str]]:
    by_mode: Dict[str, List[dict]] = {}
    for s in summaries:
        label = _mode_label(s.get("coordination", "centralized"), s.get("agent_backend"))
        by_mode.setdefault(label, []).append(s)

    rows = []
    for label, mode_summaries in by_mode.items():
        throughputs = [
            s["productivity"]["fleet_throughput_per_hour"]
            for s in mode_summaries
            if s["productivity"]["fleet_throughput_per_hour"] is not None
        ]
        latencies_ms = [
            s["agent_decisions"]["mean_decision_latency_sec"] * 1000.0
            for s in mode_summaries
            if s.get("agent_decisions")
        ]

        # Recovery Time: only meaningful for runs where a fault was actually injected
        # (Mode B, t_fail_sec set) - absent entirely (not "0") for modes/runs where no
        # central failure was ever induced, e.g. every decentralized run, or a
        # centralized run with no --t-fail.
        fault_runs = [s for s in mode_summaries if s.get("resilience")]
        recovery_time_str = NOT_MEASURED
        retention_str = NOT_MEASURED
        if fault_runs:
            recovered = [s["resilience"] for s in fault_runs if s["resilience"]["recovered"]]
            if recovered:
                times = [r["recovery_time_sec"] for r in recovered]
                recovery_time_str = (
                    f"{_fmt_mean_std(times, unit='s')} "
                    f"({len(recovered)}/{len(fault_runs)} recovered)"
                )
            else:
                # A real, honest, important result - not a missing value: the fault
                # WAS injected and NOTHING ever recovered within the run, matching
                # the documented finding for Mode B.
                recovery_time_str = f"no recovery (0/{len(fault_runs)})"

            # throughput_retention_ratio -
            # only meaningful for runs where summarize_run.py could compute a real
            # pre-failure throughput to divide by (see that module's docstring: a zero
            # or missing pre-failure throughput makes the ratio undefined, not 0/inf).
            retentions = [
                s["resilience"]["throughput_retention_ratio"]
                for s in fault_runs
                if s["resilience"].get("throughput_retention_ratio") is not None
            ]
            if retentions:
                retention_str = _fmt_mean_std(retentions, digits=3)

        energy_values = [
            r["energy_per_task"] * 100.0
            for s in mode_summaries
            for r in s.get("per_robot", {}).values()
            if r.get("energy_per_task") is not None
        ]
        energy_str = _fmt_mean_std(energy_values, unit="%SOC/task", digits=2) if energy_values else NOT_MEASURED

        safety_runs = [s["safety"] for s in mode_summaries if s.get("safety")]
        safety_str = (
            _fmt_mean_std([s["total_interventions"] for s in safety_runs], unit=" events")
            if safety_runs
            else NOT_MEASURED
        )

        rows.append(
            {
                "Mode": label,
                "Runs": str(len(mode_summaries)),
                "Throughput (tasks/hr)": _fmt_mean_std(throughputs),
                "Recovery Time": recovery_time_str,
                "Throughput Retention": retention_str,
                "Energy/Task": energy_str,
                "Collisions (safety events)": safety_str,
                "Decision Latency (ms)": _fmt_mean_std(latencies_ms, digits=3),
            }
        )
    return rows


RESILIENCE_COLUMNS = [
    "Mode",
    "Runs",
    "Recovery",
    "Recovery Time",
    "Throughput Retention",
    "Energy/Task",
    "Safety",
    "Decision Latency",
]


def build_resilience_table(summaries: List[dict]) -> List[Dict[str, str]]:
    """Builds the B/C/D/E central-failure comparison
    (`analysis/results/resilience_summary.csv`) - distinct from
    `build_comparison_table`'s general productivity table. Only includes runs that
    actually had a fault injected (`resilience` block present, i.e. `t_fail_sec` was
    set) and only under `coordination in {centralized, centralized_then_failover}` -
    a plain decentralized run has no central point of failure to recover from, so it
    is silently excluded here rather than given a misleading row. Only measured
    values are ever reported; a mode with zero fault-injected runs in this batch
    simply gets no row at all, never a fabricated one.
    """
    fault_summaries = [s for s in summaries if s.get("resilience")]

    by_mode: Dict[str, List[dict]] = {}
    for s in fault_summaries:
        label = _resilience_mode_label(s.get("coordination", "centralized"), s.get("agent_backend"))
        if label is None:
            continue
        by_mode.setdefault(label, []).append(s)

    # Fixed row order for the resilience table (No Recovery, Deterministic,
    # LLM, Hybrid) when present, any other observed label appended after.
    order = ["No Recovery", "Deterministic", "LLM", "Hybrid"]
    labels = [l for l in order if l in by_mode] + sorted(l for l in by_mode if l not in order)

    rows = []
    for label in labels:
        mode_summaries = by_mode[label]
        resiliences = [s["resilience"] for s in mode_summaries]
        recovered = [r for r in resiliences if r["recovered"]]
        recovery_str = f"{len(recovered)}/{len(resiliences)}"

        if recovered:
            times = [r["recovery_time_sec"] for r in recovered if r["recovery_time_sec"] is not None]
            recovery_time_str = _fmt_mean_std(times, unit="s") if times else NOT_MEASURED
        else:
            recovery_time_str = "no recovery"

        retentions = [
            r["throughput_retention_ratio"]
            for r in resiliences
            if r.get("throughput_retention_ratio") is not None
        ]
        retention_str = _fmt_mean_std(retentions, digits=3) if retentions else NOT_MEASURED

        energy_values = [
            r["energy_per_task"] * 100.0
            for s in mode_summaries
            for r in s.get("per_robot", {}).values()
            if r.get("energy_per_task") is not None
        ]
        energy_str = _fmt_mean_std(energy_values, unit="%SOC/task", digits=2) if energy_values else NOT_MEASURED

        safety_runs = [s["safety"] for s in mode_summaries if s.get("safety")]
        safety_str = (
            _fmt_mean_std([s["total_interventions"] for s in safety_runs], unit=" events")
            if safety_runs
            else NOT_MEASURED
        )

        latencies_ms = [
            s["agent_decisions"]["mean_decision_latency_sec"] * 1000.0
            for s in mode_summaries
            if s.get("agent_decisions")
        ]
        latency_str = _fmt_mean_std(latencies_ms, digits=3) if latencies_ms else NOT_MEASURED

        rows.append(
            {
                "Mode": label,
                "Runs": str(len(mode_summaries)),
                "Recovery": recovery_str,
                "Recovery Time": recovery_time_str,
                "Throughput Retention": retention_str,
                "Energy/Task": energy_str,
                "Safety": safety_str,
                "Decision Latency": latency_str,
            }
        )
    return rows


def write_resilience_csv(rows: List[Dict[str, str]], out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    # Explicit UTF-8: this file's values contain "±" (from _fmt_mean_std), and this
    # project runs across both WSL2 (default UTF-8) and Windows (default cp1252) -
    # without this, a value written on one platform mis-decodes as "?"/mojibake when
    # read on the other, exactly the bug csv_to_latex.py hit reading a WSL2-written
    # CSV on Windows.
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=RESILIENCE_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def render_markdown(rows: List[Dict[str, str]]) -> str:
    if not rows:
        return "(no successful runs to report)\n"
    columns = list(rows[0].keys())
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join("---" for _ in columns) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(row[c] for c in columns) + " |")
    return "\n".join(lines) + "\n"


def render_detailed_statistics(summaries: List[dict]) -> str:
    """Per-mode, per-metric N/mean/median/
    stdev/95% CI/min/max/p95 for Throughput (tasks/hr) - the one metric every
    successful run always has. Skips a mode entirely if it has no measured
    throughput values (never a fabricated all-zero row)."""
    by_mode: Dict[str, List[dict]] = {}
    for s in summaries:
        label = _mode_label(s.get("coordination", "centralized"), s.get("agent_backend"))
        by_mode.setdefault(label, []).append(s)

    lines = ["## Detailed statistics: Throughput (tasks/hr)\n"]
    lines.append("| Mode | N | mean | median | stdev | 95% CI | min | max | p95 |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    any_row = False
    for label, mode_summaries in by_mode.items():
        values = [v for s in mode_summaries if (v := _throughput(s)) is not None]
        stats = compute_metric_statistics(values)
        if stats is None:
            continue
        any_row = True
        ci = (
            f"[{stats['ci95_low']}, {stats['ci95_high']}]"
            if stats["ci95_low"] is not None
            else NOT_MEASURED
        )
        lines.append(
            f"| {label} | {stats['n']} | {stats['mean']} | {stats['median']} | "
            f"{stats['stdev'] if stats['stdev'] is not None else NOT_MEASURED} | {ci} | "
            f"{stats['min']} | {stats['max']} | {stats['p95']} |"
        )
    if not any_row:
        return ""

    # Paired differences: for every pair of modes present, on
    # Throughput, matched by random_seed. Only emitted when at least one pair
    # actually shares seeds - a batch run with disjoint seeds per mode (e.g. one
    # seed per mode, all different) legitimately has nothing to pair.
    labels = list(by_mode.keys())
    paired_lines = []
    for i in range(len(labels)):
        for j in range(i + 1, len(labels)):
            a, b = labels[i], labels[j]
            diff_stats = compute_paired_differences(by_mode[a], by_mode[b], _throughput)
            if diff_stats is None:
                continue
            ci = (
                f"[{diff_stats['ci95_low']}, {diff_stats['ci95_high']}]"
                if diff_stats["ci95_low"] is not None
                else NOT_MEASURED
            )
            paired_lines.append(
                f"| {b} - {a} | {diff_stats['n']} | {diff_stats['mean']} | "
                f"{diff_stats['stdev'] if diff_stats['stdev'] is not None else NOT_MEASURED} | {ci} |"
            )
    if paired_lines:
        lines.append("\n## Paired differences (by seed): Throughput (tasks/hr)\n")
        lines.append("| Comparison | N pairs | mean diff | stdev | 95% CI |")
        lines.append("| --- | --- | --- | --- | --- |")
        lines.extend(paired_lines)

    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("batch_dir", help="directory containing batch_manifest.json")
    parser.add_argument("--out", default=None, help="defaults to <batch_dir>/report.md")
    parser.add_argument(
        "--resilience-csv",
        default=None,
        help="where to write the B/C/D/E resilience comparison "
        "(defaults to analysis/results/resilience_summary.csv). Only written if this "
        "batch has at least one fault-injected (t_fail_sec-bearing) run.",
    )
    args = parser.parse_args()

    batch_dir = Path(args.batch_dir)
    manifest_path = batch_dir / "batch_manifest.json"
    if not manifest_path.exists():
        print(f"ERROR: no batch_manifest.json in {batch_dir}", file=sys.stderr)
        sys.exit(1)
    manifest = json.loads(manifest_path.read_text())

    summaries = _load_summaries(manifest)
    rows = build_comparison_table(summaries)
    table = render_markdown(rows)
    detailed = render_detailed_statistics(summaries)
    resilience_rows = build_resilience_table(summaries)
    resilience_table = render_markdown(resilience_rows) if resilience_rows else ""

    out_path = Path(args.out) if args.out else batch_dir / "report.md"
    report_text = (
        f"# Comparison report: {manifest.get('experiment', '?')}\n\n"
        f"{manifest['succeeded']}/{len(manifest['runs'])} runs succeeded.\n\n"
        f"{table}\n{detailed}"
    )
    if resilience_rows:
        report_text += f"\n## Resilience comparison (Mode B/C/D/E)\n\n{resilience_table}"
    out_path.write_text(report_text)
    print(table)
    if detailed:
        print(detailed)

    if resilience_rows:
        resilience_csv_path = (
            Path(args.resilience_csv)
            if args.resilience_csv
            else REPO_ROOT / "analysis" / "results" / "resilience_summary.csv"
        )
        write_resilience_csv(resilience_rows, resilience_csv_path)
        print(resilience_table)
        print(f"[generate_report] wrote {resilience_csv_path}")
    else:
        print(
            "[generate_report] no fault-injected (t_fail_sec) runs in this batch - "
            "resilience_summary.csv not written",
            file=sys.stderr,
        )
    print(f"[generate_report] wrote {out_path}")


if __name__ == "__main__":
    main()
