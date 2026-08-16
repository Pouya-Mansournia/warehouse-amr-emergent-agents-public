#!/usr/bin/env python3
"""Figures for the Phase II pilot report - reads the same per-run analysis
generate_phase2_report.py computes (recomputed here rather than piped through
a file, to keep this script runnable standalone). Same categorical palette
convention as Phase I's plot_resilience_comparison.py (dataviz skill,
validated adjacent-pair order: blue/orange/aqua/yellow).

    python3 phase2/analysis/plot_phase2_report.py phase2/experiments/pilot_001 \
        --out-dir ../paper/figures
"""
from __future__ import annotations

import argparse
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from generate_phase2_report import CONDITION_ORDER, _load_run, _top_robot_persistence, analyze_run  # noqa: E402
from role_formation import ALL_ROLES  # noqa: E402

import json
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
INK_PRIMARY, INK_SECONDARY, INK_MUTED = "#0b0b0b", "#52514e", "#898781"
GRIDLINE, BASELINE = "#e1e0d9", "#c3c2b7"
SURFACE = "#fcfcfb"


def _style(ax, y_label):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color(BASELINE)
    ax.spines["bottom"].set_color(BASELINE)
    ax.tick_params(colors=INK_MUTED, labelsize=9)
    ax.yaxis.grid(True, color=GRIDLINE, linewidth=0.8, zorder=0)
    ax.set_axisbelow(True)
    ax.set_ylabel(y_label, color=INK_SECONDARY, fontsize=10)
    for label in ax.get_xticklabels():
        label.set_color(INK_PRIMARY)
        label.set_fontsize(9)


def _grouped_bar(labels, series: dict, title, y_label, out_path):
    fig, ax = plt.subplots(figsize=(7.2, 4.2), dpi=200)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)
    n_series = len(series)
    width = 0.8 / n_series
    x = range(len(labels))
    for i, (name, values) in enumerate(series.items()):
        offs = [xi + (i - (n_series - 1) / 2) * width for xi in x]
        ax.bar(offs, values, width=width * 0.9, label=name, color=COLORS[i % len(COLORS)], zorder=3)
    ax.set_xticks(list(x))
    ax.set_xticklabels(labels, rotation=15, ha="right")
    _style(ax, y_label)
    ax.set_title(title, fontsize=11, color=INK_PRIMARY, loc="left", pad=12)
    if n_series > 1:
        ax.legend(frameon=False, fontsize=9, labelcolor=INK_SECONDARY)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, facecolor=fig.get_facecolor())
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign_dir")
    parser.add_argument("--out-dir", default="paper/figures")
    args = parser.parse_args()

    campaign_dir = Path(args.campaign_dir)
    manifest = json.loads((campaign_dir / "campaign_manifest.json").read_text(encoding="utf-8"))
    out_dir = Path(args.out_dir)

    by_condition = {c: [] for c in CONDITION_ORDER}
    for record in manifest["runs"]:
        if not record["success"]:
            continue
        run = _load_run(Path(record["run_dir"]))
        if run is None:
            continue
        run["analysis"] = analyze_run(run)
        by_condition[record["condition"]].append(run)

    labels = [c for c in CONDITION_ORDER if by_condition[c]]

    tasks = [statistics.mean(r["summary"]["tasks_completed_total"] for r in by_condition[c]) for c in labels]
    conflicts = [statistics.mean(r["summary"]["resource_conflicts_blocked"] for r in by_condition[c]) for c in labels]
    _grouped_bar(
        labels, {"Tasks completed": tasks, "Resource conflicts (blocked)": conflicts},
        "Tasks and resource conflicts by condition", "Mean count (per 3000s run)",
        out_dir / "phase2_tasks_conflicts.png",
    )

    negs = [statistics.mean(r["summary"]["negotiations_initiated"] for r in by_condition[c]) for c in labels]
    charges = [statistics.mean(r["summary"]["charging_cycles_completed"] for r in by_condition[c]) for c in labels]
    _grouped_bar(
        labels, {"Negotiations initiated": negs, "Charging cycles completed": charges},
        "Negotiations and charging cycles by condition", "Mean count (per 3000s run)",
        out_dir / "phase2_negotiations_charging.png",
    )

    role_series = {}
    for role in ALL_ROLES:
        vals = []
        for c in labels:
            ratios = [
                _top_robot_persistence(r["analysis"]["role_persistence"], role)
                for r in by_condition[c]
            ]
            ratios = [r["persistence_ratio"] for r in ratios if r]
            vals.append(statistics.mean(ratios) if ratios else 0.0)
        role_series[role] = vals
    _grouped_bar(
        labels, role_series, "Top-robot role persistence ratio by condition",
        "Mean persistence ratio (1/N robots = uniform baseline)",
        out_dir / "phase2_role_persistence.png",
    )

    jain = [
        statistics.mean(
            v for r in by_condition[c]
            if (v := r["analysis"]["resource_conventions"]["jain_fairness_index"]) is not None
        ) if any(r["analysis"]["resource_conventions"]["jain_fairness_index"] is not None for r in by_condition[c]) else 0.0
        for c in labels
    ]
    rr = [
        statistics.mean(
            v for r in by_condition[c]
            if (v := r["analysis"]["resource_conventions"]["round_robin_score"]) is not None
        ) if any(r["analysis"]["resource_conventions"]["round_robin_score"] is not None for r in by_condition[c]) else 0.0
        for c in labels
    ]
    _grouped_bar(
        labels, {"Jain fairness index": jain, "Round-robin score": rr},
        "Charger usage conventions by condition", "Index (0-1)",
        out_dir / "phase2_resource_conventions.png",
    )

    print(f"[plot_phase2_report] wrote figures to {out_dir}")


if __name__ == "__main__":
    main()
