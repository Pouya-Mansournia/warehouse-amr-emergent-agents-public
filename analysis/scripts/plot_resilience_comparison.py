#!/usr/bin/env python3
"""Two single-axis comparison figures for the Mode B/C/D/E resilience campaign,
built directly from a batch's real
summary.json files - never from resilience_summary.csv's formatted strings, so
the underlying per-seed values stay available for error bars.

    python3 analysis/scripts/plot_resilience_comparison.py \
        experiments/2026-08-12_batch_resilience_pilot --out-dir paper/figures

Produces:
  resilience_recovery.png       - recovery rate (%) per mode, direct-labeled
                                   with the raw N/M fraction and mean recovery
                                   time (only bars that have a real value).
  resilience_decision_latency.png - mean decision latency per mode, log-scale
                                   (deterministic is ~1e5x faster than LLM/
                                   hybrid - a linear axis would flatten the
                                   deterministic bar to invisible).

Each bar height and label is computed straight from summary.json's own
resilience/agent_decisions blocks - nothing here is hand-typed or estimated.
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from generate_report import _load_summaries, _resilience_mode_label  # noqa: E402

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# dataviz skill's validated categorical palette (light mode, adjacent-pair order) -
# slots 1-4: blue, orange, aqua, yellow. Bars are always adjacent to their
# neighbors only (never all-pairs), so this order clears the CVD/normal-vision
# gates as validated. Yellow/aqua sit under 3:1 contrast on the light surface -
# mitigated per the palette's own relief rule via direct value labels on every bar.
COLORS = {
    "No Recovery": "#2a78d6",
    "Deterministic": "#eb6834",
    "LLM": "#1baf7a",
    "Hybrid": "#eda100",
}
MODE_ORDER = ["No Recovery", "Deterministic", "LLM", "Hybrid"]

INK_PRIMARY = "#0b0b0b"
INK_SECONDARY = "#52514e"
INK_MUTED = "#898781"
GRIDLINE = "#e1e0d9"
BASELINE = "#c3c2b7"


def _style_axes(ax, *, y_label):
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
        label.set_fontsize(10)


def plot_recovery(by_mode: dict, out_path: Path) -> None:
    labels = [m for m in MODE_ORDER if m in by_mode]
    rates, fractions, mean_times = [], [], []
    for label in labels:
        resiliences = [s["resilience"] for s in by_mode[label]]
        recovered = [r for r in resiliences if r["recovered"]]
        n_total = len(resiliences)
        rates.append(100.0 * len(recovered) / n_total if n_total else 0.0)
        fractions.append(f"{len(recovered)}/{n_total}")
        times = [r["recovery_time_sec"] for r in recovered if r.get("recovery_time_sec") is not None]
        mean_times.append(round(statistics.mean(times), 1) if times else None)

    fig, ax = plt.subplots(figsize=(6.4, 4.2), dpi=200)
    fig.patch.set_facecolor("#fcfcfb")
    ax.set_facecolor("#fcfcfb")
    bars = ax.bar(
        labels, rates, width=0.56,
        color=[COLORS[l] for l in labels], zorder=3,
        edgecolor="#fcfcfb", linewidth=2,
    )
    for bar, frac, mean_t in zip(bars, fractions, mean_times):
        height = bar.get_height()
        label = frac if mean_t is None else f"{frac}\n(mean {mean_t}s)"
        ax.annotate(
            label, (bar.get_x() + bar.get_width() / 2, height),
            xytext=(0, 6), textcoords="offset points",
            ha="center", va="bottom", fontsize=9, color=INK_PRIMARY,
        )
    ax.set_ylim(0, 115)
    _style_axes(ax, y_label="Recovery rate (%)")
    ax.set_title(
        "Post-failure recovery rate by mode (N=3 paired seeds)",
        fontsize=11, color=INK_PRIMARY, loc="left", pad=12,
    )
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, facecolor=fig.get_facecolor())
    plt.close(fig)


def plot_decision_latency(by_mode: dict, out_path: Path) -> None:
    labels = [m for m in MODE_ORDER if m in by_mode and any(s.get("agent_decisions") for s in by_mode[m])]
    means_ms = []
    for label in labels:
        latencies = [
            s["agent_decisions"]["mean_decision_latency_sec"] * 1000.0
            for s in by_mode[label]
            if s.get("agent_decisions")
        ]
        means_ms.append(statistics.mean(latencies) if latencies else 0.0)

    fig, ax = plt.subplots(figsize=(6.4, 4.2), dpi=200)
    fig.patch.set_facecolor("#fcfcfb")
    ax.set_facecolor("#fcfcfb")
    bars = ax.bar(
        labels, means_ms, width=0.56,
        color=[COLORS[l] for l in labels], zorder=3,
        edgecolor="#fcfcfb", linewidth=2,
    )
    ax.set_yscale("log")
    for bar, val in zip(bars, means_ms):
        ax.annotate(
            f"{val:.3g} ms", (bar.get_x() + bar.get_width() / 2, bar.get_height()),
            xytext=(0, 6), textcoords="offset points",
            ha="center", va="bottom", fontsize=9, color=INK_PRIMARY,
        )
    _style_axes(ax, y_label="Mean decision latency, ms (log scale)")
    ax.set_title(
        "Cost of intelligence: mean per-decision latency by mode",
        fontsize=11, color=INK_PRIMARY, loc="left", pad=12,
    )
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, facecolor=fig.get_facecolor())
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("batch_dir")
    parser.add_argument("--out-dir", default="paper/figures")
    args = parser.parse_args()

    batch_dir = Path(args.batch_dir)
    manifest = json.loads((batch_dir / "batch_manifest.json").read_text())
    summaries = _load_summaries(manifest)

    by_mode: dict = {}
    for s in summaries:
        if not s.get("resilience"):
            continue
        label = _resilience_mode_label(s.get("coordination", "centralized"), s.get("agent_backend"))
        if label is None:
            continue
        by_mode.setdefault(label, []).append(s)

    out_dir = Path(args.out_dir)
    plot_recovery(by_mode, out_dir / "resilience_recovery.png")
    plot_decision_latency(by_mode, out_dir / "resilience_decision_latency.png")
    print(f"[plot_resilience_comparison] wrote figures to {out_dir}")


if __name__ == "__main__":
    main()
