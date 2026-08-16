#!/usr/bin/env python3
"""Ties every Phase II analysis module together into one report from a real
pilot campaign's data: interaction-density table (what the user explicitly
asked to see reported), role-formation persistence, peer-preference
concentration (memory ON vs OFF ablation: llm_memory vs llm_no_memory - the
one pair in this pilot's 4 conditions that holds the backend constant and
varies only memory), resource-convention findings, interaction-network hub
concentration, and an honest emergence-criteria evaluation.

    python3 phase2/analysis/generate_phase2_report.py phase2/experiments/pilot_001 \
        --out-dir phase2/analysis/results --paper-dir ../paper

Nothing here recomputes or overwrites anything under phase2/experiments/ -
strictly read-only over the campaign's own output.
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path
from typing import Dict, List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from analysis.emergence_criteria import evaluate_pattern  # noqa: E402
from analysis.interaction_network import (  # noqa: E402
    betweenness_centrality,
    build_edges,
    degree_centrality,
    leadership_concentration,
)
from analysis.peer_preference import (  # noqa: E402
    compute_pair_stats,
    compute_selection_probabilities,
    concentration_index,
)
from analysis.resource_conventions import (  # noqa: E402
    compute_charging_cycles,
    jain_fairness_index,
    lower_soc_priority_rate,
    round_robin_score,
    usage_share,
)
from analysis.role_formation import (  # noqa: E402
    ALL_ROLES,
    assign_candidate_roles,
    compute_role_persistence,
    compute_window_metrics,
    load_jsonl,
)

CONDITION_ORDER = ["deterministic_no_memory", "llm_no_memory", "llm_memory", "hybrid_memory"]


def _load_run(run_dir: Path) -> Optional[dict]:
    meta_path, summary_path = run_dir / "metadata.json", run_dir / "summary.json"
    if not meta_path.exists() or not summary_path.exists():
        return None
    metadata = json.loads(meta_path.read_text(encoding="utf-8"))
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    interactions = load_jsonl(run_dir / "interactions.jsonl")
    robot_states = load_jsonl(run_dir / "robot_states.jsonl")
    robots = sorted(summary["tasks_completed_per_robot"])
    return {
        "run_dir": str(run_dir), "metadata": metadata, "summary": summary,
        "interactions": interactions, "robot_states": robot_states, "robots": robots,
    }


def analyze_run(run: dict) -> dict:
    interactions, robot_states, robots = run["interactions"], run["robot_states"], run["robots"]
    duration_sec = run["metadata"]["duration_sec"]

    window_rows = compute_window_metrics(robot_states, interactions, duration_sec)
    role_assignments = assign_candidate_roles(window_rows)
    persistence = compute_role_persistence(role_assignments)

    pair_stats = compute_pair_stats(interactions)
    selection_probs = compute_selection_probabilities(pair_stats)
    mean_concentration = (
        round(statistics.mean(concentration_index(p) for p in selection_probs.values()), 4)
        if selection_probs else None
    )

    cycles = compute_charging_cycles(interactions, robot_states)
    resource = {
        "usage_share": usage_share(cycles),
        "jain_fairness_index": jain_fairness_index(cycles, robots),
        "lower_soc_priority": lower_soc_priority_rate(cycles),
        "round_robin_score": round_robin_score(cycles),
        "n_charging_cycles": len(cycles),
    }

    edges = build_edges(interactions)
    centrality = degree_centrality(edges, robots)
    betweenness = betweenness_centrality(edges, robots)
    hub = leadership_concentration(centrality)

    return {
        "role_persistence": persistence,
        "peer_preference": {
            "selection_probabilities": selection_probs,
            "mean_concentration_index": mean_concentration,
        },
        "resource_conventions": resource,
        "network": {"degree_centrality": centrality, "betweenness": betweenness, "hub": hub},
        "n_windows": len({r["window"] for r in window_rows}),
    }


def _top_robot_persistence(persistence_by_role: Dict[str, Dict[str, dict]], role: str) -> Optional[dict]:
    entries = persistence_by_role.get(role, {})
    if not entries:
        return None
    robot_id, stats = max(entries.items(), key=lambda kv: kv[1]["persistence_ratio"])
    return {"robot_id": robot_id, **stats}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign_dir")
    parser.add_argument("--out-dir", default=None)
    parser.add_argument("--paper-dir", default=None)
    args = parser.parse_args()

    campaign_dir = Path(args.campaign_dir)
    manifest = json.loads((campaign_dir / "campaign_manifest.json").read_text(encoding="utf-8"))
    out_dir = Path(args.out_dir) if args.out_dir else Path(__file__).resolve().parent / "results"
    out_dir.mkdir(parents=True, exist_ok=True)

    by_condition: Dict[str, List[dict]] = {c: [] for c in CONDITION_ORDER}
    for record in manifest["runs"]:
        if not record["success"]:
            continue
        run = _load_run(Path(record["run_dir"]))
        if run is None:
            continue
        run["analysis"] = analyze_run(run)
        by_condition[record["condition"]].append(run)

    report_lines = ["# Phase II pilot report\n"]
    report_lines.append(
        f"Campaign: `{campaign_dir}` - {manifest['succeeded']}/{len(manifest['runs'])} runs "
        f"succeeded, {manifest['campaign_wall_clock_sec_so_far']}s total wall-clock.\n"
    )

    # -- 1. interaction density table (exactly what was asked to be reported) --
    report_lines.append("## Interaction density per condition (mean over paired seeds)\n")
    report_lines.append(
        "| Condition | Seeds | Tasks | Negotiations | Charging cycles | Resource conflicts (blocked) |"
    )
    report_lines.append("| --- | --- | --- | --- | --- | --- |")
    density_rows = []
    for condition in CONDITION_ORDER:
        runs = by_condition[condition]
        if not runs:
            report_lines.append(f"| {condition} | 0 | - | - | - | - |")
            continue
        tasks = [r["summary"]["tasks_completed_total"] for r in runs]
        negs = [r["summary"]["negotiations_initiated"] for r in runs]
        charges = [r["summary"]["charging_cycles_completed"] for r in runs]
        conflicts = [r["summary"]["resource_conflicts_blocked"] for r in runs]
        row = {
            "condition": condition, "n_seeds": len(runs),
            "mean_tasks": round(statistics.mean(tasks), 1),
            "mean_negotiations": round(statistics.mean(negs), 1),
            "mean_charging_cycles": round(statistics.mean(charges), 1),
            "mean_resource_conflicts": round(statistics.mean(conflicts), 1),
        }
        density_rows.append(row)
        report_lines.append(
            f"| {condition} | {row['n_seeds']} | {row['mean_tasks']} | {row['mean_negotiations']} "
            f"| {row['mean_charging_cycles']} | {row['mean_resource_conflicts']} |"
        )

    # -- 2. role persistence per condition --
    report_lines.append("\n## Role formation: top-robot persistence ratio per condition\n")
    report_lines.append("| Condition | " + " | ".join(ALL_ROLES) + " |")
    report_lines.append("| --- | " + " | ".join("---" for _ in ALL_ROLES) + " |")
    role_summary: Dict[str, Dict[str, List[float]]] = {role: {c: [] for c in CONDITION_ORDER} for role in ALL_ROLES}
    for condition in CONDITION_ORDER:
        cells = []
        for role in ALL_ROLES:
            ratios = []
            for run in by_condition[condition]:
                top = _top_robot_persistence(run["analysis"]["role_persistence"], role)
                if top:
                    ratios.append(top["persistence_ratio"])
            role_summary[role][condition] = ratios
            cells.append(f"{round(statistics.mean(ratios), 3)}" if ratios else "n/a")
        report_lines.append(f"| {condition} | " + " | ".join(cells) + " |")

    # -- 3. peer preference: memory ON vs OFF (llm_memory vs llm_no_memory) --
    report_lines.append("\n## Peer-preference concentration (memory ablation: llm_memory vs llm_no_memory)\n")
    conc_off = [r["analysis"]["peer_preference"]["mean_concentration_index"] for r in by_condition["llm_no_memory"] if r["analysis"]["peer_preference"]["mean_concentration_index"] is not None]
    conc_on = [r["analysis"]["peer_preference"]["mean_concentration_index"] for r in by_condition["llm_memory"] if r["analysis"]["peer_preference"]["mean_concentration_index"] is not None]
    report_lines.append(f"- llm_no_memory: n={len(conc_off)}, mean concentration index = {round(statistics.mean(conc_off), 4) if conc_off else 'n/a'}")
    report_lines.append(f"- llm_memory: n={len(conc_on)}, mean concentration index = {round(statistics.mean(conc_on), 4) if conc_on else 'n/a'}")

    # -- 4. resource conventions --
    report_lines.append("\n## Resource (charger) conventions per condition\n")
    report_lines.append("| Condition | Jain fairness | Lower-SOC-priority rate | Round-robin score | Charging cycles |")
    report_lines.append("| --- | --- | --- | --- | --- |")
    for condition in CONDITION_ORDER:
        runs = by_condition[condition]
        if not runs:
            report_lines.append(f"| {condition} | n/a | n/a | n/a | n/a |")
            continue
        jain = [r["analysis"]["resource_conventions"]["jain_fairness_index"] for r in runs if r["analysis"]["resource_conventions"]["jain_fairness_index"] is not None]
        soc = [r["analysis"]["resource_conventions"]["lower_soc_priority"]["rate"] for r in runs if r["analysis"]["resource_conventions"]["lower_soc_priority"]]
        rr = [r["analysis"]["resource_conventions"]["round_robin_score"] for r in runs if r["analysis"]["resource_conventions"]["round_robin_score"] is not None]
        n_cycles = [r["analysis"]["resource_conventions"]["n_charging_cycles"] for r in runs]
        report_lines.append(
            f"| {condition} | {round(statistics.mean(jain), 3) if jain else 'n/a'} "
            f"| {round(statistics.mean(soc), 3) if soc else 'n/a'} "
            f"| {round(statistics.mean(rr), 3) if rr else 'n/a'} "
            f"| {round(statistics.mean(n_cycles), 1)} |"
        )

    # -- 5. network hub concentration --
    report_lines.append("\n## Interaction-network hub concentration per condition\n")
    report_lines.append("| Condition | Runs with a hub | Mean hub degree centrality |")
    report_lines.append("| --- | --- | --- |")
    for condition in CONDITION_ORDER:
        runs = by_condition[condition]
        hubs = [r["analysis"]["network"]["hub"] for r in runs]
        with_hub = [h for h in hubs if h["hub_robot_id"] is not None]
        mean_hub = round(statistics.mean(h["hub_degree_centrality"] for h in with_hub), 4) if with_hub else "n/a"
        report_lines.append(f"| {condition} | {len(with_hub)}/{len(hubs)} | {mean_hub} |")

    # -- 6. emergence criteria evaluation --
    report_lines.append("\n## Emergence criteria evaluation\n")
    num_robots = manifest["num_robots"]
    emergence_results = {}
    for role in ALL_ROLES:
        ratios_on = role_summary[role]["llm_memory"]
        ratios_off = role_summary[role]["llm_no_memory"]
        streaks_on = []
        for run in by_condition["llm_memory"]:
            top = _top_robot_persistence(run["analysis"]["role_persistence"], role)
            if top:
                streaks_on.append(top["longest_streak"])
        result = evaluate_pattern(
            pattern_name=f"role:{role}",
            num_robots=num_robots,
            per_seed_persistence_ratio=ratios_on or [0.0],
            per_seed_longest_streak=streaks_on or [0],
            memory_on_mean=round(statistics.mean(ratios_on), 4) if ratios_on else None,
            memory_off_mean=round(statistics.mean(ratios_off), 4) if ratios_off else None,
        )
        emergence_results[role] = result
        report_lines.append(f"- **{role}**: verdict = `{result['verdict']}`")

    peer_pref_result = evaluate_pattern(
        pattern_name="peer_preference_concentration",
        num_robots=num_robots,
        per_seed_persistence_ratio=conc_on or [0.0],
        per_seed_longest_streak=[2] * len(conc_on) if conc_on else [0],
        memory_on_mean=round(statistics.mean(conc_on), 4) if conc_on else None,
        memory_off_mean=round(statistics.mean(conc_off), 4) if conc_off else None,
    )
    emergence_results["peer_preference_concentration"] = peer_pref_result
    report_lines.append(f"- **peer_preference_concentration**: verdict = `{peer_pref_result['verdict']}`")

    report_text = "\n".join(report_lines) + "\n"
    (out_dir / "phase2_pilot_report.md").write_text(report_text, encoding="utf-8")
    (out_dir / "phase2_pilot_density.json").write_text(json.dumps(density_rows, indent=2), encoding="utf-8")
    (out_dir / "phase2_emergence_criteria.json").write_text(
        json.dumps(emergence_results, indent=2), encoding="utf-8"
    )

    print(report_text)
    print(f"[generate_phase2_report] wrote {out_dir}")


if __name__ == "__main__":
    main()
