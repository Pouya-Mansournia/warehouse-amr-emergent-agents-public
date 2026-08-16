"""Rolling-window role-formation analysis.

Per-window per-robot metrics are computed straight from `robot_states.jsonl`
(periodic state samples) and `interactions.jsonl` (real events) - nothing here
is invented. Candidate role LABELS are strictly post-hoc and descriptive: for
each window, whichever robot ranks highest on a given metric gets that
window's candidate label for that metric (ties broken by robot_id, matching
this project's other deterministic tie-breaks). No role is assigned during
the simulation itself and no robot is ever told to "be" a role - see
`world/world.py`, which has no such concept anywhere in the tick loop.

The research question here is "does role
formation repeatedly occur", not "does robot X always become the helper" -
`role_persistence` is computed per (robot, role) pair specifically so a
result like "some robot is the long-distance specialist in 80% of windows,
but which robot varies by seed" is visible and distinguishable from "one
specific robot always wins."
"""
from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional

WINDOW_SEC = 300.0

ROLE_LONG_DISTANCE = "long_distance_specialist"
ROLE_LOCAL_TASK = "local_task_specialist"
ROLE_HIGH_UTILIZATION = "high_utilization_worker"
ROLE_HELPER = "helper"
ALL_ROLES = (ROLE_LONG_DISTANCE, ROLE_LOCAL_TASK, ROLE_HIGH_UTILIZATION, ROLE_HELPER)


def load_jsonl(path: Path) -> List[dict]:
    if not path.exists():
        return []
    lines = path.read_text(encoding="utf-8").splitlines()
    return [json.loads(l) for l in lines if l.strip()]


def compute_window_metrics(
    robot_states: List[dict], interactions: List[dict], duration_sec: float,
    window_sec: float = WINDOW_SEC,
) -> List[dict]:
    """One row per (window_index, robot_id): tasks_completed, mean_task_distance
    (distance_traveled delta / tasks_completed delta within the window - a proxy,
    since per-task distance isn't logged directly), help_given, help_received,
    charger_visits, resource_conflicts_blocked, utilization (end-of-window
    cumulative value)."""
    n_windows = max(1, int(duration_sec // window_sec))
    by_robot_states: Dict[str, List[dict]] = defaultdict(list)
    for s in robot_states:
        by_robot_states[s["robot_id"]].append(s)
    for samples in by_robot_states.values():
        samples.sort(key=lambda s: s["simulation_time"])

    rows = []
    for w in range(n_windows):
        w_start, w_end = w * window_sec, (w + 1) * window_sec
        window_interactions = [e for e in interactions if w_start <= e["simulation_time"] < w_end]

        for robot_id, samples in by_robot_states.items():
            in_window = [s for s in samples if w_start <= s["simulation_time"] < w_end]
            if not in_window:
                continue
            first, last = in_window[0], in_window[-1]
            tasks_delta = last["tasks_completed"] - first["tasks_completed"]
            distance_delta = last["distance_traveled_m"] - first["distance_traveled_m"]
            mean_task_distance = round(distance_delta / tasks_delta, 3) if tasks_delta > 0 else None

            help_given = sum(
                1 for e in window_interactions
                if e["interaction_type"] == "TASK_TRANSFER" and e["to"] == robot_id and e["result"] == "SUCCESS"
            )
            help_received = sum(
                1 for e in window_interactions
                if e["interaction_type"] == "TASK_TRANSFER" and e["from"] == robot_id and e["result"] == "SUCCESS"
            )
            charger_visits = sum(
                1 for e in window_interactions
                if e["interaction_type"] == "CHARGER_YIELD" and e["from"] == robot_id and e["result"] == "RELEASED"
            )
            resource_conflicts_blocked = sum(
                1 for e in window_interactions
                if e["from"] == robot_id and e.get("result") == "BLOCKED_NO_CANDIDATES"
            )

            rows.append({
                "window": w, "robot_id": robot_id,
                "tasks_completed": tasks_delta,
                "mean_task_distance": mean_task_distance,
                "help_given": help_given,
                "help_received": help_received,
                "charger_visits": charger_visits,
                "resource_conflicts_blocked": resource_conflicts_blocked,
                "utilization": last["utilization"],
            })
    return rows


def _argmax_robot(rows: List[dict], key: str, *, minimize: bool = False) -> Optional[str]:
    """Best-by-`key` robot_id, ties broken deterministically by lowest robot_id."""
    candidates = [r for r in rows if r[key] is not None]
    if not candidates:
        return None
    best_value = min(r[key] for r in candidates) if minimize else max(r[key] for r in candidates)
    tied_ids = sorted(r["robot_id"] for r in candidates if r[key] == best_value)
    return tied_ids[0]


def assign_candidate_roles(window_rows: List[dict]) -> List[dict]:
    """One row per window: which robot (if any) holds each candidate role that
    window. A role is only assigned if at least one robot had a non-zero/non-None
    value for its metric that window (an all-idle window assigns nothing -
    never a fabricated role for a window with no real activity)."""
    by_window: Dict[int, List[dict]] = defaultdict(list)
    for r in window_rows:
        by_window[r["window"]].append(r)

    assignments = []
    for w, rows in sorted(by_window.items()):
        long_distance = _argmax_robot(
            [r for r in rows if r["mean_task_distance"]], "mean_task_distance"
        )
        local_task = _argmax_robot(
            [r for r in rows if r["mean_task_distance"]], "mean_task_distance", minimize=True
        )
        high_util = _argmax_robot([r for r in rows if r["tasks_completed"] > 0], "tasks_completed")
        helper = _argmax_robot([r for r in rows if r["help_given"] > 0], "help_given")
        assignments.append({
            "window": w,
            ROLE_LONG_DISTANCE: long_distance,
            ROLE_LOCAL_TASK: local_task,
            ROLE_HIGH_UTILIZATION: high_util,
            ROLE_HELPER: helper,
        })
    return assignments


def compute_role_persistence(role_assignments: List[dict]) -> Dict[str, Dict[str, dict]]:
    """{role: {robot_id: {windows_held, total_windows, persistence_ratio,
    longest_streak}}} - persistence_ratio is windows_held / windows where that
    role was assigned to ANYONE (not the full run, so an idle run doesn't
    silently deflate every ratio toward zero)."""
    result: Dict[str, Dict[str, dict]] = {role: defaultdict(lambda: {"windows_held": 0, "streak": 0, "longest_streak": 0}) for role in ALL_ROLES}
    windows_assigned = {role: 0 for role in ALL_ROLES}

    current_holder = {role: None for role in ALL_ROLES}
    for row in role_assignments:
        for role in ALL_ROLES:
            holder = row[role]
            if holder is None:
                current_holder[role] = None
                continue
            windows_assigned[role] += 1
            entry = result[role][holder]
            entry["windows_held"] += 1
            if current_holder[role] == holder:
                entry["streak"] += 1
            else:
                entry["streak"] = 1
            entry["longest_streak"] = max(entry["longest_streak"], entry["streak"])
            current_holder[role] = holder

    final: Dict[str, Dict[str, dict]] = {}
    for role in ALL_ROLES:
        final[role] = {}
        for robot_id, entry in result[role].items():
            total = windows_assigned[role]
            final[role][robot_id] = {
                "windows_held": entry["windows_held"],
                "windows_role_assigned": total,
                "persistence_ratio": round(entry["windows_held"] / total, 4) if total else 0.0,
                "longest_streak": entry["longest_streak"],
            }
    return final
