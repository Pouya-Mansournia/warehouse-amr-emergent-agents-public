"""Shared-resource (charger) convention analysis.

Every field here is derived from real logged events - `CHARGER_REQUEST`
(successful claim, `result=None`) / `CHARGER_YIELD` (`result="RELEASED"` on a
normal finish) from `interactions.jsonl`, cross-referenced with
`robot_states.jsonl` for battery SOC at request time. No convention label
(lower-SOC-priority, round-robin, dominant-user, ...) is asserted without a
computed rate/index backing it - see `docs/notes.md`'s
"never fabricate a result" rule, which Phase II inherits by choice, not by
being forced to (Phase II is a separate codebase).
"""
from __future__ import annotations

from bisect import bisect_right
from collections import defaultdict
from typing import Dict, List, Optional


def _battery_at(robot_states_by_robot: Dict[str, List[dict]], robot_id: str, t: float) -> Optional[float]:
    samples = robot_states_by_robot.get(robot_id)
    if not samples:
        return None
    times = [s["simulation_time"] for s in samples]
    idx = bisect_right(times, t) - 1
    if idx < 0:
        return None
    return samples[idx]["battery_soc"]


def compute_charging_cycles(interactions: List[dict], robot_states: List[dict]) -> List[dict]:
    by_robot: Dict[str, List[dict]] = defaultdict(list)
    for s in robot_states:
        by_robot[s["robot_id"]].append(s)
    for samples in by_robot.values():
        samples.sort(key=lambda s: s["simulation_time"])

    requests = sorted(
        (e for e in interactions if e["interaction_type"] == "CHARGER_REQUEST" and e["result"] is None),
        key=lambda e: e["simulation_time"],
    )
    releases = sorted(
        (e for e in interactions if e["interaction_type"] == "CHARGER_YIELD" and e["result"] == "RELEASED"),
        key=lambda e: e["simulation_time"],
    )
    blocked = [e for e in interactions if e.get("result") == "BLOCKED_NO_CANDIDATES" and e["interaction_type"] == "CHARGER_REQUEST"]

    cycles = []
    release_idx = 0
    for req in requests:
        release = None
        while release_idx < len(releases) and releases[release_idx]["simulation_time"] < req["simulation_time"]:
            release_idx += 1
        if release_idx < len(releases):
            release = releases[release_idx]
            release_idx += 1
        waiting_peers = {
            e["from"]: _battery_at(by_robot, e["from"], e["simulation_time"])
            for e in blocked
            if e["from"] != req["from"] and abs(e["simulation_time"] - req["simulation_time"]) <= 30.0
        }
        requester_soc = _battery_at(by_robot, req["from"], req["simulation_time"])
        cycles.append({
            "robot_id": req["from"],
            "request_time": req["simulation_time"],
            "release_time": release["simulation_time"] if release else None,
            "duration_sec": (release["simulation_time"] - req["simulation_time"]) if release else None,
            "requester_battery_soc": requester_soc,
            "concurrently_waiting_peers": {k: v for k, v in waiting_peers.items() if v is not None},
        })
    return cycles


def usage_share(cycles: List[dict]) -> Dict[str, float]:
    counts: Dict[str, int] = defaultdict(int)
    for c in cycles:
        counts[c["robot_id"]] += 1
    total = sum(counts.values())
    return {r: round(n / total, 4) for r, n in counts.items()} if total else {}


def jain_fairness_index(cycles: List[dict], all_robots: List[str]) -> Optional[float]:
    """1.0 = every robot used the charger an equal number of times; 1/N = one
    robot used it exclusively. None when there were no charging cycles at all."""
    counts = {r: 0 for r in all_robots}
    for c in cycles:
        counts[c["robot_id"]] = counts.get(c["robot_id"], 0) + 1
    values = list(counts.values())
    if not values or sum(values) == 0:
        return None
    n = len(values)
    return round((sum(values) ** 2) / (n * sum(v ** 2 for v in values)), 4)


def lower_soc_priority_rate(cycles: List[dict]) -> Optional[dict]:
    """Of the charging events where at least one OTHER robot was also waiting
    (blocked) around the same time, what fraction went to whichever robot
    actually had the lowest battery SOC among the requester + waiting peers?
    A rate near 1.0 is real evidence of a "lowest-SOC-first" convention
    emerging from the deterministic cost formula's energy term; near the
    1/(N waiting+1) baseline is evidence of no such convention. Returns None
    (not 0.0) when zero contested events exist - there is nothing to measure,
    which is different from measuring "never happens"."""
    contested = [c for c in cycles if c["concurrently_waiting_peers"] and c["requester_battery_soc"] is not None]
    if not contested:
        return None
    wins = 0
    for c in contested:
        socs = dict(c["concurrently_waiting_peers"])
        socs[c["robot_id"]] = c["requester_battery_soc"]
        lowest_robot = min(socs.items(), key=lambda kv: (kv[1], kv[0]))[0]
        if lowest_robot == c["robot_id"]:
            wins += 1
    return {
        "contested_events": len(contested),
        "lowest_soc_won_count": wins,
        "rate": round(wins / len(contested), 4),
    }


def round_robin_score(cycles: List[dict]) -> Optional[float]:
    """Fraction of consecutive charging events won by a DIFFERENT robot than
    the previous one - 1.0 is perfectly alternating (round-robin-like), 0.0 is
    the same robot repeatedly. None if fewer than 2 cycles occurred."""
    if len(cycles) < 2:
        return None
    ordered = sorted(cycles, key=lambda c: c["request_time"])
    switches = sum(
        1 for a, b in zip(ordered, ordered[1:]) if a["robot_id"] != b["robot_id"]
    )
    return round(switches / (len(ordered) - 1), 4)
