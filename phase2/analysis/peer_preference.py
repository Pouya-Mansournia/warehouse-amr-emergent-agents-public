"""Peer-preference analysis: does accumulated interaction
history change which peer a robot's help-request negotiation picks?

All counts come directly from `interactions.jsonl` rows - `TASK_TRANSFER`
(from=initiator, to=winner, result=SUCCESS) is the actual selection event;
`HELP_OFFER` rows are the pool of peers who bid and therefore *could* have
been selected. `selection_probability` is normalized over an initiator's own
total selections, so it answers "given this robot picked a winner, how often
was it robot X specifically" - the concentration this metric would show if
memory produced real peer preference is a probability mass moving away from
uniform across peers, not a fabricated score.
"""
from __future__ import annotations

from collections import defaultdict
from typing import Dict, List, Tuple


def compute_pair_stats(interactions: List[dict]) -> Dict[Tuple[str, str], dict]:
    """{(initiator, peer): {bids, wins, rejections, mean_response_time_sec,
    acceptance_rate}} for every ordered (initiator, peer) pair that ever
    interacted in a negotiation."""
    bids = defaultdict(int)
    wins = defaultdict(int)
    rejections = defaultdict(int)
    response_times = defaultdict(list)

    for e in interactions:
        if e["interaction_type"] == "HELP_OFFER":
            # from=peer (the bidder), to=initiator
            bids[(e["to"], e["from"])] += 1
        elif e["interaction_type"] == "TASK_TRANSFER" and e["result"] == "SUCCESS":
            wins[(e["from"], e["to"])] += 1
            if e.get("response_latency") is not None:
                response_times[(e["from"], e["to"])].append(e["response_latency"])
        elif e["interaction_type"] == "TASK_REJECT":
            rejections[(e["from"], e["to"])] += 1

    pairs = set(bids) | set(wins) | set(rejections)
    result = {}
    for pair in pairs:
        b, w = bids.get(pair, 0), wins.get(pair, 0)
        rt = response_times.get(pair, [])
        result[pair] = {
            "bids": b,
            "wins": w,
            "rejections": rejections.get(pair, 0),
            "acceptance_rate": round(w / b, 4) if b else None,
            "mean_response_time_sec": round(sum(rt) / len(rt), 4) if rt else None,
        }
    return result


def compute_selection_probabilities(pair_stats: Dict[Tuple[str, str], dict]) -> Dict[str, Dict[str, float]]:
    """{initiator: {peer: selection_probability}} - each initiator's wins
    normalized over its own total wins. A uniform distribution across peers
    (1/N_peers each) is the null hypothesis "no preference"; concentration
    away from uniform is the peer-preference signal this analysis looks
    for. Initiators with zero total wins are omitted (nothing to normalize)."""
    totals: Dict[str, int] = defaultdict(int)
    for (initiator, _peer), stats in pair_stats.items():
        totals[initiator] += stats["wins"]

    result: Dict[str, Dict[str, float]] = defaultdict(dict)
    for (initiator, peer), stats in pair_stats.items():
        if totals[initiator] == 0 or stats["wins"] == 0:
            continue
        result[initiator][peer] = round(stats["wins"] / totals[initiator], 4)
    return dict(result)


def concentration_index(selection_probs: Dict[str, float]) -> float:
    """Herfindahl-Hirschman-style concentration index (sum of squared shares) for
    one initiator's selection distribution: 1/N (uniform, no preference) to 1.0
    (always the same peer). Returns None-equivalent 0.0 for an empty input -
    callers should check for that separately rather than reading 0.0 as
    "perfectly uniform"."""
    if not selection_probs:
        return 0.0
    return round(sum(p ** 2 for p in selection_probs.values()), 4)
