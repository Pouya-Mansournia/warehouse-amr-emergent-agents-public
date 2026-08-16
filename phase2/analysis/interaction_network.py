"""Interaction-network analysis: does one robot repeatedly
become a coordination hub, without ever being explicitly elected one?

Edges are built only from real peer-to-peer events - `HELP_OFFER` (peer bid on
another robot's low-battery offer), `TASK_TRANSFER`, and `TASK_REJECT` (both
carry a real `from`/`to` pair). Station-claim events are deliberately excluded
(their `to` is almost always `None` - a claim is a broadcast to "whoever is
listening", not a directed peer interaction). No external graph library is
used (networkx etc.) - with 4 robots the graph is small enough that a plain,
dependency-free implementation is both simpler and easier to verify by hand
than pulling in a library for it.
"""
from __future__ import annotations

from collections import defaultdict, deque
from typing import Dict, List, Tuple

NETWORK_INTERACTION_TYPES = frozenset({"HELP_OFFER", "TASK_TRANSFER", "TASK_REJECT"})


def build_edges(interactions: List[dict]) -> List[Tuple[str, str]]:
    return [
        (e["from"], e["to"])
        for e in interactions
        if e["interaction_type"] in NETWORK_INTERACTION_TYPES and e["to"] is not None
    ]


def degree_centrality(edges: List[Tuple[str, str]], robots: List[str]) -> Dict[str, dict]:
    """`degree_centrality` here is frequency-weighted (one edge per interaction
    EVENT, not per unique peer relationship) - deliberately, since "robot1 and
    robot2 negotiated once" and "robot1 and robot2 negotiated 50 times" are
    different amounts of real interaction volume this analysis wants to
    distinguish. This means the value is NOT bounded to [0, 1] the way a
    classic simple-graph degree centrality would be (dividing by `2*(n-1)` is
    a fixed reference scale for comparison across robots/runs, not a ceiling)
    - only its relative ordering across robots is meaningful, which is all
    `leadership_concentration` below uses it for."""
    in_deg: Dict[str, int] = defaultdict(int)
    out_deg: Dict[str, int] = defaultdict(int)
    for a, b in edges:
        out_deg[a] += 1
        in_deg[b] += 1
    n = len(robots)
    max_possible = max(n - 1, 1)
    return {
        r: {
            "in_degree": in_deg.get(r, 0),
            "out_degree": out_deg.get(r, 0),
            "degree_centrality": round((in_deg.get(r, 0) + out_deg.get(r, 0)) / (2 * max_possible), 4),
        }
        for r in robots
    }


def _shortest_paths_through(adjacency: Dict[str, set], robots: List[str]) -> Dict[str, int]:
    """For every pair (s, t) with a shortest path, count how many such
    shortest paths pass through each intermediate node - the numerator of
    unweighted betweenness centrality. Ties (multiple shortest paths) split
    credit fractionally via BFS layer counting (a simplified, undirected
    Brandes-style pass - correct for these small, sparse graphs)."""
    betweenness: Dict[str, float] = defaultdict(float)
    for s in robots:
        # BFS distances and shortest-path counts from s
        dist = {s: 0}
        sigma = {s: 1}
        order = [s]
        queue = deque([s])
        preds: Dict[str, List[str]] = defaultdict(list)
        while queue:
            v = queue.popleft()
            for w in adjacency.get(v, ()):
                if w not in dist:
                    dist[w] = dist[v] + 1
                    sigma[w] = 0
                    queue.append(w)
                    order.append(w)
                if dist.get(w) == dist[v] + 1:
                    sigma[w] += sigma[v]
                    preds[w].append(v)
        delta: Dict[str, float] = defaultdict(float)
        for w in reversed(order):
            for v in preds[w]:
                delta[v] += (sigma[v] / sigma[w]) * (1 + delta[w])
            if w != s:
                betweenness[w] += delta[w]
    # undirected graph double-counts each pair's contribution once per direction
    return {r: round(betweenness.get(r, 0.0) / 2.0, 4) for r in robots}


def betweenness_centrality(edges: List[Tuple[str, str]], robots: List[str]) -> Dict[str, float]:
    adjacency: Dict[str, set] = defaultdict(set)
    for a, b in edges:
        adjacency[a].add(b)
        adjacency[b].add(a)
    return _shortest_paths_through(adjacency, robots)


def leadership_concentration(centrality: Dict[str, dict]) -> Dict[str, str]:
    """The single robot with the highest degree centrality this run/window -
    `None` if there was no interaction activity at all (never a fabricated
    leader for an idle graph)."""
    if not centrality or all(v["degree_centrality"] == 0 for v in centrality.values()):
        return {"hub_robot_id": None, "hub_degree_centrality": 0.0}
    best_value = max(v["degree_centrality"] for v in centrality.values())
    hub_id = min(r for r, v in centrality.items() if v["degree_centrality"] == best_value)
    return {"hub_robot_id": hub_id, "hub_degree_centrality": best_value}
