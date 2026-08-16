from analysis.interaction_network import (
    betweenness_centrality,
    build_edges,
    degree_centrality,
    leadership_concentration,
)


def _event(from_r, to_r, itype="TASK_TRANSFER"):
    return {"from": from_r, "to": to_r, "interaction_type": itype}


def test_build_edges_only_includes_directed_peer_events():
    events = [
        _event("robot1", "robot2", "TASK_TRANSFER"),
        _event("robot1", None, "RESOURCE_REQUEST"),  # broadcast, no peer - excluded
        _event("robot2", "robot1", "HELP_OFFER"),
    ]
    edges = build_edges(events)
    assert ("robot1", "robot2") in edges
    assert ("robot2", "robot1") in edges
    assert len(edges) == 2


def test_degree_centrality_counts_in_and_out():
    edges = [("robot1", "robot2"), ("robot1", "robot3"), ("robot2", "robot1")]
    centrality = degree_centrality(edges, ["robot1", "robot2", "robot3"])
    assert centrality["robot1"]["out_degree"] == 2
    assert centrality["robot1"]["in_degree"] == 1


def test_betweenness_centrality_hub_scores_higher_than_leaves():
    # star graph: robot1 is the hub, robot2/robot3/robot4 only connect through it
    edges = [
        ("robot1", "robot2"), ("robot2", "robot1"),
        ("robot1", "robot3"), ("robot3", "robot1"),
        ("robot1", "robot4"), ("robot4", "robot1"),
    ]
    robots = ["robot1", "robot2", "robot3", "robot4"]
    betweenness = betweenness_centrality(edges, robots)
    assert betweenness["robot1"] > betweenness["robot2"]
    assert betweenness["robot1"] > betweenness["robot3"]
    assert betweenness["robot1"] > betweenness["robot4"]


def test_betweenness_centrality_zero_for_a_fully_disconnected_set():
    betweenness = betweenness_centrality([], ["robot1", "robot2"])
    assert betweenness == {"robot1": 0.0, "robot2": 0.0}


def test_leadership_concentration_picks_highest_degree_hub():
    edges = [("robot1", "robot2"), ("robot1", "robot3"), ("robot1", "robot4")]
    centrality = degree_centrality(edges, ["robot1", "robot2", "robot3", "robot4"])
    result = leadership_concentration(centrality)
    assert result["hub_robot_id"] == "robot1"
    assert result["hub_degree_centrality"] > 0


def test_leadership_concentration_none_for_no_activity():
    centrality = degree_centrality([], ["robot1", "robot2"])
    result = leadership_concentration(centrality)
    assert result["hub_robot_id"] is None
