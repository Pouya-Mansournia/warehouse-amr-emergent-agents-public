from analysis.peer_preference import (
    compute_pair_stats,
    compute_selection_probabilities,
    concentration_index,
)


def _event(from_r, to_r, itype, result=None, latency=None):
    return {"from": from_r, "to": to_r, "interaction_type": itype, "result": result, "response_latency": latency}


def test_compute_pair_stats_counts_bids_wins_rejections():
    events = [
        _event("robot2", "robot1", "HELP_OFFER"),  # robot2 bid on robot1's offer
        _event("robot3", "robot1", "HELP_OFFER"),
        _event("robot1", "robot2", "TASK_TRANSFER", result="SUCCESS", latency=5.0),
        _event("robot1", "robot3", "TASK_REJECT", result="REJECTED"),
    ]
    stats = compute_pair_stats(events)
    assert stats[("robot1", "robot2")]["bids"] == 1
    assert stats[("robot1", "robot2")]["wins"] == 1
    assert stats[("robot1", "robot2")]["acceptance_rate"] == 1.0
    assert stats[("robot1", "robot3")]["rejections"] == 1


def test_selection_probabilities_normalize_over_initiators_own_wins():
    events = [
        _event("robot1", "robot2", "TASK_TRANSFER", result="SUCCESS", latency=5.0),
        _event("robot1", "robot2", "TASK_TRANSFER", result="SUCCESS", latency=5.0),
        _event("robot1", "robot3", "TASK_TRANSFER", result="SUCCESS", latency=5.0),
    ]
    stats = compute_pair_stats(events)
    probs = compute_selection_probabilities(stats)
    assert probs["robot1"]["robot2"] == 0.6667
    assert probs["robot1"]["robot3"] == 0.3333


def test_concentration_index_is_higher_for_a_single_favored_peer():
    uniform = {"robot2": 0.5, "robot3": 0.5}
    concentrated = {"robot2": 0.9, "robot3": 0.1}
    assert concentration_index(concentrated) > concentration_index(uniform)


def test_concentration_index_empty_returns_zero():
    assert concentration_index({}) == 0.0
