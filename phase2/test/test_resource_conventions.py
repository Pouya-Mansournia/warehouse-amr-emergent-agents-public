from analysis.resource_conventions import (
    compute_charging_cycles,
    jain_fairness_index,
    lower_soc_priority_rate,
    round_robin_score,
    usage_share,
)


def _req(t, robot_id):
    return {"simulation_time": t, "from": robot_id, "interaction_type": "CHARGER_REQUEST", "result": None}


def _blocked(t, robot_id):
    return {"simulation_time": t, "from": robot_id, "interaction_type": "CHARGER_REQUEST", "result": "BLOCKED_NO_CANDIDATES"}


def _release(t, robot_id):
    return {"simulation_time": t, "from": robot_id, "interaction_type": "CHARGER_YIELD", "result": "RELEASED"}


def _state(t, robot_id, soc):
    return {"simulation_time": t, "robot_id": robot_id, "battery_soc": soc}


def test_compute_charging_cycles_pairs_request_with_release():
    interactions = [_req(10, "robot1"), _release(30, "robot1")]
    states = [_state(10, "robot1", 0.2)]
    cycles = compute_charging_cycles(interactions, states)
    assert len(cycles) == 1
    assert cycles[0]["duration_sec"] == 20
    assert cycles[0]["requester_battery_soc"] == 0.2


def test_usage_share_and_jain_fairness_index_perfectly_fair():
    cycles = [{"robot_id": "robot1"}, {"robot_id": "robot2"}]
    shares = usage_share(cycles)
    assert shares == {"robot1": 0.5, "robot2": 0.5}
    assert jain_fairness_index(cycles, ["robot1", "robot2"]) == 1.0


def test_jain_fairness_index_low_for_dominant_user():
    cycles = [{"robot_id": "robot1"}] * 9 + [{"robot_id": "robot2"}]
    idx = jain_fairness_index(cycles, ["robot1", "robot2"])
    assert idx < 0.7


def test_jain_fairness_index_none_when_no_cycles():
    assert jain_fairness_index([], ["robot1", "robot2"]) is None


def test_lower_soc_priority_rate_detects_a_real_priority_pattern():
    cycles = [
        {"robot_id": "robot1", "requester_battery_soc": 0.10, "concurrently_waiting_peers": {"robot2": 0.20}},
        {"robot_id": "robot1", "requester_battery_soc": 0.05, "concurrently_waiting_peers": {"robot2": 0.30}},
    ]
    result = lower_soc_priority_rate(cycles)
    assert result["rate"] == 1.0
    assert result["contested_events"] == 2


def test_lower_soc_priority_rate_none_when_never_contested():
    cycles = [{"robot_id": "robot1", "requester_battery_soc": 0.1, "concurrently_waiting_peers": {}}]
    assert lower_soc_priority_rate(cycles) is None


def test_round_robin_score_perfect_alternation():
    cycles = [
        {"robot_id": "robot1", "request_time": 0},
        {"robot_id": "robot2", "request_time": 10},
        {"robot_id": "robot1", "request_time": 20},
    ]
    assert round_robin_score(cycles) == 1.0


def test_round_robin_score_zero_for_same_robot_repeating():
    cycles = [
        {"robot_id": "robot1", "request_time": 0},
        {"robot_id": "robot1", "request_time": 10},
    ]
    assert round_robin_score(cycles) == 0.0
