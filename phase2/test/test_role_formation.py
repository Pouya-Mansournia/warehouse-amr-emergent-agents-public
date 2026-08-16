from analysis.role_formation import (
    ROLE_HELPER,
    ROLE_HIGH_UTILIZATION,
    assign_candidate_roles,
    compute_role_persistence,
    compute_window_metrics,
)


def _state(t, robot_id, tasks, distance, utilization=0.5):
    return {
        "simulation_time": t, "robot_id": robot_id, "tasks_completed": tasks,
        "distance_traveled_m": distance, "utilization": utilization,
    }


def test_compute_window_metrics_computes_task_and_distance_deltas():
    states = [
        _state(0, "robot1", 0, 0.0),
        _state(100, "robot1", 2, 20.0),
        _state(0, "robot2", 0, 0.0),
        _state(100, "robot2", 1, 5.0),
    ]
    rows = compute_window_metrics(states, [], duration_sec=300, window_sec=300)
    r1 = next(r for r in rows if r["robot_id"] == "robot1")
    assert r1["tasks_completed"] == 2
    assert r1["mean_task_distance"] == 10.0


def test_assign_candidate_roles_picks_highest_utilization_worker():
    rows = [
        {"window": 0, "robot_id": "robot1", "tasks_completed": 5, "mean_task_distance": 3.0,
         "help_given": 0, "help_received": 0, "charger_visits": 0, "resource_conflicts_blocked": 0, "utilization": 0.9},
        {"window": 0, "robot_id": "robot2", "tasks_completed": 1, "mean_task_distance": 3.0,
         "help_given": 0, "help_received": 0, "charger_visits": 0, "resource_conflicts_blocked": 0, "utilization": 0.2},
    ]
    assignments = assign_candidate_roles(rows)
    assert assignments[0][ROLE_HIGH_UTILIZATION] == "robot1"


def test_assign_candidate_roles_never_fabricates_a_role_with_no_activity():
    rows = [
        {"window": 0, "robot_id": "robot1", "tasks_completed": 0, "mean_task_distance": None,
         "help_given": 0, "help_received": 0, "charger_visits": 0, "resource_conflicts_blocked": 0, "utilization": 0.0},
    ]
    assignments = assign_candidate_roles(rows)
    assert assignments[0][ROLE_HIGH_UTILIZATION] is None
    assert assignments[0][ROLE_HELPER] is None


def test_role_persistence_tracks_longest_streak():
    assignments = [
        {"window": 0, ROLE_HIGH_UTILIZATION: "robot1", "long_distance_specialist": None,
         "local_task_specialist": None, ROLE_HELPER: None},
        {"window": 1, ROLE_HIGH_UTILIZATION: "robot1", "long_distance_specialist": None,
         "local_task_specialist": None, ROLE_HELPER: None},
        {"window": 2, ROLE_HIGH_UTILIZATION: "robot2", "long_distance_specialist": None,
         "local_task_specialist": None, ROLE_HELPER: None},
        {"window": 3, ROLE_HIGH_UTILIZATION: "robot1", "long_distance_specialist": None,
         "local_task_specialist": None, ROLE_HELPER: None},
    ]
    persistence = compute_role_persistence(assignments)
    robot1 = persistence[ROLE_HIGH_UTILIZATION]["robot1"]
    assert robot1["windows_held"] == 3
    assert robot1["windows_role_assigned"] == 4
    assert robot1["longest_streak"] == 2
    assert robot1["persistence_ratio"] == 0.75


def test_role_persistence_ratio_uses_only_assigned_windows_as_denominator():
    # A run where the role is only ever assigned in 2 of many windows shouldn't
    # silently deflate persistence toward zero because of idle windows.
    assignments = [
        {"window": 0, ROLE_HELPER: None, ROLE_HIGH_UTILIZATION: None,
         "long_distance_specialist": None, "local_task_specialist": None},
        {"window": 1, ROLE_HELPER: "robot1", ROLE_HIGH_UTILIZATION: None,
         "long_distance_specialist": None, "local_task_specialist": None},
        {"window": 2, ROLE_HELPER: "robot1", ROLE_HIGH_UTILIZATION: None,
         "long_distance_specialist": None, "local_task_specialist": None},
    ]
    persistence = compute_role_persistence(assignments)
    assert persistence[ROLE_HELPER]["robot1"]["persistence_ratio"] == 1.0
