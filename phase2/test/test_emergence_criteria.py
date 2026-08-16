from analysis.emergence_criteria import evaluate_pattern


def test_pattern_with_strong_evidence_gets_candidate_verdict():
    result = evaluate_pattern(
        pattern_name="helper_role",
        num_robots=4,
        per_seed_persistence_ratio=[0.8, 0.75, 0.9, 0.85, 0.7],
        per_seed_longest_streak=[5, 4, 6, 5, 3],
        memory_on_mean=0.8,
        memory_off_mean=0.4,
    )
    assert result["verdict"] == "candidate_emergent_collective_behavior"
    assert result["criteria"]["7_not_trivially_a_tie_break_artifact"]["supported"] is True


def test_pattern_indistinguishable_from_uniform_is_not_established():
    result = evaluate_pattern(
        pattern_name="random_role",
        num_robots=4,
        per_seed_persistence_ratio=[0.25, 0.26, 0.24, 0.25, 0.23],
        per_seed_longest_streak=[1, 1, 1, 1, 1],
        memory_on_mean=0.25,
        memory_off_mean=0.25,
    )
    assert result["verdict"] == "not_established"
    assert result["criteria"]["3_measurable_fleet_level_pattern"]["supported"] is False


def test_pattern_explained_equally_by_cost_formula_fails_criterion_7():
    # Strong pattern, but identical with and without memory - just the cost formula.
    result = evaluate_pattern(
        pattern_name="cost_formula_artifact",
        num_robots=4,
        per_seed_persistence_ratio=[0.9, 0.9, 0.9, 0.9, 0.9],
        per_seed_longest_streak=[10, 10, 10, 10, 10],
        memory_on_mean=0.9,
        memory_off_mean=0.89,
    )
    assert result["criteria"]["7_not_trivially_a_tie_break_artifact"]["supported"] is False
    assert result["verdict"] == "not_established"


def test_missing_memory_comparison_yields_insufficient_data():
    result = evaluate_pattern(
        pattern_name="no_ablation_available",
        num_robots=4,
        per_seed_persistence_ratio=[0.8, 0.8, 0.8, 0.8, 0.8],
        per_seed_longest_streak=[5, 5, 5, 5, 5],
        memory_on_mean=None,
        memory_off_mean=None,
    )
    assert result["verdict"] == "insufficient_data"
