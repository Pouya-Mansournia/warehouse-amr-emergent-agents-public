import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from generate_report import (  # noqa: E402
    NOT_MEASURED,
    _mode_label,
    _resilience_mode_label,
    build_comparison_table,
    build_resilience_table,
    compute_metric_statistics,
    compute_paired_differences,
    render_detailed_statistics,
    render_markdown,
    write_resilience_csv,
)


def _summary(coordination, agent_backend=None, throughput=10.0, decision_latency=None, seed=None):
    s = {
        "coordination": coordination,
        "agent_backend": agent_backend,
        "random_seed": seed,
        "productivity": {"fleet_throughput_per_hour": throughput},
    }
    if decision_latency is not None:
        s["agent_decisions"] = {"mean_decision_latency_sec": decision_latency}
    else:
        s["agent_decisions"] = None
    return s


def test_compute_metric_statistics_empty_list_returns_none():
    assert compute_metric_statistics([]) is None


def test_compute_metric_statistics_single_value_has_no_stdev_or_ci():
    stats = compute_metric_statistics([5.0])
    assert stats["n"] == 1
    assert stats["mean"] == 5.0
    assert stats["stdev"] is None
    assert stats["ci95_low"] is None


def test_compute_metric_statistics_basic_shape():
    values = [10.0, 20.0, 30.0, 40.0, 50.0]
    stats = compute_metric_statistics(values)
    assert stats["n"] == 5
    assert stats["mean"] == 30.0
    assert stats["median"] == 30.0
    assert stats["min"] == 10.0
    assert stats["max"] == 50.0
    assert stats["stdev"] is not None
    assert stats["ci95_low"] <= stats["mean"] <= stats["ci95_high"]


def test_compute_metric_statistics_deterministic_across_calls():
    values = [1.0, 5.0, 3.0, 8.0, 2.0]
    a = compute_metric_statistics(values)
    b = compute_metric_statistics(values)
    assert a == b


def test_compute_paired_differences_matches_by_seed():
    a = [_summary("centralized", throughput=10.0, seed=1), _summary("centralized", throughput=20.0, seed=2)]
    b = [_summary("decentralized", "rule", throughput=15.0, seed=1), _summary("decentralized", "rule", throughput=18.0, seed=2)]
    stats = compute_paired_differences(a, b, lambda s: s["productivity"]["fleet_throughput_per_hour"])
    assert stats["n"] == 2
    assert stats["mean"] == round((5.0 + -2.0) / 2, 4)


def test_compute_paired_differences_none_when_no_shared_seeds():
    a = [_summary("centralized", throughput=10.0, seed=1)]
    b = [_summary("decentralized", "rule", throughput=15.0, seed=99)]
    assert compute_paired_differences(a, b, lambda s: s["productivity"]["fleet_throughput_per_hour"]) is None


def test_render_detailed_statistics_includes_paired_differences_when_seeds_shared():
    summaries = [
        _summary("centralized", throughput=10.0, seed=1),
        _summary("centralized", throughput=12.0, seed=2),
        _summary("decentralized", "rule", throughput=15.0, seed=1),
        _summary("decentralized", "rule", throughput=18.0, seed=2),
    ]
    text = render_detailed_statistics(summaries)
    assert "Detailed statistics" in text
    assert "Paired differences" in text


def test_render_detailed_statistics_empty_when_nothing_measured():
    assert render_detailed_statistics([]) == ""


def test_recovery_time_reported_when_fault_recovered():
    s = _summary("centralized")
    s["resilience"] = {"t_fail_sec": 30.0, "recovered": True, "recovery_time_sec": 12.5}
    row = build_comparison_table([s])[0]

    assert "12.5" in row["Recovery Time"]
    assert "1/1 recovered" in row["Recovery Time"]


def test_recovery_time_reports_honest_no_recovery():
    s = _summary("centralized")
    s["resilience"] = {"t_fail_sec": 30.0, "recovered": False, "recovery_time_sec": None}
    row = build_comparison_table([s])[0]

    assert row["Recovery Time"] == "no recovery (0/1)"


def test_energy_per_task_aggregated_from_per_robot():
    s = _summary("decentralized", "rule")
    s["per_robot"] = {
        "robot1": {"energy_per_task": 0.05},
        "robot2": {"energy_per_task": 0.03},
    }
    row = build_comparison_table([s])[0]

    assert "%SOC/task" in row["Energy/Task"]
    assert row["Energy/Task"] != NOT_MEASURED


def test_safety_events_reported_when_present():
    s = _summary("decentralized", "rule")
    s["safety"] = {"total_interventions": 3, "by_action_type": {"SLOWDOWN": 3}}
    row = build_comparison_table([s])[0]

    assert row["Collisions (safety events)"] != NOT_MEASURED
    assert "3" in row["Collisions (safety events)"]


def test_throughput_retention_reported_when_computed():
    s = _summary("centralized")
    s["resilience"] = {
        "t_fail_sec": 100.0, "recovered": True, "recovery_time_sec": 5.0,
        "throughput_retention_ratio": 0.5,
    }
    row = build_comparison_table([s])[0]

    assert row["Throughput Retention"] == "0.5"


def test_throughput_retention_not_measured_when_absent():
    s = _summary("centralized")
    s["resilience"] = {"t_fail_sec": 100.0, "recovered": False, "recovery_time_sec": None}
    row = build_comparison_table([s])[0]

    assert row["Throughput Retention"] == NOT_MEASURED


def test_mode_label_centralized():
    assert _mode_label("centralized", None) == "Centralized"


def test_mode_label_decentralized_rule():
    assert _mode_label("decentralized", "rule") == "Decentralized"


def test_mode_label_llm():
    assert _mode_label("decentralized", "llm") == "LLM"


def test_mode_label_hybrid():
    assert _mode_label("decentralized", "hybrid") == "Hybrid"


def test_build_comparison_table_groups_by_mode():
    summaries = [
        _summary("centralized", throughput=20.0),
        _summary("decentralized", "rule", throughput=15.0),
        _summary("decentralized", "rule", throughput=17.0),
    ]
    rows = build_comparison_table(summaries)
    labels = {r["Mode"] for r in rows}

    assert labels == {"Centralized", "Decentralized"}
    decentralized_row = next(r for r in rows if r["Mode"] == "Decentralized")
    assert decentralized_row["Runs"] == "2"


def test_unmeasured_columns_are_explicit_not_fabricated():
    rows = build_comparison_table([_summary("centralized", throughput=10.0)])
    row = rows[0]

    assert row["Recovery Time"] == NOT_MEASURED
    assert row["Energy/Task"] == NOT_MEASURED
    assert row["Collisions (safety events)"] == NOT_MEASURED


def test_decision_latency_only_reported_when_measured():
    rows = build_comparison_table([_summary("centralized", throughput=10.0)])
    assert rows[0]["Decision Latency (ms)"] == NOT_MEASURED

    rows = build_comparison_table(
        [_summary("decentralized", "llm", throughput=10.0, decision_latency=0.05)]
    )
    assert rows[0]["Decision Latency (ms)"] != NOT_MEASURED
    assert "50.0" in rows[0]["Decision Latency (ms)"]


def test_render_markdown_produces_a_pipe_table_with_header_and_rows():
    rows = build_comparison_table(
        [_summary("centralized", throughput=10.0), _summary("decentralized", "hybrid", throughput=12.0)]
    )
    table = render_markdown(rows)

    assert table.startswith("| Mode |")
    assert "Centralized" in table
    assert "Hybrid" in table
    assert table.count("\n") >= len(rows) + 2  # header + separator + one line per row


def test_render_markdown_handles_no_runs():
    assert "no successful runs" in render_markdown([])


def _fault_summary(coordination, agent_backend=None, recovered=True, recovery_time_sec=5.0, seed=None):
    s = _summary(coordination, agent_backend, seed=seed)
    s["resilience"] = {
        "t_fail_sec": 20.0,
        "recovered": recovered,
        "recovery_time_sec": recovery_time_sec if recovered else None,
    }
    return s


def test_resilience_mode_label_no_recovery():
    assert _resilience_mode_label("centralized", None) == "No Recovery"


def test_resilience_mode_label_deterministic():
    assert _resilience_mode_label("centralized_then_failover", "rule") == "Deterministic"


def test_resilience_mode_label_llm():
    assert _resilience_mode_label("centralized_then_failover", "llm") == "LLM"


def test_resilience_mode_label_hybrid():
    assert _resilience_mode_label("centralized_then_failover", "hybrid") == "Hybrid"


def test_resilience_mode_label_none_for_pure_decentralized():
    # No central coordinator was ever present to fail over from - this mode does not
    # belong in the B/C/D/E resilience comparison at all.
    assert _resilience_mode_label("decentralized", "rule") is None


def test_build_resilience_table_excludes_runs_without_fault_injection():
    summaries = [_summary("centralized"), _summary("decentralized", "rule")]
    assert build_resilience_table(summaries) == []


def test_build_resilience_table_excludes_pure_decentralized_even_with_resilience_block():
    s = _fault_summary("decentralized", "rule")
    assert build_resilience_table([s]) == []


def test_build_resilience_table_reports_recovery_fraction():
    summaries = [
        _fault_summary("centralized_then_failover", "rule", recovered=True, recovery_time_sec=10.0),
        _fault_summary("centralized_then_failover", "rule", recovered=True, recovery_time_sec=20.0),
        _fault_summary("centralized_then_failover", "rule", recovered=False),
    ]
    rows = build_resilience_table(summaries)
    assert len(rows) == 1
    assert rows[0]["Mode"] == "Deterministic"
    assert rows[0]["Recovery"] == "2/3"
    assert "15.0" in rows[0]["Recovery Time"]


def test_build_resilience_table_no_recovery_reports_honest_string():
    s = _fault_summary("centralized", None, recovered=False)
    rows = build_resilience_table([s])
    assert rows[0]["Mode"] == "No Recovery"
    assert rows[0]["Recovery"] == "0/1"
    assert rows[0]["Recovery Time"] == "no recovery"


def test_build_resilience_table_row_order_matches_bcde():
    summaries = [
        _fault_summary("centralized_then_failover", "hybrid"),
        _fault_summary("centralized_then_failover", "llm"),
        _fault_summary("centralized", None),
        _fault_summary("centralized_then_failover", "rule"),
    ]
    rows = build_resilience_table(summaries)
    assert [r["Mode"] for r in rows] == ["No Recovery", "Deterministic", "LLM", "Hybrid"]


def test_write_resilience_csv_writes_header_and_rows(tmp_path):
    rows = build_resilience_table(
        [_fault_summary("centralized", None, recovered=False)]
    )
    out_path = tmp_path / "resilience_summary.csv"
    write_resilience_csv(rows, out_path)

    with open(out_path, newline="") as f:
        reader = list(csv.DictReader(f))
    assert len(reader) == 1
    assert reader[0]["Mode"] == "No Recovery"
    assert reader[0]["Recovery"] == "0/1"


def test_write_resilience_csv_round_trips_plus_minus_sign_as_utf8(tmp_path):
    # Regression: write_resilience_csv previously opened without an explicit
    # encoding, which defaults to the platform locale (cp1252 on Windows) - a "±"
    # value written that way reads back as mojibake on a different platform/locale
    # (this bug was caught live when reading a WSL2-written CSV from Windows).
    rows = build_resilience_table([
        _fault_summary("centralized_then_failover", "rule", recovered=True, recovery_time_sec=10.0),
        _fault_summary("centralized_then_failover", "rule", recovered=True, recovery_time_sec=20.0),
    ])
    out_path = tmp_path / "resilience_summary.csv"
    write_resilience_csv(rows, out_path)

    raw_bytes = out_path.read_bytes()
    assert "±".encode("utf-8") in raw_bytes

    with open(out_path, newline="", encoding="utf-8") as f:
        reader = list(csv.DictReader(f))
    assert "±" in reader[0]["Recovery Time"]
