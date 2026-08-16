import csv
import json

from experiment_manager.summarize_run import build_summary, generate_plots, write_summary


def _write_metadata(run_dir, **overrides):
    metadata = {
        "experiment_mode": "phase10_test",
        "coordination": "decentralized",
        "agent_backend": "rule",
        "robot_count": 2,
        "random_seed": 5,
        "duration_sec": 100.0,
    }
    metadata.update(overrides)
    (run_dir / "metadata.json").write_text(json.dumps(metadata))
    return metadata


def _write_events(run_dir, events):
    with open(run_dir / "events.jsonl", "w") as f:
        for e in events:
            f.write(json.dumps(e) + "\n")


def _write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def test_build_summary_counts_succeeded_tasks_per_robot(tmp_path):
    _write_metadata(tmp_path)
    _write_events(
        tmp_path,
        [
            {"event": "EXPERIMENT_LOGGER_STARTED", "timestamp": 0.0, "robot_id": None},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 1.0, "robot_id": "robot1", "status": "EXECUTING"},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 11.0, "robot_id": "robot1", "status": "SUCCEEDED"},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 20.0, "robot_id": "robot2", "status": "EXECUTING"},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 25.0, "robot_id": "robot2", "status": "ABORTED"},
        ],
    )
    summary = build_summary(tmp_path)

    assert summary["productivity"]["tasks_completed"] == 1
    assert summary["per_robot"]["robot1"]["tasks_completed"] == 1
    assert summary["per_robot"]["robot2"]["tasks_completed"] == 0
    # Only SUCCEEDED goals count toward completion time - robot2's ABORTED attempt is
    # correctly excluded (it didn't complete a task).
    assert summary["productivity"]["mean_task_duration_sec"] == 10.0


def test_build_summary_computes_fleet_throughput_per_hour(tmp_path):
    _write_metadata(tmp_path, duration_sec=36.0)
    _write_events(
        tmp_path,
        [
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 0.0, "robot_id": "robot1", "status": "SUCCEEDED"},
        ],
    )
    summary = build_summary(tmp_path)
    # 1 task in 36s -> 100 tasks/hour
    assert summary["productivity"]["fleet_throughput_per_hour"] == 100.0


def test_build_summary_omits_agent_decisions_when_absent(tmp_path):
    _write_metadata(tmp_path, robot_count=1)
    _write_events(tmp_path, [])
    summary = build_summary(tmp_path)

    assert summary["agent_decisions"] is None


def test_build_summary_computes_decision_latency_stats(tmp_path):
    _write_metadata(tmp_path, robot_count=1)
    _write_events(tmp_path, [])
    with open(tmp_path / "agent_decisions.jsonl", "w") as f:
        for latency, backend in [(0.001, "rule"), (0.5, "llm"), (0.002, "rule")]:
            f.write(
                json.dumps(
                    {
                        "timestamp": 0.0,
                        "robot_id": "robot1",
                        "backend": backend,
                        "action": "BID_FOR_TASK",
                        "decision_latency_sec": latency,
                    }
                )
                + "\n"
            )
    summary = build_summary(tmp_path)

    assert summary["agent_decisions"]["decisions"] == 3
    assert summary["agent_decisions"]["backends"] == ["llm", "rule"]
    assert summary["agent_decisions"]["mean_decision_latency_sec"] == round(
        (0.001 + 0.5 + 0.002) / 3, 6
    )


def test_build_summary_computes_distance_and_min_battery(tmp_path):
    _write_metadata(tmp_path, robot_count=1)
    _write_events(tmp_path, [])
    _write_csv(
        tmp_path / "robot_robot1.csv",
        ["timestamp", "x", "y", "yaw_quat_z", "yaw_quat_w", "v", "w"],
        [[0.0, 0.0, 0.0, 0, 1, 0, 0], [1.0, 3.0, 4.0, 0, 1, 0, 0]],  # 3-4-5 triangle
    )
    _write_csv(
        tmp_path / "health_robot1.csv",
        ["timestamp", "battery_soc"] + [f"col{i}" for i in range(17)],
        [[0.0, 0.9] + [0] * 17, [1.0, 0.4] + [0] * 17],
    )
    summary = build_summary(tmp_path)

    assert summary["per_robot"]["robot1"]["distance_m"] == 5.0
    assert summary["per_robot"]["robot1"]["min_battery_soc"] == 0.4


def test_build_summary_omits_negotiation_and_faults_when_absent(tmp_path):
    _write_metadata(tmp_path, robot_count=1)
    _write_events(tmp_path, [])
    summary = build_summary(tmp_path)

    assert summary["negotiation"] is None
    assert summary["faults"] is None


def test_build_summary_includes_negotiation_stats_when_present(tmp_path):
    _write_metadata(tmp_path, robot_count=2)
    _write_events(tmp_path, [])
    _write_csv(
        tmp_path / "negotiations.csv",
        ["timestamp", "conversation_id", "performative", "from_robot_id", "to_robot_id",
         "station_name", "side", "x", "y", "cost", "reason_code"],
        [
            [0.0, "robot1-0", "OFFER", "robot1", "", "s1", "output", 1.0, 1.0, 0.1, "LOW_BATTERY"],
            [1.0, "robot1-0", "BID", "robot2", "robot1", "s1", "output", 1.0, 1.0, 0.2, "LOW_BATTERY"],
            [2.0, "robot1-0", "COMMIT", "robot1", "robot2", "s1", "output", 1.0, 1.0, 0.2, "LOW_BATTERY"],
        ],
    )
    summary = build_summary(tmp_path)

    assert summary["negotiation"] == {
        "conversations": 1,
        "offers": 1,
        "commits": 1,
        "timeouts": 0,
    }


def test_write_summary_writes_summary_json(tmp_path):
    _write_metadata(tmp_path, robot_count=1)
    _write_events(tmp_path, [])
    out_path = write_summary(tmp_path)

    assert out_path == tmp_path / "summary.json"
    assert out_path.exists()
    loaded = json.loads(out_path.read_text())
    assert loaded["experiment_mode"] == "phase10_test"


def test_generate_plots_only_produces_trajectories_when_thats_all_that_exists(tmp_path):
    _write_metadata(tmp_path, robot_count=1)
    _write_events(tmp_path, [])
    _write_csv(
        tmp_path / "robot_robot1.csv",
        ["timestamp", "x", "y", "yaw_quat_z", "yaw_quat_w", "v", "w"],
        [[0.0, 0.0, 0.0, 0, 1, 0, 0], [1.0, 1.0, 1.0, 0, 1, 0, 0]],
    )
    plots = generate_plots(tmp_path)
    names = {p.name for p in plots}

    assert names == {"trajectories.png"}
    for p in plots:
        assert p.exists()
        assert p.stat().st_size > 0


def test_generate_plots_skips_battery_plot_when_health_csv_has_no_rows(tmp_path):
    # A health_<ns>.csv file that exists but has only a header (e.g. a run that ended
    # before health_monitor's ~1Hz sample period produced a single row) must NOT
    # produce a battery_soc.png with no actual data on it.
    _write_metadata(tmp_path, robot_count=1)
    _write_events(tmp_path, [])
    _write_csv(
        tmp_path / "robot_robot1.csv",
        ["timestamp", "x", "y", "yaw_quat_z", "yaw_quat_w", "v", "w"],
        [[0.0, 0.0, 0.0, 0, 1, 0, 0]],
    )
    _write_csv(
        tmp_path / "health_robot1.csv",
        ["timestamp", "battery_soc"] + [f"col{i}" for i in range(17)],
        [],  # header only, zero data rows
    )
    plots = generate_plots(tmp_path)
    names = {p.name for p in plots}

    assert "battery_soc.png" not in names
    assert "trajectories.png" in names


def test_generate_plots_produces_battery_and_throughput_when_data_present(tmp_path):
    _write_metadata(tmp_path, robot_count=1)
    _write_events(
        tmp_path,
        [
            {"event": "EXPERIMENT_LOGGER_STARTED", "timestamp": 0.0, "robot_id": None},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 5.0, "robot_id": "robot1", "status": "SUCCEEDED"},
        ],
    )
    _write_csv(
        tmp_path / "robot_robot1.csv",
        ["timestamp", "x", "y", "yaw_quat_z", "yaw_quat_w", "v", "w"],
        [[0.0, 0.0, 0.0, 0, 1, 0, 0]],
    )
    _write_csv(
        tmp_path / "health_robot1.csv",
        ["timestamp", "battery_soc"] + [f"col{i}" for i in range(17)],
        [[0.0, 1.0] + [0] * 17],
    )
    plots = generate_plots(tmp_path)
    names = {p.name for p in plots}

    assert "battery_soc.png" in names
    assert "throughput.png" in names


def _write_decision(f, **overrides):
    record = {
        "timestamp": 0.0, "robot_id": "robot1", "backend": "rule",
        "action": "BID_FOR_TASK", "decision_latency_sec": 0.001,
        "has_llm_meta": False, "provider": None, "model": None,
        "prompt_tokens": None, "completion_tokens": None,
        "schema_valid": None, "safety_valid": None,
        "fallback_used": None, "retry_count": None,
    }
    record.update(overrides)
    f.write(json.dumps(record) + "\n")


def test_build_summary_omits_llm_block_when_no_llm_decisions(tmp_path):
    _write_metadata(tmp_path, robot_count=1)
    _write_events(tmp_path, [])
    with open(tmp_path / "agent_decisions.jsonl", "w") as f:
        _write_decision(f, backend="rule")
    summary = build_summary(tmp_path)

    assert "llm" not in summary["agent_decisions"]


def test_build_summary_computes_llm_aggregate_metrics(tmp_path):
    _write_metadata(tmp_path, robot_count=1, agent_backend="llm")
    _write_events(tmp_path, [])
    with open(tmp_path / "agent_decisions.jsonl", "w") as f:
        _write_decision(
            f, backend="llm", decision_latency_sec=0.1, has_llm_meta=True,
            provider="ollama", model="llama3.2:1b", prompt_tokens=100,
            completion_tokens=20, schema_valid=True, safety_valid=True,
            fallback_used=False, retry_count=0,
        )
        _write_decision(
            f, backend="llm", decision_latency_sec=0.2, has_llm_meta=True,
            provider="ollama", model="llama3.2:1b", prompt_tokens=110,
            completion_tokens=None, schema_valid=False, safety_valid=None,
            fallback_used=True, retry_count=2,
        )
        _write_decision(
            f, backend="llm", decision_latency_sec=0.15, has_llm_meta=True,
            provider="ollama", model="llama3.2:1b", prompt_tokens=105,
            completion_tokens=15, schema_valid=True, safety_valid=False,
            fallback_used=True, retry_count=1,
        )
    summary = build_summary(tmp_path)
    llm = summary["agent_decisions"]["llm"]

    assert llm["llm_requests"] == 3
    assert llm["llm_successes"] == 1
    assert llm["llm_schema_failures"] == 1
    assert llm["llm_safety_rejections"] == 1
    assert llm["llm_fallbacks"] == 2
    assert llm["llm_retries"] == 3
    assert llm["total_prompt_tokens"] == 315
    assert llm["total_completion_tokens"] == 35  # None dropped, not counted as 0


def test_build_summary_computes_hybrid_escalation_metrics(tmp_path):
    _write_metadata(tmp_path, robot_count=1, agent_backend="hybrid")
    _write_events(tmp_path, [])
    with open(tmp_path / "agent_decisions.jsonl", "w") as f:
        _write_decision(f, backend="hybrid", has_llm_meta=False)
        _write_decision(f, backend="hybrid", has_llm_meta=False)
        _write_decision(f, backend="hybrid", has_llm_meta=True, schema_valid=True, safety_valid=True)
    summary = build_summary(tmp_path)
    hybrid = summary["agent_decisions"]["hybrid"]

    assert hybrid["deterministic_decisions"] == 2
    assert hybrid["llm_escalations"] == 1
    assert hybrid["llm_escalation_rate"] == round(1 / 3, 4)


def test_build_summary_omits_resilience_when_no_fault_injected(tmp_path):
    _write_metadata(tmp_path, robot_count=1, t_fail_sec=None)
    _write_events(tmp_path, [])
    summary = build_summary(tmp_path)

    assert summary["resilience"] is None


def test_build_summary_reports_recovery_time_when_fault_recovered(tmp_path):
    _write_metadata(tmp_path, robot_count=1, t_fail_sec=10.0)
    _write_events(
        tmp_path,
        [
            {"event": "EXPERIMENT_LOGGER_STARTED", "timestamp": 100.0, "robot_id": None},
            # fault at absolute t=110.0; a NEW task assigned (EXECUTING) at t=112.0
            # after the fault, then succeeds at t=125.0 -> real recovery = 15.0s
            # (from fault, not from assignment).
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 112.0, "robot_id": "robot1", "status": "EXECUTING"},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 125.0, "robot_id": "robot1", "status": "SUCCEEDED"},
        ],
    )
    summary = build_summary(tmp_path)

    assert summary["resilience"]["recovered"] is True
    assert summary["resilience"]["recovery_time_sec"] == 15.0


def test_build_summary_resilience_populated_when_logger_started_sim_time_is_null(tmp_path):
    """Regression test for a real bug found live: on --time-source simulation runs,
    EXPERIMENT_LOGGER_STARTED's own simulation_time is frequently null (its /clock
    subscription hasn't received a first message yet) - `next(...)` found the event
    and returned that None value, which was wrongly treated as "no such event," so
    the entire resilience block silently stayed None even with a real, confirmed
    recovery elsewhere in the same run's events.jsonl."""
    _write_metadata(
        tmp_path, robot_count=1, t_fail_sec=8.0, time_source="simulation",
        simulation_duration_sec=45.0,
    )
    _write_events(
        tmp_path,
        [
            {
                "event": "EXPERIMENT_LOGGER_STARTED", "timestamp": 100.0,
                "simulation_time": None, "robot_id": None,
            },
            {
                "event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 200.0,
                "simulation_time": 17.6, "robot_id": "robot1", "status": "EXECUTING",
            },
            {
                "event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 300.0,
                "simulation_time": 35.6, "robot_id": "robot1", "status": "SUCCEEDED",
            },
        ],
    )
    summary = build_summary(tmp_path)

    assert summary["resilience"] is not None
    assert summary["resilience"]["recovered"] is True
    assert summary["resilience"]["recovery_time_sec"] == 27.6  # 35.6 - 8.0


def test_build_summary_reports_honest_no_recovery(tmp_path):
    _write_metadata(tmp_path, robot_count=1, t_fail_sec=10.0)
    _write_events(
        tmp_path,
        [
            {"event": "EXPERIMENT_LOGGER_STARTED", "timestamp": 100.0, "robot_id": None},
            # only success is BEFORE the fault - nothing ever succeeds again afterward.
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 103.0, "robot_id": "robot1", "status": "EXECUTING"},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 105.0, "robot_id": "robot1", "status": "SUCCEEDED"},
        ],
    )
    summary = build_summary(tmp_path)

    assert summary["resilience"]["recovered"] is False
    assert summary["resilience"]["recovery_time_sec"] is None


def test_build_summary_does_not_count_a_task_assigned_before_fault_as_recovery(tmp_path):
    """Regression test for the real false-positive found live during failover
    validation: a task ASSIGNED before the fault that simply finishes late (already
    in flight - Nav2 doesn't need the central node to complete an in-progress goal)
    must NOT count as recovery, even though it SUCCEEDS after t_fail_sec."""
    _write_metadata(tmp_path, robot_count=1, t_fail_sec=10.0)
    _write_events(
        tmp_path,
        [
            {"event": "EXPERIMENT_LOGGER_STARTED", "timestamp": 100.0, "robot_id": None},
            # assigned BEFORE the fault (t=103, fault at t=110), but only succeeds
            # well after it (t=125) - purely late-finishing pre-existing work.
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 103.0, "robot_id": "robot1", "status": "EXECUTING"},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 125.0, "robot_id": "robot1", "status": "SUCCEEDED"},
        ],
    )
    summary = build_summary(tmp_path)

    assert summary["resilience"]["recovered"] is False
    assert summary["resilience"]["recovery_time_sec"] is None


def test_build_summary_pre_post_failure_throughput(tmp_path):
    _write_metadata(tmp_path, robot_count=1, t_fail_sec=100.0, duration_sec=200.0)
    _write_events(
        tmp_path,
        [
            {"event": "EXPERIMENT_LOGGER_STARTED", "timestamp": 0.0, "robot_id": None},
            # 2 tasks assigned+completed entirely within [0, 100) - pre-failure.
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 0.0, "robot_id": "robot1", "status": "EXECUTING"},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 20.0, "robot_id": "robot1", "status": "SUCCEEDED"},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 20.0, "robot_id": "robot1", "status": "EXECUTING"},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 40.0, "robot_id": "robot1", "status": "SUCCEEDED"},
            # 1 task assigned+completed entirely within [100, 200) - post-failure.
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 120.0, "robot_id": "robot1", "status": "EXECUTING"},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 140.0, "robot_id": "robot1", "status": "SUCCEEDED"},
        ],
    )
    summary = build_summary(tmp_path)
    r = summary["resilience"]

    # pre-failure window: 100s -> 2 tasks -> 72.0 tasks/hr; post-failure window:
    # 100s -> 1 task -> 36.0 tasks/hr; retention = 36.0 / 72.0 = 0.5.
    assert r["pre_failure_tasks_completed"] == 2
    assert r["post_failure_tasks_completed"] == 1
    assert r["pre_failure_throughput_per_hour"] == 72.0
    assert r["post_failure_throughput_per_hour"] == 36.0
    assert r["throughput_retention_ratio"] == 0.5


def test_build_summary_throughput_retention_none_when_no_pre_failure_activity(tmp_path):
    _write_metadata(tmp_path, robot_count=1, t_fail_sec=10.0, duration_sec=20.0)
    _write_events(
        tmp_path,
        [
            {"event": "EXPERIMENT_LOGGER_STARTED", "timestamp": 0.0, "robot_id": None},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 12.0, "robot_id": "robot1", "status": "EXECUTING"},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 15.0, "robot_id": "robot1", "status": "SUCCEEDED"},
        ],
    )
    summary = build_summary(tmp_path)
    r = summary["resilience"]

    assert r["pre_failure_throughput_per_hour"] == 0.0
    assert r["throughput_retention_ratio"] is None


def test_build_summary_resilience_uses_simulation_time_when_time_source_simulation(tmp_path):
    """Regression test for the clock-basis bug: when time_source=simulation,
    t_fail_sec is in SIMULATED seconds and must be compared against simulation_time,
    not the wall-clock timestamp (which can be a wildly different scale)."""
    _write_metadata(
        tmp_path, robot_count=1, t_fail_sec=10.0, time_source="simulation",
        simulation_duration_sec=20.0,
    )
    _write_events(
        tmp_path,
        [
            {
                "event": "EXPERIMENT_LOGGER_STARTED", "timestamp": 100000.0,
                "simulation_time": 0.0, "robot_id": None,
            },
            # Real wall-clock gap is huge (this run is very slow: realtime factor
            # ~0.05), but in SIMULATED time this task is assigned/completed entirely
            # after t_fail_sec=10 - must count as recovered.
            {
                "event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 100300.0,
                "simulation_time": 15.0, "robot_id": "robot1", "status": "EXECUTING",
            },
            {
                "event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 100320.0,
                "simulation_time": 17.0, "robot_id": "robot1", "status": "SUCCEEDED",
            },
        ],
    )
    summary = build_summary(tmp_path)

    assert summary["resilience"]["recovered"] is True
    assert summary["resilience"]["recovery_time_sec"] == 7.0
    # Regression test for the task-duration clock-field bug (found during the
    # paper-oriented gap analysis): mean_task_duration_sec must use simulation_time
    # (17.0 - 15.0 = 2.0) on a --time-source simulation run, not the wall-clock gap
    # (100320.0 - 100300.0 = 20.0).
    assert summary["productivity"]["mean_task_duration_sec"] == 2.0


def test_build_summary_computes_energy_per_task(tmp_path):
    _write_metadata(tmp_path, robot_count=1)
    _write_events(
        tmp_path,
        [{"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 1.0, "robot_id": "robot1", "status": "SUCCEEDED"}],
    )
    _write_csv(
        tmp_path / "health_robot1.csv",
        ["timestamp", "battery_soc"] + [f"col{i}" for i in range(17)],
        [[0.0, 0.9] + [0] * 17, [10.0, 0.7] + [0] * 17],
    )
    summary = build_summary(tmp_path)

    assert summary["per_robot"]["robot1"]["battery_soc_start"] == 0.9
    assert summary["per_robot"]["robot1"]["battery_soc_end"] == 0.7
    assert summary["per_robot"]["robot1"]["energy_per_task"] == 0.2  # (0.9-0.7)/1 task


def test_build_summary_energy_per_task_none_when_no_tasks_completed(tmp_path):
    _write_metadata(tmp_path, robot_count=1)
    _write_events(tmp_path, [])
    _write_csv(
        tmp_path / "health_robot1.csv",
        ["timestamp", "battery_soc"] + [f"col{i}" for i in range(17)],
        [[0.0, 0.9] + [0] * 17],
    )
    summary = build_summary(tmp_path)

    assert summary["per_robot"]["robot1"]["energy_per_task"] is None


def test_build_summary_omits_safety_when_absent(tmp_path):
    _write_metadata(tmp_path, robot_count=1)
    _write_events(tmp_path, [])
    summary = build_summary(tmp_path)

    assert summary["safety"] is None


def test_build_summary_reports_safety_interventions_when_present(tmp_path):
    _write_metadata(tmp_path, robot_count=1)
    _write_events(tmp_path, [])
    _write_csv(
        tmp_path / "safety_events.csv",
        ["timestamp", "robot_id", "action_type", "polygon_name"],
        [
            [0.0, "robot1", "SLOWDOWN", "front"],
            [1.0, "robot1", "STOP", "front"],
            [2.0, "robot1", "SLOWDOWN", "front"],
        ],
    )
    summary = build_summary(tmp_path)

    assert summary["safety"]["total_interventions"] == 3
    assert summary["safety"]["by_action_type"] == {"SLOWDOWN": 2, "STOP": 1}


def test_build_summary_omits_degradation_when_no_degradation_fault(tmp_path):
    _write_metadata(tmp_path, robot_count=2)
    _write_events(tmp_path, [])
    _write_csv(
        tmp_path / "faults.csv",
        ["timestamp", "simulation_time", "fault_id", "robot_id", "fault_type", "severity", "clear"],
        [[5.0, 5.0, "f1", "robot1", "LOW_BATTERY", 0.8, "False"]],
    )
    summary = build_summary(tmp_path)

    assert summary["degradation"] is None


def test_build_summary_computes_workload_redistribution_after_degradation_onset(tmp_path):
    _write_metadata(tmp_path, robot_count=2, duration_sec=100.0)
    _write_events(
        tmp_path,
        [
            {"event": "EXPERIMENT_LOGGER_STARTED", "timestamp": 0.0, "robot_id": None},
            # Pre-onset (onset at t=10): robot1 completes 2, robot2 completes 0.
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 1.0, "robot_id": "robot1", "status": "EXECUTING"},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 2.0, "robot_id": "robot1", "status": "SUCCEEDED"},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 3.0, "robot_id": "robot1", "status": "EXECUTING"},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 4.0, "robot_id": "robot1", "status": "SUCCEEDED"},
            # Post-onset: robot2 picks up the workload instead (robot1 degraded).
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 20.0, "robot_id": "robot2", "status": "EXECUTING"},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 21.0, "robot_id": "robot2", "status": "SUCCEEDED"},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 22.0, "robot_id": "robot2", "status": "EXECUTING"},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 23.0, "robot_id": "robot2", "status": "SUCCEEDED"},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 24.0, "robot_id": "robot2", "status": "EXECUTING"},
            {"event": "NAV_GOAL_STATUS_CHANGED", "timestamp": 25.0, "robot_id": "robot2", "status": "SUCCEEDED"},
        ],
    )
    _write_csv(
        tmp_path / "faults.csv",
        ["timestamp", "simulation_time", "fault_id", "robot_id", "fault_type", "severity", "clear"],
        [[10.0, 10.0, "f1", "robot1", "ACCELERATED_BATTERY_DISCHARGE", 1.0, "False"]],
    )
    summary = build_summary(tmp_path)

    assert summary["degradation"]["degraded_robots"] == ["robot1"]
    assert summary["degradation"]["onset_sec"] == 10.0
    assert summary["degradation"]["per_robot"]["robot1"]["tasks_completed_pre_onset"] == 2
    assert summary["degradation"]["per_robot"]["robot1"]["tasks_completed_post_onset"] == 0
    assert summary["degradation"]["per_robot"]["robot2"]["tasks_completed_pre_onset"] == 0
    assert summary["degradation"]["per_robot"]["robot2"]["tasks_completed_post_onset"] == 3
    # Workload share shifted entirely to the non-degraded robot after onset.
    assert summary["degradation"]["per_robot"]["robot1"]["workload_share_pre_onset"] == 1.0
    assert summary["degradation"]["per_robot"]["robot1"]["workload_share_post_onset"] == 0.0
    assert summary["degradation"]["per_robot"]["robot2"]["workload_share_post_onset"] == 1.0
