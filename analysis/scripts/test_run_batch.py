import json
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_batch import build_run_command, parse_seeds, run_batch  # noqa: E402


def test_parse_seeds_range():
    assert parse_seeds("1:5") == [1, 2, 3, 4, 5]


def test_parse_seeds_single_value_range():
    assert parse_seeds("7:7") == [7]


def test_parse_seeds_rejects_backwards_range():
    with pytest.raises(ValueError):
        parse_seeds("5:1")


def test_parse_seeds_comma_list():
    assert parse_seeds("1,5,9") == [1, 5, 9]


def test_parse_seeds_single_value():
    assert parse_seeds("42") == [42]


def test_build_run_command_centralized_omits_agent_backend():
    cmd = build_run_command(
        experiment="central_failure",
        coordination="centralized",
        seed=1,
        num_robots=2,
        duration=60.0,
        agent_backend="hybrid",
        headless=True,
    )
    assert "--coordination" in cmd
    assert "centralized" in cmd
    assert "--agent-backend" not in cmd
    assert "--headless" in cmd


def test_build_run_command_decentralized_includes_agent_backend():
    cmd = build_run_command(
        experiment="central_failure",
        coordination="decentralized",
        seed=1,
        num_robots=2,
        duration=60.0,
        agent_backend="hybrid",
        headless=True,
    )
    assert "--agent-backend" in cmd
    idx = cmd.index("--agent-backend")
    assert cmd[idx + 1] == "hybrid"


def test_build_run_command_defaults_to_wall_time_source():
    cmd = build_run_command(
        experiment="central_failure", coordination="centralized", seed=1,
        num_robots=2, duration=60.0, agent_backend="rule", headless=True,
    )
    idx = cmd.index("--time-source")
    assert cmd[idx + 1] == "wall"


def test_build_run_command_passes_through_simulation_time_source():
    cmd = build_run_command(
        experiment="central_failure", coordination="centralized", seed=1,
        num_robots=2, duration=60.0, agent_backend="rule", headless=True,
        time_source="simulation",
    )
    idx = cmd.index("--time-source")
    assert cmd[idx + 1] == "simulation"


def test_run_batch_sweeps_every_mode_seed_combination_and_writes_manifest(tmp_path, monkeypatch):
    calls = []

    def fake_run(cmd, cwd, stdout, stderr):
        calls.append(cmd)
        stdout.write(
            f"[run_experiment] experiment directory: {tmp_path}/fake_run\n"
            "[run_experiment] done. duration=1.0s dir=...\n"
        )

        class _Result:
            returncode = 0

        return _Result()

    monkeypatch.setattr(subprocess, "run", fake_run)

    manifest = run_batch(
        experiment="test_experiment",
        coordination_modes=["centralized", "decentralized"],
        seeds=[1, 2],
        num_robots=2,
        duration=10.0,
        agent_backend="rule",
        headless=True,
        batch_dir=tmp_path / "batch",
    )

    assert len(calls) == 4  # 2 modes x 2 seeds
    assert manifest["succeeded"] == 4
    assert manifest["failed"] == 0
    assert len(manifest["runs"]) == 4
    for run in manifest["runs"]:
        assert run["run_dir"] == f"{tmp_path}/fake_run"

    manifest_path = tmp_path / "batch" / "batch_manifest.json"
    assert manifest_path.exists()
    assert json.loads(manifest_path.read_text())["succeeded"] == 4


def test_run_batch_continues_after_a_failed_run(tmp_path, monkeypatch):
    call_count = 0

    def fake_run(cmd, cwd, stdout, stderr):
        nonlocal call_count
        call_count += 1
        is_failure = call_count == 1

        class _Result:
            returncode = 1 if is_failure else 0

        if not is_failure:
            stdout.write(f"[run_experiment] experiment directory: {tmp_path}/ok\n")
        return _Result()

    monkeypatch.setattr(subprocess, "run", fake_run)

    manifest = run_batch(
        experiment="test_experiment",
        coordination_modes=["centralized"],
        seeds=[1, 2],
        num_robots=2,
        duration=10.0,
        agent_backend="rule",
        headless=True,
        batch_dir=tmp_path / "batch",
    )

    # Both seeds ran despite the first one failing - the batch didn't abort.
    assert call_count == 2
    assert manifest["succeeded"] == 1
    assert manifest["failed"] == 1


def test_build_run_command_includes_t_fail_for_centralized():
    cmd = build_run_command(
        experiment="central_failure",
        coordination="centralized",
        seed=1,
        num_robots=2,
        duration=300.0,
        agent_backend="rule",
        headless=True,
        t_fail=120.0,
    )
    assert "--t-fail" in cmd
    idx = cmd.index("--t-fail")
    assert cmd[idx + 1] == "120.0"


def test_build_run_command_omits_t_fail_for_decentralized():
    # run_experiment.py itself rejects --t-fail with decentralized (no central node to
    # kill) - the batch command builder must not even offer it, not just rely on
    # run_experiment.py to reject it downstream.
    cmd = build_run_command(
        experiment="central_failure",
        coordination="decentralized",
        seed=1,
        num_robots=2,
        duration=300.0,
        agent_backend="rule",
        headless=True,
        t_fail=120.0,
    )
    assert "--t-fail" not in cmd


def test_build_run_command_centralized_then_failover_includes_agent_backend():
    # Mode D/E as failover targets: once agents activate, they run the
    # same decentralized_agent loop as pure 'decentralized' - agent_backend matters here.
    cmd = build_run_command(
        experiment="central_failure",
        coordination="centralized_then_failover",
        seed=1,
        num_robots=2,
        duration=60.0,
        agent_backend="llm",
        headless=True,
    )
    assert "--agent-backend" in cmd
    idx = cmd.index("--agent-backend")
    assert cmd[idx + 1] == "llm"


def test_build_run_command_centralized_then_failover_includes_t_fail():
    cmd = build_run_command(
        experiment="central_failure",
        coordination="centralized_then_failover",
        seed=1,
        num_robots=2,
        duration=60.0,
        agent_backend="hybrid",
        headless=True,
        t_fail=20.0,
    )
    assert "--t-fail" in cmd
    idx = cmd.index("--t-fail")
    assert cmd[idx + 1] == "20.0"


def test_build_run_command_centralized_then_failover_includes_manager_timeout():
    cmd = build_run_command(
        experiment="central_failure",
        coordination="centralized_then_failover",
        seed=1,
        num_robots=2,
        duration=60.0,
        agent_backend="rule",
        headless=True,
        t_fail=20.0,
        manager_timeout_sec=5.0,
    )
    assert "--manager-timeout-sec" in cmd
    idx = cmd.index("--manager-timeout-sec")
    assert cmd[idx + 1] == "5.0"


def test_build_run_command_omits_manager_timeout_for_centralized():
    # --manager-timeout-sec only means anything once per-robot agents exist to time
    # out a heartbeat against - a plain 'centralized' run never reaches DORMANT state.
    cmd = build_run_command(
        experiment="central_failure",
        coordination="centralized",
        seed=1,
        num_robots=2,
        duration=60.0,
        agent_backend="rule",
        headless=True,
        t_fail=20.0,
        manager_timeout_sec=5.0,
    )
    assert "--manager-timeout-sec" not in cmd


def test_build_run_command_omits_manager_timeout_for_decentralized():
    cmd = build_run_command(
        experiment="central_failure",
        coordination="decentralized",
        seed=1,
        num_robots=2,
        duration=60.0,
        agent_backend="rule",
        headless=True,
        manager_timeout_sec=5.0,
    )
    assert "--manager-timeout-sec" not in cmd


def test_run_batch_records_manager_timeout_sec_in_manifest(tmp_path, monkeypatch):
    def fake_run(cmd, cwd, stdout, stderr):
        stdout.write(f"[run_experiment] experiment directory: {tmp_path}/fake_run\n")

        class _Result:
            returncode = 0

        return _Result()

    monkeypatch.setattr(subprocess, "run", fake_run)

    manifest = run_batch(
        experiment="test_experiment",
        coordination_modes=["centralized_then_failover"],
        seeds=[1],
        num_robots=2,
        duration=60.0,
        agent_backend="hybrid",
        headless=True,
        batch_dir=tmp_path / "batch",
        t_fail=20.0,
        manager_timeout_sec=5.0,
    )

    assert manifest["manager_timeout_sec"] == 5.0
    assert manifest["t_fail_sec"] == 20.0


def _fake_run_factory(tmp_path, run_dir_name="fake_run"):
    def fake_run(cmd, cwd, stdout, stderr):
        stdout.write(f"[run_experiment] experiment directory: {tmp_path}/{run_dir_name}\n")

        class _Result:
            returncode = 0

        return _Result()

    return fake_run


def test_run_batch_append_merges_runs_across_invocations(tmp_path, monkeypatch):
    batch_dir = tmp_path / "batch"

    monkeypatch.setattr(subprocess, "run", _fake_run_factory(tmp_path, "run_rule"))
    manifest1 = run_batch(
        experiment="resilience_pilot",
        coordination_modes=["centralized_then_failover"],
        seeds=[1],
        num_robots=2,
        duration=30.0,
        agent_backend="rule",
        headless=True,
        batch_dir=batch_dir,
        t_fail=10.0,
        manager_timeout_sec=3.0,
        append=True,
    )
    assert len(manifest1["runs"]) == 1

    monkeypatch.setattr(subprocess, "run", _fake_run_factory(tmp_path, "run_llm"))
    manifest2 = run_batch(
        experiment="resilience_pilot",
        coordination_modes=["centralized_then_failover"],
        seeds=[1],
        num_robots=2,
        duration=30.0,
        agent_backend="llm",
        headless=True,
        batch_dir=batch_dir,
        t_fail=10.0,
        manager_timeout_sec=3.0,
        append=True,
    )

    # Both invocations' runs are present, not overwritten.
    assert len(manifest2["runs"]) == 2
    assert manifest2["succeeded"] == 2
    assert len(manifest2["invocations"]) == 2
    assert manifest2["invocations"][0]["agent_backend"] == "rule"
    assert manifest2["invocations"][1]["agent_backend"] == "llm"
    # The manifest on disk reflects the merged state, not just the last call.
    on_disk = json.loads((batch_dir / "batch_manifest.json").read_text())
    assert len(on_disk["runs"]) == 2


def test_run_batch_without_append_overwrites_prior_manifest(tmp_path, monkeypatch):
    batch_dir = tmp_path / "batch"

    monkeypatch.setattr(subprocess, "run", _fake_run_factory(tmp_path, "run_a"))
    run_batch(
        experiment="resilience_pilot", coordination_modes=["centralized"], seeds=[1],
        num_robots=2, duration=30.0, agent_backend="rule", headless=True,
        batch_dir=batch_dir,
    )

    monkeypatch.setattr(subprocess, "run", _fake_run_factory(tmp_path, "run_b"))
    manifest2 = run_batch(
        experiment="resilience_pilot", coordination_modes=["centralized"], seeds=[2],
        num_robots=2, duration=30.0, agent_backend="rule", headless=True,
        batch_dir=batch_dir,
    )

    # Default (append=False) behavior is unchanged: only the second call's run exists.
    assert len(manifest2["runs"]) == 1
    assert manifest2["runs"][0]["seed"] == 2


def test_run_batch_log_filenames_are_backend_qualified(tmp_path, monkeypatch):
    # Without this, two --append invocations at the same (coordination, seed) but
    # different --agent-backend would silently clobber each other's log file.
    monkeypatch.setattr(subprocess, "run", _fake_run_factory(tmp_path))
    manifest = run_batch(
        experiment="resilience_pilot",
        coordination_modes=["centralized_then_failover"],
        seeds=[1],
        num_robots=2,
        duration=30.0,
        agent_backend="hybrid",
        headless=True,
        batch_dir=tmp_path / "batch",
        t_fail=10.0,
    )
    assert "centralized_then_failover_hybrid_seed1.log" in manifest["runs"][0]["log"]
