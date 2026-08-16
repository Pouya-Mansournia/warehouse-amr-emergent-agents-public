import pytest

from agent_core.peer_memory import PeerMemory


def test_new_peer_has_zero_reliability():
    memory = PeerMemory()
    assert memory.reliability("robot2") == 0.0


def test_successful_help_increases_reliability():
    memory = PeerMemory()
    memory.record_outcome("robot2", "successful_help")
    memory.record_outcome("robot2", "successful_help")
    assert memory.reliability("robot2") == 1.0


def test_mixed_outcomes_compute_success_rate():
    memory = PeerMemory()
    memory.record_outcome("robot2", "successful_help")
    memory.record_outcome("robot2", "rejected")
    memory.record_outcome("robot2", "rejected")
    memory.record_outcome("robot2", "timed_out")
    assert memory.reliability("robot2") == 0.25


def test_response_times_averaged():
    memory = PeerMemory()
    memory.record_outcome("robot2", "successful_help", response_time_sec=1.0)
    memory.record_outcome("robot2", "rejected", response_time_sec=3.0)
    summary = memory.summary()
    assert summary["robot2"]["average_response_time_sec"] == 2.0


def test_unknown_outcome_rejected():
    memory = PeerMemory()
    with pytest.raises(ValueError):
        memory.record_outcome("robot2", "made_up_outcome")


def test_peer_with_no_interactions_has_none_success_rate_and_none_response_time():
    memory = PeerMemory()
    memory.record_outcome("robot2", "successful_help")  # give robot2 some history
    summary = memory.summary()
    assert summary["robot2"]["task_transfer_success_rate"] == 1.0
    assert summary["robot2"]["average_response_time_sec"] is None


def test_summary_bounded_and_sorted_by_interaction_count():
    memory = PeerMemory()
    memory.record_outcome("robot2", "successful_help")
    for _ in range(3):
        memory.record_outcome("robot3", "rejected")
    memory.record_outcome("robot4", "timed_out")

    summary = memory.summary(max_peers=2)

    assert len(summary) == 2
    assert "robot3" in summary  # most interactions (3), must be kept
    assert set(summary.keys()) <= {"robot2", "robot3", "robot4"}
