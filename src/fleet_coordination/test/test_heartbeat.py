from fleet_coordination.heartbeat import HeartbeatMonitor


def test_alive_before_any_heartbeat_ever_seen():
    monitor = HeartbeatMonitor(manager_timeout_sec=5.0)
    assert monitor.is_alive(now_monotonic=100.0) is True
    assert monitor.has_ever_seen_heartbeat is False


def test_alive_immediately_after_a_heartbeat():
    monitor = HeartbeatMonitor(manager_timeout_sec=5.0)
    monitor.on_heartbeat(now_monotonic=100.0)
    assert monitor.is_alive(now_monotonic=100.1) is True
    assert monitor.has_ever_seen_heartbeat is True


def test_alive_right_up_to_the_timeout_boundary():
    monitor = HeartbeatMonitor(manager_timeout_sec=5.0)
    monitor.on_heartbeat(now_monotonic=100.0)
    assert monitor.is_alive(now_monotonic=104.9) is True


def test_dead_once_timeout_elapsed():
    monitor = HeartbeatMonitor(manager_timeout_sec=5.0)
    monitor.on_heartbeat(now_monotonic=100.0)
    assert monitor.is_alive(now_monotonic=105.1) is False


def test_a_fresh_heartbeat_resets_the_timeout():
    monitor = HeartbeatMonitor(manager_timeout_sec=5.0)
    monitor.on_heartbeat(now_monotonic=100.0)
    monitor.on_heartbeat(now_monotonic=104.0)
    assert monitor.is_alive(now_monotonic=108.0) is True
    assert monitor.is_alive(now_monotonic=109.5) is False
