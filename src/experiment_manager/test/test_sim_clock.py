from experiment_manager.sim_clock import ClockTracker


def test_clock_tracker_has_no_clock_before_first_message():
    tracker = ClockTracker()
    assert tracker.has_clock is False
    assert tracker.sim_time_sec is None


def test_clock_tracker_reports_time_from_sec_and_nanosec():
    tracker = ClockTracker()
    tracker.on_clock(sec=100, nanosec=500_000_000)
    assert tracker.has_clock is True
    assert tracker.sim_time_sec == 100.5


def test_clock_tracker_tracks_only_the_most_recent_message():
    tracker = ClockTracker()
    tracker.on_clock(sec=10, nanosec=0)
    tracker.on_clock(sec=20, nanosec=0)
    assert tracker.sim_time_sec == 20.0


def test_clock_tracker_correct_regardless_of_advance_rate():
    """The whole point of simulation-time tracking: elapsed sim-time between two
    /clock messages must be exactly what the messages say, independent of how much
    real wall-clock time passed between them - a fast (0.5x realtime) or slow
    (0.05x realtime) host must both compute the identical simulated-duration answer
    from the same two Clock messages."""
    tracker = ClockTracker()
    tracker.on_clock(sec=0, nanosec=0)
    start = tracker.sim_time_sec
    tracker.on_clock(sec=100, nanosec=0)
    elapsed = tracker.sim_time_sec - start
    assert elapsed == 100.0
