"""Unit tests for fleet_coordination.claim_book - pure Python, no rclpy/ROS graph."""
from fleet_coordination.claim_book import ClaimBook


def test_station_is_free_by_default():
    book = ClaimBook()
    assert book.is_free("output_station_1")
    assert book.winner_of("output_station_1") is None


def test_first_claim_wins_uncontested():
    book = ClaimBook()
    book.observe("output_station_1", "robot1", cost=0.5, release=False)
    assert book.winner_of("output_station_1") == "robot1"
    assert not book.is_free("output_station_1")


def test_lower_cost_claim_wins_regardless_of_arrival_order():
    book = ClaimBook()
    book.observe("output_station_1", "robot1", cost=0.9, release=False)
    book.observe("output_station_1", "robot2", cost=0.3, release=False)
    assert book.winner_of("output_station_1") == "robot2"


def test_lower_cost_claim_wins_even_if_it_arrives_first():
    book = ClaimBook()
    book.observe("output_station_1", "robot2", cost=0.3, release=False)
    book.observe("output_station_1", "robot1", cost=0.9, release=False)
    assert book.winner_of("output_station_1") == "robot2"


def test_exact_cost_tie_broken_by_robot_id_deterministically():
    book_a = ClaimBook()
    book_a.observe("output_station_1", "robot2", cost=0.5, release=False)
    book_a.observe("output_station_1", "robot1", cost=0.5, release=False)

    book_b = ClaimBook()  # same two claims, opposite arrival order
    book_b.observe("output_station_1", "robot1", cost=0.5, release=False)
    book_b.observe("output_station_1", "robot2", cost=0.5, release=False)

    # Both independent "agents" must converge on the identical winner regardless of
    # message arrival order - this is the whole point of the deterministic tiebreak.
    assert book_a.winner_of("output_station_1") == book_b.winner_of("output_station_1") == "robot1"


def test_release_by_holder_frees_the_station():
    book = ClaimBook()
    book.observe("output_station_1", "robot1", cost=0.5, release=False)
    book.observe("output_station_1", "robot1", cost=0.5, release=True)
    assert book.is_free("output_station_1")


def test_release_by_non_holder_is_ignored():
    book = ClaimBook()
    book.observe("output_station_1", "robot1", cost=0.5, release=False)
    book.observe("output_station_1", "robot2", cost=0.5, release=True)  # not the holder
    assert book.winner_of("output_station_1") == "robot1"


def test_free_stations_filters_candidates_by_current_belief():
    book = ClaimBook()
    candidates = [
        ("output_station_1", "output", 6.9, -9.0),
        ("output_station_2", "output", 6.9, -7.0),
    ]
    book.observe("output_station_1", "robot1", cost=0.5, release=False)
    free = book.free_stations(candidates)
    assert free == [("output_station_2", "output", 6.9, -7.0)]


def test_two_robots_never_converge_on_the_same_station_after_seeing_both_claims():
    """End-to-end sanity check of the design claim made in claim_book.py's module
    docstring: two independently-instantiated ClaimBooks that observe the SAME two
    competing claims (in either order) must agree on exactly one winner."""
    claims = [("robot1", 0.42), ("robot2", 0.41)]
    for ordering in (claims, list(reversed(claims))):
        book = ClaimBook()
        for robot_id, cost in ordering:
            book.observe("input_station_5", robot_id, cost, release=False)
        assert book.winner_of("input_station_5") == "robot2"
