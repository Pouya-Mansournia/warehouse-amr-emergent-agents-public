from fleet_coordination.negotiation import (
    ACCEPT,
    BID,
    COMMIT,
    COUNTER_PROPOSAL,
    OFFER,
    REJECT,
    TIMEOUT,
    Conversation,
)


def _conv(**overrides):
    defaults = dict(
        conversation_id="robot1-0",
        initiator="robot1",
        station_name="output_station_5",
        side="output",
        x=6.9,
        y=-1.0,
        reason_code="LOW_BATTERY",
        deadline=100.0,
    )
    defaults.update(overrides)
    return Conversation(**defaults)


def test_no_bids_selects_no_winner():
    conv = _conv()
    assert conv.select_winner() is None


def test_single_bid_wins():
    conv = _conv()
    conv.record_bid("robot2", BID, 0.4)
    assert conv.select_winner() == "robot2"


def test_lowest_cost_bid_wins():
    conv = _conv()
    conv.record_bid("robot2", BID, 0.6)
    conv.record_bid("robot3", BID, 0.3)
    assert conv.select_winner() == "robot3"


def test_exact_tie_broken_by_robot_id():
    conv = _conv()
    conv.record_bid("robot3", BID, 0.5)
    conv.record_bid("robot2", BID, 0.5)
    assert conv.select_winner() == "robot2"


def test_counter_proposal_treated_like_bid_for_selection():
    conv = _conv()
    conv.record_bid("robot2", BID, 0.5)
    conv.record_bid("robot3", COUNTER_PROPOSAL, 0.2)
    assert conv.select_winner() == "robot3"


def test_only_the_best_bid_per_robot_is_kept():
    conv = _conv()
    conv.record_bid("robot2", BID, 0.9)
    conv.record_bid("robot2", BID, 0.3)  # a better later bid should replace the worse one
    conv.record_bid("robot2", BID, 0.7)  # a worse later bid should NOT overwrite the best
    assert conv.bids["robot2"] == 0.3


def test_initiator_cannot_bid_on_its_own_offer():
    conv = _conv(initiator="robot1")
    conv.record_bid("robot1", BID, 0.01)
    assert conv.bids == {}
    assert conv.select_winner() is None


def test_non_bid_performatives_are_ignored_by_record_bid():
    conv = _conv()
    for performative in (OFFER, ACCEPT, REJECT, COMMIT, TIMEOUT):
        conv.record_bid("robot2", performative, 0.1)
    assert conv.bids == {}


def test_is_expired():
    conv = _conv(deadline=100.0)
    assert conv.is_expired(100.0) is True
    assert conv.is_expired(150.0) is True
    assert conv.is_expired(99.9) is False


def test_select_winner_with_no_reliability_map_behaves_as_before():
    conv = _conv()
    conv.record_bid("robot2", BID, 0.5)
    conv.record_bid("robot3", BID, 0.52)
    assert conv.select_winner(reliability=None) == "robot2"


def test_select_winner_reliability_can_flip_a_close_tie():
    conv = _conv()
    conv.record_bid("robot2", BID, 0.50)
    conv.record_bid("robot3", BID, 0.52)
    # robot3's small cost disadvantage (0.02) is smaller than its reliability bonus
    # (0.05 weight * 1.0 reliability = 0.05), so it should win despite the higher raw bid.
    assert conv.select_winner(reliability={"robot3": 1.0}) == "robot3"


def test_select_winner_reliability_cannot_overturn_a_large_cost_gap():
    conv = _conv()
    conv.record_bid("robot2", BID, 0.10)
    conv.record_bid("robot3", BID, 0.90)
    assert conv.select_winner(reliability={"robot3": 1.0}) == "robot2"


def test_select_winner_unknown_peer_gets_no_reliability_bonus():
    conv = _conv()
    conv.record_bid("robot2", BID, 0.50)
    conv.record_bid("robot3", BID, 0.52)
    assert conv.select_winner(reliability={"robot9": 1.0}) == "robot2"
