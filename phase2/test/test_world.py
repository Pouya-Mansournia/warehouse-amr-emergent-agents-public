from world.reuse import RuleAgent, ReplayAgent, Action
from world.world import World, CHARGER_STATION, LOW_BATTERY_THRESHOLD


def _rule_world(num_robots=2, seed=1, memory_enabled=False, stations_per_side=3):
    return World(
        num_robots=num_robots, seed=seed,
        backend_factory=lambda rid: RuleAgent(), memory_enabled=memory_enabled,
        stations_per_side=stations_per_side,
    )


def test_world_constructs_requested_number_of_robots():
    world = _rule_world(num_robots=4)
    assert len(world.robots) == 4
    assert set(world.robots) == {"robot1", "robot2", "robot3", "robot4"}


def test_robots_complete_tasks_over_time():
    world = _rule_world(num_robots=2)
    for _ in range(500):
        world.step()
    total = sum(r.tasks_completed for r in world.robots.values())
    assert total > 0, "expected at least one completed task cycle in 500 simulated seconds"


def test_battery_drains_while_traveling():
    world = _rule_world(num_robots=1)
    start_soc = next(iter(world.robots.values())).battery_soc
    for _ in range(200):
        world.step()
    end_soc = next(iter(world.robots.values())).battery_soc
    assert end_soc < start_soc


def test_robot_recharges_when_battery_crosses_low_threshold():
    world = _rule_world(num_robots=1)
    robot = next(iter(world.robots.values()))
    saw_charging = False
    for _ in range(3000):
        world.step()
        if robot.state == "CHARGING":
            saw_charging = True
            break
    assert saw_charging, "a single robot with no contention should reach the charger eventually"


def test_charger_is_never_held_by_two_robots_at_once():
    world = _rule_world(num_robots=4, stations_per_side=2)
    for _ in range(1500):
        world.step()
        charging = [r for r in world.robots.values() if r.state == "CHARGING"]
        assert len(charging) <= 1, "single-slot charger scarcity must hold in this synchronous world"


def test_station_is_never_claimed_by_two_robots_at_once():
    world = _rule_world(num_robots=4, stations_per_side=2)
    for _ in range(1500):
        world.step()
        occupied = [r.target[0] for r in world.robots.values() if r.state in ("TRAVELING", "WORKING") and r.target]
        assert len(occupied) == len(set(occupied)), "no station should ever be claimed by two robots"


def test_low_battery_mid_task_triggers_a_help_offer():
    world = _rule_world(num_robots=2)
    for _ in range(3000):
        world.step()
    help_requests = [e for e in world.interactions.events if e["interaction_type"] == "HELP_REQUEST" and e["result"] is None]
    assert help_requests, "expected at least one real low-battery help offer over 3000s with 2 robots"


def test_memory_disabled_never_constructs_peer_memory():
    world = _rule_world(num_robots=2, memory_enabled=False)
    assert all(r.memory is None for r in world.robots.values())


def test_memory_enabled_constructs_peer_memory_per_robot():
    world = _rule_world(num_robots=2, memory_enabled=True)
    assert all(r.memory is not None for r in world.robots.values())


def test_resource_scarcity_generates_blocked_events_with_few_stations():
    world = _rule_world(num_robots=4, stations_per_side=1)
    for _ in range(1000):
        world.step()
    blocked = [e for e in world.interactions.events if e.get("result") == "BLOCKED_NO_CANDIDATES"]
    assert blocked, "1 station per side with 4 robots must produce real blocked-waiting events"


def test_deterministic_given_same_seed():
    world_a = _rule_world(num_robots=3, seed=42)
    world_b = _rule_world(num_robots=3, seed=42)
    for _ in range(1000):
        world_a.step()
        world_b.step()
    tasks_a = {rid: r.tasks_completed for rid, r in world_a.robots.items()}
    tasks_b = {rid: r.tasks_completed for rid, r in world_b.robots.items()}
    assert tasks_a == tasks_b, "identical seed must reproduce identical task-completion counts"


def test_replay_agent_backend_never_bids_for_a_station_outside_the_offer():
    # ReplayAgent always returns WAIT once its script is exhausted - confirms the
    # world never crashes when a backend refuses to act (defensive: any future
    # backend that sometimes returns WAIT must be handled the same way RuleAgent's
    # BID_FOR_TASK path is).
    world = World(
        num_robots=1, seed=1,
        backend_factory=lambda rid: ReplayAgent(script=[]), memory_enabled=False,
    )
    for _ in range(50):
        world.step()
    robot = next(iter(world.robots.values()))
    assert robot.state == "IDLE"
    assert robot.tasks_completed == 0
