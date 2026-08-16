"""Lightweight, tick-based multi-robot world (Phase II / Experiment Family II).

Deliberately NOT a physics simulation: no Gazebo, no Nav2, no ROS. Travel time is
distance/speed, battery drain is distance-proportional, station contention reuses
Phase I's exact `ClaimBook` tie-break, and low-battery task-transfer reuses Phase I's
exact `Conversation` negotiation state machine (see `reuse.py` for what's imported
and why). This is the explicit, documented tradeoff behind Phase II's existence: Phase
I's embodied Gazebo platform measures real navigation/physics but costs 15-40+ minutes
of wall-clock time per run (this project's own measured realtime factor), which makes
the long-horizon, many-seed runs Experiment Family II needs infeasible on this
hardware. This world runs the same decision logic at effectively unlimited speed
(thousands of simulated seconds in real seconds), at the honest cost of not
re-validating navigation/physics - that validation is Phase I's job, frozen and not
repeated here.

One simplification worth stating plainly: because every robot lives in the same
Python process and station-claim resolution is a synchronous function call (not a
real network broadcast with delay), the specific double-claim RACE Phase I
found (two claims arriving within the real contention window each
resolving locally before seeing the other) cannot occur here - contention is still
real (two robots CAN want the same station; the lower-cost one deterministically
wins), but the timing race that defeated charger scarcity in the embodied world is
absent by construction. Analysis code checks and reports whether single-slot charger
scarcity actually holds in THIS environment rather than assuming it does either way.
"""
from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from world.interactions import (
    CHARGER_REQUEST,
    CHARGER_YIELD,
    HELP_OFFER,
    HELP_REQUEST,
    InteractionLog,
    RESOURCE_REQUEST,
    RESOURCE_YIELD,
    TASK_REJECT,
    TASK_TRANSFER,
)
from world.reuse import (
    Action,
    AgentBackend,
    ALLOWED_ACTIONS,
    ClaimBook,
    Conversation,
    Observation,
    OPPOSITE_SIDE,
    PeerMemory,
    REJECTED,
    STATIONS_BY_SIDE,
    StationCandidate,
    SUCCESSFUL_HELP,
    TIMED_OUT,
)

SPEED_M_S = 1.2  # matches Phase I's observed live robot speed (~1.2-1.3 m/s)
PICKUP_DROPOFF_PAUSE_SEC = 2.0  # matches Phase I's PICKUP_DROPOFF_PAUSE_SEC
LOW_BATTERY_THRESHOLD = 0.25  # matches Phase I's LOW_BATTERY_THRESHOLD
FULL_CHARGE_THRESHOLD = 0.95
CHARGE_DURATION_SEC = 20.0  # matches Phase I's CHARGING_BOOST window
BATTERY_DRAIN_PER_METER = 0.004  # tuned so a robot needs to charge roughly every
                                  # ~300-400s at this world's station spacing - frequent
                                  # enough to produce real charger contention over a
                                  # long-horizon run, not so frequent it dominates
NEGOTIATION_TIMEOUT_SEC = 5.0  # matches Phase I's NEGOTIATION_TIMEOUT_SEC
RELIABILITY_WEIGHT = 0.05  # matches Phase I's memory-nudge weight

CHARGER_STATION = ("charger_1", "charger", 0.0, 0.0)


def _distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5


def _normalized_distance(d: float, max_d: float = 20.0) -> float:
    return min(d / max_d, 1.0)


def _energy_risk(battery_soc: float) -> float:
    return 1.0 - battery_soc


def _bid_cost(x: float, y: float, battery_soc: float, station: tuple) -> float:
    """Peer-bidding cost formula for a low-battery task offer - duplicated from
    RuleAgent's own weights rather than routed through the peer's AgentBackend,
    matching Phase I's documented convention (decentralized_agent.py): any idle
    peer's bid uses this fixed formula regardless of which backend it normally
    decides with."""
    d = _distance(x, y, station[2], station[3])
    return 0.7 * _normalized_distance(d) + 0.3 * _energy_risk(battery_soc)


@dataclass
class RobotState:
    robot_id: str
    backend: AgentBackend
    x: float = 0.0
    y: float = 0.0
    battery_soc: float = 1.0
    state: str = "IDLE"  # IDLE | TRAVELING | WORKING | CHARGING
    current_side: str = "input"
    target: Optional[tuple] = None  # (name, side, x, y)
    remaining_work_sec: float = 0.0
    tasks_completed: int = 0
    distance_traveled_m: float = 0.0
    memory: Optional[PeerMemory] = None
    _pending_claim: Optional[str] = None
    _offer_made_for_current_task: bool = False
    _resume_side_after_charge: Optional[str] = None
    busy_ticks: int = 0
    total_ticks: int = 0

    def utilization(self) -> float:
        return round(self.busy_ticks / self.total_ticks, 4) if self.total_ticks else 0.0


@dataclass
class _ActiveConversation:
    conversation: Conversation
    initiator_target: tuple  # the station the initiator was heading to/working at


class World:
    """One simulated warehouse fleet. `step()` advances one simulated second and
    is meant to be called in a plain loop - no real-time pacing, no ROS executor."""

    def __init__(
        self,
        *,
        num_robots: int,
        seed: int,
        backend_factory,
        memory_enabled: bool,
        interaction_log: Optional[InteractionLog] = None,
        stations_per_side: Optional[int] = 3,
    ) -> None:
        self.rng = random.Random(seed)
        self.sim_time = 0.0
        self.memory_enabled = memory_enabled
        self.claim_book = ClaimBook()
        self.interactions = interaction_log or InteractionLog()
        self.active_conversations: Dict[str, _ActiveConversation] = {}
        self._conv_counter = 0
        # Deliberately fewer stations than the fleet size (default 3 per side vs.
        # e.g. 4 robots): Phase I's full 10-per-side layout produced essentially zero
        # real station contention with only 2-4 robots (confirmed empirically - a
        # 3000-tick, 4-robot calibration run against all 20 stations logged ZERO
        # RESOURCE_YIELD/CHARGER_YIELD-from-contention events). Experiment Family
        # II calls for deliberately limited shared infrastructure - this is that constraint
        # applied to task stations too, not just the charger, so resource-negotiation
        # interactions (the whole point of this experiment) actually occur.
        self.stations_by_side = {
            side: stations[:stations_per_side] if stations_per_side else stations
            for side, stations in STATIONS_BY_SIDE.items()
        }

        self.robots: Dict[str, RobotState] = {}
        for i in range(num_robots):
            robot_id = f"robot{i + 1}"
            self.robots[robot_id] = RobotState(
                robot_id=robot_id,
                backend=backend_factory(robot_id),
                memory=PeerMemory() if memory_enabled else None,
            )

    # -- one simulated second -------------------------------------------------
    def step(self, dt: float = 1.0) -> None:
        self.sim_time += dt
        order = list(self.robots.values())
        self.rng.shuffle(order)

        for robot in order:
            robot.total_ticks += 1
            if robot.state != "IDLE":
                robot.busy_ticks += 1
            if robot.state == "CHARGING":
                self._tick_charging(robot, dt)
            elif robot.state == "WORKING":
                self._tick_working(robot, dt)
            elif robot.state == "TRAVELING":
                self._tick_traveling(robot, dt)
            elif robot.state == "IDLE":
                self._tick_idle(robot)

        # resolve this tick's fresh claims only after every IDLE robot has bid,
        # so simultaneous contention is resolved by cost, not by iteration order
        for robot in order:
            if robot._pending_claim is not None:
                self._resolve_claim(robot)

        self._resolve_expired_conversations()

    # -- per-state tick handlers ------------------------------------------------
    def _tick_charging(self, robot: RobotState, dt: float) -> None:
        robot.battery_soc = min(1.0, robot.battery_soc + dt / CHARGE_DURATION_SEC)
        robot.remaining_work_sec -= dt
        if robot.battery_soc >= FULL_CHARGE_THRESHOLD or robot.remaining_work_sec <= 0:
            self.claim_book.observe(CHARGER_STATION[0], robot.robot_id, 0.0, release=True)
            self.interactions.record(
                simulation_time=self.sim_time, from_robot=robot.robot_id, to_robot=None,
                interaction_type=CHARGER_YIELD, resource=CHARGER_STATION[0], result="RELEASED",
            )
            robot.state = "IDLE"
            robot.current_side = robot._resume_side_after_charge or robot.current_side
            robot._resume_side_after_charge = None

    def _tick_working(self, robot: RobotState, dt: float) -> None:
        robot.remaining_work_sec -= dt
        if robot.remaining_work_sec <= 0:
            robot.tasks_completed += 1
            self.claim_book.observe(robot.target[0], robot.robot_id, 0.0, release=True)
            robot.current_side = OPPOSITE_SIDE[robot.target[1]]
            robot.target = None
            robot.state = "IDLE"
            robot._offer_made_for_current_task = False

    def _tick_traveling(self, robot: RobotState, dt: float) -> None:
        dest_x, dest_y = robot.target[2], robot.target[3]
        remaining = _distance(robot.x, robot.y, dest_x, dest_y)
        step_dist = min(SPEED_M_S * dt, remaining)
        if remaining > 1e-9:
            robot.x += (dest_x - robot.x) * (step_dist / remaining)
            robot.y += (dest_y - robot.y) * (step_dist / remaining)
        robot.distance_traveled_m += step_dist
        robot.battery_soc = max(0.0, robot.battery_soc - step_dist * BATTERY_DRAIN_PER_METER)

        is_charging_trip = robot.target[1] == "charger"
        if (
            not is_charging_trip
            and not robot._offer_made_for_current_task
            and robot.battery_soc < LOW_BATTERY_THRESHOLD
        ):
            self._initiate_help_offer(robot)

        if _distance(robot.x, robot.y, dest_x, dest_y) < 1e-6:
            if is_charging_trip:
                robot.state = "CHARGING"
                robot.remaining_work_sec = CHARGE_DURATION_SEC
            else:
                robot.state = "WORKING"
                robot.remaining_work_sec = PICKUP_DROPOFF_PAUSE_SEC

    def _tick_idle(self, robot: RobotState) -> None:
        needs_charge = robot.battery_soc < LOW_BATTERY_THRESHOLD
        if needs_charge:
            candidates_raw = [CHARGER_STATION] if self.claim_book.is_free(CHARGER_STATION[0]) else []
        else:
            candidates_raw = self.claim_book.free_stations(self.stations_by_side[robot.current_side])
        if not candidates_raw:
            # Genuine resource scarcity: every station this robot could use is
            # currently held by a peer, so it never even gets to broadcast a claim.
            # This is the dominant real-scarcity signal with few stations relative
            # to fleet size (confirmed by calibration: explicit simultaneous
            # double-claims are rare, but this "nothing free" case is common) - log
            # it so resource-conflict analysis sees the real pressure, not just the
            # rarer two-way race.
            self.interactions.record(
                simulation_time=self.sim_time, from_robot=robot.robot_id, to_robot=None,
                interaction_type=CHARGER_REQUEST if needs_charge else RESOURCE_REQUEST,
                resource=CHARGER_STATION[0] if needs_charge else robot.current_side,
                result="BLOCKED_NO_CANDIDATES",
            )
            return

        candidates = tuple(
            StationCandidate(name=c[0], side=c[1], x=c[2], y=c[3]) for c in candidates_raw
        )
        obs = Observation(
            robot_id=robot.robot_id, x=robot.x, y=robot.y, battery_soc=robot.battery_soc,
            degradation_risk=0.0, utilization=0.0, candidate_stations=candidates,
        )
        action = robot.backend.decide(obs, ALLOWED_ACTIONS)
        if action.action != "BID_FOR_TASK":
            return

        self.claim_book.observe(action.station_name, robot.robot_id, action.cost, release=False)
        is_charger = action.station_name == CHARGER_STATION[0]
        self.interactions.record(
            simulation_time=self.sim_time, from_robot=robot.robot_id, to_robot=None,
            interaction_type=CHARGER_REQUEST if is_charger else RESOURCE_REQUEST,
            resource=action.station_name,
        )
        robot._pending_claim = action.station_name

    def _resolve_claim(self, robot: RobotState) -> None:
        station_name = robot._pending_claim
        robot._pending_claim = None
        winner = self.claim_book.winner_of(station_name)
        if winner != robot.robot_id:
            is_charger = station_name == CHARGER_STATION[0]
            self.interactions.record(
                simulation_time=self.sim_time, from_robot=robot.robot_id, to_robot=winner,
                interaction_type=CHARGER_YIELD if is_charger else RESOURCE_YIELD,
                resource=station_name, result="LOST_CONTENTION",
            )
            return
        station = CHARGER_STATION if station_name == CHARGER_STATION[0] else next(
            c for c in self.stations_by_side[robot.current_side] if c[0] == station_name
        )
        robot.target = station
        robot.state = "TRAVELING"
        if station_name == CHARGER_STATION[0]:
            robot._resume_side_after_charge = robot.current_side

    # -- low-battery task-transfer negotiation (reuses Conversation exactly) ----
    def _initiate_help_offer(self, robot: RobotState) -> None:
        robot._offer_made_for_current_task = True
        self._conv_counter += 1
        conv_id = f"c{self._conv_counter}"
        conv = Conversation(
            conversation_id=conv_id, initiator=robot.robot_id, station_name=robot.target[0],
            side=robot.target[1], x=robot.target[2], y=robot.target[3],
            reason_code="LOW_BATTERY", deadline=self.sim_time + NEGOTIATION_TIMEOUT_SEC,
        )
        self.active_conversations[conv_id] = _ActiveConversation(
            conversation=conv, initiator_target=robot.target,
        )
        self.interactions.record(
            simulation_time=self.sim_time, from_robot=robot.robot_id, to_robot=None,
            interaction_type=HELP_REQUEST, resource=robot.target[0], task_id=conv_id,
        )
        # any currently-idle peer with no claim of its own, and enough battery not to
        # immediately need the same rescue itself, may bid this same tick - without
        # this guard a chronically-low-battery robot could keep bidding on peers'
        # offers (increasing its own distance) instead of ever reaching the charger
        for peer in self.robots.values():
            if (
                peer.robot_id == robot.robot_id
                or peer.state != "IDLE"
                or peer.battery_soc < LOW_BATTERY_THRESHOLD
            ):
                continue
            cost = _bid_cost(peer.x, peer.y, peer.battery_soc, robot.target)
            conv.record_bid(peer.robot_id, "BID", cost)
            self.interactions.record(
                simulation_time=self.sim_time, from_robot=peer.robot_id, to_robot=robot.robot_id,
                interaction_type=HELP_OFFER, resource=robot.target[0], task_id=conv_id,
            )

    def _resolve_expired_conversations(self) -> None:
        expired = [
            cid for cid, ac in self.active_conversations.items()
            if ac.conversation.is_expired(self.sim_time)
        ]
        for conv_id in expired:
            ac = self.active_conversations.pop(conv_id)
            conv = ac.conversation
            initiator = self.robots[conv.initiator]
            reliability = None
            if self.memory_enabled and initiator.memory is not None:
                reliability = {p: initiator.memory.reliability(p) for p in conv.bids}
            winner_id = conv.select_winner(reliability=reliability, reliability_weight=RELIABILITY_WEIGHT)

            # A winning bidder can have become busy with its own claim between
            # bidding and this deadline (bids are honored at bid time, but a peer's
            # own _tick_idle can claim a different station in a later tick before
            # this negotiation resolves) - honoring the transfer anyway would
            # silently overwrite that robot's real target, which a real regression
            # test caught. Phase I documented the identical race in
            # decentralized_agent.py's _on_transfer_accepted and handles it the
            # same way: the already-busy winner's acceptance is ignored, not
            # retried against the next-best bidder, and the initiator keeps its
            # own task - a real, intentional race in a deliberately simple
            # one-round protocol, not something masked.
            winner_still_available = winner_id is not None and self.robots[winner_id].state == "IDLE"

            if winner_id is None or not winner_still_available:
                self.interactions.record(
                    simulation_time=self.sim_time, from_robot=conv.initiator, to_robot=winner_id,
                    interaction_type=HELP_REQUEST, resource=conv.station_name,
                    task_id=conv_id,
                    result="TIMEOUT" if winner_id is None else "WINNER_BUSY",
                )
                continue

            response_latency = NEGOTIATION_TIMEOUT_SEC  # bids are recorded at offer time in this model
            for bidder_id in conv.bids:
                accepted = bidder_id == winner_id
                if self.memory_enabled and initiator.memory is not None:
                    outcome = SUCCESSFUL_HELP if accepted else REJECTED
                    initiator.memory.record_outcome(bidder_id, outcome, response_latency)
                self.interactions.record(
                    simulation_time=self.sim_time, from_robot=conv.initiator, to_robot=bidder_id,
                    interaction_type=TASK_TRANSFER if accepted else TASK_REJECT,
                    resource=conv.station_name, task_id=conv_id, accepted=accepted,
                    response_latency=response_latency,
                    result="SUCCESS" if accepted else "REJECTED",
                )

            # ownership actually transfers: winner heads to the station, initiator
            # frees up (still low-battery, will seek the charger on its next idle tick)
            self.claim_book.observe(conv.station_name, conv.initiator, 0.0, release=True)
            self.claim_book.observe(conv.station_name, winner_id, 0.0, release=False)
            winner = self.robots[winner_id]
            winner.target = ac.initiator_target
            winner.state = "TRAVELING"
            initiator.target = None
            initiator.state = "IDLE"
            initiator._offer_made_for_current_task = False
