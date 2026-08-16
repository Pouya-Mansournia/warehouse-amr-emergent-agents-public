#!/usr/bin/env python3
"""Decentralized per-robot coordination agent (Mode C).

One OS process per robot (unlike amr_ros_dg/fleet_manager.py, which is ONE node
governing every robot from a single in-process reservation dict). Each agent:

  - owns only ITS OWN robot's NavigateToPose action client and state machine;
  - has no connection to any other robot's process, no shared memory, no central
    broker - the only channels between robots are the shared /fleet/agent/claim topic
    (amr_interfaces/StationClaim) and /fleet/agent/task_transfer
    (amr_interfaces/TaskTransfer), which every agent both publishes AND
    subscribes to;
  - decides which station to bid for by delegating to a provider-independent
    `agent_core.AgentBackend` - `RuleAgent` by default, reproducing the
    `task_cost` weighted-sum formula; `agent_backend:=llm`
    or `agent_backend:=hybrid` (Mode E - RuleAgent normally, `LLMAgent` only for
    ambiguous near-tied decisions, see hybrid_agent.py) are also selectable - and
    resolves simultaneous-claim conflicts against its peers using ClaimBook's
    deterministic lowest-cost-wins rule (see claim_book.py's module docstring for why
    this converges without a coordinator);
  - if its own battery drops below LOW_BATTERY_THRESHOLD while it's mid-task, initiates
    a peer-to-peer task-transfer negotiation for that task instead of finishing
    it itself - see negotiation.py's module docstring for the OFFER/BID/ACCEPT/REJECT/
    COMMIT/TIMEOUT protocol this drives.

Deferred work (contention-window waits, retries, the pickup/dropoff pause, negotiation
deadlines) uses the same single-persistent-poll-timer pattern established in
amr_ros_dg/fleet_manager.py after TROUBLESHOOTING.md #25 - a fresh create_timer() per
retry was found to silently stop firing under real WSL2 CPU load; this node never calls
create_timer() again after __init__ either.
"""
from __future__ import annotations

import random
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from amr_interfaces.msg import (
    AgentDecision,
    CoordinationModeChanged,
    FaultCommand,
    FleetManagerHeartbeat,
    RobotHealth,
    StationClaim,
    TaskTransfer,
)
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node

from agent_core.interfaces import ALLOWED_ACTIONS, Observation, StationCandidate
from agent_core.peer_memory import PeerMemory
from agent_core.rule_agent import RuleAgent
from fleet_coordination.claim_book import ClaimBook
from fleet_coordination.heartbeat import HeartbeatMonitor
from fleet_coordination.negotiation import ACCEPT, BID, COMMIT, COUNTER_PROPOSAL, OFFER, REJECT, TIMEOUT, Conversation
from fleet_coordination.stations import OPPOSITE_SIDE, STATIONS_BY_SIDE

PICKUP_DROPOFF_PAUSE_SEC = 2.0
CONTENTION_WINDOW_SEC = 1.0
POLL_PERIOD_SEC = 0.2

# A robot offers to transfer its currently-claimed task once its own battery
# drops below this - the same "LOW_BATTERY" concept used as the canonical
# PROPOSE_TASK_TRANSFER example. Negotiation is capped at
# NEGOTIATION_TIMEOUT_SEC to prevent infinite negotiation -
# if nobody bids in time, the initiator simply keeps its own task rather
# than blocking or retrying forever.
LOW_BATTERY_THRESHOLD = 0.25
NEGOTIATION_TIMEOUT_SEC = 5.0

# Charging-station scarcity reuses the exact
# station-claim broadcast/ClaimBook mechanism already used for task stations
# (see stations.py) - a charger is just another named "station" in the SAME shared
# /fleet/agent/claim topic, so a single ClaimBook.winner_of(CHARGER_STATION_NAME) call
# gives free, coordinator-free, capacity-1 slot contention with zero new message types
# or classes. Deliberately capacity=1 (only one robot's claim can ever hold this one
# name at a time) - this models a genuinely scarce shared resource. Location is
# the warehouse center (0, 0), clear of every input/output station in stations.py, not
# a real Gazebo charger entity (no such model exists in this simulation - a documented
# simplification, not a hidden one).
CHARGER_STATION_NAME = "charger_1"
CHARGER_X = 0.0
CHARGER_Y = 0.0
# Lower than LOW_BATTERY_THRESHOLD so a robot tries a peer task-transfer FIRST
# (while still NAVIGATING) and only falls back to physically charging once it's IDLE
# (task finished or handed off) and still critically low - avoids the two mechanisms
# fighting over the same low-battery signal, and avoids ever canceling an in-flight Nav2
# goal (this repo's existing, documented policy - see negotiation.py's module docstring).
CRITICAL_BATTERY_THRESHOLD = 0.15
# "Charged enough" to resume normal task cycling - deliberately not 1.0, so charging is
# a real, observable, non-instantaneous state (CHARGING_BOOST's 0.02/tick rate takes
# ~20 simulated seconds to cross this gap from a freshly-critical robot) without holding
# a scarce charger slot indefinitely once the emergency has genuinely passed.
CHARGED_ENOUGH_THRESHOLD = 0.9
CHARGING_RETRY_SEC = 2.0

# Mirrors agent_core/rule_agent.py's RuleAgent formula (distance + battery risk)
# - duplicated rather than imported because bidding on a PEER's
# transfer offer needs a one-off single-point cost, not RuleAgent's candidate-list
# decide() shape. Same duplication-with-a-comment convention as stations.py.
_BID_W_DISTANCE = 0.7
_BID_W_ENERGY = 0.3
_BID_MAX_EXPECTED_DISTANCE_M = 20.0


@dataclass
class _PendingWork:
    fn: Optional[object] = None
    fire_at: float = 0.0


class DecentralizedAgent(Node):
    def __init__(self) -> None:
        super().__init__("decentralized_agent")

        self.declare_parameter("robot_id", "")
        self.declare_parameter("seed", 0)
        self.declare_parameter("agent_backend", "rule")
        # Centralized -> decentralized failover: start_dormant=True means
        # this agent touches NOTHING (no claims, no Nav2 goals) until it concludes the
        # central fleet_manager is gone - see HeartbeatMonitor / _poll_pending below.
        # Always False (the normal decentralized behavior, unchanged) unless launched under
        # --coordination centralized_then_failover.
        self.declare_parameter("start_dormant", False)
        self.declare_parameter("manager_timeout_sec", 5.0)
        # Episodic peer memory: off by
        # default (identical behavior to the baseline). When true, this robot
        # tracks real ACCEPT/REJECT/TIMEOUT outcomes of task-transfer negotiations it
        # initiated, per peer, and uses the resulting reliability score to nudge (not
        # override) Conversation.select_winner's deterministic lowest-cost-wins rule -
        # see negotiation.py's select_winner docstring and agent_core/peer_memory.py.
        self.declare_parameter("memory_enabled", False)
        robot_id = self.get_parameter("robot_id").get_parameter_value().string_value
        if not robot_id:
            raise RuntimeError("decentralized_agent requires the 'robot_id' parameter")
        self.robot_id = robot_id

        base_seed = self.get_parameter("seed").get_parameter_value().integer_value
        # Distinct-but-reproducible per-robot stream from one shared experiment seed -
        # same reasoning as fleet_manager.py's seeding, just
        # split per OS process here since each robot IS its own process in Mode C.
        robot_index = int("".join(ch for ch in robot_id if ch.isdigit()) or "0")
        self._rng = random.Random(base_seed * 1000 + robot_index)

        self._claim_book = ClaimBook()
        # Phase 6: the "which station to bid for" decision is delegated to a
        # provider-independent AgentBackend (agent_core) rather than being computed
        # inline here - RuleAgent's defaults reproduce Phase 5's exact cost formula
        # (see rule_agent.py's module docstring). A second selectable
        # backend, LLMAgent, always falls back to RuleAgent on malformed/unsafe/
        # unreachable-server responses - so 'llm' mode can never leave a robot with no
        # decision at all, only ever degrades to deterministic behavior.
        # 'hybrid' (Mode E): RuleAgent handles the common case, LLMAgent is
        # only consulted when the top two candidate stations are within
        # HybridAgent's ambiguity_margin of each other - see hybrid_agent.py's
        # module docstring for the full Mode E framing.
        backend_choice = (
            self.get_parameter("agent_backend").get_parameter_value().string_value
        )
        if backend_choice == "llm":
            from agent_core.llm_agent import LLMAgent
            from agent_core.ollama_client import OllamaClient

            self._agent_backend = LLMAgent(client=OllamaClient(), fallback=RuleAgent())
        elif backend_choice == "hybrid":
            from agent_core.hybrid_agent import HybridAgent
            from agent_core.llm_agent import LLMAgent
            from agent_core.ollama_client import OllamaClient

            deterministic = RuleAgent()
            llm = LLMAgent(client=OllamaClient(), fallback=deterministic)
            self._agent_backend = HybridAgent(deterministic=deterministic, llm=llm)
        else:
            backend_choice = "rule"
            self._agent_backend = RuleAgent()
        self._backend_name = backend_choice
        self._x = 0.0
        self._y = 0.0
        self._battery_soc = 1.0  # optimistic default until a real health sample arrives

        start_dormant = self.get_parameter("start_dormant").get_parameter_value().bool_value
        manager_timeout_sec = (
            self.get_parameter("manager_timeout_sec").get_parameter_value().double_value
        )
        self._start_dormant = start_dormant
        self._heartbeat_monitor = HeartbeatMonitor(manager_timeout_sec=manager_timeout_sec)

        self._memory_enabled = (
            self.get_parameter("memory_enabled").get_parameter_value().bool_value
        )
        self._peer_memory = PeerMemory() if self._memory_enabled else None
        # Sim-time each peer's BID arrived, keyed by conversation_id then
        # robot_id - used only to compute average_response_time_sec once a conversation
        # resolves; never influences select_winner itself (only reliability does).
        self._bid_received_at: dict = {}

        self._state = "DORMANT" if start_dormant else "IDLE"
        self._current_side = self._rng.choice(["input", "output"])
        self._contending_station: Optional[tuple] = None  # (name, side, x, y, cost)
        self._claimed_station: Optional[tuple] = None
        self._pending = _PendingWork()

        # Phase 8: negotiation state. `_active_conversation` is only ever set while THIS
        # robot is the initiator of an in-flight transfer offer (bidder-side state is
        # transient - a bid is fire-and-forget, nothing to track until ACCEPT/REJECT
        # arrives). `_low_battery_offer_sent` guards against re-offering the SAME task
        # repeatedly every time a new (still-low) health sample arrives - reset whenever
        # a new task starts, so each task gets at most one low-battery offer attempt.
        self._active_conversation: Optional[Conversation] = None
        self._negotiation_seq = 0
        self._low_battery_offer_sent = False
        self._negotiation_pending = _PendingWork()

        self._action_client = ActionClient(
            self, NavigateToPose, f"/{robot_id}/navigate_to_pose"
        )
        self.create_subscription(
            Odometry, f"/{robot_id}/diff_drive_controller/odom", self._on_odom, 10
        )
        self.create_subscription(
            RobotHealth, f"/{robot_id}/health/state", self._on_health, 10
        )
        self._claim_pub = self.create_publisher(StationClaim, "/fleet/agent/claim", 10)
        self.create_subscription(
            StationClaim, "/fleet/agent/claim", self._on_claim_msg, 10
        )
        self._transfer_pub = self.create_publisher(
            TaskTransfer, "/fleet/agent/task_transfer", 10
        )
        self.create_subscription(
            TaskTransfer, "/fleet/agent/task_transfer", self._on_task_transfer_msg, 10
        )
        # Phase 12: real, measured decision latency per AgentBackend.decide() call -
        # what lets the final research-analysis comparison table report an actual
        # "Decision Latency" column instead of leaving it blank or invented.
        self._decision_pub = self.create_publisher(
            AgentDecision, "/fleet/agent/decision", 10
        )
        # Heartbeat subscription is always created (harmless overhead if
        # never in failover mode - fleet_manager may not even be running to publish it)
        # rather than conditionally wired, so start_dormant is the only thing that
        # decides whether it's ever ACTED on.
        self.create_subscription(
            FleetManagerHeartbeat, "/fleet/manager/heartbeat", self._on_heartbeat, 10
        )
        self._mode_change_pub = self.create_publisher(
            CoordinationModeChanged, "/fleet/agent/coordination_mode", 10
        )
        # The same topic fault_injector.py already publishes on - no new
        # message type or plumbing needed to make a robot "charge itself."
        self._fault_pub = self.create_publisher(FaultCommand, "/fleet/faults/command", 10)

        # This agent's ONLY timer - see this module's docstring / TROUBLESHOOTING.md #25.
        self.create_timer(POLL_PERIOD_SEC, self._poll_pending)

        if start_dormant:
            self.get_logger().info(
                f"{robot_id}: decentralized agent started DORMANT (failover standby, "
                f"manager_timeout_sec={manager_timeout_sec}) - waiting for central "
                f"manager heartbeat loss (seed={base_seed})"
            )
        else:
            self._schedule(1.0, self._attempt_claim)
            self.get_logger().info(f"{robot_id}: decentralized agent started (seed={base_seed})")

    # ------------------------------------------------------------------ telemetry inputs

    def _on_odom(self, msg: Odometry) -> None:
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y

    def _on_health(self, msg: RobotHealth) -> None:
        self._battery_soc = msg.battery_soc
        self._maybe_offer_transfer()

    # ------------------------------------------------------------------ centralized-failover

    def _sim_now(self) -> float:
        """Simulation time (seconds), from this node's own ROS clock - meaningful
        because use_sim_time:=true is already set unconditionally on every
        decentralized_agent (decentralized_agents.launch.py, since Phase 5), same as
        every Nav2/SLAM node. Critically, fleet_manager.py ALSO has use_sim_time:=true,
        so its heartbeat timer (create_timer(1.0, ...)) fires every 1.0 SIMULATED
        second, not 1.0 real second - at this project's documented ~0.09x realtime
        factor that's roughly every ~11 real seconds. Comparing heartbeat freshness
        against time.monotonic() (real wall-clock) was found live to cause a false-
        positive failover: agents declared the manager dead at simulation_time=4.2s
        (a real elapsed gap of only a few real seconds since the last - entirely
        normal for a 1-simulated-second heartbeat period at this realtime factor) while
        fleet_manager was still genuinely alive and running. Using sim time on both
        sides (the timer's own firing cadence AND this comparison) keeps
        manager_timeout_sec meaningful regardless of the host's realtime factor -
        the same simulation-time-control principle applied elsewhere in this project,
        now applied to heartbeat detection too."""
        return self.get_clock().now().nanoseconds / 1e9

    def _on_heartbeat(self, msg: FleetManagerHeartbeat) -> None:
        self._heartbeat_monitor.on_heartbeat(self._sim_now())

    def _publish_coordination_mode_changed(self, from_mode: str, to_mode: str, reason: str) -> None:
        msg = CoordinationModeChanged()
        msg.robot_id = self.robot_id
        msg.from_mode = from_mode
        msg.to_mode = to_mode
        msg.reason = reason
        msg.stamp = self.get_clock().now().to_msg()
        self._mode_change_pub.publish(msg)

    def _activate_after_failover(self) -> None:
        """DORMANT -> normal active cycle, triggered once the central manager's
        heartbeat has been missing for longer than manager_timeout_sec (see
        HeartbeatMonitor). This robot has no knowledge of what fleet_manager may have
        already reserved centrally - its own ClaimBook starts empty, exactly as if this
        were a fresh --coordination decentralized run starting now. A station
        fleet_manager already sent this (or another) robot toward is not tracked here;
        any resulting double-claim is resolved the same way any other simultaneous
        claim is (ClaimBook's deterministic lowest-cost-wins rule) - an explicit,
        documented limitation of this one-round failover, same honesty as Phase 5's
        ClaimBook / Phase 8's negotiation protocol limitations."""
        self._state = "IDLE"
        self._publish_coordination_mode_changed(
            "CENTRALIZED", "DECENTRALIZED", "CENTRAL_MANAGER_TIMEOUT"
        )
        self.get_logger().warn(
            f"{self.robot_id}: central manager heartbeat lost - activating "
            f"decentralized failover"
        )
        self._attempt_claim()

    # ------------------------------------------------------------------ peer claims

    def _on_claim_msg(self, msg: StationClaim) -> None:
        # Every agent (including this one, for its own broadcasts) funnels every claim
        # through the identical ClaimBook rule - see claim_book.py's module docstring for
        # why that's what makes independent agents converge on the same belief.
        self._claim_book.observe(msg.station_name, msg.robot_id, msg.cost, msg.release)

    # ------------------------------------------------------------------ scheduling
    # (see amr_ros_dg/fleet_manager.py's _schedule/_poll_pending - identical base
    # pattern.) TWO independent slots, not one: `_pending` for the primary task cycle
    # (contention window, nav-goal retries, pickup/dropoff pause) and
    # `_negotiation_pending` for an in-flight Phase 8 negotiation deadline. Found live
    # (2026-08-11, Phase 8 validation) that a single shared slot is NOT safe here - a
    # robot can genuinely need both scheduled at once (an OFFER's 5s deadline running
    # while its own Nav2 goal is still resolving), and when a nav-goal result callback
    # fired microseconds after `_initiate_transfer` had just claimed the one slot, it
    # silently overwrote the negotiation timeout - `_resolve_transfer` never fired,
    # `_active_conversation` was never cleared, and the robot could never negotiate
    # again for the rest of the run (worse than just "one lost negotiation": a
    # permanently wedged flag). Two slots polled independently every tick fixes this
    # with no coordination needed between them, since they're never used for the same
    # purpose.

    def _schedule(self, delay_sec: float, fn) -> None:
        self._pending.fn = fn
        self._pending.fire_at = time.monotonic() + delay_sec

    def _schedule_negotiation(self, delay_sec: float, fn) -> None:
        self._negotiation_pending.fn = fn
        self._negotiation_pending.fire_at = time.monotonic() + delay_sec

    def _poll_pending(self) -> None:
        if self._state == "DORMANT":
            # Only meaningful check while dormant - once active, this
            # agent behaves exactly like a normal decentralized agent forever after
            # (no reverting to DORMANT if a heartbeat somehow reappears; a central
            # manager coming back from the dead mid-run is out of scope here, same as
            # Mode B's own "no recovery" framing never considers a resurrected
            # fleet_manager either). Uses _sim_now(), not time.monotonic() - see that
            # method's docstring for why (fleet_manager's heartbeat period is itself
            # sim-time-throttled, so the comparison must be too).
            if not self._heartbeat_monitor.is_alive(self._sim_now()):
                self._activate_after_failover()
            return
        if self._state == "CHARGING" and self._battery_soc >= CHARGED_ENOUGH_THRESHOLD:
            # No separate timer needed - battery_soc is already updated by
            # _on_health every time a fresh RobotHealth sample arrives, this just checks
            # it on the same 0.2s cadence as everything else (TROUBLESHOOTING.md #25's
            # single-persistent-timer discipline, no new create_timer() call).
            self._finish_charging()
        now = time.monotonic()
        fn = self._pending.fn
        if fn is not None and now >= self._pending.fire_at:
            self._pending.fn = None
            fn()
        negotiation_fn = self._negotiation_pending.fn
        if negotiation_fn is not None and now >= self._negotiation_pending.fire_at:
            self._negotiation_pending.fn = None
            negotiation_fn()

    # ------------------------------------------------------------------ task cycle

    def _attempt_claim(self) -> None:
        # A robot that becomes IDLE while still critically low goes to
        # charge instead of contending for a new task - checked here (not in _on_health)
        # so it only ever triggers between tasks, never by interrupting an in-flight one.
        if self._battery_soc < CRITICAL_BATTERY_THRESHOLD:
            self._attempt_charge()
            return
        candidates = self._claim_book.free_stations(STATIONS_BY_SIDE[self._current_side])
        observation = Observation(
            robot_id=self.robot_id,
            x=self._x,
            y=self._y,
            battery_soc=self._battery_soc,
            degradation_risk=0.0,  # no mechanical-health model exists yet (RuleAgent's w_health=0.0)
            utilization=0.0,       # no workload model exists yet (RuleAgent's w_workload=0.0)
            candidate_stations=tuple(
                StationCandidate(name=c[0], side=c[1], x=c[2], y=c[3]) for c in candidates
            ),
        )
        decide_started = time.perf_counter()
        action = self._agent_backend.decide(observation, ALLOWED_ACTIONS)
        decision_latency_sec = time.perf_counter() - decide_started
        self._publish_decision(action.action, decision_latency_sec)

        if action.action == "WAIT":
            self.get_logger().info(
                f"{self.robot_id}: no free {self._current_side} station (per local belief), retrying in 1s"
            )
            self._schedule(1.0, self._attempt_claim)
            return

        name = action.station_name
        side, x, y = next((c[1], c[2], c[3]) for c in candidates if c[0] == name)
        cost = action.cost
        self._contending_station = (name, side, x, y, cost)
        self._publish_claim(name, side, x, y, cost, release=False)
        self.get_logger().info(
            f"{self.robot_id}: contending for {side} station '{name}' at ({x:.1f}, {y:.1f}), cost={cost:.3f}"
        )
        self._schedule(CONTENTION_WINDOW_SEC, self._resolve_contention)

    def _resolve_contention(self) -> None:
        name, side, x, y, cost = self._contending_station
        winner = self._claim_book.winner_of(name)
        if winner != self.robot_id:
            self.get_logger().info(
                f"{self.robot_id}: lost contention for '{name}' to {winner}, retrying"
            )
            self._contending_station = None
            self._schedule(0.3, self._attempt_claim)
            return

        self.get_logger().info(f"{self.robot_id}: won contention for '{name}', navigating")
        self._claimed_station = (name, side, x, y)
        self._contending_station = None
        self._low_battery_offer_sent = False  # new task - eligible for a fresh offer
        self._state = "NAVIGATING"
        self._send_nav_goal(x, y)

    # ------------------------------------------------------------------ charging

    def _attempt_charge(self) -> None:
        # cost = battery_soc itself (lower battery = lower cost = wins ClaimBook's
        # lowest-cost-wins rule) - the most critical robot wins the single charger slot
        # when two robots contend for it simultaneously, without a coordinator, same
        # convergence property Phase 5's task-station contention already relies on.
        cost = self._battery_soc
        self._publish_claim(CHARGER_STATION_NAME, "charging", CHARGER_X, CHARGER_Y, cost, release=False)
        self.get_logger().warn(
            f"{self.robot_id}: battery critical ({self._battery_soc:.2f}), contending "
            f"for charger (cost={cost:.3f})"
        )
        self._schedule(CONTENTION_WINDOW_SEC, self._resolve_charge_contention)

    def _resolve_charge_contention(self) -> None:
        winner = self._claim_book.winner_of(CHARGER_STATION_NAME)
        if winner != self.robot_id:
            self.get_logger().info(
                f"{self.robot_id}: lost charger contention to {winner}, retrying in "
                f"{CHARGING_RETRY_SEC}s"
            )
            self._schedule(CHARGING_RETRY_SEC, self._attempt_charge)
            return

        self.get_logger().warn(f"{self.robot_id}: won charger contention, navigating to charge")
        self._claimed_station = (CHARGER_STATION_NAME, "charging", CHARGER_X, CHARGER_Y)
        self._state = "NAVIGATING"
        self._send_nav_goal(CHARGER_X, CHARGER_Y)

    def _start_charging(self) -> None:
        self._state = "CHARGING"
        self._fault_pub.publish(
            self._make_fault_command(fault_type="CHARGING_BOOST", severity=1.0, clear=False)
        )
        self.get_logger().warn(
            f"{self.robot_id}: arrived at charger, charging (soc={self._battery_soc:.2f})"
        )

    def _finish_charging(self) -> None:
        self._fault_pub.publish(
            self._make_fault_command(fault_type="CHARGING_BOOST", severity=1.0, clear=True)
        )
        self._publish_claim(CHARGER_STATION_NAME, "charging", CHARGER_X, CHARGER_Y, 0.0, release=True)
        self.get_logger().warn(f"{self.robot_id}: charged (soc={self._battery_soc:.2f}), resuming tasks")
        self._claimed_station = None
        self._state = "IDLE"
        self._attempt_claim()

    def _make_fault_command(self, *, fault_type: str, severity: float, clear: bool) -> FaultCommand:
        msg = FaultCommand()
        msg.robot_id = self.robot_id
        msg.fault_type = fault_type
        msg.severity = severity
        msg.duration_sec = 0.0
        msg.trigger_source = "manual"
        msg.fault_id = f"{self.robot_id}-charging"
        msg.clear = clear
        msg.stamp = self.get_clock().now().to_msg()
        return msg

    def _publish_claim(self, name: str, side: str, x: float, y: float, cost: float, *, release: bool) -> None:
        msg = StationClaim()
        msg.robot_id = self.robot_id
        msg.station_name = name
        msg.side = side
        msg.x = x
        msg.y = y
        msg.cost = cost
        msg.release = release
        msg.stamp = self.get_clock().now().to_msg()
        self._claim_pub.publish(msg)

    def _publish_decision(self, action: str, decision_latency_sec: float) -> None:
        msg = AgentDecision()
        msg.robot_id = self.robot_id
        msg.backend = self._backend_name
        msg.action = action
        msg.decision_latency_sec = decision_latency_sec
        msg.stamp = self.get_clock().now().to_msg()
        # LLMAgent/HybridAgent set last_decision_meta after every
        # decide() call (agent_core.llm_agent.DecisionMeta); RuleAgent has no such
        # attribute, so getattr's default correctly yields "no LLM was involved" for
        # backend=="rule" and for "hybrid" decisions answered deterministically.
        meta = getattr(self._agent_backend, "last_decision_meta", None)
        if meta is not None:
            msg.has_llm_meta = True
            msg.provider = meta.provider or ""
            msg.model = meta.model or ""
            msg.prompt_tokens = meta.prompt_tokens if meta.prompt_tokens is not None else -1
            msg.completion_tokens = (
                meta.completion_tokens if meta.completion_tokens is not None else -1
            )
            msg.schema_valid = bool(meta.schema_valid)
            msg.safety_valid = bool(meta.safety_valid) if meta.safety_valid is not None else False
            msg.fallback_used = meta.fallback_used
            msg.retry_count = meta.retry_count
        else:
            msg.has_llm_meta = False
            msg.provider = ""
            msg.model = ""
            msg.prompt_tokens = -1
            msg.completion_tokens = -1
            msg.schema_valid = False
            msg.safety_valid = False
            msg.fallback_used = False
            msg.retry_count = 0
        self._decision_pub.publish(msg)

    # ------------------------------------------------------------------ Phase 8: negotiation

    def _publish_transfer(
        self,
        conversation_id: str,
        performative: str,
        *,
        to_robot_id: str,
        station_name: str,
        side: str,
        x: float,
        y: float,
        cost: float,
        reason_code: str,
    ) -> None:
        msg = TaskTransfer()
        msg.conversation_id = conversation_id
        msg.performative = performative
        msg.from_robot_id = self.robot_id
        msg.to_robot_id = to_robot_id
        msg.station_name = station_name
        msg.side = side
        msg.x = x
        msg.y = y
        msg.cost = cost
        msg.reason_code = reason_code
        msg.stamp = self.get_clock().now().to_msg()
        self._transfer_pub.publish(msg)

    def _maybe_offer_transfer(self) -> None:
        # A robot NAVIGATING to the charger (side=="charging")
        # was indistinguishable from a robot navigating to a real task station, so this
        # low-battery trigger offered the charging TRIP itself for transfer - a
        # peer would accept it, take over navigating to the charger, while the original
        # robot (now IDLE, still critical) immediately re-won a fresh charger contention
        # of its own via _attempt_claim -> _attempt_charge. Both ends up charging
        # concurrently at the one-slot charger, silently defeating the scarcity
        # constraint the whole mechanism exists to create - live-confirmed via
        # health_robot1.csv/health_robot2.csv both showing simultaneous soc recovery
        # windows. Excluding "charging" trips here is the fix: charging is this robot's
        # own emergency response, not a task it should ever hand off to a peer.
        if (
            self._state == "NAVIGATING"
            and self._claimed_station is not None
            and self._claimed_station[1] != "charging"
            and self._active_conversation is None
            and not self._low_battery_offer_sent
            and self._battery_soc < LOW_BATTERY_THRESHOLD
        ):
            self._initiate_transfer()

    def _initiate_transfer(self) -> None:
        name, side, x, y = self._claimed_station
        conversation_id = f"{self.robot_id}-{self._negotiation_seq}"
        self._negotiation_seq += 1
        self._active_conversation = Conversation(
            conversation_id=conversation_id,
            initiator=self.robot_id,
            station_name=name,
            side=side,
            x=x,
            y=y,
            reason_code="LOW_BATTERY",
            deadline=time.monotonic() + NEGOTIATION_TIMEOUT_SEC,
        )
        self._low_battery_offer_sent = True
        if self._memory_enabled:
            self._bid_received_at[conversation_id] = {}
            self._offer_sent_at = self._sim_now()
        self._publish_transfer(
            conversation_id,
            OFFER,
            to_robot_id="",
            station_name=name,
            side=side,
            x=x,
            y=y,
            cost=self._battery_soc,
            reason_code="LOW_BATTERY",
        )
        self.get_logger().warn(
            f"{self.robot_id}: battery low ({self._battery_soc:.2f}), offering to "
            f"transfer '{name}' (conversation={conversation_id})"
        )
        self._schedule_negotiation(NEGOTIATION_TIMEOUT_SEC, self._resolve_transfer)

    def _resolve_transfer(self) -> None:
        conv = self._active_conversation
        self._active_conversation = None
        reliability = None
        if self._memory_enabled and self._peer_memory is not None:
            reliability = {
                robot_id: self._peer_memory.reliability(robot_id) for robot_id in conv.bids
            }
        winner = conv.select_winner(reliability=reliability)

        if self._memory_enabled and self._peer_memory is not None:
            arrivals = self._bid_received_at.pop(conv.conversation_id, {})
            for robot_id in conv.bids:
                response_time = (
                    arrivals[robot_id] - self._offer_sent_at
                    if robot_id in arrivals
                    else None
                )
                outcome = "successful_help" if robot_id == winner else "rejected"
                self._peer_memory.record_outcome(robot_id, outcome, response_time)

        if winner is None:
            self._publish_transfer(
                conv.conversation_id,
                TIMEOUT,
                to_robot_id="",
                station_name=conv.station_name,
                side=conv.side,
                x=conv.x,
                y=conv.y,
                cost=0.0,
                reason_code=conv.reason_code,
            )
            self.get_logger().info(
                f"{self.robot_id}: no bids for '{conv.station_name}' "
                f"(conversation={conv.conversation_id}), keeping task"
            )
            return

        winning_cost = conv.bids[winner]
        for robot_id, bid_cost in conv.bids.items():
            self._publish_transfer(
                conv.conversation_id,
                ACCEPT if robot_id == winner else REJECT,
                to_robot_id=robot_id,
                station_name=conv.station_name,
                side=conv.side,
                x=conv.x,
                y=conv.y,
                cost=bid_cost,
                reason_code=conv.reason_code,
            )
        self._publish_transfer(
            conv.conversation_id,
            COMMIT,
            to_robot_id=winner,
            station_name=conv.station_name,
            side=conv.side,
            x=conv.x,
            y=conv.y,
            cost=winning_cost,
            reason_code=conv.reason_code,
        )
        self.get_logger().warn(
            f"{self.robot_id}: transferring '{conv.station_name}' to {winner} "
            f"(cost={winning_cost:.3f}, conversation={conv.conversation_id})"
        )
        # I never actually cancel the in-flight Nav2 goal for this station (no cancel
        # client is wired up - out of scope for this phase, same class of documented
        # limitation as Phase 4's finding that Nav2 keeps executing whatever goal it
        # already had). Releasing the claim and going IDLE just stops ME from treating
        # it as mine any further and lets the fleet's belief converge correctly even if
        # my body is still physically mid-transit for a moment.
        self._publish_claim(conv.station_name, conv.side, conv.x, conv.y, 0.0, release=True)
        self._claimed_station = None
        self._state = "IDLE"
        self._attempt_claim()

    def _cost_for(self, x: float, y: float) -> float:
        normalized_distance = min(
            ((self._x - x) ** 2 + (self._y - y) ** 2) ** 0.5 / _BID_MAX_EXPECTED_DISTANCE_M,
            1.0,
        )
        energy_risk = 1.0 - self._battery_soc
        return _BID_W_DISTANCE * normalized_distance + _BID_W_ENERGY * energy_risk

    def _on_task_transfer_msg(self, msg: TaskTransfer) -> None:
        if msg.from_robot_id == self.robot_id:
            return

        if msg.performative == OFFER:
            self._maybe_bid_on_offer(msg)
        elif (
            msg.performative in (BID, COUNTER_PROPOSAL)
            and self._active_conversation is not None
            and msg.conversation_id == self._active_conversation.conversation_id
        ):
            self._active_conversation.record_bid(msg.from_robot_id, msg.performative, msg.cost)
            if self._memory_enabled:
                self._bid_received_at.setdefault(msg.conversation_id, {})[
                    msg.from_robot_id
                ] = self._sim_now()
        elif msg.performative == ACCEPT and msg.to_robot_id == self.robot_id:
            self._on_transfer_accepted(msg)
        elif msg.performative == REJECT and msg.to_robot_id == self.robot_id:
            self.get_logger().info(
                f"{self.robot_id}: bid for '{msg.station_name}' not selected "
                f"(conversation={msg.conversation_id})"
            )

    def _maybe_bid_on_offer(self, msg: TaskTransfer) -> None:
        if self._claimed_station is not None:
            return  # already committed to a task of my own - single-task-at-a-time
        cost = self._cost_for(msg.x, msg.y)
        self._publish_transfer(
            msg.conversation_id,
            BID,
            to_robot_id=msg.from_robot_id,
            station_name=msg.station_name,
            side=msg.side,
            x=msg.x,
            y=msg.y,
            cost=cost,
            reason_code=msg.reason_code,
        )
        self.get_logger().info(
            f"{self.robot_id}: bidding {cost:.3f} on {msg.from_robot_id}'s offer for "
            f"'{msg.station_name}' (conversation={msg.conversation_id})"
        )

    def _on_transfer_accepted(self, msg: TaskTransfer) -> None:
        if self._claimed_station is not None:
            # Became busy with something else between bidding and hearing back - can't
            # take this on too (single-task-at-a-time model). The initiator believes
            # I've taken it; this is a known race in this simple one-round protocol,
            # same class of honesty as claim_book.py's own "not Byzantine/partition-safe"
            # limitation.
            self.get_logger().warn(
                f"{self.robot_id}: accepted transfer for '{msg.station_name}' but "
                f"already busy with another task - ignoring "
                f"(conversation={msg.conversation_id})"
            )
            return
        if self._contending_station is not None:
            # Abandon my own in-flight (unwon) contention bid in favor of the negotiated
            # assignment - no Nav2 goal was ever sent for it, so there's nothing to
            # cancel, just release the claim broadcast so peers' ClaimBooks don't hold a
            # stale entry.
            name, side, x, y, _cost = self._contending_station
            self._publish_claim(name, side, x, y, 0.0, release=True)
            self._contending_station = None

        self._claimed_station = (msg.station_name, msg.side, msg.x, msg.y)
        self._low_battery_offer_sent = False
        self._state = "NAVIGATING"
        self.get_logger().warn(
            f"{self.robot_id}: accepted task transfer for '{msg.station_name}' from "
            f"{msg.from_robot_id} (conversation={msg.conversation_id})"
        )
        self._send_nav_goal(msg.x, msg.y)

    def _send_nav_goal(self, x: float, y: float) -> None:
        if not self._action_client.server_is_ready():
            self.get_logger().warn(
                f"{self.robot_id}: navigate_to_pose server not ready yet, retrying in 2s"
            )
            self._schedule(2.0, lambda: self._send_nav_goal(x, y))
            return

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        goal.pose = pose

        send_future = self._action_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        name, side, x, y = self._claimed_station
        if not goal_handle.accepted:
            self.get_logger().warn(f"{self.robot_id}: goal to '{name}' rejected, retrying in 2s")
            self._publish_claim(name, side, x, y, 0.0, release=True)
            self._claimed_station = None
            # A rejected charging goal retries the charge attempt, not a
            # normal task claim - same distinction _on_nav_result makes below.
            retry_fn = self._attempt_charge if side == "charging" else self._attempt_claim
            self._schedule(2.0, retry_fn)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        result = future.result()
        ok = result.status == GoalStatus.STATUS_SUCCEEDED
        name, side, x, y = self._claimed_station
        if side == "charging":
            # Even a failed nav result still starts charging "in place" -
            # the same honest simplification the pickup/dropoff pause below already
            # makes for task stations (this repo never cancels/retries an
            # in-flight Nav2 goal on failure, it treats the robot as having done the
            # action anyway - documented, not hidden).
            if not ok:
                self.get_logger().warn(
                    f"{self.robot_id}: navigation to charger failed (status={result.status}), "
                    f"still charging in place"
                )
            self._start_charging()
            return
        action = "pick up" if side == "input" else "drop off"
        if ok:
            self.get_logger().info(f"{self.robot_id}: arrived at '{name}', {action} (2s)...")
        else:
            self.get_logger().warn(
                f"{self.robot_id}: navigation to '{name}' failed (status={result.status}), "
                f"still doing {action} in place"
            )
        self._state = "PAUSED"
        self._schedule(PICKUP_DROPOFF_PAUSE_SEC, self._on_pause_complete)

    def _on_pause_complete(self) -> None:
        name, side, x, y = self._claimed_station
        self.get_logger().info(f"{self.robot_id}: done at '{name}', releasing")
        self._publish_claim(name, side, x, y, 0.0, release=True)
        self._claimed_station = None
        self._current_side = OPPOSITE_SIDE[side]
        self._state = "IDLE"
        self._attempt_claim()


def main() -> None:
    rclpy.init()
    node = DecentralizedAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
