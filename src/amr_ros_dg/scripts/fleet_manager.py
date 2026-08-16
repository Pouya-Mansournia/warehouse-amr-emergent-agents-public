#!/usr/bin/env python3
"""
Fleet task manager for the amr_ros_dg warehouse simulation.

For each robot in the fleet (namespaced robot1..robotN, matching fleet.launch.py):
    IDLE -> pick a random free station on one side -> claim it -> NavigateToPose there
         -> "pick up" (2s pause) -> release -> pick a random free station on the OPPOSITE
         side -> claim it -> NavigateToPose there -> "drop off" (2s pause) -> release -> repeat

No real manipulation - this is a mobility/logistics sim only. No-collision is handled by
each robot's own Nav2 local costmap (populated from its own lidar), not by this node; this
node's only synchronization job is making sure two robots never target the SAME station at
the same time (a station-level reservation, not a full multi-agent motion planner).

STATIONS below are hand-written to match worlds/warehouse_fleet_world.sdf exactly (model
names input_station_1..10 / output_station_1..10, x=-6.9/+6.9, y=-9..9 step 2 - the world
was resized from the original 50x50m/25-station spec down to a lighter 15x20m/10-station
footprint for fleet testing). The world file is hand-authored SDF, not generated from this
list (or vice versa) - if you change one, change the other.
"""

import random
import threading
import time
from dataclasses import dataclass

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

try:
    from amr_interfaces.msg import FleetManagerHeartbeat
except ImportError:  # amr_interfaces not built - fleet_manager still works (Mode A only)
    FleetManagerHeartbeat = None

# Centralized -> decentralized failover: how often fleet_manager announces
# it's alive on /fleet/manager/heartbeat. A DecentralizedAgent in DORMANT (failover
# standby) state watches for these - this node's process dying (Mode B kill, or a real
# crash) simply stops them. A single persistent timer, created once in __init__ and never
# touched again - the same safe pattern every other periodic/deferred action in this node
# already uses (see _schedule()'s docstring for why a fresh create_timer() per call was
# found unsafe under real load; this is a fixed-period timer, not that pattern at all, but
# kept to the same "created once, never again" discipline regardless).
_HEARTBEAT_PERIOD_SEC = 1.0


def _build_stations():
    """(name, side, x, y) for all 10 input + 10 output stations. Must mirror the SDF."""
    stations = []
    for i, y in enumerate(range(-9, 10, 2), start=1):
        stations.append((f"input_station_{i}", "input", -6.9, float(y)))
    for i, y in enumerate(range(-9, 10, 2), start=1):
        stations.append((f"output_station_{i}", "output", 6.9, float(y)))
    return stations


STATIONS = _build_stations()
STATIONS_BY_SIDE = {
    "input": [s for s in STATIONS if s[1] == "input"],
    "output": [s for s in STATIONS if s[1] == "output"],
}
OPPOSITE_SIDE = {"input": "output", "output": "input"}

PICKUP_DROPOFF_PAUSE_SEC = 2.0

# Poll period for each robot's persistent scheduling timer - see RobotTask.pending_fn below.
# Bounds the extra latency any retry/pause can pick up (e.g. a 2.0s retry actually fires
# within [2.0, 2.0 + _SCHEDULE_POLL_SEC)), which is negligible next to Nav2/SLAM bringup
# timescales, in exchange for removing every runtime create_timer() call in this node.
_SCHEDULE_POLL_SEC = 0.2


@dataclass
class RobotTask:
    """Per-robot state machine state. One instance per robot namespace."""

    namespace: str
    state: str = "IDLE"  # IDLE -> NAVIGATING -> PAUSED -> (back to IDLE, opposite side)
    current_side: str = "input"  # overwritten with an RNG-drawn side in FleetManager.__init__
    claimed_station: str = ""
    action_client: ActionClient = None
    # Deferred work for this robot, dispatched by its own persistent poll timer (see
    # FleetManager._schedule/_poll_pending) instead of ever calling create_timer() again
    # after node startup - see that method's docstring for why.
    pending_fn = None
    pending_fire_at: float = 0.0


class FleetManager(Node):
    def __init__(self):
        super().__init__("fleet_manager")

        self.declare_parameter("num_robots", 5)
        num_robots = self.get_parameter("num_robots").get_parameter_value().integer_value

        # Deterministic seeding is required for every stochastic
        # component; this fleet_manager's station-selection/initial-side randomness was the
        # one part of it never actually wired to run_experiment.py's --seed (it only ever
        # landed in metadata.json, never reached this node) - fixed here by seeding a
        # per-node rng instead of using the `random` module's shared global state, so this
        # node's randomness is reproducible independent of what else in the process might
        # also call `random.*`.
        self.declare_parameter("seed", 0)
        seed = self.get_parameter("seed").get_parameter_value().integer_value
        self._rng = random.Random(seed)
        self.get_logger().info(f"fleet_manager seeded with seed={seed}")

        # Reservation dict: station_name -> robot_namespace claiming it. A real lock is used
        # (not just check-then-set) because this node runs a MultiThreadedExecutor so per-robot
        # callback groups genuinely execute concurrently - rclpy's usual "callbacks are
        # effectively single-threaded" assumption does not hold here.
        self._reservation_lock = threading.Lock()
        self._reservations = {}  # station_name -> robot_namespace

        self._robots = {}
        for i in range(1, num_robots + 1):
            ns = f"robot{i}"
            cb_group = MutuallyExclusiveCallbackGroup()
            client = ActionClient(
                self, NavigateToPose, f"/{ns}/navigate_to_pose", callback_group=cb_group
            )
            task = RobotTask(
                namespace=ns,
                action_client=client,
                current_side=self._rng.choice(["input", "output"]),
            )
            self._robots[ns] = task
            # This robot's ONLY timer, created once, ever, right here - see _schedule()'s
            # docstring for why every retry/pause below is dispatched through it instead of
            # each calling create_timer() itself when it needs to defer work.
            self.create_timer(
                _SCHEDULE_POLL_SEC, lambda t=task: self._poll_pending(t), callback_group=cb_group
            )
            self.get_logger().info(f"Registered {ns}, waiting for its Nav2 action server...")

        # Kick off every robot's first assignment once, then each robot re-triggers itself
        # at the end of its own cycle (drop-off complete -> assign again) - no shared poll
        # loop/timer is needed since each robot's cycle is event-driven off its own action
        # result callback.
        self._start_timer = self.create_timer(1.0, self._bootstrap_idle_robots)

        # Heartbeat for decentralized failover, only if amr_interfaces was built (same optional-import
        # degradation pattern event_logger_node.py already uses for its own amr_interfaces
        # subscriptions) - a --coordination centralized run with amr_interfaces absent
        # still works exactly as before, just with no failover signal to watch for.
        self._heartbeat_pub = None
        if FleetManagerHeartbeat is not None:
            self._heartbeat_pub = self.create_publisher(
                FleetManagerHeartbeat, "/fleet/manager/heartbeat", 10
            )
            self.create_timer(_HEARTBEAT_PERIOD_SEC, self._publish_heartbeat)

    def _publish_heartbeat(self) -> None:
        msg = FleetManagerHeartbeat()
        msg.stamp = self.get_clock().now().to_msg()
        self._heartbeat_pub.publish(msg)

    # ------------------------------------------------------------------ bootstrap

    def _bootstrap_idle_robots(self):
        # One-shot: assign a first task to every still-IDLE robot, then cancel this timer.
        for task in self._robots.values():
            if task.state == "IDLE":
                self._assign_next_task(task)
        self._start_timer.cancel()

    # ------------------------------------------------------------------ reservation helpers

    def _claim_random_station(self, side: str):
        """Atomic check-and-set over the free stations on `side`. Returns (name, x, y) or None."""
        with self._reservation_lock:
            free = [s for s in STATIONS_BY_SIDE[side] if s[0] not in self._reservations]
            if not free:
                return None
            station = self._rng.choice(free)
            self._reservations[station[0]] = "claimed"  # placeholder set inside the lock
            return station

    def _release_station(self, station_name: str):
        with self._reservation_lock:
            self._reservations.pop(station_name, None)

    # ------------------------------------------------------------------ task cycle

    def _schedule(self, task: RobotTask, delay_sec: float, fn):
        """Defer `fn` (a no-arg closure) until `delay_sec` from now, for this robot.

        History: an earlier version of this node called `self.create_timer(delay, fn)`
        fresh for every retry - first broken because create_timer() is PERIODIC, not
        one-shot (fixed by a self-cancelling wrapper, `_one_shot`), then STILL broken
        under real WSL load even after that fix: confirmed live, twice, in separate
        sessions, with the identical symptom - `navigate_to_pose server not ready yet,
        retrying in 2s` logged exactly once per robot and then fleet_manager never sent
        another goal, ever, for the rest of a 150s run, with no exception/traceback
        anywhere. The common factor both times: a NEW timer entity was being added to a
        MultiThreadedExecutor from INSIDE a callback that same executor was running -
        rclpy is supposed to wake its rcl_wait() for this via a guard condition, but
        under heavy CPU contention (the same WSL load documented in
        TROUBLESHOOTING.md #21/#22b/#22c) that wake-up is apparently not reliably timely
        enough, and with fleet_manager having no other frequently-firing entity to
        incidentally re-wake it, the new timer could then wait arbitrarily long.

        Fix: never call create_timer() again after node startup. Every robot gets ONE
        persistent poll timer for its entire lifetime (see __init__), already sitting in
        the executor's wait-set from the start; deferring work here is just a plain
        Python attribute write, not a new entity, so there is nothing for the executor to
        need waking up for."""
        task.pending_fn = fn
        task.pending_fire_at = time.monotonic() + delay_sec

    def _poll_pending(self, task: RobotTask):
        fn = task.pending_fn
        if fn is None or time.monotonic() < task.pending_fire_at:
            return
        task.pending_fn = None
        fn()

    def _assign_next_task(self, task: RobotTask):
        station = self._claim_random_station(task.current_side)
        if station is None:
            # All stations on this side are currently claimed - retry shortly rather than
            # blocking the executor. Deferred via _schedule(), not a blocking sleep.
            self.get_logger().info(
                f"{task.namespace}: no free {task.current_side} station, retrying in 1s"
            )
            self._schedule(task, 1.0, lambda: self._assign_next_task(task))
            return

        name, side, x, y = station
        task.claimed_station = name
        task.state = "NAVIGATING"
        self.get_logger().info(
            f"{task.namespace}: assigned {side} station '{name}' at ({x:.1f}, {y:.1f})"
        )
        self._send_nav_goal(task, x, y)

    def _send_nav_goal(self, task: RobotTask, x: float, y: float):
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        # frame_id left as plain "map" - each robot's own namespace/tf tree has its own
        # "map" frame (per-robot SLAM Toolbox instance), see fleet.launch.py's TF design
        # comment. Nav2 resolves this within the robot's own namespace, not a global frame.
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        goal.pose = pose

        if not task.action_client.server_is_ready():
            # Nav2 for this robot may not have finished bringing up yet (SLAM/costmaps take
            # a few seconds after Gazebo spawn) - retry rather than dropping the task.
            self.get_logger().warn(
                f"{task.namespace}: navigate_to_pose server not ready yet, retrying in 2s"
            )
            self._schedule(task, 2.0, lambda: self._send_nav_goal(task, x, y))
            return

        send_future = task.action_client.send_goal_async(goal)
        send_future.add_done_callback(lambda f: self._on_goal_response(task, f))

    def _on_goal_response(self, task: RobotTask, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(
                f"{task.namespace}: goal to '{task.claimed_station}' rejected, retrying in 2s"
            )
            self._release_station(task.claimed_station)
            # Same reason as the "server not ready" case in _send_nav_goal: a rejection right
            # after Nav2 bringup usually means bt_navigator/controller_server aren't fully
            # active yet. Without this delay, _assign_next_task -> _send_nav_goal ->
            # (server_is_ready() true, but goal still rejected) -> here -> _assign_next_task
            # formed a zero-delay hot loop - confirmed live in WSL via the log showing
            # hundreds of assign/reject pairs within ~90ms.
            self._schedule(task, 2.0, lambda: self._assign_next_task(task))
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_nav_result(task, f))

    def _on_nav_result(self, task: RobotTask, future):
        result = future.result()
        ok = result.status == GoalStatus.STATUS_SUCCEEDED
        action = "pick up" if task.current_side == "input" else "drop off"
        if ok:
            self.get_logger().info(
                f"{task.namespace}: arrived at '{task.claimed_station}', {action} (2s)..."
            )
        else:
            self.get_logger().warn(
                f"{task.namespace}: navigation to '{task.claimed_station}' failed "
                f"(status={result.status}), still doing {action} in place"
            )
        task.state = "PAUSED"
        # Non-blocking pause: deferred via this robot's persistent poll timer (see
        # _schedule()'s docstring), not time.sleep(), so other robots' callbacks (running on
        # their own MutuallyExclusiveCallbackGroup) are never stalled by this.
        self._schedule(task, PICKUP_DROPOFF_PAUSE_SEC, lambda: self._on_pause_complete(task))

    def _on_pause_complete(self, task: RobotTask):
        self.get_logger().info(f"{task.namespace}: done at '{task.claimed_station}', releasing")
        self._release_station(task.claimed_station)
        task.claimed_station = ""
        task.current_side = OPPOSITE_SIDE[task.current_side]
        task.state = "IDLE"
        self._assign_next_task(task)


def main():
    rclpy.init()
    node = FleetManager()
    # MultiThreadedExecutor: N robots each waiting on their own action server / timers must
    # make progress concurrently, not queue behind each other on a single-threaded executor
    # (that would serialize e.g. one robot's 2s pickup pause behind another's goal response).
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
