"""One-command experiment runner for Phase 1/2/3 (Mode A) and Phase 4 (Mode B).

    ros2 run experiment_manager run_experiment \
        --mode centralized_baseline --num-robots 3 --seed 42 \
        --duration 300

    # Mode B: kill fleet_manager (centralized coordination) 60s in, keep
    # everything else (Gazebo, Nav2, event_logger, rosbag) running/logging:
    ros2 run experiment_manager run_experiment \
        --mode central_failure --num-robots 3 --seed 42 \
        --duration 300 --t-fail 60

Launches the unmodified amr_ros_dg fleet simulation, starts the passive
event_logger and an rosbag2 recording of the topics needed to
reconstruct a run, waits for --duration seconds (or Ctrl+C), then stops
everything and finalizes metadata.json.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time

from experiment_manager.run_paths import finalize_duration, make_run_dir, write_metadata
from experiment_manager.summarize_run import generate_plots, write_summary

_FLEET_MANAGER_CMD_PATTERN = "lib/amr_ros_dg/fleet_manager.py"


def _ros_distro() -> str:
    return os.environ.get("ROS_DISTRO", "unknown")


def _popen_own_group(args) -> subprocess.Popen:
    """Every ros2 subprocess launched here (fleet, health, logger, bag) is itself a
    process TREE, not a single process - `ros2 launch` in particular spawns further
    children (gz sim, spawners, Nav2/SLAM nodes) via its own launch actions.
    subprocess.Popen.kill()/.send_signal() only ever targets the direct child; if that
    child doesn't forward the signal fast enough (or dies before its own children do),
    grandchildren like `gz sim` are silently orphaned - reparented to init, and left
    running (and burning CPU) indefinitely. Confirmed live in WSL: after several
    "successful" (exit code 0) experiment runs, `ps aux` still showed multiple leftover
    `gz sim` processes from PAST runs, each still consuming 60-150% CPU, which is what
    was actually making later runs' controller_manager spawners fail under load - not a
    bug in those later runs at all. start_new_session=True makes this child (and
    everything IT spawns) its own process group; _terminate_group below then signals
    that whole group at once via os.killpg, which every descendant receives regardless
    of how many launch layers deep it is."""
    return subprocess.Popen(args, start_new_session=True)


def _terminate_group(proc: subprocess.Popen, *, timeout: float = 15.0) -> None:
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return  # already dead
    try:
        os.killpg(pgid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=timeout)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass


def _reap_stray_gz_sim() -> None:
    """Last-resort safety net, unconditionally run after every experiment shuts down.

    `_terminate_group`'s os.killpg assumes every descendant of a launched subprocess
    stays in that subprocess's process group - true for a plain fork, but `ros2 launch`
    itself puts EACH of its launched actions (gz sim included) into its OWN process
    group internally, specifically so it can signal them individually with its own
    multi-stage graceful-shutdown cascade (SIGINT, wait ~5s, SIGTERM, wait more,
    SIGKILL - visible in its own log output). If that internal cascade is still mid-way
    through a heavy multi-robot teardown when OUR 15s top-level timeout fires and we
    SIGKILL the launch process's group, `gz sim` - already outside that group - survives
    as an orphan. Confirmed live in WSL: this happened even AFTER the process-group fix
    below was in place and passing its own earlier test. Since this repo's whole design
    assumes one experiment runs at a time (no parallel runs), unconditionally killing
    every `gz sim` process for THIS world file once we've already finished our own
    shutdown sequence is safe and removes the failure mode entirely rather than trying
    to out-guess `ros2 launch`'s internal process-group behavior."""
    try:
        subprocess.run(
            ["pkill", "-9", "-f", "gz sim .*warehouse_fleet_world.sdf"],
            check=False,
            timeout=5,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass


def _log_run_event(run_dir, event: str, *, sim_time_source=None, **fields) -> None:
    """Appends one JSON line to <run_dir>/events.jsonl, in the same format
    event_logger_node.py's own `_log_event` uses - written directly here (not via a ROS
    topic round-trip) because run_experiment.py already knows run_dir directly, and this
    event (central coordination being killed) is about the EXPERIMENT, not about
    anything a ROS node observed. Logs both wall_clock timestamp and, when a
    _SimTimeSource is running (--time-source simulation), simulation_time, so this
    event stays comparable to every other logged event regardless of clock basis."""
    record = {
        "event": event,
        "robot_id": None,
        "timestamp": time.time(),
        "simulation_time": sim_time_source.sim_now() if sim_time_source is not None else None,
        **fields,
    }
    with open(run_dir / "events.jsonl", "a") as f:
        f.write(json.dumps(record) + "\n")


class _SimTimeSource:
    """Tracks Gazebo's simulation clock (/clock, bridged by fleet.launch.py's
    clock_bridge node) so run_experiment.py's --duration/--t-fail can be measured in
    simulated seconds instead of host wall-clock seconds: at this project's WSL2
    realtime factor of ~0.07-0.1x, a "300s" wall-clock run is only ~21-30 simulated
    seconds, nowhere near enough for a full task cycle.

    run_experiment.py itself is a plain subprocess-orchestration script with no
    pre-existing rclpy context (unlike every actual ROS node in this repo) - this class
    owns a single minimal rclpy.init()/Node/rclpy.shutdown() lifecycle of its own,
    entirely separate from the fleet/health/logger/bag subprocesses it's timing, and is
    only ever constructed when --time-source simulation is explicitly requested (the
    default --time-source wall path never touches rclpy at all, so every prior run's
    behavior is completely unchanged unless this flag is passed).
    """

    def __init__(self) -> None:
        import threading

        import rclpy
        from rclpy.node import Node

        from experiment_manager.sim_clock import SimClockNode

        self._rclpy = rclpy
        self._rclpy.init(args=[])
        self._node = Node("run_experiment_sim_clock_probe")
        self._sim_clock = SimClockNode(self._node)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        while not self._stop.is_set() and self._rclpy.ok():
            self._rclpy.spin_once(self._node, timeout_sec=0.1)

    def sim_now(self):
        return self._sim_clock.sim_now()

    def wait_for_clock(self, timeout_sec: float) -> bool:
        """Blocks (polling, not spinning - spinning happens on the background thread
        started in __init__) until the first /clock message arrives or timeout_sec
        elapses. Gazebo doesn't publish /clock until the world is actually running, so
        this must be a real wait, not an assumption that it's already there."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self._sim_clock.has_clock:
                return True
            time.sleep(0.1)
        return self._sim_clock.has_clock

    def shutdown(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


def _kill_fleet_manager(run_dir, t_fail: float, sim_time_source=None) -> None:
    """Mode B: deterministically remove centralized
    coordination at T_fail while leaving Gazebo/Nav2/event_logger/rosbag running -
    fleet_manager.py is its own OS process within the `ros2 launch amr_ros_dg
    fleet.launch.py` tree (see fleet.launch.py's fleet_manager_node), so it can be
    killed by matching its own command line without touching anything else launched
    alongside it. SIGINT first (fleet_manager.py has no special SIGINT handling beyond
    rclpy's default, but this keeps the same graceful-then-forceful pattern as
    _terminate_group); SIGKILL only if it's still alive after a short grace period."""
    subprocess.run(["pkill", "-INT", "-f", _FLEET_MANAGER_CMD_PATTERN], check=False)
    time.sleep(2.0)
    subprocess.run(["pkill", "-9", "-f", _FLEET_MANAGER_CMD_PATTERN], check=False)
    _log_run_event(
        run_dir, "CENTRAL_MANAGER_KILLED", t_fail_sec=t_fail, sim_time_source=sim_time_source
    )
    print(f"[run_experiment] Mode B: killed fleet_manager at t={t_fail:.1f}s")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", default="centralized_baseline")
    parser.add_argument("--num-robots", type=int, default=3)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="seconds to run before automatic shutdown; omit to run until Ctrl+C",
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--no-health",
        action="store_true",
        help="skip robot_health's health_monitor/fault_injector nodes (Phase 2)",
    )
    parser.add_argument(
        "--fault-schedule",
        default="",
        help="path to a fault schedule YAML for robot_health's fault_injector "
        "(see src/robot_health/config/faults/example_schedule.yaml); "
        "empty = manual-trigger-only",
    )
    parser.add_argument(
        "--t-fail",
        type=float,
        default=None,
        help="Mode B: seconds into the run at which to "
        "deterministically kill fleet_manager (centralized coordination), while Gazebo/"
        "Nav2/event_logger/rosbag keep running and logging. Requires --duration and "
        "--coordination centralized. Omit for no central-failure injection.",
    )
    parser.add_argument(
        "--coordination",
        choices=["centralized", "decentralized", "centralized_then_failover"],
        default="centralized",
        help="'centralized' (Mode A, default): fleet_manager.py assigns every robot's "
        "tasks. 'decentralized' (Mode C): fleet_coordination's per-robot "
        "agents negotiate peer-to-peer over /fleet/agent/claim, no central broker - "
        "--t-fail is meaningless here (there is no central node to kill) and rejected. "
        "'centralized_then_failover' (Modes C/D/E as failover targets): "
        "starts centralized; per-robot agents stand by DORMANT watching "
        "/fleet/manager/heartbeat and activate their --agent-backend cycle once it's "
        "been missing for --manager-timeout-sec - pairs with --t-fail for a fair "
        "recovery comparison against Mode B's no-recovery baseline.",
    )
    parser.add_argument(
        "--manager-timeout-sec",
        type=float,
        default=5.0,
        help="Only used with --coordination centralized_then_failover: seconds of "
        "missing /fleet/manager/heartbeat before agents activate decentralized failover.",
    )
    parser.add_argument(
        "--agent-backend",
        choices=["rule", "llm", "hybrid"],
        default="rule",
        help="Only used with --coordination decentralized. 'rule' (default, "
        "RuleAgent), 'llm' (LLMAgent, backed by a local Ollama server, "
        "falling back to RuleAgent on malformed/unsafe/unreachable-server responses), "
        "or 'hybrid' (Mode E: RuleAgent by default, LLMAgent only for "
        "ambiguous near-tied decisions).",
    )
    parser.add_argument(
        "--memory-enabled",
        action="store_true",
        help="Enables episodic peer memory. Only used with --coordination "
        "decentralized or centralized_then_failover. Each agent tracks real "
        "ACCEPT/REJECT/TIMEOUT outcomes of task-transfer negotiations it initiated, "
        "per peer, and nudges Conversation.select_winner toward historically-reliable "
        "peers. Off by default (unchanged prior behavior).",
    )
    parser.add_argument(
        "--time-source",
        choices=["wall", "simulation"],
        default="wall",
        help="Selects the clock used for --duration/--t-fail. 'wall' (default): "
        "--duration/--t-fail are host wall-clock seconds. 'simulation': --duration/"
        "--t-fail are simulated seconds, tracked via "
        "/clock (bridged from Gazebo by fleet.launch.py's clock_bridge node) - fixes "
        "this project's documented WSL2 realtime factor (~0.07-0.1x) making short "
        "wall-clock runs measure almost no simulated activity. event_logger is also "
        "told to log a simulation_time column alongside its existing wall-clock one.",
    )
    parser.add_argument(
        "--clock-wait-timeout",
        type=float,
        default=90.0,
        help="Only used with --time-source simulation: seconds to wait for the first "
        "/clock message before giving up (Gazebo bring-up, not instant).",
    )
    args = parser.parse_args()

    if args.t_fail is not None and args.duration is None:
        print("ERROR: --t-fail requires --duration", file=sys.stderr)
        sys.exit(1)
    if args.t_fail is not None and args.t_fail >= (args.duration or float("inf")):
        print("ERROR: --t-fail must be less than --duration", file=sys.stderr)
        sys.exit(1)
    if args.t_fail is not None and args.coordination == "decentralized":
        print(
            "ERROR: --t-fail requires --coordination centralized or "
            "centralized_then_failover (there is no single central node to kill in "
            "pure decentralized mode)",
            file=sys.stderr,
        )
        sys.exit(1)

    if shutil.which("ros2") is None:
        print(
            "ERROR: 'ros2' not found on PATH. Source your ROS 2 Jazzy "
            "workspace (and colcon build this repo) before running this.",
            file=sys.stderr,
        )
        sys.exit(1)

    robot_namespaces = [f"robot{i}" for i in range(1, args.num_robots + 1)]

    run_dir = make_run_dir(args.mode)
    write_metadata(
        run_dir,
        mode=args.mode,
        num_robots=args.num_robots,
        seed=args.seed,
        ros_distro=_ros_distro(),
        gazebo_version="harmonic",
        t_fail_sec=args.t_fail,
        coordination=args.coordination,
        agent_backend=args.agent_backend,
        time_source=args.time_source,
        # Matches OllamaClient's own defaults (agent_core/ollama_client.py)
        # - decentralized_agent.py always constructs a plain OllamaClient() with no
        # override when agent_backend is llm/hybrid, so these are genuinely what ran,
        # not a guess.
        llm_provider="ollama" if args.agent_backend in ("llm", "hybrid") else None,
        llm_model="llama3.2:1b" if args.agent_backend in ("llm", "hybrid") else None,
        memory_enabled=args.memory_enabled,
    )
    print(f"[run_experiment] experiment directory: {run_dir}")

    fleet_proc = _popen_own_group(
        [
            "ros2",
            "launch",
            "amr_ros_dg",
            "fleet.launch.py",
            f"num_robots:={args.num_robots}",
            f"seed:={args.seed}",
            f"coordination:={args.coordination}",
            f"agent_backend:={args.agent_backend}",
            f"manager_timeout_sec:={args.manager_timeout_sec}",
            f"memory_enabled:={'true' if args.memory_enabled else 'false'}",
            f"headless:={'true' if args.headless else 'false'}",
        ]
    )

    logger_proc = _popen_own_group(
        [
            "ros2",
            "run",
            "experiment_manager",
            "event_logger",
            "--ros-args",
            "-p",
            f"run_dir:={run_dir}",
            "-p",
            "robot_namespaces:=[" + ",".join(robot_namespaces) + "]",
            "-p",
            f"time_source:={args.time_source}",
            "-p",
            f"use_sim_time:={'true' if args.time_source == 'simulation' else 'false'}",
        ]
    )

    bag_topics = ["/fleet/experiment/event"]
    for ns in robot_namespaces:
        bag_topics += [
            f"/{ns}/navigate_to_pose/_action/status",
            f"/{ns}/diff_drive_controller/odom",
            f"/{ns}/diff_drive_controller/cmd_vel",
        ]
    if args.coordination in ("decentralized", "centralized_then_failover"):
        bag_topics += [
            "/fleet/agent/claim",
            "/fleet/agent/task_transfer",
            "/fleet/agent/decision",
            "/fleet/agent/coordination_mode",
        ]
    if args.coordination in ("centralized", "centralized_then_failover"):
        bag_topics += ["/fleet/manager/heartbeat"]

    health_proc = None
    if not args.no_health:
        health_args = [
            "ros2", "launch", "robot_health", "health.launch.py",
            f"num_robots:={args.num_robots}",
            f"schedule_file:={args.fault_schedule}",
        ]
        health_proc = _popen_own_group(health_args)
        bag_topics += ["/fleet/faults/command"]
        bag_topics += [f"/{ns}/health/state" for ns in robot_namespaces]

    bag_proc = _popen_own_group(
        ["ros2", "bag", "record", "-o", str(run_dir / "rosbag" / "run"), *bag_topics]
    )

    sim_time_source = None
    sim_start = None
    if args.time_source == "simulation":
        sim_time_source = _SimTimeSource()
        print("[run_experiment] --time-source simulation: waiting for first /clock message...")
        if not sim_time_source.wait_for_clock(args.clock_wait_timeout):
            # Honest failure, not a silent fall-back to wall-clock (section 42: never
            # hide a timeout) - a run whose duration was requested in simulated seconds
            # but never actually got simulation time would silently run forever (or not
            # at all), which is worse than refusing to start.
            print(
                f"[run_experiment] ERROR: no /clock message received within "
                f"{args.clock_wait_timeout:.0f}s - is Gazebo actually running (not "
                f"paused), and is clock_bridge up? Aborting.",
                file=sys.stderr,
            )
            sim_time_source.shutdown()
            for proc in (bag_proc, logger_proc, fleet_proc):
                _terminate_group(proc)
            if health_proc is not None:
                _terminate_group(health_proc)
            _reap_stray_gz_sim()
            sys.exit(1)
        sim_start = sim_time_source.sim_now()
        print(f"[run_experiment] simulation clock acquired, sim_time={sim_start:.2f}s")

    def _elapsed_sec() -> float:
        """Elapsed time on whichever clock --time-source selected - simulated seconds
        via /clock if 'simulation', otherwise host wall-clock seconds (unchanged
        default behavior)."""
        if sim_time_source is not None:
            now = sim_time_source.sim_now()
            return (now - sim_start) if now is not None else 0.0
        return time.time() - start

    start = time.time()
    try:
        if args.duration is not None:
            fleet_manager_killed = False
            # Sleep in small increments (rather than one time.sleep(args.duration)) so
            # --t-fail can be checked and acted on partway through, instead of only
            # being able to signal shutdown at the very end.
            while _elapsed_sec() < args.duration:
                if (
                    args.t_fail is not None
                    and not fleet_manager_killed
                    and _elapsed_sec() >= args.t_fail
                ):
                    _kill_fleet_manager(run_dir, args.t_fail, sim_time_source=sim_time_source)
                    fleet_manager_killed = True
                time.sleep(0.5)
        else:
            fleet_proc.wait()
    except KeyboardInterrupt:
        pass
    finally:
        duration = time.time() - start
        simulation_duration = None
        if sim_time_source is not None:
            final_sim_now = sim_time_source.sim_now()
            if final_sim_now is not None and sim_start is not None:
                simulation_duration = final_sim_now - sim_start
            sim_time_source.shutdown()
        procs = [bag_proc, logger_proc, fleet_proc]
        if health_proc is not None:
            procs.insert(0, health_proc)
        # Sequential, not "signal all then wait all": bag/logger first (so they capture
        # the fleet's own shutdown messages), fleet last - and _terminate_group's SIGKILL
        # fallback targets the whole process GROUP (see its docstring), not just the
        # direct child, so grandchildren like `gz sim` can't be left running.
        for proc in procs:
            _terminate_group(proc)
        _reap_stray_gz_sim()
        finalize_duration(run_dir, duration, simulation_duration_sec=simulation_duration)
        if simulation_duration is not None:
            print(
                f"[run_experiment] simulated duration={simulation_duration:.1f}s, "
                f"wall duration={duration:.1f}s, realtime_factor="
                f"{(simulation_duration / duration if duration else float('nan')):.3f}"
            )
        # Phase 10: "at termination - save all logs, calculate summary, generate
        # plots, print experiment directory." Logs are already flushed by this point
        # (event_logger/rosbag were the first two processes terminated above, before
        # fleet/gazebo); summary.json and plots/ are calculated from those now-final
        # files. Never let a summarization bug take down an otherwise-successful run -
        # the raw data is still valid and usable even if this step fails.
        try:
            summary_path = write_summary(run_dir)
            plot_paths = generate_plots(run_dir)
            print(f"[run_experiment] summary: {summary_path}")
            for p in plot_paths:
                print(f"[run_experiment] plot: {p}")
        except Exception as exc:  # noqa: BLE001 - deliberately broad, see comment above
            print(f"[run_experiment] WARNING: summary/plot generation failed: {exc}")
        print(f"[run_experiment] done. duration={duration:.1f}s dir={run_dir}")


if __name__ == "__main__":
    main()
