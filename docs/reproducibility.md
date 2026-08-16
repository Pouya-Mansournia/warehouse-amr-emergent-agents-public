# Reproducibility

## Simulation-time experiment control

Every experiment metric this platform reports (throughput, task duration,
recovery time, decision latency) is only comparable across runs, seeds, and
sessions if the *thing being measured* - elapsed experiment time - means the
same thing in every run. Host wall-clock time does not satisfy this: WSL2
hosts have measured realtime factors from ~0.07x to ~0.5x depending on
session load (see `docs/current_limitations.md`), so a `--duration 300`
(wall-clock) run captures a wildly different amount of *simulated* activity
depending on the host's momentary load - a direct cause of spurious
zero-throughput results if not accounted for.

### Using `--time-source simulation`

```bash
ros2 run experiment_manager run_experiment \
 --mode my_experiment --num-robots 2 --seed 7 \
 --coordination centralized --time-source simulation \
 --duration 1800 --t-fail 600
```

With `--time-source simulation` (the opt-in alternative to the default
`--time-source wall`, which is unchanged from every prior phase):

- `--duration`/`--t-fail` are interpreted as **simulated seconds**, tracked
 via `/clock` (`rosgraph_msgs/msg/Clock`, bridged from Gazebo by
 `fleet.launch.py`'s `clock_bridge` node).
- `event_logger_node.py` is launched with `use_sim_time:=true` and
 `time_source:=simulation`, so every row it writes (`events.jsonl`,
 `robot_<ns>.csv`, `health_<ns>.csv`, `claims.csv`, `negotiations.csv`,
 `safety_events.csv`, `agent_decisions.jsonl`) carries a `simulation_time`
 column alongside the existing wall-clock `timestamp` column. Both are kept
 - `simulation_time` for anything semantically about "how long did the
 experiment run," `timestamp` for anything about real compute cost (e.g.
 LLM decision latency, which genuinely is a wall-clock question).
- `metadata.json` records `time_source: "simulation"`,
 `simulation_duration_sec` (the actual simulated duration observed), and a
 computed `realtime_factor = simulation_duration_sec / duration_sec` - a
 real, measured number for that specific run, not an assumed constant.
- `run_experiment.py` waits up to `--clock-wait-timeout` seconds (default
 90) for the first `/clock` message before starting the duration clock. If
 none arrives (Gazebo failed to start, or `clock_bridge` isn't up), the run
 aborts loudly with a non-zero exit code rather than silently running
 forever or falling back to wall-clock.

### Reading `simulation_time` from a completed run

`simulation_time` is `None`/blank in any row logged before this node's own
clock has received its first `/clock` message (a brief window right at
startup) - treat a blank value as "not yet known," not as t=0. Every value
after that first message is a real, monotonically-increasing simulated
second count from Gazebo's own clock, independent of how much wall-clock
time actually elapsed to get there.

### Backward compatibility

`--time-source` defaults to `wall`. Every existing script, batch config, and
prior experiment's data is unaffected - `simulation_time` columns/fields
simply don't appear (or appear as `null`) in wall-clock-mode runs, and every
prior consumer of `events.jsonl`/`robot_<ns>.csv`/`metadata.json` that reads
by column/key name (not position) continues to work unchanged.

### Live-validated realtime factor

A representative validation run (1 robot, `--time-source simulation
--duration 30`) measured 30 simulated seconds taking 332.2 wall-clock
seconds - `realtime_factor: 0.0904`. Use this as a planning baseline, not a
promise: budget generously (realtime factors as low as ~0.07x have been
observed depending on host load) when choosing `simulation_duration_sec` for
a real campaign, since bring-up overhead (Nav2/SLAM activation, a fixed
wall-clock delay before `fleet_manager` starts issuing goals) is excluded
from the simulated-duration budget but still consumes real wall-clock time
before the simulated clock is meaningfully "started."
