# Technical Notes

Short summary of how the platform works, what its metrics mean, and what's
still limited. Kept intentionally brief; the code and tests are the source
of truth.

## Architecture

`fleet.launch.py` brings up Gazebo, one Nav2 + SLAM stack per robot
(namespaced `robot{i}`), and a coordination layer that runs in one of
several modes: a single centralized `fleet_manager` node, fully
decentralized peer agents, or a hybrid that starts centralized and fails
over to decentralized if the manager's heartbeat goes silent. Each
decentralized agent picks its next action with a rule-based, LLM-backed, or
hybrid (rule-first, LLM-escalation) decision backend. Station claims and
task transfers between robots go through a broadcast claim protocol
(`fleet_coordination/`), not a central lock, once agents are decentralized.

## Reproducibility

`run_experiment.py --time-source simulation` measures durations and fault
timing in simulated seconds (via Gazebo's `/clock`) instead of wall-clock
seconds, so results don't depend on host load. Every run writes an
immutable output directory under `experiments/<run_id>/` with raw event
logs, per-robot telemetry, a rosbag, and a `summary.json`, enough to
reconstruct what happened without re-running anything.

## Recovery metric

A run is marked `recovered: true` only if at least one task was both
assigned and completed entirely after the injected fault. Completions that
were already in flight before the fault don't count, even if they finish
late, since Nav2 keeps executing goals it already has regardless of whether
the coordinator that assigned them is still alive. `recovery_time_sec` is
the gap between the fault and the first genuinely post-fault completion.
Throughput retention (`post_failure / pre_failure` task rate) is reported
as `not measured` rather than a fabricated number when there's no
pre-failure activity to compare against.

## Known limitations

- Live validation has used at most 2 robots (Family I) and 4 robots
  (`phase2/`). Claims about larger fleets are projections, not direct
  measurements.
- Seed counts are pilot scale (3-5 paired seeds), enough to confirm the
  pipeline is correct end to end, not enough for a statistically powered
  comparison.
- No collision-based safety metric; only Nav2's own `collision_monitor`
  interventions are logged.
- Charging-scarcity mutual exclusion has a known race: two robots can end
  up charging concurrently under the current one-round claim protocol when
  their low-battery events land close together in time.
- Peer-memory state lives only in process memory for the duration of a run
  and isn't persisted to disk.
- No communication-delay, packet-loss, or task-surge fault injection, and
  no emergence-analysis metrics in the Family I line of experiments; that
  side of things is covered separately by `phase2/`.

Full run-by-run numbers backing these results are in
`analysis/results/resilience_summary.csv` and `phase2/analysis/results/`.
