# Known Limitations

Honest, technical status of each major subsystem in this platform. This
document is intentionally specific about what has been implemented, what has
been validated against live simulation runs, and what gaps remain open - it
is not a marketing summary.

## Simulation-time experiment control

`run_experiment.py --time-source simulation` measures `--duration`/`--t-fail`
in simulated seconds (via `/clock`, bridged from Gazebo by
`fleet.launch.py`'s `clock_bridge` node) instead of host wall-clock seconds.
`event_logger_node.py` logs a `simulation_time` column alongside its existing
wall-clock `timestamp` in every output file when `time_source:=simulation` is
passed. `metadata.json` records `time_source`, `simulation_duration_sec`, and
a computed `realtime_factor`.

Validated end to end: simulated durations reliably measure the intended
simulated window regardless of the host's realtime factor (observed in the
~0.03-0.1x range depending on system load - see "Structural notes" below).
`events.jsonl` and per-robot CSV logs carry correctly-increasing
`simulation_time` values throughout a run. One cosmetic edge case: the very
first `EXPERIMENT_LOGGER_STARTED` event can log a `null` `simulation_time`
because it fires before the logger's `/clock` subscription has received its
first message - every subsequent event is populated correctly, so this is
not a defect.

`--time-source` defaults to `wall`, so existing scripts/docs that call
`run_experiment.py` without the new flag are unaffected; `simulation` mode is
opt-in per run.

**Known gap:** no dedicated stress test exists for Gazebo `/clock` publish-rate
jitter under CPU pressure. The acceptance test suite covers the underlying
logic (a fake `/clock` publisher advancing at a configurable rate drives
correct termination regardless of rate) but not a live run under load. If a
future run shows `--t-fail` firing late or `/clock` messages arriving in
bursts, this is the mechanism to check first.

## Failover: centralized to decentralized

`fleet_manager.py` publishes `FleetManagerHeartbeat` on
`/fleet/manager/heartbeat` (1 Hz, `use_sim_time`). A `--coordination
centralized_then_failover` mode launches both `fleet_manager` and every
robot's `decentralized_agent` from the start - agents begin `DORMANT` (no
claims, no Nav2 goals) and watch the heartbeat via a ROS-free
`HeartbeatMonitor` (`fleet_coordination/heartbeat.py`). Once the heartbeat has
been missing for `manager_timeout_sec`, an agent activates its normal
decentralized decision cycle (rule/LLM/hybrid backend) and publishes
`CoordinationModeChanged`, logged by `event_logger_node.py` as a
`COORDINATION_MODE_CHANGED` event. `RobotHealth.msg`'s
`central_manager_alive` field is a separate, still-unused mechanism - the
real heartbeat check lives entirely on the `/fleet/manager/heartbeat` topic.

A real bug was found and fixed during validation: an early implementation
compared heartbeat freshness against wall-clock `time.monotonic()`, while
`fleet_manager`'s heartbeat timer runs on simulated time. At a low realtime
factor this caused agents to falsely declare failover almost immediately
while the manager was still alive. Fixed by comparing heartbeat freshness
using the agent's own sim-time clock (`decentralized_agent.py`'s
`_sim_now()`).

Validated live: `CENTRAL_MANAGER_KILLED` and `COORDINATION_MODE_CHANGED`
fire at the expected simulated timestamps (`t_fail + manager_timeout_sec`,
exact to within the logging tick), and post-failover claim contention (e.g.
two robots broadcasting a claim on the same station simultaneously) resolves
correctly through the existing `ClaimBook` rules.

**Known limitation:** there is no reverting to `DORMANT`/`CENTRALIZED` if a
heartbeat reappears mid-run - a resurrected `fleet_manager` is out of scope.
Double-claim collisions right after failover are resolved by the existing
`ClaimBook` rule but are not separately measured/counted as a dedicated
metric.

## Task completion and recovery metrics

`summarize_run.py` computes `resilience.recovered` by pairing each robot's
`EXECUTING`->`SUCCEEDED` events and only counting a cycle toward recovery
when its *assignment* timestamp is after the injected fault - a task already
in flight before the fault that simply finishes late does not count as
recovery. This fixed an earlier false-positive where any post-fault
completion (including one assigned pre-fault) was counted as recovery. Also
computed: `pre_failure_tasks_completed`, `post_failure_tasks_completed`,
`pre_failure_throughput_per_hour`, `post_failure_throughput_per_hour`, and
`throughput_retention_ratio` (only computed when the run duration exceeds
`t_fail_sec`; `None`, not `0`, when there is no pre-failure activity to
divide by). Both wall-clock and simulation-time bases are supported. See
`docs/recovery_definition.md` for the full definition and rationale.

Validated live: a genuine post-failover assignment and completion was
observed and correctly reported (`recovered: true`, a positive
`recovery_time_sec`, and `throughput_retention_ratio: null` when the
pre-failure window had zero completions - the honest value, not a fabricated
zero).

A related bug was found and fixed: the resilience computation used
`next(...)` to find the run's start-of-simulation event, and a `null`
`simulation_time` on that first logged event (see above) was wrongly treated
as "no such event," silently skipping the entire resilience block. Fixed by
defaulting the simulation clock's start timestamp to `0.0` (correct by
construction, since every observed `CENTRAL_MANAGER_KILLED.simulation_time`
lands within noise of `t_fail_sec` with no offset).

## LLM observability

`agent_core/llm_agent.py` has a `SchemaError`/`SafetyError` exception split
(malformed/missing-field/disallowed-action vs. structurally-valid-but-unsafe)
and a `DecisionMeta` dataclass (`provider`, `model`, `prompt_tokens`,
`completion_tokens`, `schema_valid`, `safety_valid`, `fallback_used`,
`retry_count`) exposed as `LLMAgent.last_decision_meta` after every
`decide()` call, on both the success and fallback paths.
`agent_core/ollama_client.py` captures Ollama's own
`prompt_eval_count`/`eval_count` response fields into
`last_prompt_tokens`/`last_completion_tokens`. `agent_core/hybrid_agent.py`
mirrors `last_decision_meta` from the underlying `LLMAgent` on escalation,
`None` on the deterministic path. `amr_interfaces/msg/AgentDecision.msg`
carries `has_llm_meta`, `provider`, `model`, `prompt_tokens`/
`completion_tokens` (-1 sentinel), `schema_valid`, `safety_valid`,
`fallback_used`, `retry_count`, wired through `decentralized_agent.py` and
persisted to `agent_decisions.jsonl` by `event_logger_node.py`.
`summarize_run.py`'s `decision_summary` includes an `llm` block
(request/success/schema-failure/safety-rejection/fallback/retry counts,
mean/median/p95 latency, total prompt/completion tokens) and a `hybrid` block
(deterministic-decision count, LLM-escalation count and rate).

Validated against a real local Ollama server (`llama3.2:1b`): every decision
in the validation run carried real `DecisionMeta` with genuinely varying
token counts, all schema- and safety-valid, and decision latency roughly two
orders of magnitude slower than the rule-based backend - a real,
expected cost-of-intelligence signature. A couple of decisions needed a real
retry before succeeding, exercising that path against genuine model output
rather than a mock.

## Multi-seed comparison and statistics

`analysis/scripts/generate_report.py` implements
`compute_metric_statistics()` (n, mean, median, stdev, min, max, p95, and a
percentile-bootstrap 95% CI on the mean, with a seeded RNG for determinism;
`None` for CI/stdev when n<=1, with no assumption of normality) and
`compute_paired_differences()` (matches runs across two coordination modes by
`random_seed`, computes per-pair metric differences, reuses the same
statistics helper). `render_detailed_statistics()` renders a per-metric
statistics table plus a paired-differences table when a mode pair shares
seeds.

A real bug was found while preparing a multi-run batch: `run_batch.py` had
not been threaded through with the `--time-source simulation` control that
single runs already supported, so every batch run to that point was
wall-clock only (with most of `--duration` spent on Nav2/SLAM bring-up
rather than the experiment itself). Fixed: `build_run_command`/`run_one`/
`run_batch`/the CLI all accept `--time-source`, recorded in
`batch_manifest.json`.

**Honest scope statement:** the small pilot batches run against this
pipeline (N=2-3 seeds per mode) confirm the entire statistics pipeline
(batch runner -> per-run summary -> cross-mode aggregation -> bootstrap CI ->
paired differences -> rendered report) is correct and produces real numbers
from real simulation data. With N in the low single digits, the reported
confidence intervals and paired differences are a pipeline-correctness
demonstration, not a statistically powered comparison. A campaign with
10-30 seeds per mode is supported by `run_batch.py --seeds` but was not
run to completion in this environment, given the multi-hour wall-clock cost
per mode at this host's measured realtime factor.

## Health degradation

`ACCELERATED_BATTERY_DISCHARGE` (a x5 discharge multiplier at severity 1.0,
defined in `robot_health/faults.py`) is injected via the fault schedule
`src/robot_health/config/faults/battery_degradation.yaml`, and
`summarize_run.py` computes a `degradation` block: per-robot task counts and
workload share before vs. after the fault's onset timestamp.

Validated live: the fault was genuinely injected and had a measured effect - 
the affected robot ended the run at a visibly lower battery state of charge
than its peer, confirming the discharge multiplier took effect rather than
being logged-but-inert.

**Honest negative result:** workload was not redistributed away from the
degraded robot in the validation run - both robots completed an equal
share of tasks. This is because the deterministic cost formula
(`0.7*distance + 0.3*energy_risk`) only mildly penalizes moderate battery
loss, and the affected robot's battery never approached the low-battery
threshold that triggers a task-transfer negotiation within the run's
duration. This is a genuine finding (decentralized coordination as currently
weighted does not proactively redistribute work in response to moderate
battery degradation, only reacts once battery crosses a hard emergency
threshold), not an experiment failure.

**Known design limitation:** in the validated run, the fault's onset offset
was measured from fault-injector node startup (wall-clock), not simulation
time, so at a low realtime factor it fired almost immediately - leaving no
real pre-fault baseline window to contrast against. A future run should
either account for this offset in wall-clock terms, or switch to a direct
fault command timed off simulation time, to get a genuine pre/post contrast.

## Peer memory / negotiation

`agent_core/peer_memory.py` implements `PeerMemory` - per-peer
`successful_help`/`rejected_requests`/`timed_out_requests` counts, response
times, and a derived reliability score in [0, 1] - wired into
`fleet_coordination/negotiation.py`'s `Conversation.select_winner()` as an
optional bounded nudge (never overriding the deterministic lowest-cost-wins
rule by more than `reliability_weight`, default 0.05), and into
`decentralized_agent.py` via a `memory_enabled` parameter (off by default)
that records real accept/reject outcomes from task-transfer negotiations a
robot initiated. Threaded through the launch files and `run_experiment.py`'s
`--memory-enabled` flag, and recorded in `metadata.json`.

**Known, documented gap:** memory state is not persisted to disk. It only
affects in-process decisions during the run itself; a completed run's
peer-interaction history cannot currently be reconstructed after the fact.
Persisting it would need a new message type and event-logger wiring.

Validated live for crash-safety across real `OFFER`->`TIMEOUT` task-transfer
negotiations, but the `record_outcome()` accept/reject recording path itself
was not exercised live in the available 2-robot validation runs - with only
two robots, the peer robot was always mid-task and never had the
opportunity to bid, so only the `TIMEOUT` path (which never calls
`record_outcome`) was exercised outside of unit tests. Exercising the
accept/reject path live would need three or more robots, or a staggered-start
scenario.

## Charging scarcity

A minimal charging-scarcity mechanism exists: a fixed-coordinate charger
station (reusing the existing `StationClaim`/`ClaimBook` broadcast machinery
as a capacity-1 station) and a `CHARGING_BOOST` fault effect (reusing the
existing declarative fault-command system to raise `battery_soc` while a
robot is charging), rather than a dedicated Gazebo charger entity or new
message types.

Validated live: the core recharge mechanism works unambiguously - a robot's
battery state of charge rises correctly while stationary at the charger, and
the claim -> navigate -> charge -> release cycle completes end to end through
Gazebo/Nav2.

**Known, confirmed race condition - single-slot scarcity is not enforced.**
An initial validation run showed two robots charging concurrently rather
than sequentially. The first hypothesis (the charging trip being
indistinguishable from a normal task to the low-battery task-transfer
trigger, and therefore offered away to a peer) was fixed by excluding
`side=="charging"` claims from that trigger, but a follow-up run showed
concurrent charging persisting. The actual root cause is a genuine race in
the one-round `ClaimBook` contention protocol: at a low realtime factor,
wall-clock message-passing delay can exceed the contention window, so two
robots claiming within a fraction of a second of each other (in simulated
time) can each resolve their own contention window before receiving the
other's claim broadcast. This breaks down specifically for the charging use
case because low-battery events tend to be temporally correlated across
robots (similar discharge schedules), pushing claims closer together than
normal task-station contention typically sees.

This is a known, documented limitation, not a claimed success: recharge
behavior works, but the mutual-exclusion property does not hold under this
protocol as currently timed. Candidate fixes not yet implemented: lengthen
the contention window specifically for charger contention, make charger
claim resolution two-round (broadcast intent, wait, then confirm) instead of
one-round, or have the charger arbitrate via a single authoritative claim
owner instead of distributed consensus by broadcast timing.

No communication-delay/packet-loss injection, task-surge injection, or
emergence-analysis metrics (specialization entropy, interaction-graph
centrality, etc.) are implemented in this line of experiments - see
`phase2/` for the separate, isolated emergent-behavior analysis suite. No
3-robot charging-scarcity scenario has been validated (see "Structural
notes" below on this platform's tested robot-count range).

## Resilience comparison (no-recovery / deterministic / LLM / hybrid failover)

`run_batch.py`'s `--coordination-modes` accepts `centralized_then_failover`
with `--agent-backend` passed through (previously only supported for plain
`decentralized`), a `--manager-timeout-sec` CLI flag, and an `--append` mode
that merges multiple invocations' runs into one `batch_manifest.json` (needed
because deterministic/LLM/hybrid failover targets each need a different
`--agent-backend` in one invocation). `generate_report.py` implements
`build_resilience_table()`/`write_resilience_csv()`, producing
`analysis/results/resilience_summary.csv` with columns for recovery outcome,
recovery time, throughput retention, energy per task, safety, and decision
latency, restricted to runs that had a fault injected and labeled by
recovery strategy (No Recovery / Deterministic / LLM / Hybrid).

A paired-seed pilot campaign (`analysis/scripts/run_resilience_campaign.sh`)
comparing the four modes across matched seeds produced the summary in
`analysis/results/resilience_summary.csv`. At N=3 paired seeds:

| Mode | Recovery | Recovery Time | Decision Latency (mean) |
| --- | --- | --- | --- |
| No Recovery | 0/3 | no recovery | not applicable |
| Deterministic | 3/3 | ~28s | ~0.07ms |
| LLM | 2/3 | ~31s | ~5.6s |
| Hybrid | 2/3 | ~30s | ~5.2s |

The completed-task counts in the no-recovery mode were all assignments made
before the induced fault that simply finished late - no task was ever
assigned after the failure in any of the three seeds, confirming the
"no recovery" mode genuinely never reassigns work. The deterministic mode's
3/3 result was consistent and tightly clustered across seeds. The LLM/hybrid
modes' non-recovering seeds are explained by real, measured data rather than
a bug: LLM/hybrid mean decision latency (multiple seconds) is several orders
of magnitude higher than the deterministic backend's (well under a
millisecond), and this latency gap directly competes with the fixed
post-failure time budget for completing a task at all. In this campaign's
scenario geometry, the hybrid controller's LLM-escalation rate was 1.0 in
every run (i.e. it never took its cheaper deterministic fast path), because
the world's symmetric robot-start geometry makes early post-failover
decisions genuinely near-tied - a scenario-geometry effect worth controlling
for (e.g. asymmetric spawns) in a larger campaign, not a design flaw. One LLM
decision was rejected live by the schema/safety validator and correctly fell
back, direct evidence the validation pipeline functions against genuine
(not synthetic) model output.

**Honest scope statement:** N=3 is a pilot, not a large-sample campaign.
`run_batch.py --seeds` supports an arbitrary range; the bound is wall-clock
cost per run on this environment, not the pipeline itself. The deterministic
vs. LLM/hybrid recovery-rate gap (3/3 vs 2/3) is suggestive at this sample
size, not statistically conclusive. The fixed recovery-time budget used for
this campaign was reused from a single prior validated timing, not
independently swept - the LLM/hybrid modes' non-recovering seeds might
recover reliably given a longer budget; this was not tested.

A tooling bug was found and fixed during this campaign's reporting step (not
the simulation pipeline itself): the CSV writer and LaTeX-table generator
opened files without an explicit encoding, so a "+/-" character written on
one platform's default encoding could decode incorrectly when converted to a
table on a different platform's default encoding. Fixed by specifying
`encoding="utf-8"` on both the write and read sides.

## Structural notes that apply across the whole platform

- **Robot count:** live validation throughout this project has used 2
 robots; 3+ robots has not been confirmed reliable in this development
 environment (see `docs/reproducibility.md` and `TROUBLESHOOTING.md`).
 Any claim about 3-robot behavior is a projection from 2-robot data, not a
 direct measurement, until a host is confirmed to run 3+ robots reliably.
- **Realtime factor varies by session:** observed in roughly the 0.03-0.1x
 range depending on host load - consistent but not a fixed constant. Any
 `--duration`/`simulation_duration_sec` planning should budget for the low
 end of this range, not assume the middle.
- **No emergent behavior has been searched for in this line of experiments.**
 Nothing in this document should be read as evidence toward or against
 emergence - that question is addressed separately by the analysis suite
 under `phase2/`.
