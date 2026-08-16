# Recovery Definition

## What "recovered" means in this platform's data

A fault-injected run (`t_fail_sec` set in `metadata.json` - centralized or
`centralized_then_failover`) is reported as `recovered: true` by
`summarize_run.py`'s `resilience` block **only if at least one task was
both assigned (`NAV_GOAL_STATUS_CHANGED` -> `EXECUTING`) AND completed
(`-> SUCCEEDED`) entirely after `t_fail_sec`.**

```python
recovered = any(assigned_ts > fail_ts for assigned_ts, completed_ts in cycles
 if completed_ts > fail_ts)
```

`recovery_time_sec` is the gap, in whichever clock basis the run used
(simulated seconds under `--time-source simulation`, wall-clock seconds
otherwise - see `docs/reproducibility.md`), between the fault and the first
such genuinely-post-fault task's completion.

## Why "assigned after the fault," not just "completed after the fault"

An earlier implementation of this metric only checked "did any task SUCCEED
after the fault." This produces a **real, live-confirmed false positive**:
Nav2 keeps executing whatever goal it already has in flight independent of
whether the central coordinator that assigned it is still alive - so a task
assigned *before* the fault that simply takes a long time to finish will
show up as a `SUCCEEDED` event after `t_fail_sec`, even though nothing about
the coordination system actually recovered. A validation run hit exactly
this: both completed tasks were assigned well before the induced fault, and
the old metric still reported `"recovered": true`. Fixed by requiring the
*assignment*, not just the completion, to fall after the fault - 
`summarize_run.py`'s `_completed_task_cycles()`.

## Clock basis

Under `--time-source simulation`, `t_fail_sec` is itself measured in
**simulated** seconds (see `docs/reproducibility.md`). The resilience
computation must compare it against `simulation_time`, not the wall-clock
`timestamp` field - comparing a simulated-seconds fault offset against a
wall-clock event stream is a unit mismatch, the same class of bug found and
fixed in the failover heartbeat-timeout logic (see
`docs/current_limitations.md`). `summarize_run.py` selects the clock field
from `metadata.json`'s own `time_source` value, so this is handled
automatically and correctly regardless of which mode a run used.

## Pre/post-failure throughput and retention ratio

Also computed in the `resilience` block, only when the run's total duration
exceeds `t_fail_sec` (so both windows are non-empty):

- `pre_failure_tasks_completed` / `post_failure_tasks_completed`: counts of
 completed task cycles (`_completed_task_cycles`, same pairing as above,
 requiring a real `EXECUTING`->`SUCCEEDED` pair) whose completion falls
 before/after the fault.
- `pre_failure_throughput_per_hour` / `post_failure_throughput_per_hour`:
 those counts divided by the pre-fault window (`t_fail_sec`) and post-fault
 window (`total_duration - t_fail_sec`) respectively, in tasks/hour.
- `throughput_retention_ratio`: `post_failure_throughput_per_hour /
 pre_failure_throughput_per_hour`, only computed when the pre-failure
 throughput is non-zero (a zero or missing denominator makes the ratio
 undefined - reported as `None`/`not measured`, never a fabricated `0` or
 `inf`).

This is a simpler operational definition than a richer alternative (a
sliding-window throughput threshold - "recovered once throughput over a
trailing window returns to >= 80% of pre-failure throughput"). That richer,
windowed definition is **not implemented** - this platform's
`recovered`/`recovery_time_sec` answer a narrower, more directly measurable
question ("did coordination resume assigning new work at all, and how long
did that take") rather than "did full productivity return." Implementing the
sliding-window definition would need bucketing `_completed_task_cycles()`'s
completions into a rolling window and comparing each window's rate against
the pre-failure rate - a real, larger addition, not implemented here, and
not silently approximated by the current fields.

## What this does NOT measure

- Whether the *specific task* fleet_manager had assigned before dying ever
 got completed by anyone (decentralized failover has no knowledge of what
 was centrally reserved - see the documented double-claim limitation in
 `docs/current_limitations.md`).
- Multi-robot partial recovery (one robot recovers, another never does) -
 the fleet-wide `recovered` flag is `true` as soon as ANY robot's
 post-fault task succeeds, not a per-robot breakdown.
- Recovery quality/efficiency - only that new work started and finished at
 all, not whether it was assigned efficiently.
