# Phase II pilot report

Campaign: `phase2/experiments/pilot_001` - 20/20 runs succeeded, 23271.06s total wall-clock.

## Interaction density per condition (mean over paired seeds)

| Condition | Seeds | Tasks | Negotiations | Charging cycles | Resource conflicts (blocked) |
| --- | --- | --- | --- | --- | --- |
| deterministic_no_memory | 5 | 713.8 | 48.6 | 48 | 213 |
| llm_no_memory | 5 | 675.6 | 47.8 | 47 | 306 |
| llm_memory | 5 | 676.6 | 48 | 47.4 | 270.6 |
| hybrid_memory | 5 | 683.6 | 48 | 48 | 374.8 |

## Role formation: top-robot persistence ratio per condition

| Condition | long_distance_specialist | local_task_specialist | high_utilization_worker | helper |
| --- | --- | --- | --- | --- |
| deterministic_no_memory | 0.4 | 0.52 | 0.58 | n/a |
| llm_no_memory | 0.46 | 0.44 | 0.52 | 1.0 |
| llm_memory | 0.44 | 0.48 | 0.64 | 1.0 |
| hybrid_memory | 0.44 | 0.36 | 0.62 | 1.0 |

## Peer-preference concentration (memory ablation: llm_memory vs llm_no_memory)

- llm_no_memory: n=1, mean concentration index = 1.0
- llm_memory: n=1, mean concentration index = 1.0

## Resource (charger) conventions per condition

| Condition | Jain fairness | Lower-SOC-priority rate | Round-robin score | Charging cycles |
| --- | --- | --- | --- | --- |
| deterministic_no_memory | 1.0 | 0.451 | 1.0 | 48 |
| llm_no_memory | 0.999 | 0.608 | 1.0 | 47.6 |
| llm_memory | 1.0 | 0.509 | 1.0 | 48 |
| hybrid_memory | 1.0 | 0.428 | 1.0 | 48 |

## Interaction-network hub concentration per condition

| Condition | Runs with a hub | Mean hub degree centrality |
| --- | --- | --- |
| deterministic_no_memory | 5/5 | 1.4333 |
| llm_no_memory | 5/5 | 1.7667 |
| llm_memory | 5/5 | 1.6667 |
| hybrid_memory | 5/5 | 1.7333 |

## Emergence criteria evaluation

- **long_distance_specialist**: verdict = `not_established`
- **local_task_specialist**: verdict = `not_established`
- **high_utilization_worker**: verdict = `candidate_emergent_collective_behavior`
- **helper**: verdict = `not_established`
- **peer_preference_concentration**: verdict = `not_established`
