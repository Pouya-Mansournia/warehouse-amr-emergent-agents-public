"""Evaluate a candidate collective-behavior pattern against a 7-point
emergence bar. This module does not decide FOR the pattern - it
computes each criterion's supporting number and leaves the verdict
("candidate_emergent" / "not_established") to be read off the numbers, so a
"no" is exactly as visible as a "yes" (this project's own research-integrity
rule, applied here rather than only in Phase I - see docs/README.md
"Research integrity").

Criteria:
  1. Not explicitly encoded as a global coordination rule.
  2. Arises from repeated local interactions.
  3. Forms a measurable fleet-level pattern.
  4. Persists for a meaningful duration.
  5. Occurs across multiple independent runs (seeds).
  6. Frequency is meaningfully above an appropriate baseline.
  7. Cannot be trivially explained by a deterministic tie-break or fixed cost
     function.

Criteria 1 and 2 are architectural facts about `world/world.py` itself (true
by construction - no role/preference/convention is ever assigned by the
simulation code, only computed post-hoc by this analysis package) and are
reported as such rather than computed from data. Criteria 3-7 are computed.
"""
from __future__ import annotations

from typing import List, Optional

UNIFORM_BASELINE_MARGIN = 0.15  # how far above 1/N counts as "meaningfully above baseline"
MIN_MEANINGFUL_STREAK_WINDOWS = 2  # a single-window hold is a fluke, not persistence
MIN_SEED_COVERAGE_FRACTION = 0.6  # pattern must appear in most seeds to count as "multiple runs"
MEMORY_EFFECT_MARGIN = 0.10  # memory ON must beat memory OFF by this much to rule out "just cost formula"


def evaluate_pattern(
    *,
    pattern_name: str,
    num_robots: int,
    per_seed_persistence_ratio: List[float],
    per_seed_longest_streak: List[int],
    memory_on_mean: Optional[float],
    memory_off_mean: Optional[float],
) -> dict:
    baseline = 1.0 / num_robots
    n_seeds = len(per_seed_persistence_ratio)

    mean_persistence = round(sum(per_seed_persistence_ratio) / n_seeds, 4) if n_seeds else 0.0
    above_baseline = mean_persistence >= baseline + UNIFORM_BASELINE_MARGIN
    seeds_above_baseline = sum(1 for p in per_seed_persistence_ratio if p >= baseline + UNIFORM_BASELINE_MARGIN)
    seed_coverage = round(seeds_above_baseline / n_seeds, 4) if n_seeds else 0.0

    mean_streak = round(sum(per_seed_longest_streak) / n_seeds, 2) if n_seeds else 0.0
    persists = mean_streak >= MIN_MEANINGFUL_STREAK_WINDOWS

    multi_run = seed_coverage >= MIN_SEED_COVERAGE_FRACTION

    memory_effect = None
    not_just_cost_formula = None
    if memory_on_mean is not None and memory_off_mean is not None:
        memory_effect = round(memory_on_mean - memory_off_mean, 4)
        not_just_cost_formula = memory_effect >= MEMORY_EFFECT_MARGIN

    criteria = {
        "1_not_explicitly_programmed": {
            "supported": True,
            "reasoning": "world/world.py contains no role/preference/convention "
                         "assignment anywhere in its tick loop - roles/preferences/"
                         "conventions exist only as this analysis package's post-hoc "
                         "labels, verifiable by reading world/world.py directly.",
        },
        "2_arises_from_repeated_local_interactions": {
            "supported": True,
            "reasoning": "every input to this pattern (claims, negotiations, charger "
                         "events) is a real per-tick local decision by RuleAgent/"
                         "LLMAgent/HybridAgent/ClaimBook/Conversation - no global "
                         "scheduler exists in this world.",
        },
        "3_measurable_fleet_level_pattern": {
            "supported": above_baseline,
            "mean_persistence_ratio": mean_persistence,
            "uniform_baseline": round(baseline, 4),
            "reasoning": f"mean persistence ratio {mean_persistence} vs uniform-random "
                         f"baseline {round(baseline, 4)} (margin {UNIFORM_BASELINE_MARGIN}).",
        },
        "4_persists_meaningful_duration": {
            "supported": persists,
            "mean_longest_streak_windows": mean_streak,
            "reasoning": f"mean longest consecutive-window streak {mean_streak} "
                         f"(threshold {MIN_MEANINGFUL_STREAK_WINDOWS} windows).",
        },
        "5_multiple_independent_runs": {
            "supported": multi_run,
            "seed_coverage": seed_coverage,
            "seeds_above_baseline": seeds_above_baseline,
            "total_seeds": n_seeds,
            "reasoning": f"{seeds_above_baseline}/{n_seeds} seeds independently show "
                         f"this pattern above baseline (threshold {MIN_SEED_COVERAGE_FRACTION}).",
        },
        "6_frequency_above_baseline": {
            "supported": above_baseline,
            "reasoning": "same computation as criterion 3 - both ask whether the "
                         "observed rate exceeds what a uniform/random null model predicts.",
        },
        "7_not_trivially_a_tie_break_artifact": {
            "supported": not_just_cost_formula,
            "memory_on_mean": memory_on_mean,
            "memory_off_mean": memory_off_mean,
            "memory_effect": memory_effect,
            "reasoning": (
                "no memory-ablation comparison available for this pattern"
                if memory_effect is None else
                f"memory ON mean ({memory_on_mean}) minus memory OFF mean "
                f"({memory_off_mean}) = {memory_effect} (threshold {MEMORY_EFFECT_MARGIN}) - "
                "if the pattern is just as strong without memory, it is more likely "
                "explained by the deterministic cost formula alone, not a genuinely "
                "emergent, history-dependent convention."
            ),
        },
    }

    all_supported = all(
        c["supported"] is True for c in criteria.values()
    )
    any_unknown = any(c["supported"] is None for c in criteria.values())

    verdict = (
        "candidate_emergent_collective_behavior" if all_supported else
        "insufficient_data" if any_unknown else
        "not_established"
    )

    return {
        "pattern_name": pattern_name,
        "verdict": verdict,
        "criteria": criteria,
    }
