#!/usr/bin/env bash
# Paired-seed Mode B/C/D/E resilience comparison.
#
# Runs a pilot-scale (N seeds),
# paired-seed comparison of Mode B (centralized, no recovery), Mode C
# (centralized -> deterministic failover), Mode D (centralized -> LLM
# failover), and Mode E (centralized -> hybrid failover), then generates
# analysis/results/resilience_summary.csv from real data only.
#
# Timing (--duration/--t-fail/--manager-timeout-sec) matches the one
# combination this platform has already live-confirmed produces a genuine
# post-failover task completion (t_fail=8,
# manager_timeout_sec=3, duration=45 -> real recovery at
# simulation_time=35.6s, confirmed in an earlier recovery-validation run).
# An earlier sanity pilot's t_fail=10/
# manager_timeout=3/duration=30 (only 17s of post-failover window) showed
# zero recovery across all four modes - too short a window, not a defect;
# this script deliberately uses the longer, already-validated timing instead.
#
# Usage:
#   source /opt/ros/jazzy/setup.bash
#   source install/setup.bash
#   bash analysis/scripts/run_resilience_campaign.sh <seeds> <batch_dir>
#
# Example (3-seed pilot):
#   bash analysis/scripts/run_resilience_campaign.sh "20,21,22" \
#       experiments/2026-08-12_batch_resilience_pilot
set -euo pipefail

SEEDS="${1:?usage: run_resilience_campaign.sh <seeds> <batch_dir>}"
BATCH_DIR="${2:?usage: run_resilience_campaign.sh <seeds> <batch_dir>}"
DURATION=45
T_FAIL=8
MANAGER_TIMEOUT=3
NUM_ROBOTS=2

echo "[resilience] seeds=${SEEDS} batch_dir=${BATCH_DIR} duration=${DURATION} t_fail=${T_FAIL} manager_timeout=${MANAGER_TIMEOUT}"

# Mode B (centralized) + Mode C (centralized_then_failover, rule) share one
# invocation since agent_backend is irrelevant to Mode B.
python3 analysis/scripts/run_batch.py \
    --experiment resilience_pilot \
    --coordination-modes centralized centralized_then_failover \
    --agent-backend rule \
    --seeds "${SEEDS}" \
    --num-robots "${NUM_ROBOTS}" \
    --duration "${DURATION}" \
    --t-fail "${T_FAIL}" \
    --manager-timeout-sec "${MANAGER_TIMEOUT}" \
    --time-source simulation \
    --batch-dir "${BATCH_DIR}" \
    --append

# Mode D (centralized_then_failover, llm)
python3 analysis/scripts/run_batch.py \
    --experiment resilience_pilot \
    --coordination-modes centralized_then_failover \
    --agent-backend llm \
    --seeds "${SEEDS}" \
    --num-robots "${NUM_ROBOTS}" \
    --duration "${DURATION}" \
    --t-fail "${T_FAIL}" \
    --manager-timeout-sec "${MANAGER_TIMEOUT}" \
    --time-source simulation \
    --batch-dir "${BATCH_DIR}" \
    --append

# Mode E (centralized_then_failover, hybrid)
python3 analysis/scripts/run_batch.py \
    --experiment resilience_pilot \
    --coordination-modes centralized_then_failover \
    --agent-backend hybrid \
    --seeds "${SEEDS}" \
    --num-robots "${NUM_ROBOTS}" \
    --duration "${DURATION}" \
    --t-fail "${T_FAIL}" \
    --manager-timeout-sec "${MANAGER_TIMEOUT}" \
    --time-source simulation \
    --batch-dir "${BATCH_DIR}" \
    --append

python3 analysis/scripts/generate_report.py "${BATCH_DIR}"

echo "[resilience] done. batch_dir=${BATCH_DIR}"
