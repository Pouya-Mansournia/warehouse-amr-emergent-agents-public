#!/usr/bin/env python3
"""Frame-corrected separation audit for the legacy three-seed subset.

robot_robotN.csv stores wheel-odometry positions, and each robot's odometry
frame starts at that robot's own spawn pose. The original separation audit
compared these positions directly, which ignored the spawn offset between the robots.
fleet.launch.py spawns robot i at x=0, y=(i-(n+1)/2)*1.0 m with zero yaw (the
same layout was used for the legacy runs), so a common frame is obtained by
adding that y offset. Claim replay is unchanged from the original audit.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

RUN_SUFFIXES = [f"{i:03d}" for i in range(4, 13)]
NEAR_SEPARATION_M = 1.10  # twice the configured 0.55 m costmap inflation radius

SPAWN_Y_SPACING_M = 1.0


def rows(path: Path):
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def claims(run_dir: Path):
    owners = {}
    conflicts = []
    malformed_releases = []
    for row in rows(run_dir / "claims.csv"):
        station = row["station_name"]
        robot = row["robot_id"]
        is_release = row["release"].lower() == "true"
        if is_release:
            if owners.get(station) not in (None, robot):
                malformed_releases.append({"station": station, "releasing_robot": robot, "owner": owners[station]})
            if owners.get(station) == robot:
                del owners[station]
        else:
            if station in owners and owners[station] != robot:
                conflicts.append({"station": station, "first_owner": owners[station], "second_owner": robot, "simulation_time": float(row["simulation_time"])})
            owners[station] = robot
    return {
        "simultaneous_cross_robot_claim_conflicts": conflicts,
        "cross_robot_release_mismatches": malformed_releases,
        "unreleased_claims_at_observation_end": owners,
    }


def spawn_y(robot_index: int, num_robots: int) -> float:
    return (robot_index - (num_robots + 1) / 2.0) * SPAWN_Y_SPACING_M


def separation(run_dir: Path, start_sim_sec: float, num_robots: int):
    first = rows(run_dir / "robot_robot1.csv")
    second = rows(run_dir / "robot_robot2.csv")
    off1, off2 = spawn_y(1, num_robots), spawn_y(2, num_robots)
    j = 0
    samples = []
    for a in first:
        ta = float(a["simulation_time"])
        while j + 1 < len(second) and abs(float(second[j + 1]["simulation_time"]) - ta) <= abs(float(second[j]["simulation_time"]) - ta):
            j += 1
        b = second[j]
        if ta >= start_sim_sec and abs(float(b["simulation_time"]) - ta) <= 0.15:
            distance = math.hypot(float(a["x"]) - float(b["x"]),
                                  (float(a["y"]) + off1) - (float(b["y"]) + off2))
            samples.append((ta, distance))
    minimum = min(samples, key=lambda item: item[1])
    near = sum(distance < NEAR_SEPARATION_M for _, distance in samples)
    return {
        "matched_samples": len(samples),
        "maximum_timestamp_mismatch_sec": 0.15,
        "spawn_y_offsets_m": [off1, off2],
        "minimum_center_separation_m": round(minimum[1], 4),
        "minimum_at_simulation_time_sec": round(minimum[0], 3),
        "near_separation_threshold_m": NEAR_SEPARATION_M,
        "near_separation_samples": near,
        "near_separation_fraction": round(near / len(samples), 6),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiments-dir", required=True)
    parser.add_argument("--out-dir", required=True)
    args = parser.parse_args()
    base = Path(args.experiments_dir)
    output = []
    for suffix in RUN_SUFFIXES:
        run_dir = base / f"2026-08-12_milestone9_pilot_{suffix}"
        meta = json.loads((run_dir / "metadata.json").read_text(encoding="utf-8"))
        output.append({
            "run": run_dir.name,
            "backend": meta["agent_backend"],
            "seed": meta["random_seed"],
            "separation": separation(run_dir, float(meta["t_fail_sec"]), int(meta["robot_count"])),
            "claim_audit": claims(run_dir),
        })
    result = {
        "method": "odometry positions shifted by each robot's spawn offset into a common frame; nearest-time pose matching with at most 0.15 s mismatch; center-to-center planar distance from fault injection to observation end",
        "near_separation_definition": "distance below twice the configured 0.55 m costmap inflation radius; this is not a contact detector",
        "supersedes": "safety_and_claim_audit.json separation fields (no spawn-offset correction)",
        "runs": output,
    }
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    target = out / "safety_and_claim_audit_frame_corrected.json"
    target.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(target)
    for r in output:
        s = r["separation"]
        print(r["backend"], r["seed"], s["minimum_center_separation_m"], s["near_separation_samples"],
              len(r["claim_audit"]["simultaneous_cross_robot_claim_conflicts"]))


if __name__ == "__main__":
    main()
