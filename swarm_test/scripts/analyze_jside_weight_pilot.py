#!/usr/bin/env python3
"""Summarize the controlled J_side weight pilot."""

import argparse
import bisect
import csv
import math
from collections import defaultdict
from pathlib import Path


def number(row, key):
    try:
        return float(row.get(key, ""))
    except (TypeError, ValueError):
        return float("nan")


def read_rows(path):
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def dynamic_tau(lx, ly, vx, vy, radius, ke=0.30, t_max=2.0, max_tau=2.0):
    distance = math.hypot(lx, ly)
    speed = math.hypot(vx, vy)
    if distance <= 1e-6 or speed <= 1e-6:
        return 0.0
    nx, ny = lx / distance, ly / distance
    nvx, nvy = vx / speed, vy / speed
    cos_delta = nx * nvx + ny * nvy
    if cos_delta >= 0.0:
        return 0.0
    cone = (lx * vx + ly * vy) ** 2 + (radius * radius - distance * distance) * speed * speed
    if cone <= 0.0:
        return 0.0
    interaction_time = max(0.0, distance - radius) * max(0.0, -cos_delta) / speed
    if interaction_time <= 0.0 or interaction_time >= t_max:
        return 0.0
    return min(ke * interaction_time, max_tau)


def trial_side_metrics(run_dir, epsilon_n=1e-3):
    robot = read_rows(run_dir / "robot_log.csv")
    obstacles = read_rows(run_dir / "obstacle_log.csv")
    if not robot or not obstacles:
        return {"interactions": 0, "consistent": 0, "sample_count": 0,
                "sample_consistent": 0, "g_mean": float("nan"),
                "executed_interactions": 0, "executed_consistent": 0,
                "executed_margin_mean": float("nan")}
    robot_times = [number(row, "t") for row in robot]
    grouped = defaultdict(list)
    for row in obstacles:
        if math.hypot(number(row, "vx"), number(row, "vy")) > 1e-3:
            grouped[row["id"]].append(row)
    interaction_signs = []
    sample_signs = []
    g_values = []
    executed_signs = []
    executed_margins = []
    for rows in grouped.values():
        active_g = []
        relative_samples = []
        for row in rows:
            t = number(row, "t")
            index = bisect.bisect_left(robot_times, t)
            index = min(max(index, 0), len(robot) - 1)
            if index and abs(robot_times[index - 1] - t) < abs(robot_times[index] - t):
                index -= 1
            rob = robot[index]
            yaw = number(rob, "yaw")
            speed = number(rob, "v")
            robot_vx = speed * math.cos(yaw)
            robot_vy = speed * math.sin(yaw)
            lx = number(row, "x") - number(rob, "x")
            ly = number(row, "y") - number(rob, "y")
            rvx = number(row, "vx") - robot_vx
            rvy = number(row, "vy") - robot_vy
            radius = number(row, "radius") + 0.4
            tau = dynamic_tau(lx, ly, rvx, rvy, radius)
            relative_samples.append({
                "t": t,
                "lx": lx,
                "ly": ly,
                "distance": math.hypot(lx, ly),
                "tau": tau,
            })
            if tau <= 0.0:
                continue
            denom = math.sqrt(lx * lx + ly * ly + epsilon_n * epsilon_n)
            tx, ty = -ly / denom, lx / denom  # s0=+1 and J=[[0,-1],[1,0]]
            g_side = tx * (lx + tau * rvx) + ty * (ly + tau * rvy)
            active_g.append(g_side)
            sample_signs.append(g_side >= 0.0)
            g_values.append(g_side)
        if active_g:
            ordered = sorted(active_g)
            median = ordered[len(ordered) // 2]
            interaction_signs.append(median >= 0.0)

        # Formal executed-side metric: freeze the preferred lateral frame at
        # the first applicable dynamic interaction sample, then classify the
        # robot's side at the subsequent closest approach.  The evaluation
        # window is the active interaction-time estimate plus a 1 s tolerance;
        # it remains within the 3 s upper bound implied by T_max=2 s and avoids
        # using a later obstacle wrap-around or second encounter.
        entry_index = next(
            (index for index, sample in enumerate(relative_samples) if sample["tau"] > 0.0),
            None,
        )
        if entry_index is not None:
            entry = relative_samples[entry_index]
            denom = math.sqrt(
                entry["lx"] * entry["lx"]
                + entry["ly"] * entry["ly"]
                + epsilon_n * epsilon_n
            )
            tx, ty = -entry["ly"] / denom, entry["lx"] / denom
            interaction_time = entry["tau"] / 0.30
            evaluation_end = entry["t"] + min(3.0, interaction_time + 1.0)
            candidates = [
                sample for sample in relative_samples[entry_index:]
                if sample["t"] <= evaluation_end
            ]
            closest = min(candidates, key=lambda sample: sample["distance"])
            executed_margin = tx * closest["lx"] + ty * closest["ly"]
            executed_margins.append(executed_margin)
            executed_signs.append(executed_margin >= 0.0)
    return {
        "interactions": len(interaction_signs),
        "consistent": sum(interaction_signs),
        "sample_count": len(sample_signs),
        "sample_consistent": sum(sample_signs),
        "g_mean": sum(g_values) / len(g_values) if g_values else float("nan"),
        "executed_interactions": len(executed_signs),
        "executed_consistent": sum(executed_signs),
        "executed_margin_mean": (
            sum(executed_margins) / len(executed_margins)
            if executed_margins else float("nan")
        ),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    rows = list(csv.DictReader((args.input_root / "summary.csv").open()))
    grouped = defaultdict(list)
    for row in rows:
        side = trial_side_metrics(Path(row["output_dir"]))
        row.update({f"side_{key}": value for key, value in side.items()})
        grouped[row["baseline"]].append(row)

    output = []
    for baseline, values in sorted(grouped.items()):
        def mean(key):
            data = [number(row, key) for row in values]
            data = [value for value in data if value == value]
            return sum(data) / len(data) if data else float("nan")

        output.append({
            "baseline": baseline,
            "n": len(values),
            "success_count": sum(row.get("success") == "1" for row in values),
            "collision_trials": sum(number(row, "nav_collision_count") > 0 for row in values),
            "side_preference_enabled": int(baseline != "No_J_side"),
            "side_weight": 0.0 if baseline == "No_J_side" else mean("side_weight"),
            "path_length_mean_m": mean("robot_path_length_m"),
            "travel_time_mean_s": mean("nav_travel_time_s"),
            "final_goal_error_mean_m": mean("robot_final_goal_distance_m"),
            "solve_time_mean_ms": mean("solve_time_mean_ms"),
            "solve_time_max_ms": max(number(row, "solve_time_max_ms") for row in values),
            "side_cost_mean": mean("side_cost_mean"),
            "side_cost_max": max(number(row, "side_cost_max") for row in values),
            "side_interactions": int(sum(number(row, "side_interactions") for row in values)),
            "preferred_side_consistency_rate": (
                sum(number(row, "side_consistent") for row in values)
                / sum(number(row, "side_interactions") for row in values)
                if sum(number(row, "side_interactions") for row in values) else float("nan")
            ),
            "preferred_side_sample_rate": (
                sum(number(row, "side_sample_consistent") for row in values)
                / sum(number(row, "side_sample_count") for row in values)
                if sum(number(row, "side_sample_count") for row in values) else float("nan")
            ),
            "g_side_mean": mean("side_g_mean"),
            "executed_side_interactions": int(
                sum(number(row, "side_executed_interactions") for row in values)
            ),
            "executed_side_consistency_rate": (
                sum(number(row, "side_executed_consistent") for row in values)
                / sum(number(row, "side_executed_interactions") for row in values)
                if sum(number(row, "side_executed_interactions") for row in values)
                else float("nan")
            ),
            "executed_side_margin_mean_m": mean("side_executed_margin_mean"),
            "first_infeasible_total": int(sum(number(row, "first_infeasible_count") for row in values)),
            "mpc_guard_used_total": int(sum(number(row, "mpc_guard_used_count") for row in values)),
            "no_cbf_fallback_total": int(sum(number(row, "no_cbf_fallback_count") for row in values)),
        })

    fields = list(output[0])
    with args.output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(output)
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
