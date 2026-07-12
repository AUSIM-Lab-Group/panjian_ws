#!/usr/bin/env python3
"""Generate deterministic paired perturbation manifests for SEESM trials."""

import argparse
import csv
from pathlib import Path
import random
import sys

import yaml


HEADER = [
    "trial_id", "seed", "scenario_id", "obstacle_id",
    "start_x_offset_m", "start_y_offset_m", "speed_scale", "start_delay_offset_s",
]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--count", type=int, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--prefix", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    config = Path(__file__).resolve().parents[1] / "config/secbf_scenarios.yaml"
    with config.open("r", encoding="utf-8") as f:
        scenarios = yaml.safe_load(f)["scenarios"]
    scenario_ids = [value.strip() for value in args.scenario.split(",") if value.strip()]
    unknown = [scenario_id for scenario_id in scenario_ids if scenario_id not in scenarios]
    if unknown:
        raise SystemExit(f"unknown scenario: {','.join(unknown)}")
    rows = []
    for scenario_id in scenario_ids:
        for number in range(1, args.count + 1):
            trial_id = f"{args.prefix}_{number:03d}"
            trial_seed = args.seed + number - 1
            rng = random.Random(trial_seed)
            for index, obstacle in enumerate(scenarios[scenario_id].get("obstacles", []), start=1):
                rows.append({
                    "trial_id": trial_id, "seed": trial_seed, "scenario_id": scenario_id,
                    "obstacle_id": obstacle.get("obstacle_id", f"obs_{index:03d}"),
                    "start_x_offset_m": f"{rng.uniform(-0.15, 0.15):.6f}",
                    "start_y_offset_m": f"{rng.uniform(-0.15, 0.15):.6f}",
                    "speed_scale": f"{rng.uniform(0.95, 1.05):.6f}",
                    "start_delay_offset_s": f"{rng.uniform(-0.20, 0.20):.6f}",
                })
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=HEADER)
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
