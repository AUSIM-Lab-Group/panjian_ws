#!/usr/bin/env python3
"""Check that a run directory contains the Phase 5 paper CSV fields."""

import argparse
import csv
from pathlib import Path
import sys


FILE_FIELDS = {
    "robot_log.csv": {"t", "x", "y", "yaw", "v", "w", "cmd_v", "cmd_w"},
    "obstacle_log.csv": {"t", "id", "class", "x", "y", "radius", "vx", "vy", "d_i", "rel_v", "TTC", "h_EE"},
    "margin_guard_log.csv": {
        "time", "obs_id", "class", "d_i", "rel_v_norm", "ttc", "mu", "beta_bar",
        "beta_requested", "beta_applied", "guard_upper_bound", "h_ee", "h_see", "guard_status",
    },
    "planner_log.csv": {"t", "mpc_status", "cmd_v", "cmd_w", "slack", "solve_time_ms"},
    "timing_log.csv": {"t", "mpc_secbf_ms", "total_loop_time_ms"},
    "event_log.csv": {"t", "event", "detail"},
}

PAPER_FIELDS = {
    "t": [("robot_log.csv", "t"), ("margin_guard_log.csv", "time")],
    "id": [("obstacle_log.csv", "id"), ("margin_guard_log.csv", "obs_id")],
    "class": [("obstacle_log.csv", "class"), ("margin_guard_log.csv", "class")],
    "d_i": [("obstacle_log.csv", "d_i"), ("margin_guard_log.csv", "d_i")],
    "rel_v": [("obstacle_log.csv", "rel_v"), ("margin_guard_log.csv", "rel_v_norm")],
    "TTC": [("obstacle_log.csv", "TTC"), ("margin_guard_log.csv", "ttc")],
    "r_i/mu": [("margin_guard_log.csv", "mu")],
    "beta_bar": [("margin_guard_log.csv", "beta_bar")],
    "beta_hat": [("margin_guard_log.csv", "beta_requested")],
    "beta": [("margin_guard_log.csv", "beta_applied")],
    "guard_upper_bound": [("margin_guard_log.csv", "guard_upper_bound")],
    "h_EE": [("obstacle_log.csv", "h_EE"), ("margin_guard_log.csv", "h_ee")],
    "h_SEE": [("margin_guard_log.csv", "h_see")],
    "slack": [("planner_log.csv", "slack")],
    "mpc_status": [("planner_log.csv", "mpc_status")],
}


def read_header(path):
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        try:
            return set(next(reader))
        except StopIteration:
            return set()


def main():
    parser = argparse.ArgumentParser(description="Check Phase 5 CSV field contract")
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()

    headers = {}
    errors = []
    for file_name, required in FILE_FIELDS.items():
        path = args.run_dir / file_name
        if not path.exists():
            errors.append(f"missing file: {file_name}")
            continue
        header = read_header(path)
        headers[file_name] = header
        missing = sorted(required - header)
        if missing:
            errors.append(f"{file_name}: missing fields {missing}")

    for field, choices in PAPER_FIELDS.items():
        if not any(file_name in headers and column in headers[file_name] for file_name, column in choices):
            errors.append(f"paper field unavailable: {field}")

    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1

    print(f"Phase 5 CSV field checks passed: {args.run_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
