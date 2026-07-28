#!/usr/bin/env python3
"""Run non-canonical Head-on Ext feasibility probes behind the path-ready gate."""

from __future__ import annotations

import argparse
from collections import Counter
import csv
import importlib.util
import json
import os
from pathlib import Path
import signal
import statistics
import sys
import time

import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
RUNNER_PATH = SCRIPT_DIR / "run_secbf_sim_experiments.py"


def load_runner():
    spec = importlib.util.spec_from_file_location("teacher_probe_runner", RUNNER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


VARIANTS = (
    {
        "id": "beta0_full_constraints",
        "description": "beta=0, original CBF/slack/control-increment constraints",
        "overrides": {},
    },
    {
        "id": "no_cbf_original_control",
        "description": "exclude all obstacles from the CBF active set",
        "overrides": {"active_set_distance_m": 0.01},
    },
    {
        "id": "beta0_relaxed_delta_u",
        "description": "beta=0 with effectively inactive control-increment bound",
        "overrides": {"delta_u_max": 10.0},
    },
    {
        "id": "beta0_relaxed_slack",
        "description": "beta=0 with epsilon_max increased from 0.05 to 1.0",
        "overrides": {"epsilon_max": 1.0},
    },
    {
        "id": "beta0_relaxed_delta_u_and_slack",
        "description": "beta=0 with delta_u_max=10 and epsilon_max=1.0",
        "overrides": {"delta_u_max": 10.0, "epsilon_max": 1.0},
    },
)


def replace_launch_arg(command: list[str], key: str, value) -> None:
    prefix = f"{key}:="
    replacement = prefix + str(value)
    for index, token in enumerate(command):
        if token.startswith(prefix):
            command[index] = replacement
            return
    command.append(replacement)


def read_rows(path: Path):
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def numeric(row, field, default=0.0):
    try:
        return float(row.get(field) or default)
    except (TypeError, ValueError):
        return default


def summarize_variant(run_dir: Path, variant: dict, gate_record: dict):
    planner_rows = read_rows(run_dir / "planner_log.csv")
    timing_rows = read_rows(run_dir / "timing_log.csv")
    obstacle_rows = read_rows(run_dir / "obstacle_log.csv")
    statuses = Counter(row.get("final_status", "") for row in planner_rows)
    sources = Counter(row.get("accepted_beta_source", "") for row in planner_rows)
    nonzero_commands = [
        row for row in planner_rows
        if abs(numeric(row, "cmd_v")) > 1.0e-4 or abs(numeric(row, "cmd_w")) > 1.0e-4
    ]
    solve_times = [numeric(row, "total_loop_time_ms") for row in timing_rows]
    initial_success = sum(int(numeric(row, "initial_success")) for row in timing_rows)
    open_set_empty = 0
    planner_text_path = run_dir / "planner.log"
    if planner_text_path.exists():
        open_set_empty = planner_text_path.read_text(
            encoding="utf-8", errors="replace"
        ).count("open set empty, no path!")
    obstacle_motion = 0.0
    if obstacle_rows:
        first_by_id = {}
        max_displacement = {}
        for row in obstacle_rows:
            obstacle_id = row["id"]
            point = (numeric(row, "x"), numeric(row, "y"))
            first_by_id.setdefault(obstacle_id, point)
            start = first_by_id[obstacle_id]
            displacement = ((point[0] - start[0]) ** 2 + (point[1] - start[1]) ** 2) ** 0.5
            max_displacement[obstacle_id] = max(
                max_displacement.get(obstacle_id, 0.0), displacement
            )
        obstacle_motion = max(max_displacement.values(), default=0.0)
    return {
        "variant": variant["id"],
        "description": variant["description"],
        "gate_ready": bool(gate_record.get("ready")),
        "gate_wait_sec": float(gate_record.get("wait_sec", 0.0)),
        "reference_span_m": float(gate_record.get("reference_span_m", 0.0)),
        "planner_records": len(planner_rows),
        "initial_success_count": initial_success,
        "success_status_count": statuses.get("success", 0),
        "infeasible_status_count": sum(
            value for key, value in statuses.items() if "infeasible" in key
        ),
        "safe_stop_count": sources.get("safe_stop", 0),
        "nonzero_command_count": len(nonzero_commands),
        "first_nonzero_cmd_v": numeric(nonzero_commands[0], "cmd_v") if nonzero_commands else 0.0,
        "first_nonzero_cmd_w": numeric(nonzero_commands[0], "cmd_w") if nonzero_commands else 0.0,
        "mean_solve_time_ms": statistics.mean(solve_times) if solve_times else 0.0,
        "max_solve_time_ms": max(solve_times, default=0.0),
        "open_set_empty_count": open_set_empty,
        "max_obstacle_displacement_m": obstacle_motion,
        "accepted_sources": json.dumps(dict(sources), sort_keys=True),
        "overrides": json.dumps(variant["overrides"], sort_keys=True),
        "output_dir": str(run_dir.resolve()),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--duration-sec", type=float, default=10.0)
    parser.add_argument("--gate-timeout-sec", type=float, default=15.0)
    parser.add_argument("--gate-min-span-m", type=float, default=0.25)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=SCRIPT_DIR.parent / "config/seed_manifests/20260712_nine_condition_pilot5.csv",
    )
    parser.add_argument(
        "--freeze",
        type=Path,
        default=SCRIPT_DIR.parent / "config/experiment_freezes/main_smoke.yaml",
    )
    args = parser.parse_args()
    if args.duration_sec <= 0.0:
        parser.error("--duration-sec must be positive")

    runner = load_runner()
    args.output_root.mkdir(parents=True, exist_ok=True)
    scenarios = runner.load_scenarios(SCRIPT_DIR.parent / "config/secbf_scenarios.yaml")
    trials = runner.load_seed_manifest(args.manifest)
    trial = next(
        item for item in trials
        if item["scenario_id"] == "head_on_context_ext" and item["trial_id"] == "pilot_001"
    )
    freeze = runner.load_parameter_freeze(args.freeze, "smoke", campaign="main")
    scenario, obstacles = runner.materialize_trial(
        scenarios["head_on_context_ext"], trial
    )
    scenario = runner.apply_parameter_freeze(scenario, freeze)

    roscore_env = os.environ.copy()
    roscore_env["ROS_HOME"] = str((args.output_root / ".roscore").resolve())
    roscore_env["ROS_LOG_DIR"] = str((args.output_root / ".roscore_log").resolve())
    Path(roscore_env["ROS_HOME"]).mkdir(parents=True, exist_ok=True)
    Path(roscore_env["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
    roscore, roscore_log = runner.start_process(
        ["roscore"], args.output_root / "roscore.log", env=roscore_env
    )
    time.sleep(3.0)
    summaries = []
    try:
        for variant in VARIANTS:
            run_dir = args.output_root / variant["id"]
            if run_dir.exists():
                raise RuntimeError(f"probe output already exists: {run_dir}")
            run_dir.mkdir(parents=True)
            obstacle_params = runner.write_obstacle_params(run_dir, obstacles)
            classes_arg = runner.obstacle_classes(obstacles)
            planner_cmd, start_cmd = runner.build_commands(
                "head_on_context_ext", "No_semantic", run_dir,
                obstacle_params, classes_arg, len(obstacles), scenario,
                global_path_ready_gate=True,
            )
            for key, value in variant["overrides"].items():
                replace_launch_arg(planner_cmd, key, value)
            (run_dir / "probe_contract.yaml").write_text(
                yaml.safe_dump({
                    "scenario": "head_on_context_ext",
                    "trial_id": "pilot_001",
                    "baseline": "No_semantic",
                    "variant": variant,
                    "freeze": str(args.freeze.resolve()),
                    "duration_sec": args.duration_sec,
                    "global_path_ready_gate": True,
                }, sort_keys=False),
                encoding="utf-8",
            )
            env = runner.process_env(run_dir)
            processes = []
            try:
                planner_proc, planner_log = runner.start_process(
                    planner_cmd, run_dir / "planner.log", env=env
                )
                processes.append((planner_proc, planner_log))
                time.sleep(3.0)
                start_proc, start_log = runner.start_process(
                    start_cmd, run_dir / "start_test.log", env=env
                )
                processes.append((start_proc, start_log))
                gate_started = time.time()
                gate_record = runner.wait_for_global_path_ready(
                    "/global_path",
                    args.gate_timeout_sec,
                    args.gate_min_span_m,
                    env=env,
                )
                gate_record["wait_sec"] = time.time() - gate_started
                if not gate_record["ready"]:
                    raise RuntimeError(
                        f"path-ready gate failed for {variant['id']}: {gate_record}"
                    )
                runner.release_global_path_start_gate(
                    "/teacher_v1/global_path_ready", env=env
                )
                (run_dir / "global_path_ready_gate.yaml").write_text(
                    yaml.safe_dump(gate_record, sort_keys=False), encoding="utf-8"
                )
                deadline = time.time() + args.duration_sec
                while time.time() < deadline:
                    if any(proc.poll() is not None for proc, _ in processes):
                        raise RuntimeError(f"probe process exited early: {variant['id']}")
                    time.sleep(0.25)
            finally:
                for proc, log_file in reversed(processes):
                    runner.stop_process(proc)
                    log_file.close()
            summary = summarize_variant(run_dir, variant, gate_record)
            summaries.append(summary)
            print(
                f"COMPLETE {variant['id']}: initial_success="
                f"{summary['initial_success_count']}/{summary['planner_records']} "
                f"nonzero_cmd={summary['nonzero_command_count']}"
            )
            time.sleep(1.0)
    finally:
        runner.stop_process(roscore)
        roscore_log.close()

    fieldnames = list(summaries[0])
    with (args.output_root / "probe_summary.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader(); writer.writerows(summaries)
    (args.output_root / "probe_summary.json").write_text(
        json.dumps(summaries, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(args.output_root / "probe_summary.csv")


if __name__ == "__main__":
    main()
