#!/usr/bin/env python3
"""Run MPC-SECBF numerical simulation experiment matrix."""

import argparse
import csv
import datetime as dt
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

try:
    import yaml
except ImportError:
    yaml = None


SCENARIO_INDEX = {
    "S1_pedestrian_crossing": 1,
    "S2_child_sudden": 2,
    "S3_feasibility_critical": 3,
    "S4_mixed": 4,
}

BASELINES = {
    "B1_ACBF_fixed": {
        "planner": "acbf0_planner.launch",
        "controller_index": 4,
        "guard_enabled": None,
    },
    "B2_SECBF_no_guard": {
        "planner": "secbf_planner.launch",
        "controller_index": 6,
        "guard_enabled": "false",
    },
    "B3_SECBF_with_guard": {
        "planner": "secbf_planner.launch",
        "controller_index": 6,
        "guard_enabled": "true",
    },
}


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def load_scenarios(config_path: Path) -> dict:
    if yaml is None:
        raise RuntimeError("PyYAML is required: install python3-yaml or pip3 install pyyaml")
    with config_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data.get("scenarios", {})


def selected(values, requested):
    if requested == "all":
        return list(values)
    if requested not in values:
        raise ValueError(f"Unknown selection '{requested}'. Valid values: {', '.join(values)} or all")
    return [requested]


def write_obstacle_params(run_dir: Path, obstacles: list) -> Path:
    param_path = run_dir / "obstacles_param.yaml"
    clean_obstacles = []
    for obs in obstacles:
        clean = {k: v for k, v in obs.items() if k != "semantic_class"}
        clean_obstacles.append(clean)
    with param_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump({"obstacle_params": clean_obstacles}, f, sort_keys=False)
    return param_path


def obstacle_classes(obstacles: list) -> str:
    classes = [obs.get("semantic_class", "unknown") for obs in obstacles]
    return "[" + ",".join(classes) + "]"


def build_commands(scenario_id: str, baseline_id: str, run_dir: Path, obstacle_params: Path,
                   classes_arg: str, num_obs: int):
    baseline = BASELINES[baseline_id]
    common_start = [
        "roslaunch", "swarm_test", "start_test.launch",
        f"scenario_index:={SCENARIO_INDEX[scenario_id]}",
        f"controller_index:={baseline['controller_index']}",
        f"output_dir:={run_dir}",
        f"num_of_obs:={num_obs}",
        f"obstacle_params_file:={obstacle_params}",
        "record_data:=true",
    ]

    if baseline_id == "B1_ACBF_fixed":
        planner = [
            "roslaunch", "swarm_test", baseline["planner"],
            "show_rviz:=false",
        ]
    else:
        planner = [
            "roslaunch", "swarm_test", baseline["planner"],
            "show_rviz:=false",
            f"scenario_id:={scenario_id}",
            f"baseline_id:={baseline_id}",
            f"guard_enabled:={baseline['guard_enabled']}",
            f"output_dir:={run_dir}",
            f"obstacle_classes:={classes_arg}",
        ]

    return planner, common_start


def start_process(cmd, log_path: Path):
    log_file = log_path.open("w", encoding="utf-8")
    proc = subprocess.Popen(
        cmd,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        text=True,
    )
    return proc, log_file


def stop_process(proc):
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=5)
    except ProcessLookupError:
        pass


def run_verify(run_dir: Path):
    guard_log = run_dir / "guard_log.csv"
    if not guard_log.exists():
        return False, "guard_log.csv not found"
    cmd = [
        sys.executable,
        str(repo_root() / "swarm_test/scripts/verify_safety_bound.py"),
        "--csv", str(guard_log),
        "--gamma", "0.35",
        "--eps_max", "0.05",
        "--delta_bar_beta", "0.3",
    ]
    result = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    (run_dir / "verify_safety_bound.txt").write_text(result.stdout, encoding="utf-8")
    return result.returncode == 0, "verify_safety_bound.txt"


def summarize_guard_log(guard_log: Path) -> dict:
    metrics = {
        "guard_records": 0,
        "semantic_classes": "",
        "beta_applied_mean": "",
        "beta_applied_max": "",
        "h_ee_min": "",
        "h_see_min": "",
        "guard_rollback_count": "",
        "guard_rollback_rate": "",
    }
    if not guard_log.exists():
        return metrics

    with guard_log.open("r", newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return metrics

    def floats(name):
        values = []
        for row in rows:
            try:
                values.append(float(row[name]))
            except (KeyError, TypeError, ValueError):
                pass
        return values

    beta_applied = floats("beta_applied")
    h_ee = floats("h_ee")
    h_see = floats("h_see")
    guard_failed = sum(1 for row in rows if row.get("guard_passed") in {"0", "False", "false"})
    classes = sorted({row.get("class", "") for row in rows if row.get("class", "")})

    metrics["guard_records"] = len(rows)
    metrics["semantic_classes"] = ";".join(classes)
    if beta_applied:
        metrics["beta_applied_mean"] = f"{sum(beta_applied) / len(beta_applied):.6f}"
        metrics["beta_applied_max"] = f"{max(beta_applied):.6f}"
    if h_ee:
        metrics["h_ee_min"] = f"{min(h_ee):.6f}"
    if h_see:
        metrics["h_see_min"] = f"{min(h_see):.6f}"
    metrics["guard_rollback_count"] = guard_failed
    metrics["guard_rollback_rate"] = f"{guard_failed / len(rows):.6f}"
    return metrics


def write_summary(run_dir: Path, scenario_id: str, baseline_id: str, commands,
                  verify_passed, verify_note, duration_sec):
    summary_md = run_dir / "summary.md"
    summary_csv = run_dir / "summary.csv"
    guard_log = run_dir / "guard_log.csv"
    data_summary = run_dir / "data_processor_summary.csv"
    data_distance = run_dir / "data_processor_distance.csv"
    guard_metrics = summarize_guard_log(guard_log)

    lines = [
        f"# {scenario_id} / {baseline_id}",
        "",
        f"- duration_sec: {duration_sec}",
        f"- guard_log_exists: {guard_log.exists()}",
        f"- data_processor_summary_exists: {data_summary.exists()}",
        f"- data_processor_distance_exists: {data_distance.exists()}",
        f"- safety_bound_passed: {verify_passed}",
        f"- verification_note: {verify_note}",
        f"- guard_records: {guard_metrics['guard_records']}",
        f"- semantic_classes: {guard_metrics['semantic_classes']}",
        f"- beta_applied_mean: {guard_metrics['beta_applied_mean']}",
        f"- beta_applied_max: {guard_metrics['beta_applied_max']}",
        f"- h_ee_min: {guard_metrics['h_ee_min']}",
        f"- h_see_min: {guard_metrics['h_see_min']}",
        f"- guard_rollback_count: {guard_metrics['guard_rollback_count']}",
        f"- guard_rollback_rate: {guard_metrics['guard_rollback_rate']}",
        "",
        "## Commands",
        "",
    ]
    for name, cmd in commands:
        lines.append(f"- {name}: `{' '.join(cmd)}`")
    summary_md.write_text("\n".join(lines) + "\n", encoding="utf-8")

    with summary_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "scenario", "baseline", "duration_sec", "guard_log_exists",
                "data_processor_summary_exists", "data_processor_distance_exists",
                "safety_bound_passed", "guard_records", "semantic_classes",
                "beta_applied_mean", "beta_applied_max", "h_ee_min", "h_see_min",
                "guard_rollback_count", "guard_rollback_rate", "output_dir",
            ],
        )
        writer.writeheader()
        writer.writerow({
            "scenario": scenario_id,
            "baseline": baseline_id,
            "duration_sec": duration_sec,
            "guard_log_exists": guard_log.exists(),
            "data_processor_summary_exists": data_summary.exists(),
            "data_processor_distance_exists": data_distance.exists(),
            "safety_bound_passed": verify_passed,
            **guard_metrics,
            "output_dir": str(run_dir),
        })


def run_one(scenario_id: str, baseline_id: str, scenario: dict, args, timestamp: str):
    run_dir = Path(args.output_root) / f"{timestamp}_{scenario_id}_{baseline_id}"
    obstacles = scenario.get("obstacles", [])
    if not obstacles:
        raise RuntimeError(f"Scenario {scenario_id} has no obstacles")

    classes_arg = obstacle_classes(obstacles)
    obstacle_params = run_dir / "obstacles_param.yaml"
    planner_cmd, start_cmd = build_commands(
        scenario_id, baseline_id, run_dir, obstacle_params, classes_arg, len(obstacles)
    )

    commands = [("planner", planner_cmd), ("start_test", start_cmd)]
    if args.dry_run:
        print(f"\n[{scenario_id} / {baseline_id}]")
        print(f"output_dir: {run_dir}")
        print(f"obstacle_classes: {classes_arg}")
        for name, cmd in commands:
            print(f"{name}: {' '.join(cmd)}")
        return

    run_dir.mkdir(parents=True, exist_ok=True)
    obstacle_params = write_obstacle_params(run_dir, obstacles)
    planner_cmd, start_cmd = build_commands(
        scenario_id, baseline_id, run_dir, obstacle_params, classes_arg, len(obstacles)
    )

    processes = []
    roscore = None
    roscore_log = None
    try:
        if args.roscore == "auto":
            roscore, roscore_log = start_process(["roscore"], run_dir / "roscore.log")
            time.sleep(3.0)

        planner, planner_log = start_process(planner_cmd, run_dir / "planner.log")
        processes.append((planner, planner_log))
        time.sleep(3.0)

        starter, starter_log = start_process(start_cmd, run_dir / "start_test.log")
        processes.append((starter, starter_log))

        deadline = time.time() + args.duration_sec
        while time.time() < deadline:
            if any(proc.poll() not in (None, 0) for proc, _ in processes):
                break
            time.sleep(1.0)
    finally:
        for proc, log_file in reversed(processes):
            stop_process(proc)
            log_file.close()
        if roscore is not None:
            stop_process(roscore)
        if roscore_log is not None:
            roscore_log.close()

    if baseline_id == "B1_ACBF_fixed":
        verify_passed, verify_note = None, "B1 has no Guard log"
    else:
        verify_passed, verify_note = run_verify(run_dir)

    write_summary(
        run_dir,
        scenario_id,
        baseline_id,
        [("planner", planner_cmd), ("start_test", start_cmd)],
        verify_passed,
        verify_note,
        args.duration_sec,
    )
    print(f"Completed {scenario_id} / {baseline_id}: {run_dir}")


def main():
    root = repo_root()
    default_output = root / "swarm_test/output/secbf_runs"
    parser = argparse.ArgumentParser(description="Run MPC-SECBF simulation experiments")
    parser.add_argument("--scenario", default="all")
    parser.add_argument("--baseline", default="all")
    parser.add_argument("--duration-sec", type=int, default=90)
    parser.add_argument("--output-root", default=str(default_output))
    parser.add_argument("--roscore", choices=["auto", "external"], default="auto")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--config",
        default=str(root / "swarm_test/config/secbf_scenarios.yaml"),
    )
    args = parser.parse_args()

    scenarios = load_scenarios(Path(args.config))
    scenario_ids = selected(SCENARIO_INDEX.keys(), args.scenario)
    baseline_ids = selected(BASELINES.keys(), args.baseline)
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")

    for scenario_id in scenario_ids:
        for baseline_id in baseline_ids:
            run_one(scenario_id, baseline_id, scenarios[scenario_id], args, timestamp)


if __name__ == "__main__":
    main()
