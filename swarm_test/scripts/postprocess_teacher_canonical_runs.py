#!/usr/bin/env python3
from __future__ import annotations

"""Post-process teacher-style MPC-SECBF canonical simulation batches."""

import argparse
import csv
import math
from pathlib import Path
from typing import Optional

try:
    import yaml
except ImportError:
    yaml = None


METHOD_LABELS = {
    "Fixed_margin": "Standard MPC-CBF",
    "Standard_MPC_CBF": "Standard MPC-CBF",
    "No_semantic": "EESM-MPC-ECBF",
    "Unguarded_SEESM": "SEESM w/o FPU",
    "SEESM_Ours": "Proposed MPC-SECBF",
}


RUN_FIELDS = [
    "repeat_id",
    "trial_id",
    "seed",
    "scenario",
    "scenario_family",
    "context_level",
    "baseline",
    "method_label",
    "run_id",
    "output_dir",
    "success",
    "goal_reached",
    "collision_count",
    "collision_episode_count",
    "d_min_m",
    "min_h_seesm",
    "semantic_violation_ratio",
    "semantic_violation_pair_ratio",
    "mpc_feasibility_rate",
    "mpc_first_attempt_feasibility_rate",
    "path_length_m",
    "travel_time_s",
    "solve_time_mean_ms",
    "solve_time_p95_ms",
    "solve_time_max_ms",
    "planner_records",
    "guard_records",
    "safety_bound_passed",
    "global_beta_applied_max",
    "global_semantic_rejection_count",
    "global_stale_margin_count",
    "global_missing_margin_count",
    "global_replan_mean_ms",
]

MANIFEST_FIELDS = [
    "repeat_id",
    "trial_id",
    "seed",
    "scenario",
    "baseline",
    "source_batch",
    "output_dir",
    "run_id",
    "scenario_family",
    "context_level",
    "method_label",
    "summary_csv",
]

TABLE_FIELDS = [
    "scenario",
    "scenario_family",
    "context_level",
    "baseline",
    "method_label",
    "n_trials",
    "success_rate",
    "goal_reached_rate",
    "collision_rate",
    "collision_episode_mean",
    "d_min_mean_m",
    "d_min_min_m",
    "min_h_seesm_mean",
    "min_h_seesm_min",
    "semantic_violation_ratio_mean",
    "mpc_feasibility_rate_mean",
    "path_length_mean_m",
    "travel_time_mean_s",
    "solve_time_mean_ms",
    "solve_time_p95_ms",
    "global_beta_applied_max",
    "global_semantic_rejection_count_mean",
    "global_stale_margin_count_mean",
    "global_missing_margin_count_mean",
    "global_replan_mean_ms",
]


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, object]], fieldnames: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def parse_float(value, default: Optional[float] = None) -> Optional[float]:
    if value in ("", None):
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def fmt(value: Optional[float]) -> str:
    if value is None or not math.isfinite(value):
        return ""
    return f"{value:.6f}"


def mean(values: list[float]) -> Optional[float]:
    values = [value for value in values if math.isfinite(value)]
    if not values:
        return None
    return sum(values) / len(values)


def percentile(values: list[float], q: float) -> Optional[float]:
    values = sorted(value for value in values if math.isfinite(value))
    if not values:
        return None
    index = max(0, min(len(values) - 1, math.ceil(q * len(values)) - 1))
    return values[index]


def load_scenario_meta(config_path: Path) -> dict[str, dict[str, str]]:
    if yaml is None or not config_path.exists():
        return {}
    with config_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    scenarios = data.get("scenarios", {})
    meta = {}
    for scenario, cfg in scenarios.items():
        meta[scenario] = {
            "scenario_family": cfg.get("scenario_family", ""),
            "context_level": cfg.get("context_level", ""),
        }
    return meta


def repeat_id_from_run(run_id: str) -> str:
    parts = run_id.split("_")
    for part in parts:
        if len(part) == 3 and part.startswith("r") and part[1:].isdigit():
            return part
    return "r01"


def collect_summary_rows(output_root: Path) -> list[dict[str, str]]:
    aggregate = output_root / "summary.csv"
    rows = read_csv(aggregate)
    if rows:
        return rows

    collected: list[dict[str, str]] = []
    for summary in sorted(output_root.glob("*/summary.csv")):
        run_rows = read_csv(summary)
        if run_rows:
            collected.append(run_rows[0])
    return collected


def semantic_violation_metrics(run_dir: Path) -> dict[str, object]:
    rows = read_csv(run_dir / "margin_guard_log.csv")
    if not rows:
        return {
            "guard_records": 0,
            "min_h_seesm_from_log": None,
            "semantic_violation_ratio": None,
            "semantic_violation_pair_ratio": None,
        }

    time_groups: dict[str, bool] = {}
    h_values: list[float] = []
    pair_violations = 0
    for row in rows:
        h_see = parse_float(row.get("h_see"))
        if h_see is None:
            continue
        h_values.append(h_see)
        violated = h_see < 0.0
        pair_violations += int(violated)
        time_key = row.get("time", "")
        time_groups[time_key] = time_groups.get(time_key, False) or violated

    if not h_values:
        return {
            "guard_records": len(rows),
            "min_h_seesm_from_log": None,
            "semantic_violation_ratio": None,
            "semantic_violation_pair_ratio": None,
        }
    return {
        "guard_records": len(rows),
        "min_h_seesm_from_log": min(h_values),
        "semantic_violation_ratio": (
            sum(1 for violated in time_groups.values() if violated) / len(time_groups)
            if time_groups else None
        ),
        "semantic_violation_pair_ratio": pair_violations / len(h_values),
    }


def planner_metrics(run_dir: Path, summary_row: dict[str, str]) -> dict[str, object]:
    rows = read_csv(run_dir / "planner_log.csv")
    if not rows:
        first_infeasible = parse_float(summary_row.get("first_infeasible_rate"))
        return {
            "planner_records": 0,
            "mpc_feasibility_rate": None,
            "mpc_first_attempt_feasibility_rate": (
                1.0 - first_infeasible if first_infeasible is not None else None
            ),
            "solve_time_mean_ms": parse_float(summary_row.get("solve_time_mean_ms")),
            "solve_time_p95_ms": None,
            "solve_time_max_ms": parse_float(summary_row.get("solve_time_max_ms")),
        }

    final_success = 0
    first_success = 0
    solve_times = []
    for row in rows:
        final_status = row.get("final_status") or row.get("mpc_status")
        first_status = row.get("first_attempt_status")
        final_success += int(final_status == "success")
        first_success += int(first_status == "success")
        solve_time = parse_float(row.get("solve_time_ms"))
        if solve_time is not None:
            solve_times.append(solve_time)

    total = len(rows)
    return {
        "planner_records": total,
        "mpc_feasibility_rate": final_success / total if total else None,
        "mpc_first_attempt_feasibility_rate": first_success / total if total else None,
        "solve_time_mean_ms": mean(solve_times),
        "solve_time_p95_ms": percentile(solve_times, 0.95),
        "solve_time_max_ms": max(solve_times) if solve_times else None,
    }


def global_seesm_metrics(run_dir: Path) -> dict[str, object]:
    rows = read_csv(run_dir / "global_seesm_log.csv")
    beta_values: list[float] = []
    replan_values: list[float] = []
    semantic_rejections = 0
    stale_count = 0
    missing_count = 0
    for row in rows:
        beta = parse_float(row.get("beta_applied"))
        if beta is not None:
            beta_values.append(beta)
        replan_ms = parse_float(row.get("global_replan_ms"))
        if replan_ms is not None:
            replan_values.append(replan_ms)
        reason = (row.get("reason") or "").strip().lower()
        rejected = row.get("primitive_rejected") in {"1", "true", "True"} or row.get("shot_rejected") in {"1", "true", "True"}
        semantic_rejections += int(reason == "semantic" and rejected)
        stale_count += int(reason == "stale")
        missing_count += int(reason == "missing")
    return {
        "global_beta_applied_max": max(beta_values) if beta_values else None,
        "global_semantic_rejection_count": semantic_rejections,
        "global_stale_margin_count": stale_count,
        "global_missing_margin_count": missing_count,
        "global_replan_mean_ms": mean(replan_values),
    }


def trial_identity(run_dir: Path) -> tuple[str, str]:
    if yaml is None or not (run_dir / "meta.yaml").exists():
        return "", ""
    try:
        with (run_dir / "meta.yaml").open("r", encoding="utf-8") as handle:
            meta = yaml.safe_load(handle) or {}
        trial = meta.get("trial_manifest") or {}
        return str(trial.get("trial_id", "")), str(trial.get("seed", meta.get("random_seed", "")))
    except (OSError, TypeError, ValueError, yaml.YAMLError):
        return "", ""


def metric_from_summary(summary_row: dict[str, str], primary: str, fallback: str) -> float | None:
    value = parse_float(summary_row.get(primary))
    if value is not None:
        return value
    return parse_float(summary_row.get(fallback))


def paper_outcome(summary_row: dict[str, str]) -> tuple[int, int, int]:
    goal_reached = int((parse_float(summary_row.get("success")) or 0.0) > 0.0)
    collision_count = int(parse_float(summary_row.get("nav_collision_count")) or 0.0)
    return int(goal_reached == 1 and collision_count == 0), goal_reached, collision_count


def collision_episode_metrics(run_dir: Path) -> dict[str, int]:
    robot_radius = 0.4
    meta_path = run_dir / "meta.yaml"
    if yaml is not None and meta_path.exists():
        try:
            with meta_path.open("r", encoding="utf-8") as handle:
                meta = yaml.safe_load(handle) or {}
            robot_radius = float(meta.get("robot_radius", robot_radius))
        except (OSError, TypeError, ValueError, yaml.YAMLError):
            pass

    samples_by_obstacle: dict[str, list[tuple[float, float]]] = {}
    for row in read_csv(run_dir / "obstacle_log.csv"):
        timestamp = parse_float(row.get("t"))
        radius = parse_float(row.get("radius"))
        distance = parse_float(row.get("d_i"))
        if timestamp is None or radius is None or distance is None:
            continue
        obstacle_id = str(row.get("id") or "unknown")
        samples_by_obstacle.setdefault(obstacle_id, []).append(
            (timestamp, distance - radius - robot_radius)
        )

    episode_count = 0
    for samples in samples_by_obstacle.values():
        samples.sort()
        intervals = [
            samples[index][0] - samples[index - 1][0]
            for index in range(1, len(samples))
            if samples[index][0] > samples[index - 1][0]
        ]
        nominal_period = percentile(intervals, 0.90)
        max_gap = 2.5 * nominal_period if nominal_period is not None else 0.0
        in_episode = False
        previous_negative_time = None
        for timestamp, clearance in samples:
            if clearance >= 0.0:
                in_episode = False
                previous_negative_time = None
                continue
            if (
                not in_episode
                or previous_negative_time is None
                or timestamp - previous_negative_time > max_gap
            ):
                episode_count += 1
                in_episode = True
            previous_negative_time = timestamp

    return {"collision_episode_count": episode_count}


def build_rows(output_root: Path, config_path: Path) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    scenario_meta = load_scenario_meta(config_path)
    summary_rows = collect_summary_rows(output_root)
    manifest_rows: list[dict[str, object]] = []
    metric_rows: list[dict[str, object]] = []
    source_batch = output_root.name

    for summary in summary_rows:
        baseline = summary.get("baseline", "")
        if baseline not in METHOD_LABELS:
            continue
        output_dir = Path(summary.get("output_dir", ""))
        if not output_dir.exists():
            continue
        run_id = output_dir.name
        scenario = summary.get("scenario", "")
        repeat_id = repeat_id_from_run(run_id)
        trial_id, seed = trial_identity(output_dir)
        meta = scenario_meta.get(scenario, {})
        method_label = METHOD_LABELS[baseline]
        summary_csv = output_dir / "summary.csv"

        manifest_rows.append(
            {
                "repeat_id": repeat_id,
                "trial_id": trial_id,
                "seed": seed,
                "scenario": scenario,
                "baseline": baseline,
                "source_batch": source_batch,
                "output_dir": str(output_dir),
                "run_id": run_id,
                "scenario_family": meta.get("scenario_family", ""),
                "context_level": meta.get("context_level", ""),
                "method_label": method_label,
                "summary_csv": str(summary_csv),
            }
        )

        sem = semantic_violation_metrics(output_dir)
        plan = planner_metrics(output_dir, summary)
        global_metrics = global_seesm_metrics(output_dir)
        collision_episodes = collision_episode_metrics(output_dir)
        success, goal_reached, collision_count = paper_outcome(summary)
        min_h = parse_float(summary.get("h_see_min"))
        if min_h is None:
            min_h = sem["min_h_seesm_from_log"]

        metric_rows.append(
            {
                "repeat_id": repeat_id,
                "trial_id": trial_id,
                "seed": seed,
                "scenario": scenario,
                "scenario_family": meta.get("scenario_family", ""),
                "context_level": meta.get("context_level", ""),
                "baseline": baseline,
                "method_label": method_label,
                "run_id": run_id,
                "output_dir": str(output_dir),
                "success": success,
                "goal_reached": goal_reached,
                "collision_count": collision_count,
                "collision_episode_count": collision_episodes["collision_episode_count"],
                "d_min_m": fmt(metric_from_summary(summary, "log_min_distance_m", "nav_min_distance_m")),
                "min_h_seesm": fmt(min_h),
                "semantic_violation_ratio": fmt(sem["semantic_violation_ratio"]),
                "semantic_violation_pair_ratio": fmt(sem["semantic_violation_pair_ratio"]),
                "mpc_feasibility_rate": fmt(plan["mpc_feasibility_rate"]),
                "mpc_first_attempt_feasibility_rate": fmt(plan["mpc_first_attempt_feasibility_rate"]),
                "path_length_m": fmt(metric_from_summary(summary, "robot_path_length_m", "nav_path_length_m")),
                "travel_time_s": fmt(metric_from_summary(summary, "robot_travel_time_s", "nav_travel_time_s")),
                "solve_time_mean_ms": fmt(plan["solve_time_mean_ms"]),
                "solve_time_p95_ms": fmt(plan["solve_time_p95_ms"]),
                "solve_time_max_ms": fmt(plan["solve_time_max_ms"]),
                "planner_records": plan["planner_records"],
                "guard_records": sem["guard_records"],
                "safety_bound_passed": summary.get("safety_bound_passed", ""),
                "global_beta_applied_max": fmt(global_metrics["global_beta_applied_max"]),
                "global_semantic_rejection_count": global_metrics["global_semantic_rejection_count"],
                "global_stale_margin_count": global_metrics["global_stale_margin_count"],
                "global_missing_margin_count": global_metrics["global_missing_margin_count"],
                "global_replan_mean_ms": fmt(global_metrics["global_replan_mean_ms"]),
            }
        )

    return manifest_rows, metric_rows


def aggregate_rows(metric_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    grouped: dict[tuple[str, str, str, str, str], list[dict[str, object]]] = {}
    for row in metric_rows:
        key = (
            str(row["scenario"]),
            str(row["scenario_family"]),
            str(row["context_level"]),
            str(row["baseline"]),
            str(row["method_label"]),
        )
        grouped.setdefault(key, []).append(row)

    summary_rows: list[dict[str, object]] = []
    for key, rows in sorted(grouped.items()):
        scenario, family, context, baseline, method = key

        def values(name: str) -> list[float]:
            return [value for value in (parse_float(row.get(name)) for row in rows) if value is not None]

        success = values("success")
        goal_reached = values("goal_reached")
        collision_count = values("collision_count")
        collision_episode_count = values("collision_episode_count")
        d_min = values("d_min_m")
        min_h = values("min_h_seesm")
        summary_rows.append(
            {
                "scenario": scenario,
                "scenario_family": family,
                "context_level": context,
                "baseline": baseline,
                "method_label": method,
                "n_trials": len(rows),
                "success_rate": fmt(mean(success)),
                "goal_reached_rate": fmt(mean(goal_reached)),
                "collision_rate": fmt(mean([float(value > 0.0) for value in collision_count])),
                "collision_episode_mean": fmt(mean(collision_episode_count)),
                "d_min_mean_m": fmt(mean(d_min)),
                "d_min_min_m": fmt(min(d_min) if d_min else None),
                "min_h_seesm_mean": fmt(mean(min_h)),
                "min_h_seesm_min": fmt(min(min_h) if min_h else None),
                "semantic_violation_ratio_mean": fmt(mean(values("semantic_violation_ratio"))),
                "mpc_feasibility_rate_mean": fmt(mean(values("mpc_feasibility_rate"))),
                "path_length_mean_m": fmt(mean(values("path_length_m"))),
                "travel_time_mean_s": fmt(mean(values("travel_time_s"))),
                "solve_time_mean_ms": fmt(mean(values("solve_time_mean_ms"))),
                "solve_time_p95_ms": fmt(mean(values("solve_time_p95_ms"))),
                "global_beta_applied_max": fmt(max(values("global_beta_applied_max")) if values("global_beta_applied_max") else None),
                "global_semantic_rejection_count_mean": fmt(mean(values("global_semantic_rejection_count"))),
                "global_stale_margin_count_mean": fmt(mean(values("global_stale_margin_count"))),
                "global_missing_margin_count_mean": fmt(mean(values("global_missing_margin_count"))),
                "global_replan_mean_ms": fmt(mean(values("global_replan_mean_ms"))),
            }
        )
    return summary_rows


def main() -> None:
    parser = argparse.ArgumentParser(description="Post-process teacher canonical MPC-SECBF experiment batches.")
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument(
        "--config",
        type=Path,
        default=repo_root() / "swarm_test/config/secbf_scenarios.yaml",
    )
    args = parser.parse_args()
    output_root = args.output_root.expanduser().resolve()
    config = args.config.expanduser().resolve()

    manifest_rows, metric_rows = build_rows(output_root, config)
    table_rows = aggregate_rows(metric_rows)

    write_csv(output_root / "manifest.csv", manifest_rows, MANIFEST_FIELDS)
    write_csv(output_root / "teacher_run_metrics.csv", metric_rows, RUN_FIELDS)
    write_csv(output_root / "teacher_table_summary.csv", table_rows, TABLE_FIELDS)
    print(
        {
            "output_root": str(output_root),
            "manifest_rows": len(manifest_rows),
            "metric_rows": len(metric_rows),
            "table_rows": len(table_rows),
            "manifest": str(output_root / "manifest.csv"),
            "teacher_run_metrics": str(output_root / "teacher_run_metrics.csv"),
            "teacher_table_summary": str(output_root / "teacher_table_summary.csv"),
        }
    )


if __name__ == "__main__":
    main()
