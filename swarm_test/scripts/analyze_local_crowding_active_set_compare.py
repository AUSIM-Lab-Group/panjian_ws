#!/usr/bin/env python3
"""Audit and compare the paired max6/max3 local-crowding pilot."""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
from pathlib import Path

import yaml


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        raise ValueError(f"refusing to write empty CSV: {path}")
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def number(value) -> float:
    try:
        result = float(value)
        return result if math.isfinite(result) else math.nan
    except (TypeError, ValueError):
        return math.nan


def mean(values: list[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return math.nan
    return sum(finite) / len(finite)


def load_variant(root: Path, expected_max: int) -> dict[tuple[str, str], dict]:
    metrics_path = root / "teacher_run_metrics.csv"
    summary_path = root / "summary.csv"
    if not metrics_path.is_file() or not summary_path.is_file():
        raise ValueError(f"missing post-processed outputs under {root}")
    metrics = read_csv(metrics_path)
    summary_by_run = {
        Path(row["output_dir"]).name: row for row in read_csv(summary_path)
    }
    rows: dict[tuple[str, str], dict] = {}
    for metric in metrics:
        run_id = metric["run_id"]
        run_dir = root / run_id
        summary = summary_by_run.get(run_id)
        if summary is None:
            raise ValueError(f"missing summary row for {run_id}")
        if metric.get("common_eval_status") != "ok":
            raise ValueError(f"common evaluation failed for {run_id}")
        if not (run_dir / "RUN_COMPLETE.txt").is_file():
            raise ValueError(f"unsealed trial: {run_dir}")
        if (run_dir / "RUN_INVALID.txt").exists():
            raise ValueError(f"invalid trial: {run_dir}")
        meta = yaml.safe_load((run_dir / "meta.yaml").read_text(encoding="utf-8"))
        actual_max = int(meta["max_cbf_obstacles"])
        if actual_max != expected_max:
            raise ValueError(
                f"{run_id}: max_cbf_obstacles={actual_max}, expected {expected_max}"
            )
        key = (metric["trial_id"], metric["seed"])
        if key in rows:
            raise ValueError(f"duplicate trial identity {key} under {root}")
        rows[key] = {
            "trial_id": key[0],
            "seed": key[1],
            "run_id": run_id,
            "obstacle_sha256": hashlib.sha256(
                (run_dir / "obstacles_param.yaml").read_bytes()
            ).hexdigest(),
            "success": int(float(metric["success"])),
            "executed_collision": int(float(metric["collision_episode_count"]) > 0.0),
            "d_min_m": number(metric["d_min_m"]),
            "min_h_eval": number(metric["min_h_eval"]),
            "semantic_violation": number(metric["semantic_violation_eval_ratio"]),
            "mpc_feasibility": number(metric["mpc_feasibility_rate"]),
            "solve_mean_ms": number(metric["solve_time_mean_ms"]),
            "solve_p95_ms": number(metric["solve_time_p95_ms"]),
            "constrained_obs_mean": number(summary["constrained_obs_count_mean"]),
            "constrained_obs_max": number(summary["constrained_obs_count_max"]),
        }
    return rows


def evaluate_gates(pairs: list[dict], acceptance: dict) -> tuple[list[dict], bool]:
    expected_pairs = int(acceptance["expected_pairs"])
    if len(pairs) != expected_pairs:
        raise ValueError(f"expected {expected_pairs} pairs, found {len(pairs)}")
    added_collisions = sum(row["executed_collision_added"] for row in pairs)
    success_regressions = sum(row["success_regression"] for row in pairs)
    mean_d_min_delta = mean([row["delta_d_min_m"] for row in pairs])
    mean_semantic_delta = mean([row["delta_semantic_violation"] for row in pairs])
    gates = [
        {
            "gate": "added_executed_collision_pairs",
            "observed": added_collisions,
            "limit": int(acceptance["max_added_executed_collision_pairs"]),
            "operator": "<=",
            "passed": int(
                added_collisions <= int(acceptance["max_added_executed_collision_pairs"])
            ),
        },
        {
            "gate": "mean_d_min_delta_m",
            "observed": mean_d_min_delta,
            "limit": float(acceptance["min_mean_d_min_delta_m"]),
            "operator": ">=",
            "passed": int(
                mean_d_min_delta >= float(acceptance["min_mean_d_min_delta_m"])
            ),
        },
        {
            "gate": "mean_semantic_violation_delta",
            "observed": mean_semantic_delta,
            "limit": float(acceptance["max_mean_semantic_violation_delta"]),
            "operator": "<=",
            "passed": int(
                mean_semantic_delta
                <= float(acceptance["max_mean_semantic_violation_delta"])
            ),
        },
        {
            "gate": "success_regression_pairs",
            "observed": success_regressions,
            "limit": int(acceptance["max_success_regression_pairs"]),
            "operator": "<=",
            "passed": int(
                success_regressions <= int(acceptance["max_success_regression_pairs"])
            ),
        },
    ]
    return gates, all(row["passed"] for row in gates)


def build_pairs(reference: dict, candidate: dict) -> list[dict]:
    if set(reference) != set(candidate):
        missing_reference = sorted(set(candidate) - set(reference))
        missing_candidate = sorted(set(reference) - set(candidate))
        raise ValueError(
            f"pair mismatch: missing reference={missing_reference}, "
            f"missing candidate={missing_candidate}"
        )
    pairs = []
    for key in sorted(reference):
        ref = reference[key]
        cand = candidate[key]
        if ref["obstacle_sha256"] != cand["obstacle_sha256"]:
            raise ValueError(f"paired obstacle hash mismatch for {key}")
        pairs.append({
            "trial_id": key[0],
            "seed": key[1],
            "reference_run_id": ref["run_id"],
            "candidate_run_id": cand["run_id"],
            "obstacle_sha256": ref["obstacle_sha256"],
            "reference_success": ref["success"],
            "candidate_success": cand["success"],
            "success_regression": int(cand["success"] < ref["success"]),
            "reference_executed_collision": ref["executed_collision"],
            "candidate_executed_collision": cand["executed_collision"],
            "executed_collision_added": int(
                cand["executed_collision"] > ref["executed_collision"]
            ),
            "reference_d_min_m": ref["d_min_m"],
            "candidate_d_min_m": cand["d_min_m"],
            "delta_d_min_m": cand["d_min_m"] - ref["d_min_m"],
            "reference_min_h_eval": ref["min_h_eval"],
            "candidate_min_h_eval": cand["min_h_eval"],
            "delta_min_h_eval": cand["min_h_eval"] - ref["min_h_eval"],
            "reference_semantic_violation": ref["semantic_violation"],
            "candidate_semantic_violation": cand["semantic_violation"],
            "delta_semantic_violation": (
                cand["semantic_violation"] - ref["semantic_violation"]
            ),
            "reference_mpc_feasibility": ref["mpc_feasibility"],
            "candidate_mpc_feasibility": cand["mpc_feasibility"],
            "delta_mpc_feasibility": cand["mpc_feasibility"] - ref["mpc_feasibility"],
            "reference_solve_mean_ms": ref["solve_mean_ms"],
            "candidate_solve_mean_ms": cand["solve_mean_ms"],
            "delta_solve_mean_ms": cand["solve_mean_ms"] - ref["solve_mean_ms"],
            "reference_solve_p95_ms": ref["solve_p95_ms"],
            "candidate_solve_p95_ms": cand["solve_p95_ms"],
            "delta_solve_p95_ms": cand["solve_p95_ms"] - ref["solve_p95_ms"],
            "reference_constrained_obs_mean": ref["constrained_obs_mean"],
            "candidate_constrained_obs_mean": cand["constrained_obs_mean"],
            "reference_constrained_obs_max": ref["constrained_obs_max"],
            "candidate_constrained_obs_max": cand["constrained_obs_max"],
        })
    return pairs


def write_report(path: Path, config: dict, pairs: list[dict], gates: list[dict], passed: bool) -> None:
    lines = [
        "# Local-crowding max6/max3 active-set pilot",
        "",
        f"- Comparison: `{config['comparison_id']}`",
        f"- Scenario: `{config['scenario']}`",
        f"- Baseline: `{config['baseline']}`",
        f"- Pairs: `{len(pairs)}`",
        f"- Selection: `{config['selection_policy']['ordering']}` within "
        f"`{config['selection_policy']['active_set_distance_m']:.1f} m`",
        f"- Decision: `{'PASS' if passed else 'FAIL'}`",
        "",
        "## Aggregate deltas (max3 - max6)",
        "",
        f"- Added executed-collision pairs: `{sum(row['executed_collision_added'] for row in pairs)}`",
        f"- Success-regression pairs: `{sum(row['success_regression'] for row in pairs)}`",
        f"- Mean minimum-distance delta: `{mean([row['delta_d_min_m'] for row in pairs]):.6f} m`",
        f"- Mean semantic-violation delta: `{100.0 * mean([row['delta_semantic_violation'] for row in pairs]):.4f} percentage points`",
        f"- Mean common-clearance delta: `{mean([row['delta_min_h_eval'] for row in pairs]):.6f} m`",
        f"- Mean MPC-feasibility delta: `{100.0 * mean([row['delta_mpc_feasibility'] for row in pairs]):.4f} percentage points`",
        f"- Mean solve-time delta: `{mean([row['delta_solve_mean_ms'] for row in pairs]):.4f} ms`",
        f"- Mean trial-p95 delta: `{mean([row['delta_solve_p95_ms'] for row in pairs]):.4f} ms`",
        "",
        "## Gates",
        "",
    ]
    for gate in gates:
        lines.append(
            f"- `{gate['gate']}`: observed `{gate['observed']}`, required "
            f"`{gate['operator']} {gate['limit']}` -> "
            f"`{'PASS' if gate['passed'] else 'FAIL'}`"
        )
    lines.extend([
        "",
        "## Boundary",
        "",
        "- A PASS is screening evidence only. Keep the default at 6 until the same gates pass on 30 paired seeds.",
        "- This comparison retains the current distance-ascending active-set ordering and changes no other controller switch.",
    ])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--reference-root", type=Path, required=True)
    parser.add_argument("--candidate-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    reference_max = int(config["variants"]["reference"]["max_cbf_obstacles"])
    candidate_max = int(config["variants"]["candidate"]["max_cbf_obstacles"])
    reference = load_variant(args.reference_root, reference_max)
    candidate = load_variant(args.candidate_root, candidate_max)
    pairs = build_pairs(reference, candidate)
    gates, passed = evaluate_gates(pairs, config["acceptance"])
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "paired_trials.csv", pairs)
    write_csv(args.output_dir / "acceptance_gates.csv", gates)
    write_report(args.output_dir / "local_crowding_max3_pilot_report.md", config, pairs, gates, passed)
    sentinel = args.output_dir / (
        "ACTIVE_SET_MAX3_PILOT_PASS.txt" if passed else "ACTIVE_SET_MAX3_PILOT_FAIL.txt"
    )
    sentinel.write_text(
        f"active-set max3 pilot {'passed' if passed else 'failed'}: "
        f"{len(pairs)}/{config['acceptance']['expected_pairs']} paired trials.\n",
        encoding="utf-8",
    )
    print({"pairs": len(pairs), "passed": passed, "sentinel": str(sentinel)})
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
