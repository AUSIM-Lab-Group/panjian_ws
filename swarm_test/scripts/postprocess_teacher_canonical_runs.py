#!/usr/bin/env python3
from __future__ import annotations

"""Post-process teacher-style MPC-SECBF canonical simulation batches."""

import argparse
import csv
import hashlib
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
    "Category_only": "Category-only SEESM",
    "Unguarded_SEESM": "SEESM w/o FPU",
    "No_J_side": r"SEESM w/o J_side",
    "SEESM_Ours": "Proposed MPC-SECBF",
}

COMMON_EVALUATION_CONTRACT_VERSION = "teacher_v1_common_offline_evaluation_001"
COMMON_EVALUATION_TAU_MODE = "teacher_tca"
COMMON_EVALUATION_CATEGORIES = (
    "box",
    "adult",
    "pedestrian",
    "child",
    "child_like",
    "cyclist",
    "vehicle",
    "unknown",
)


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
    "min_h_eval",
    "semantic_violation_eval_ratio",
    "eval_records",
    "common_eval_contract_id",
    "common_eval_contract_version",
    "common_eval_contract_status",
    "common_eval_contract_sha256",
    "common_eval_status",
    "h_eesm_eval_log_error_max",
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
    "common_eval_contract_id",
    "common_eval_contract_version",
    "common_eval_contract_status",
    "common_eval_contract_sha256",
    "common_eval_status",
    "summary_csv",
]

TABLE_FIELDS = [
    "scenario",
    "scenario_family",
    "context_level",
    "baseline",
    "method_label",
    "common_eval_contract_id",
    "common_eval_contract_version",
    "common_eval_contract_status",
    "common_eval_contract_sha256",
    "common_eval_status",
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
    "min_h_eval_mean",
    "min_h_eval_min",
    "semantic_violation_eval_ratio_mean",
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


def parse_bool(value, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    return default


class CommonEvaluationContractError(ValueError):
    """Raised when the frozen common-offline evaluator is unusable."""


def sha256_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require_finite_number(value, label: str, *, positive: bool = False,
                          nonnegative: bool = False) -> float:
    parsed = parse_float(value)
    if parsed is None or not math.isfinite(parsed):
        raise CommonEvaluationContractError(f"{label} must be a finite number")
    if positive and parsed <= 0.0:
        raise CommonEvaluationContractError(f"{label} must be positive")
    if nonnegative and parsed < 0.0:
        raise CommonEvaluationContractError(f"{label} must be nonnegative")
    return float(parsed)


def load_common_evaluation_contract(path: Path) -> dict[str, object]:
    """Load the method-independent offline evaluator used for Table I/E2.

    This deliberately does not inspect a controller's run metadata.  The
    controller may use a fixed-distance barrier, no semantic margin, or a
    Guard-reduced margin; the teacher plan instead requires one frozen TCA,
    Phi, and beta_max mapping for *every* realised trajectory.
    """
    path = Path(path).expanduser().resolve()
    if yaml is None:
        raise CommonEvaluationContractError("PyYAML is required for common evaluation")
    if not path.exists():
        raise CommonEvaluationContractError(f"common evaluation contract not found: {path}")
    try:
        with path.open("r", encoding="utf-8") as handle:
            payload = yaml.safe_load(handle) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise CommonEvaluationContractError(
            f"cannot read common evaluation contract {path}: {exc}"
        ) from exc
    if not isinstance(payload, dict):
        raise CommonEvaluationContractError("common evaluation contract must be a YAML mapping")
    contract_id = str(payload.get("id", "")).strip()
    if not contract_id:
        raise CommonEvaluationContractError("common evaluation contract id is required")
    if payload.get("version") != COMMON_EVALUATION_CONTRACT_VERSION:
        raise CommonEvaluationContractError("common evaluation contract version mismatch")
    if payload.get("status") not in {"smoke_frozen", "formal_frozen"}:
        raise CommonEvaluationContractError(
            "common evaluation contract status must be smoke_frozen or formal_frozen"
        )

    tau = payload.get("dynamic_tau")
    if not isinstance(tau, dict) or tau.get("mode") != COMMON_EVALUATION_TAU_MODE:
        raise CommonEvaluationContractError("common evaluation must use fixed teacher_tca")
    delta_tau = require_finite_number(
        tau.get("delta_tau"), "dynamic_tau.delta_tau", positive=True
    )
    max_tau = require_finite_number(
        tau.get("max_tau_sec"), "dynamic_tau.max_tau_sec", nonnegative=True
    )

    phi = payload.get("phi")
    if not isinstance(phi, dict):
        raise CommonEvaluationContractError("common evaluation phi must be a mapping")
    if phi.get("form") != "explicit_linear_current_mapping":
        raise CommonEvaluationContractError("common evaluation phi form mismatch")
    weights_raw = phi.get("weights")
    if not isinstance(weights_raw, dict):
        raise CommonEvaluationContractError("common evaluation phi weights must be a mapping")
    weights = {
        name: require_finite_number(weights_raw.get(name), f"phi.weights.{name}")
        for name in ("bias", "head_on", "ttc_norm", "density_norm")
    }
    if any(weights[name] < 0.0 for name in ("head_on", "ttc_norm", "density_norm")):
        raise CommonEvaluationContractError("common evaluation interaction weights must be nonnegative")
    if phi.get("multiplier_clip") != [0.0, 1.0]:
        raise CommonEvaluationContractError("common evaluation phi multiplier clip mismatch")

    def category_table(name: str) -> dict[str, float]:
        raw = payload.get(name)
        if not isinstance(raw, dict):
            raise CommonEvaluationContractError(f"{name} must be a mapping")
        missing = [category for category in COMMON_EVALUATION_CATEGORIES if category not in raw]
        if missing:
            raise CommonEvaluationContractError(f"{name} is missing categories: {missing}")
        return {
            category: require_finite_number(
                raw[category], f"{name}.{category}", nonnegative=True
            )
            for category in COMMON_EVALUATION_CATEGORIES
        }

    beta_bar = category_table("beta_bar_m")
    beta_max = category_table("beta_max_m")

    features = payload.get("features")
    if not isinstance(features, dict):
        raise CommonEvaluationContractError("common evaluation features must be a mapping")
    ttc_horizon = require_finite_number(
        features.get("ttc_horizon_sec"), "features.ttc_horizon_sec", positive=True
    )
    min_distance = require_finite_number(
        features.get("min_feature_distance_m"),
        "features.min_feature_distance_m", nonnegative=True,
    )
    min_speed = require_finite_number(
        features.get("min_feature_speed_mps"),
        "features.min_feature_speed_mps", nonnegative=True,
    )
    min_closing_speed = require_finite_number(
        features.get("min_closing_speed_mps"),
        "features.min_closing_speed_mps", nonnegative=True,
    )
    density_divisor = require_finite_number(
        features.get("density_divisor"), "features.density_divisor", positive=True
    )
    if payload.get("category_policy") != "error":
        raise CommonEvaluationContractError("common evaluation category_policy must be error")

    return {
        "id": contract_id,
        "version": COMMON_EVALUATION_CONTRACT_VERSION,
        "status": payload["status"],
        "path": str(path),
        "sha256": sha256_file(path),
        "delta_tau": delta_tau,
        "max_tau_sec": max_tau,
        "weights": weights,
        "beta_bar_m": beta_bar,
        "beta_max_m": beta_max,
        "ttc_horizon_sec": ttc_horizon,
        "min_feature_distance_m": min_distance,
        "min_feature_speed_mps": min_speed,
        "min_closing_speed_mps": min_closing_speed,
        "density_divisor": density_divisor,
    }


def common_evaluation_tau(distance: float, speed: float, cos_delta: float,
                          contract: dict[str, object]) -> float:
    denominator = speed * speed + float(contract["delta_tau"])
    raw = -(distance * speed * cos_delta) / denominator
    return max(0.0, min(raw, float(contract["max_tau_sec"])))


def common_evaluation_density(rows: list[dict[str, str]],
                              contract: dict[str, object]) -> dict[str, float]:
    obstacle_ids_by_cycle: dict[str, set[str]] = {}
    for row_index, row in enumerate(rows, start=1):
        cycle = str(row.get("obstacle_cycle_id", "")).strip()
        obstacle_id = str(row.get("obs_id", "")).strip()
        if not cycle or not obstacle_id:
            raise CommonEvaluationContractError(
                "margin_guard_log.csv row "
                f"{row_index} is missing obstacle_cycle_id or obs_id"
            )
        obstacle_ids_by_cycle.setdefault(cycle, set()).add(obstacle_id)
    divisor = float(contract["density_divisor"])
    return {
        cycle: min(1.0, max(0.0, (len(ids) - 1) / divisor))
        for cycle, ids in obstacle_ids_by_cycle.items()
    }


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


def parse_seal(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    try:
        for line in Path(path).read_text(encoding="utf-8").splitlines():
            key, separator, value = line.partition("=")
            if separator:
                values[key.strip()] = value.strip()
    except OSError:
        return {}
    return values


def has_valid_integrity_manifest(run_dir: Path) -> bool:
    manifest = run_dir / "trial_integrity.sha256"
    if not manifest.exists() or manifest.stat().st_size <= 0:
        return False
    try:
        for line in manifest.read_text(encoding="utf-8").splitlines():
            expected, separator, relative_name = line.partition("  ")
            relative = Path(relative_name)
            if (not separator or not expected or not relative_name or
                    relative.is_absolute() or ".." in relative.parts):
                return False
            target = run_dir / relative
            if not target.is_file() or sha256_file(target) != expected:
                return False
    except OSError:
        return False
    return True


def is_sealed_complete_run_dir(run_dir: Path) -> bool:
    """Accept only a runner-sealed, intact trial; never trust root summary.csv."""
    run_dir = Path(run_dir)
    complete = run_dir / "RUN_COMPLETE.txt"
    invalid = run_dir / "RUN_INVALID.txt"
    if (not complete.exists() or complete.stat().st_size <= 0 or invalid.exists() or
            not (run_dir / "summary.csv").exists() or
            not (run_dir / "run_meta.yaml").exists()):
        return False
    seal = parse_seal(complete)
    if seal.get("trial_valid") != "true":
        return False
    if seal.get("run_meta_sha256") != sha256_file(run_dir / "run_meta.yaml"):
        return False
    manifest = run_dir / "trial_integrity.sha256"
    if (not manifest.exists() or
            seal.get("integrity_manifest_sha256") != sha256_file(manifest) or
            not has_valid_integrity_manifest(run_dir)):
        return False
    if yaml is None:
        return False
    try:
        with (run_dir / "run_meta.yaml").open("r", encoding="utf-8") as handle:
            meta = yaml.safe_load(handle) or {}
    except (OSError, TypeError, ValueError, yaml.YAMLError):
        return False
    return isinstance(meta, dict) and meta.get("run_state") == "complete"


def collect_summary_rows(output_root: Path) -> list[dict[str, str]]:
    collected: list[dict[str, str]] = []
    for summary in sorted(output_root.glob("*/summary.csv")):
        if not is_sealed_complete_run_dir(summary.parent):
            continue
        run_rows = read_csv(summary)
        if run_rows:
            collected.append(run_rows[0])
    return collected


def validate_run_common_evaluation_provenance(
    run_dir: Path, common_evaluation: dict[str, object]
) -> None:
    """Reject a sealed trajectory whose metadata names another evaluator."""
    if yaml is None:
        raise CommonEvaluationContractError("PyYAML is required for run provenance")
    meta_path = Path(run_dir) / "run_meta.yaml"
    try:
        with meta_path.open("r", encoding="utf-8") as handle:
            meta = yaml.safe_load(handle) or {}
    except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise CommonEvaluationContractError(
            f"cannot read run metadata for common-evaluation provenance: {exc}"
        ) from exc
    if not isinstance(meta, dict):
        raise CommonEvaluationContractError("run metadata must be a mapping")
    record = meta.get("common_offline_evaluation_contract")
    if not isinstance(record, dict):
        raise CommonEvaluationContractError(
            "run metadata lacks common_offline_evaluation_contract"
        )
    for field in ("id", "version", "status", "sha256"):
        if record.get(field) != common_evaluation.get(field):
            raise CommonEvaluationContractError(
                f"run common-evaluation provenance {field} mismatch"
            )


def semantic_violation_metrics(
    run_dir: Path, common_evaluation: dict[str, object]
) -> dict[str, object]:
    rows = read_csv(run_dir / "margin_guard_log.csv")
    if not rows:
        return {
            "guard_records": 0,
            "min_h_seesm_from_log": None,
            "semantic_violation_ratio": None,
            "semantic_violation_pair_ratio": None,
            "min_h_eval": None,
            "semantic_violation_eval_ratio": None,
            "eval_records": 0,
            "common_eval_status": "missing_margin_guard_log",
            "common_eval_error": "margin_guard_log.csv has no rows",
            "h_eesm_eval_log_error_max": None,
        }

    time_groups: dict[str, bool] = {}
    h_values: list[float] = []
    h_eval_values: list[float] = []
    h_eesm_eval_log_errors: list[float] = []
    pair_violations = 0
    try:
        density_by_cycle = common_evaluation_density(rows, common_evaluation)
    except CommonEvaluationContractError as exc:
        density_by_cycle = {}
        common_eval_error = str(exc)
    else:
        common_eval_error = ""

    for row_index, row in enumerate(rows, start=1):
        h_see = parse_float(row.get("h_seesm"))
        if h_see is None:
            h_see = parse_float(row.get("h_see"))
        if h_see is not None:
            h_values.append(h_see)
            violated = h_see < 0.0
            pair_violations += int(violated)
            time_key = row.get("time", "")
            time_groups[time_key] = time_groups.get(time_key, False) or violated

        distance = parse_float(row.get("d_i"))
        speed = parse_float(row.get("rel_v_norm"))
        cos_delta = parse_float(row.get("cos_delta"))
        inflated_radius = parse_float(row.get("R_base"))
        geometry_values = (distance, speed, cos_delta, inflated_radius)
        if any(
            value is None or not math.isfinite(value)
            for value in geometry_values
        ):
            common_eval_error = (
                "margin_guard_log.csv row "
                f"{row_index} has missing/nonfinite common geometry"
            )
            break

        distance = max(0.0, float(distance))
        speed = max(0.0, float(speed))
        cos_delta = max(-1.0, min(1.0, float(cos_delta)))
        inflated_radius = max(0.0, float(inflated_radius))
        category = str(row.get("class", "")).strip()
        if category not in common_evaluation["beta_bar_m"]:
            common_eval_error = (
                "margin_guard_log.csv row "
                f"{row_index} has unknown common-evaluation category {category!r}"
            )
            break
        cycle = str(row.get("obstacle_cycle_id", "")).strip()
        if cycle not in density_by_cycle:
            common_eval_error = (
                "margin_guard_log.csv row "
                f"{row_index} has no common-evaluation density group"
            )
            break

        # This is intentionally reconstructed from raw state features and the
        # frozen evaluator contract.  In particular, do not use beta_requested
        # (which is zero/fixed for some baselines), beta_pre_guard,
        # beta_applied, a logged tau, or a controller-specific dynamic_tau
        # contract.  That is the teacher-required common offline comparison.
        direction_is_observable = (
            distance > float(common_evaluation["min_feature_distance_m"])
            and speed > float(common_evaluation["min_feature_speed_mps"])
        )
        if direction_is_observable:
            f_head = max(0.0, -cos_delta)
        else:
            f_head = 0.0
        closing_speed = max(-speed * cos_delta, 0.0)
        if closing_speed > float(common_evaluation["min_closing_speed_mps"]):
            ttc_norm = max(
                0.0,
                min(
                    1.0,
                    1.0 - distance / (
                        closing_speed * float(common_evaluation["ttc_horizon_sec"])
                    ),
                ),
            )
        else:
            ttc_norm = 0.0
        rho_norm = density_by_cycle[cycle]
        weights = common_evaluation["weights"]
        mu_eval = max(
            0.0,
            min(
                1.0,
                float(weights["bias"])
                + float(weights["head_on"]) * f_head
                + float(weights["ttc_norm"]) * ttc_norm
                + float(weights["density_norm"]) * rho_norm,
            ),
        )
        beta_tilde_eval = float(common_evaluation["beta_bar_m"][category]) * mu_eval
        beta_eval = min(
            beta_tilde_eval, float(common_evaluation["beta_max_m"][category])
        )
        tau = common_evaluation_tau(distance, speed, cos_delta, common_evaluation)

        norm_squared = (
            distance * distance
            + 2.0 * tau * distance * speed * cos_delta
            + tau * tau * speed * speed
        )
        h_eesm_eval = math.sqrt(max(0.0, norm_squared)) - inflated_radius
        h_eval_values.append(h_eesm_eval - beta_eval)

        logged_h_eesm = parse_float(row.get("h_eesm"))
        if logged_h_eesm is None:
            logged_h_eesm = parse_float(row.get("h_ee"))
        # beta_ground_truth_node intentionally logs cos_delta=0 below the
        # feature-speed gate, even though its internal dynamic-tau calculation
        # still has the full relative-velocity vector.  In that regime the
        # logged scalar geometry cannot reconstruct the controller h_eesm
        # unambiguously, so do not treat the resulting difference as a
        # dynamic-EESM contract error.  H_eval itself remains well-defined by
        # the frozen public evaluator and is still included above.
        if logged_h_eesm is not None and direction_is_observable:
            h_eesm_eval_log_errors.append(abs(h_eesm_eval - logged_h_eesm))

    common_eval_status = "ok" if not common_eval_error else "invalid_raw_input"
    if common_eval_error:
        # Never turn a partial common evaluator into a valid Table-I result.
        h_eval_values = []
        h_eesm_eval_log_errors = []

    if not h_values:
        return {
            "guard_records": len(rows),
            "min_h_seesm_from_log": None,
            "semantic_violation_ratio": None,
            "semantic_violation_pair_ratio": None,
            "min_h_eval": min(h_eval_values) if h_eval_values else None,
            "semantic_violation_eval_ratio": (
                sum(value < 0.0 for value in h_eval_values) / len(h_eval_values)
                if h_eval_values else None
            ),
            "eval_records": len(h_eval_values),
            "common_eval_status": common_eval_status,
            "common_eval_error": common_eval_error,
            "h_eesm_eval_log_error_max": (
                max(h_eesm_eval_log_errors) if h_eesm_eval_log_errors else None
            ),
        }
    return {
        "guard_records": len(rows),
        "min_h_seesm_from_log": min(h_values),
        "semantic_violation_ratio": (
            sum(1 for violated in time_groups.values() if violated) / len(time_groups)
            if time_groups else None
        ),
        "semantic_violation_pair_ratio": pair_violations / len(h_values),
        "min_h_eval": min(h_eval_values) if h_eval_values else None,
        "semantic_violation_eval_ratio": (
            sum(value < 0.0 for value in h_eval_values) / len(h_eval_values)
            if h_eval_values else None
        ),
        "eval_records": len(h_eval_values),
        "common_eval_status": common_eval_status,
        "common_eval_error": common_eval_error,
        "h_eesm_eval_log_error_max": (
            max(h_eesm_eval_log_errors) if h_eesm_eval_log_errors else None
        ),
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
        mpc_status = row.get("mpc_status")
        final_status = row.get("final_status") or row.get("mpc_status")
        first_status = row.get("first_attempt_status")
        accepted_source = row.get("accepted_beta_source")
        final_success += int(
            final_status == "success"
            and mpc_status != "no_cbf_fallback"
            and accepted_source != "no_cbf"
        )
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


def paper_travel_time(summary_row: dict[str, str]) -> float | None:
    """Return navigation completion time, with legacy-log compatibility.

    ``robot_travel_time_s`` spans the recorder window and is therefore not a
    time-to-goal metric.  ``nav_travel_time_s`` stops at navigation completion
    and is the quantity defined as T_travel in the paper.
    """
    return metric_from_summary(summary_row, "nav_travel_time_s", "robot_travel_time_s")


def paper_outcome(summary_row: dict[str, str]) -> tuple[int, int, int]:
    goal_value = parse_float(summary_row.get("goal_reached"))
    if goal_value is None:
        # Compatibility with historical summaries written before the explicit
        # goal_reached field was added.
        goal_value = parse_float(summary_row.get("success"))
    goal_reached = int((goal_value or 0.0) > 0.0)
    collision_count = int(parse_float(summary_row.get("nav_collision_count")) or 0.0)
    # ``nav_collision_count`` is sampled by the navigation logger and can miss
    # a brief footprint overlap that is present in the higher-rate obstacle
    # log.  The paper defines collision geometrically, so either source is
    # sufficient evidence.  Keep the historical count when it is available and
    # use one as a conservative event marker when only D_min detects contact.
    log_min_distance = parse_float(summary_row.get("log_min_distance_m"))
    if log_min_distance is not None and log_min_distance <= 0.0:
        collision_count = max(collision_count, 1)
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
        # The recorder can emit an all-zero placeholder while obstacle IDs are
        # being initialized.  The trial summary already excludes this row; the
        # collision-episode path must use the same contract or it creates a
        # false negative-clearance episode of -(radius + robot_radius).
        initialization_values = (
            distance,
            parse_float(row.get("rel_v")),
            parse_float(row.get("TTC")),
            parse_float(row.get("h_EE")),
        )
        if all(value is not None and abs(value) <= 1.0e-12 for value in initialization_values):
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


def build_rows(
    output_root: Path, config_path: Path, common_evaluation: dict[str, object]
) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
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
        if not output_dir.exists() or not is_sealed_complete_run_dir(output_dir):
            continue
        validate_run_common_evaluation_provenance(output_dir, common_evaluation)
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
                "common_eval_contract_id": common_evaluation["id"],
                "common_eval_contract_version": common_evaluation["version"],
                "common_eval_contract_status": common_evaluation["status"],
                "common_eval_contract_sha256": common_evaluation["sha256"],
                "common_eval_status": "pending",
                "summary_csv": str(summary_csv),
            }
        )

        sem = semantic_violation_metrics(output_dir, common_evaluation)
        if sem["common_eval_status"] != "ok":
            raise CommonEvaluationContractError(
                f"{output_dir}: common offline evaluation failed: "
                f"{sem['common_eval_error']}"
            )
        manifest_rows[-1]["common_eval_status"] = sem["common_eval_status"]
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
                "min_h_eval": fmt(sem["min_h_eval"]),
                "semantic_violation_eval_ratio": fmt(sem["semantic_violation_eval_ratio"]),
                "eval_records": sem["eval_records"],
                "common_eval_contract_id": common_evaluation["id"],
                "common_eval_contract_version": common_evaluation["version"],
                "common_eval_contract_status": common_evaluation["status"],
                "common_eval_contract_sha256": common_evaluation["sha256"],
                "common_eval_status": sem["common_eval_status"],
                "h_eesm_eval_log_error_max": fmt(sem["h_eesm_eval_log_error_max"]),
                "mpc_feasibility_rate": fmt(plan["mpc_feasibility_rate"]),
                "mpc_first_attempt_feasibility_rate": fmt(plan["mpc_first_attempt_feasibility_rate"]),
                "path_length_m": fmt(metric_from_summary(summary, "robot_path_length_m", "nav_path_length_m")),
                "travel_time_s": fmt(paper_travel_time(summary)),
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
        contract_ids = {str(row.get("common_eval_contract_id", "")) for row in rows}
        contract_hashes = {
            str(row.get("common_eval_contract_sha256", "")) for row in rows
        }
        contract_versions = {
            str(row.get("common_eval_contract_version", "")) for row in rows
        }
        contract_freeze_statuses = {
            str(row.get("common_eval_contract_status", "")) for row in rows
        }
        contract_statuses = {str(row.get("common_eval_status", "")) for row in rows}
        if (len(contract_ids) > 1 or len(contract_hashes) > 1 or
                len(contract_versions) > 1 or len(contract_freeze_statuses) > 1 or
                len(contract_statuses) > 1):
            raise CommonEvaluationContractError(
                "cannot aggregate rows with mixed common-evaluation provenance"
            )

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
                "common_eval_contract_id": next(iter(contract_ids)),
                "common_eval_contract_version": next(iter(contract_versions)),
                "common_eval_contract_status": next(iter(contract_freeze_statuses)),
                "common_eval_contract_sha256": next(iter(contract_hashes)),
                "common_eval_status": next(iter(contract_statuses)),
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
                "min_h_eval_mean": fmt(mean(values("min_h_eval"))),
                "min_h_eval_min": fmt(
                    min(values("min_h_eval")) if values("min_h_eval") else None
                ),
                "semantic_violation_eval_ratio_mean": fmt(
                    mean(values("semantic_violation_eval_ratio"))
                ),
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
    parser.add_argument(
        "--evaluation-contract",
        type=Path,
        required=True,
        help=(
            "Frozen method-independent Teacher-TCA/Phi/beta_max evaluator. "
            "Required so Table-I/E2 metrics cannot inherit controller margins."
        ),
    )
    args = parser.parse_args()
    output_root = args.output_root.expanduser().resolve()
    config = args.config.expanduser().resolve()
    common_evaluation = load_common_evaluation_contract(args.evaluation_contract)

    manifest_rows, metric_rows = build_rows(output_root, config, common_evaluation)
    table_rows = aggregate_rows(metric_rows)

    write_csv(output_root / "manifest.csv", manifest_rows, MANIFEST_FIELDS)
    write_csv(output_root / "teacher_run_metrics.csv", metric_rows, RUN_FIELDS)
    write_csv(output_root / "teacher_table_summary.csv", table_rows, TABLE_FIELDS)
    provenance_path = output_root / "common_evaluation_provenance.yaml"
    provenance_path.write_text(
        yaml.safe_dump(
            {
                "id": common_evaluation["id"],
                "version": common_evaluation["version"],
                "status": common_evaluation["status"],
                "sha256": common_evaluation["sha256"],
                "path": common_evaluation["path"],
                "semantic_evaluation_status": "all_rows_must_be_ok",
            },
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    print(
        {
            "output_root": str(output_root),
            "common_evaluation_contract": {
                "id": common_evaluation["id"],
                "version": common_evaluation["version"],
                "status": common_evaluation["status"],
                "sha256": common_evaluation["sha256"],
                "path": common_evaluation["path"],
            },
            "manifest_rows": len(manifest_rows),
            "metric_rows": len(metric_rows),
            "table_rows": len(table_rows),
            "manifest": str(output_root / "manifest.csv"),
            "teacher_run_metrics": str(output_root / "teacher_run_metrics.csv"),
            "teacher_table_summary": str(output_root / "teacher_table_summary.csv"),
            "common_evaluation_provenance": str(provenance_path),
        }
    )


if __name__ == "__main__":
    main()
