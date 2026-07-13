#!/usr/bin/env python3
"""Check that a run directory contains the Phase 5 paper CSV fields."""

import argparse
import csv
import math
from pathlib import Path
import sys

try:
    import yaml
except ImportError:
    yaml = None


FILE_FIELDS = {
    "robot_log.csv": {"t", "x", "y", "yaw", "v", "w", "cmd_v", "cmd_w"},
    "obstacle_log.csv": {"t", "id", "class", "x", "y", "radius", "vx", "vy", "d_i", "rel_v", "TTC", "h_EE"},
    "margin_guard_log.csv": {
        "time", "obs_id", "class", "d_i", "rel_v_norm", "ttc", "mu", "beta_bar",
        "beta_requested", "beta_applied", "guard_upper_bound", "h_ee", "h_see", "guard_status",
        "semantic_mode", "delta_beta", "rate_limit_active", "projection_active",
    },
    "planner_log.csv": {
        "t", "mpc_status", "first_attempt_status", "final_status", "accepted_beta_source",
        "cmd_v", "cmd_w", "slack", "slack_sum", "slack_mean", "slack_max",
        "solve_time_ms", "mpc_feasibility_guard_used",
    },
    "timing_log.csv": {"t", "mpc_secbf_ms", "total_loop_time_ms"},
    "event_log.csv": {"t", "event", "detail"},
}

OPTIONAL_FILE_FIELDS = {
    "global_seesm_log.csv": {
        "t", "replan_id", "global_seesm_enable", "obs_id", "beta_applied",
        "accepted_source", "margin_age_ms", "h_ee", "h_see",
        "primitive_rejected", "shot_rejected", "reason", "global_replan_ms",
    },
}

TAU_FIELDS = frozenset({"tau", "T_i", "f_r", "f_v", "f_T", "tau_valid", "tau_reason"})
TAU_NUMERIC_FIELDS = ("tau", "T_i", "f_r", "f_v", "f_T")
KNOWN_TAU_REASONS = {
    "invalid", "non_finite_input", "invalid_config", "speed_degenerate",
    "distance_degenerate", "angle_invalid", "receding_or_nonclosing",
    "velocity_gate", "time_gate", "tau_invalid", "active", "fixed_config",
    "disabled", "no_constrained_obstacle",
}
BOOL_VALUES = {"0", "1", "true", "false", "yes", "no"}

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
    "delta_beta": [("margin_guard_log.csv", "delta_beta")],
    "guard_upper_bound": [("margin_guard_log.csv", "guard_upper_bound")],
    "h_EE": [("obstacle_log.csv", "h_EE"), ("margin_guard_log.csv", "h_ee")],
    "h_SEE": [("margin_guard_log.csv", "h_see")],
    "slack": [("planner_log.csv", "slack"), ("planner_log.csv", "slack_max")],
    "mpc_status": [("planner_log.csv", "mpc_status")],
    "mpc_feasibility_guard_used": [("planner_log.csv", "mpc_feasibility_guard_used")],
}


def read_header(path):
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        try:
            return set(next(reader))
        except StopIteration:
            return set()


def read_rows(path):
    with path.open("r", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def dynamic_tau_enabled(run_dir, errors=None):
    meta_path = run_dir / "meta.yaml"
    if not meta_path.exists():
        return False
    if yaml is None:
        if errors is not None:
            errors.append("meta.yaml: PyYAML is unavailable; cannot validate dynamic_tau")
        return False
    try:
        with meta_path.open("r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)
        if meta is None:
            meta = {}
        if not isinstance(meta, dict):
            raise ValueError("top-level YAML value must be a mapping")
        if "dynamic_tau" not in meta:
            return False
        dynamic_tau = meta["dynamic_tau"]
        if not isinstance(dynamic_tau, dict):
            raise ValueError("dynamic_tau must be a mapping")
        if "enabled" not in dynamic_tau:
            raise ValueError("dynamic_tau.enabled is missing")
        value = dynamic_tau["enabled"]
        if isinstance(value, bool):
            return value
        normalized = str(value).strip().lower()
        if normalized in {"1", "true", "yes", "on"}:
            return True
        if normalized in {"0", "false", "no", "off"}:
            return False
        raise ValueError("dynamic_tau.enabled must be a boolean")
    except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        if errors is not None:
            errors.append(f"meta.yaml: invalid dynamic_tau metadata: {exc}")
        return False


def validate_tau_rows(file_name, path, errors):
    rows = read_rows(path)
    for row_index, row in enumerate(rows, start=2):
        for field in TAU_NUMERIC_FIELDS:
            try:
                value = float(row[field])
            except (KeyError, TypeError, ValueError):
                errors.append(f"{file_name}:{row_index}: invalid numeric field {field}")
                continue
            if not math.isfinite(value):
                errors.append(f"{file_name}:{row_index}: non-finite field {field}")

        tau_valid = str(row.get("tau_valid", "")).strip().lower()
        if tau_valid not in BOOL_VALUES:
            errors.append(f"{file_name}:{row_index}: invalid tau_valid {tau_valid!r}")

        tau_reason = str(row.get("tau_reason", "")).strip()
        if tau_reason not in KNOWN_TAU_REASONS:
            errors.append(f"{file_name}:{row_index}: unknown tau_reason {tau_reason!r}")


def validate_tau_file(file_name, path, errors, required=False):
    header = read_header(path)
    present = header & TAU_FIELDS
    if not present:
        if required:
            errors.append(f"{file_name}: missing complete tau field group")
        return header
    missing = sorted(TAU_FIELDS - header)
    if missing:
        errors.append(f"{file_name}: incomplete tau field group, missing {missing}")
        return header
    validate_tau_rows(file_name, path, errors)
    return header


def main():
    parser = argparse.ArgumentParser(description="Check Phase 5 CSV field contract")
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()

    headers = {}
    errors = []
    dynamic_enabled = dynamic_tau_enabled(args.run_dir, errors)
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
        elif file_name in {"margin_guard_log.csv", "planner_log.csv"}:
            validate_tau_file(file_name, path, errors, required=dynamic_enabled)

    for file_name, required in OPTIONAL_FILE_FIELDS.items():
        path = args.run_dir / file_name
        if not path.exists():
            continue
        header = read_header(path)
        headers[file_name] = header
        missing = sorted(required - header)
        if missing:
            errors.append(f"{file_name}: missing fields {missing}")
        else:
            validate_tau_file(file_name, path, errors, required=dynamic_enabled)

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
