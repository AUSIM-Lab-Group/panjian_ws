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
        "beta_requested", "beta_pre_guard", "beta_applied", "guard_upper_bound", "h_ee", "h_see", "guard_status",
        "semantic_mode", "delta_beta", "rate_limit_active", "projection_active",
    },
    "planner_log.csv": {
        "t", "mpc_status", "first_attempt_status", "final_status", "accepted_beta_source",
        "cmd_v", "cmd_w", "slack", "slack_sum", "slack_mean", "slack_max",
        "solve_time_ms", "mpc_feasibility_guard_enabled", "candidate_feasibility_checked",
        "mpc_feasibility_guard_used",
    },
    "timing_log.csv": {"t", "mpc_secbf_ms", "total_loop_time_ms"},
    "event_log.csv": {"t", "event", "detail"},
}

OPTIONAL_FILE_FIELDS = {
    "mpc_margin_log.csv": {
        "t", "obs_id", "beta_pre_guard", "beta_applied", "accepted_beta_source",
        "first_attempt_status", "final_status", "mpc_feasibility_guard_enabled",
        "candidate_feasibility_checked", "mpc_feasibility_guard_used",
    },
    "global_seesm_log.csv": {
        "t", "replan_id", "global_seesm_enable", "obs_id", "beta_applied",
        "accepted_source", "margin_age_ms",
        "primitive_rejected", "shot_rejected", "reason", "global_replan_ms",
    },
}

TAU_FIELDS = frozenset({"tau", "T_i", "f_r", "f_v", "f_T", "tau_valid", "tau_reason"})
TAU_NUMERIC_FIELDS = ("tau", "T_i", "f_r", "f_v", "f_T")
TAU_STAGE_FIELDS = frozenset({
    "t", "obs_id", "stage", "tau_mode", "lx", "ly", "vrel_x", "vrel_y",
    "tca_raw", "tca_clipped", "tau", "tau_active", "beta", "h_eesm",
    "h_seesm", "tau_valid", "tau_reason",
})
TAU_STAGE_NUMERIC_FIELDS = (
    "stage", "lx", "ly", "vrel_x", "vrel_y", "tca_raw", "tca_clipped",
    "tau", "beta", "h_eesm", "h_seesm",
)
KNOWN_TAU_REASONS = {
    "invalid", "non_finite_input", "invalid_config", "speed_degenerate",
    "distance_degenerate", "angle_invalid", "receding_or_nonclosing",
    "velocity_gate", "time_gate", "tau_invalid", "active", "fixed_config",
    "disabled", "no_constrained_obstacle", "arithmetic_invalid", "invalid_mode",
    "stage_audit_unavailable", "teacher_receding", "teacher_tangent",
    "teacher_tca_active", "teacher_tca_clipped", "teacher_ke_tca_active",
    "teacher_ke_tca_clipped",
}
BOOL_VALUES = {"0", "1", "true", "false", "yes", "no"}
DYNAMIC_TAU_MODES = {"legacy_gate", "teacher_tca", "teacher_ke_tca"}
TEACHER_TAU_MODES = {"teacher_tca", "teacher_ke_tca"}
TEACHER_TAU_FORMULAS = {
    "teacher_tca": "tau=clip(-(l dot v_rel)/(||v_rel||^2+delta_tau),0,max_tau)",
    "teacher_ke_tca": (
        "tau=clip(Ke*clip(-(l dot v_rel)/(||v_rel||^2+delta_tau),0,max_tau),"
        "0,max_tau)"
    ),
}
RELATIVE_POSITION_CONVENTION = "l=p_robot-p_obstacle"
RELATIVE_VELOCITY_CONVENTION = "v_rel=v_robot-v_obstacle"
PREDICTION_SIGN_CONVENTION = "l(t+tau)=l+tau*v_rel"

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
    "beta_pre_guard": [("mpc_margin_log.csv", "beta_pre_guard"), ("margin_guard_log.csv", "beta_pre_guard")],
    "beta": [("mpc_margin_log.csv", "beta_applied"), ("margin_guard_log.csv", "beta_applied")],
    "delta_beta": [("margin_guard_log.csv", "delta_beta")],
    "guard_upper_bound": [("margin_guard_log.csv", "guard_upper_bound")],
    "h_EE": [("obstacle_log.csv", "h_EE"), ("margin_guard_log.csv", "h_ee")],
    "h_SEE": [("margin_guard_log.csv", "h_see")],
    "slack": [("planner_log.csv", "slack"), ("planner_log.csv", "slack_max")],
    "mpc_status": [("planner_log.csv", "mpc_status")],
    "mpc_feasibility_guard_enabled": [("planner_log.csv", "mpc_feasibility_guard_enabled")],
    "candidate_feasibility_checked": [("planner_log.csv", "candidate_feasibility_checked")],
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


def parse_meta_bool(value, field):
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"{field} must be a boolean")


def parse_finite_meta_number(mapping, key, *, minimum=None, strict_minimum=False):
    if key not in mapping:
        raise ValueError(f"dynamic_tau.{key} is missing")
    try:
        value = float(mapping[key])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"dynamic_tau.{key} must be numeric") from exc
    if not math.isfinite(value):
        raise ValueError(f"dynamic_tau.{key} must be finite")
    if minimum is not None:
        invalid = value <= minimum if strict_minimum else value < minimum
        if invalid:
            operator = ">" if strict_minimum else ">="
            raise ValueError(f"dynamic_tau.{key} must be {operator} {minimum}")
    return value


def dynamic_tau_contract(run_dir, errors=None):
    contract = {"enabled": False, "mode": "legacy_gate", "legacy_metadata": True}
    meta_path = run_dir / "meta.yaml"
    if not meta_path.exists():
        return contract
    if yaml is None:
        if errors is not None:
            errors.append("meta.yaml: PyYAML is unavailable; cannot validate dynamic_tau")
        return contract
    try:
        with meta_path.open("r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)
        if meta is None:
            meta = {}
        if not isinstance(meta, dict):
            raise ValueError("top-level YAML value must be a mapping")
        if "dynamic_tau" not in meta:
            return contract
        dynamic_tau = meta["dynamic_tau"]
        if not isinstance(dynamic_tau, dict):
            raise ValueError("dynamic_tau must be a mapping")
        if "enabled" not in dynamic_tau:
            raise ValueError("dynamic_tau.enabled is missing")
        enabled = parse_meta_bool(dynamic_tau["enabled"], "dynamic_tau.enabled")
        mode_present = "mode" in dynamic_tau
        mode = str(dynamic_tau.get("mode", "legacy_gate")).strip()
        if mode not in DYNAMIC_TAU_MODES:
            raise ValueError(f"dynamic_tau.mode is unknown: {mode!r}")
        contract = {
            "enabled": enabled,
            "mode": mode,
            "legacy_metadata": not mode_present,
        }

        # Mode-less archived runs are Legacy-v1 and stay readable.  Any
        # explicitly mode-tagged Teacher run is fail-closed: enough metadata
        # must be present to reproduce its formula and relative-vector signs.
        if mode in TEACHER_TAU_MODES:
            delta_tau = parse_finite_meta_number(
                dynamic_tau, "delta_tau", minimum=0.0, strict_minimum=True
            )
            max_tau = parse_finite_meta_number(
                dynamic_tau, "max_tau", minimum=0.0, strict_minimum=True
            )
            ke = None
            if mode == "teacher_ke_tca":
                ke = parse_finite_meta_number(
                    dynamic_tau, "Ke", minimum=0.0, strict_minimum=True
                )
            contract.update({
                "delta_tau": delta_tau,
                "max_tau": max_tau,
                "Ke": ke,
            })
            expected = {
                "formula": TEACHER_TAU_FORMULAS[mode],
                "relative_position_convention": RELATIVE_POSITION_CONVENTION,
                "relative_velocity_convention": RELATIVE_VELOCITY_CONVENTION,
                "prediction_sign": PREDICTION_SIGN_CONVENTION,
            }
            for key, expected_value in expected.items():
                if dynamic_tau.get(key) != expected_value:
                    raise ValueError(
                        f"dynamic_tau.{key} must be {expected_value!r} for {mode}"
                    )
            expected_policy = "symbolic_stagewise" if enabled else "disabled"
            if dynamic_tau.get("mpc_stage_policy") != expected_policy:
                raise ValueError(
                    "Teacher dynamic tau production must use mpc_stage_policy="
                    f"{expected_policy!r}"
                )
        elif mode == "legacy_gate" and "mpc_stage_policy" in dynamic_tau:
            expected_policy = "numeric_frozen_per_stage" if enabled else "disabled"
            if dynamic_tau["mpc_stage_policy"] != expected_policy:
                raise ValueError(
                    "numeric frozen tau is valid only for legacy_gate metadata"
                )
        return contract
    except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        if errors is not None:
            errors.append(f"meta.yaml: invalid dynamic_tau metadata: {exc}")
        return contract


def dynamic_tau_enabled(run_dir, errors=None):
    return dynamic_tau_contract(run_dir, errors)["enabled"]


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

        bool_fields = {"tau_valid": str(row.get("tau_valid", "")).strip().lower()}
        for optional_field in ("tau_computed", "tau_active"):
            if optional_field in row:
                bool_fields[optional_field] = str(
                    row.get(optional_field, "")
                ).strip().lower()
        for field, value in bool_fields.items():
            if value not in BOOL_VALUES:
                errors.append(
                    f"{file_name}:{row_index}: invalid {field} {value!r}"
                )

        if "tau_computed" in bool_fields and "tau_active" in bool_fields:
            computed = bool_fields["tau_computed"] in {"1", "true", "yes"}
            active = bool_fields["tau_active"] in {"1", "true", "yes"}
            if active and not computed:
                errors.append(
                    f"{file_name}:{row_index}: tau_active requires tau_computed"
                )

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


def validate_tau_stage_file(path, dynamic_contract, errors, required=False):
    file_name = "tau_stage_log.csv"
    if not path.exists():
        if required:
            errors.append(f"missing file: {file_name}")
        return set()
    header = read_header(path)
    missing = sorted(TAU_STAGE_FIELDS - header)
    if missing:
        errors.append(f"{file_name}: missing fields {missing}")
        return header

    expected_mode = dynamic_contract["mode"]
    rows = read_rows(path)
    if required and not rows:
        errors.append(
            f"{file_name}: no data rows for enabled Teacher dynamic tau"
        )
        return header

    def parse_bool(row, field, row_index):
        value = str(row.get(field, "")).strip().lower()
        if value not in BOOL_VALUES:
            errors.append(
                f"{file_name}:{row_index}: invalid {field} {value!r}"
            )
            return None
        return value in {"1", "true", "yes"}

    def check_close(row_index, field, actual, expected):
        # C++ CSV streams use their default significant-digit precision.  This
        # tolerance accepts decimal serialization while still catching a
        # different formula, sign convention, clipping bound, or Ke scaling.
        if not math.isclose(actual, expected, rel_tol=5.0e-5, abs_tol=5.0e-6):
            errors.append(
                f"{file_name}:{row_index}: {field}={actual:.12g} does not "
                f"match recomputed {expected:.12g}"
            )

    for row_index, row in enumerate(rows, start=2):
        if row.get("tau_mode") != expected_mode:
            errors.append(
                f"{file_name}:{row_index}: tau_mode {row.get('tau_mode')!r} "
                f"does not match meta mode {expected_mode!r}"
            )
        numeric_values = {}
        for field in TAU_STAGE_NUMERIC_FIELDS:
            try:
                value = float(row[field])
            except (KeyError, TypeError, ValueError):
                errors.append(f"{file_name}:{row_index}: invalid numeric field {field}")
                continue
            if not math.isfinite(value):
                errors.append(f"{file_name}:{row_index}: non-finite field {field}")
                continue
            numeric_values[field] = value
        stage = numeric_values.get("stage")
        if stage is not None and (stage < 0.0 or not stage.is_integer()):
            errors.append(f"{file_name}:{row_index}: stage must be a nonnegative integer")
        tau = numeric_values.get("tau")
        if tau is not None and tau < 0.0:
            errors.append(f"{file_name}:{row_index}: tau must be nonnegative")

        tau_valid = parse_bool(row, "tau_valid", row_index)
        tau_active = parse_bool(row, "tau_active", row_index)
        reason = str(row.get("tau_reason", "")).strip()
        if reason not in KNOWN_TAU_REASONS:
            errors.append(
                f"{file_name}:{row_index}: unknown tau_reason {reason!r}"
            )

        teacher_inputs = (
            "lx", "ly", "vrel_x", "vrel_y", "tca_raw", "tca_clipped", "tau"
        )
        if (
            expected_mode in TEACHER_TAU_MODES
            and all(field in numeric_values for field in teacher_inputs)
            and all(
                key in dynamic_contract for key in ("delta_tau", "max_tau")
            )
        ):
            lx = numeric_values["lx"]
            ly = numeric_values["ly"]
            vrel_x = numeric_values["vrel_x"]
            vrel_y = numeric_values["vrel_y"]
            relative_dot = lx * vrel_x + ly * vrel_y
            speed_squared = vrel_x * vrel_x + vrel_y * vrel_y
            expected_raw = -relative_dot / (
                speed_squared + dynamic_contract["delta_tau"]
            )
            expected_clipped = min(
                max(expected_raw, 0.0), dynamic_contract["max_tau"]
            )
            if expected_mode == "teacher_ke_tca":
                expected_tau_unclipped = (
                    dynamic_contract["Ke"] * expected_clipped
                )
                expected_tau = min(
                    expected_tau_unclipped, dynamic_contract["max_tau"]
                )
            else:
                expected_tau_unclipped = expected_clipped
                expected_tau = expected_clipped

            check_close(
                row_index, "tca_raw", numeric_values["tca_raw"], expected_raw
            )
            check_close(
                row_index, "tca_clipped", numeric_values["tca_clipped"],
                expected_clipped,
            )
            check_close(row_index, "tau", numeric_values["tau"], expected_tau)

            expected_active = expected_tau > 0.0
            if expected_clipped <= 0.0:
                expected_reason = (
                    "teacher_receding"
                    if relative_dot > 0.0
                    else "teacher_tangent"
                )
            elif expected_mode == "teacher_ke_tca":
                expected_reason = (
                    "teacher_ke_tca_clipped"
                    if expected_tau_unclipped > dynamic_contract["max_tau"]
                    else "teacher_ke_tca_active"
                )
            else:
                expected_reason = (
                    "teacher_tca_clipped"
                    if expected_raw > dynamic_contract["max_tau"]
                    else "teacher_tca_active"
                )

            # tau_valid is computational validity, whereas tau_active says
            # whether the valid formula produced a strictly positive horizon.
            # Receding/tangent rows therefore require (valid=true, active=false).
            if tau_valid is not None and not tau_valid:
                errors.append(
                    f"{file_name}:{row_index}: tau_valid must be true for a "
                    "finite Teacher formula evaluation"
                )
            if tau_active is not None and tau_active != expected_active:
                errors.append(
                    f"{file_name}:{row_index}: tau_active={tau_active} does not "
                    f"match recomputed active={expected_active}"
                )
            if reason != expected_reason:
                errors.append(
                    f"{file_name}:{row_index}: tau_reason {reason!r} does not "
                    f"match recomputed {expected_reason!r}"
                )
        if all(field in numeric_values for field in ("h_eesm", "h_seesm", "beta")):
            residual = abs(
                numeric_values["h_seesm"]
                - (numeric_values["h_eesm"] - numeric_values["beta"])
            )
            if residual > 1.0e-6:
                errors.append(
                    f"{file_name}:{row_index}: h_seesm != h_eesm - beta"
                )
    return header


def validate_mpc_margin_rows(path, errors):
    rows = read_rows(path)
    for row_index, row in enumerate(rows, start=2):
        try:
            beta_pre = float(row["beta_pre_guard"])
            beta_applied = float(row["beta_applied"])
        except (KeyError, TypeError, ValueError):
            errors.append(f"mpc_margin_log.csv:{row_index}: invalid beta value")
            continue
        if not math.isfinite(beta_pre) or not math.isfinite(beta_applied):
            errors.append(f"mpc_margin_log.csv:{row_index}: non-finite beta value")
            continue

        enabled = str(row.get("mpc_feasibility_guard_enabled", "")).strip().lower()
        checked = str(row.get("candidate_feasibility_checked", "")).strip().lower()
        used = str(row.get("mpc_feasibility_guard_used", "")).strip().lower()
        for field, value in (
            ("mpc_feasibility_guard_enabled", enabled),
            ("candidate_feasibility_checked", checked),
            ("mpc_feasibility_guard_used", used),
        ):
            if value not in BOOL_VALUES:
                errors.append(f"mpc_margin_log.csv:{row_index}: invalid {field} {value!r}")

        first_status = row.get("first_attempt_status", "")
        final_status = row.get("final_status", "")
        source = row.get("accepted_beta_source", "")
        checked_true = checked in {"1", "true", "yes"}
        enabled_true = enabled in {"1", "true", "yes"}
        used_true = used in {"1", "true", "yes"}
        if checked_true != (first_status in {"success", "infeasible"}):
            errors.append(
                f"mpc_margin_log.csv:{row_index}: candidate check/status mismatch"
            )
        if used_true and (not enabled_true or first_status != "infeasible"):
            errors.append(
                f"mpc_margin_log.csv:{row_index}: Guard retry lacks enabled infeasible candidate"
            )
        if source == "candidate":
            if first_status != "success" or final_status != "success":
                errors.append(
                    f"mpc_margin_log.csv:{row_index}: candidate source lacks successful MPC status"
                )
            if abs(beta_pre - beta_applied) > 1e-8:
                errors.append(
                    f"mpc_margin_log.csv:{row_index}: candidate source changed accepted beta"
                )
        if source in {"zero", "no_cbf"} and abs(beta_applied) > 1e-8:
            errors.append(
                f"mpc_margin_log.csv:{row_index}: zero/no_cbf source has nonzero beta"
            )


def main():
    parser = argparse.ArgumentParser(description="Check Phase 5 CSV field contract")
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()

    headers = {}
    errors = []
    dynamic_contract = dynamic_tau_contract(args.run_dir, errors)
    dynamic_enabled = dynamic_contract["enabled"]
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
        elif file_name == "mpc_margin_log.csv":
            validate_mpc_margin_rows(path, errors)
        elif file_name == "global_seesm_log.csv":
            teacher_barriers = {"h_phys", "h_eesm", "h_seesm"}
            legacy_barriers = {"h_ee", "h_see"}
            if dynamic_contract["mode"] in TEACHER_TAU_MODES:
                barrier_missing = sorted(teacher_barriers - header)
                if barrier_missing:
                    errors.append(
                        f"{file_name}: missing Teacher barrier fields {barrier_missing}"
                    )
            elif not (teacher_barriers.issubset(header) or legacy_barriers.issubset(header)):
                errors.append(
                    f"{file_name}: missing complete legacy or Teacher barrier field group"
                )
            validate_tau_file(file_name, path, errors, required=dynamic_enabled)

    teacher_mode = dynamic_contract["mode"] in TEACHER_TAU_MODES
    tau_stage_path = args.run_dir / "tau_stage_log.csv"
    tau_stage_header = validate_tau_stage_file(
        tau_stage_path,
        dynamic_contract,
        errors,
        required=dynamic_enabled and teacher_mode,
    )
    if tau_stage_header:
        headers["tau_stage_log.csv"] = tau_stage_header

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
