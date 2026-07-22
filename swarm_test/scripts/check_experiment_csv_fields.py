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
TEACHER_TAU_STAGE_EXTRA_FIELDS = frozenset({"tau_computed", "R_base"})
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

ALGORITHM_VERSION = "teacher_v1"
FORMULA_VERSION = "teacher_v1_formula_001"
LOG_SCHEMA_VERSION = "teacher_v1_log_schema_001"
TEACHER_MANUSCRIPT_SHA256 = (
    "c4482f2acda626162a830859db1745d12ee3afaf0c8820b47a3dd94b641ebdf5"
)
LOG_PROFILE_BY_BASELINE = {
    "B1_ACBF_fixed": "legacy_b1_v1",
    "Standard_MPC_CBF": "teacher_distance_mpc_v1",
}
TEACHER_REQUIRED_LOGS = [
    "robot_log.csv", "obstacle_log.csv", "margin_guard_log.csv",
    "planner_log.csv", "timing_log.csv", "mpc_margin_log.csv", "event_log.csv",
    "tau_stage_log.csv", "global_seesm_log.csv", "data_processor_summary.csv",
    "data_processor_distance.csv",
]
B1_REQUIRED_LOGS = [
    "robot_log.csv", "obstacle_log.csv", "event_log.csv",
    "data_processor_summary.csv", "data_processor_distance.csv",
]
TEACHER_CANONICAL_FILE_FIELDS = {
    "obstacle_log.csv": {
        "tau", "tau_mode", "h_phys", "h_eesm", "tau_computed",
        "tau_active", "tau_reason",
    },
    "margin_guard_log.csv": {
        "h_phys", "h_eesm", "h_seesm", "tau_computed", "tau_active",
        "tau_valid", "tca_raw", "tca_clipped",
    },
    "planner_log.csv": {
        "tau_computed", "tau_active", "tau_valid", "tca_raw", "tca_clipped",
    },
    "global_seesm_log.csv": {
        "h_phys", "h_eesm", "h_seesm", "tau_computed", "tau_active",
        "tau_valid", "tca_raw", "tca_clipped",
    },
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


def expected_log_profile(baseline_id):
    return LOG_PROFILE_BY_BASELINE.get(baseline_id, "teacher_seesm_v1")


def validate_teacher_metadata(run_dir, errors):
    meta_path = run_dir / "meta.yaml"
    canonical_path = run_dir / "run_meta.yaml"
    if not meta_path.exists():
        errors.append("missing file: meta.yaml")
    if not canonical_path.exists():
        errors.append("missing file: run_meta.yaml")
    if not meta_path.exists() or not canonical_path.exists():
        return {}, ""
    if yaml is None:
        errors.append("PyYAML is unavailable; cannot validate Teacher metadata")
        return {}, ""
    try:
        if meta_path.read_bytes() != canonical_path.read_bytes():
            errors.append("meta.yaml is not byte-identical to run_meta.yaml")
        with canonical_path.open("r", encoding="utf-8") as stream:
            meta = yaml.safe_load(stream)
        if not isinstance(meta, dict):
            raise ValueError("top-level YAML value must be a mapping")
    except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        errors.append(f"invalid Teacher metadata: {exc}")
        return {}, ""

    for key, expected in (
        ("algorithm_version", ALGORITHM_VERSION),
        ("formula_version", FORMULA_VERSION),
        ("log_schema_version", LOG_SCHEMA_VERSION),
        ("teacher_manuscript_sha256", TEACHER_MANUSCRIPT_SHA256),
    ):
        if meta.get(key) != expected:
            errors.append(f"metadata {key} must be {expected!r}")
    if not str(meta.get("protocol_id", "")).strip():
        errors.append("metadata protocol_id must be nonempty")
    if meta.get("run_state") not in {"running", "complete", "invalid"}:
        errors.append("metadata run_state is invalid")

    baseline_id = str(meta.get("baseline_id", ""))
    profile = expected_log_profile(baseline_id)
    if meta.get("log_profile") != profile:
        errors.append(
            f"metadata log_profile {meta.get('log_profile')!r} does not "
            f"match baseline profile {profile!r}"
        )
    expected_logs = B1_REQUIRED_LOGS if profile == "legacy_b1_v1" else TEACHER_REQUIRED_LOGS
    if meta.get("required_logs") != expected_logs:
        errors.append("metadata required_logs does not match code-controlled profile")
    if not isinstance(meta.get("provenance"), dict) or not meta.get("provenance"):
        errors.append("metadata provenance must be a nonempty mapping")
    artifacts = meta.get("artifact_hashes")
    if not isinstance(artifacts, dict) or not artifacts.get("obstacles_param.yaml"):
        errors.append("metadata artifact_hashes.obstacles_param.yaml is required")

    def require_bool(field, expected):
        try:
            actual = parse_meta_bool(meta.get(field), field)
        except ValueError as exc:
            errors.append(f"metadata {exc}")
            return
        if actual is not expected:
            errors.append(
                f"metadata {field} must be {str(expected).lower()} for {baseline_id}"
            )

    if baseline_id == "B1_ACBF_fixed":
        for field, expected in {
            "semantic_mode": "not_applicable",
            "cbf_metric": "legacy_acbf",
            "fixed_beta": 0.30,
            "front_adsm": "true",
        }.items():
            if meta.get(field) != expected:
                errors.append(f"metadata {field} must be {expected!r} for B1")
        for field in (
            "guard_enable", "enable_rate_limit", "enable_available_projection",
            "enable_guard_fallback", "mpc_feasibility_guard_enabled",
            "global_seesm_enable", "side_preference_enabled",
        ):
            require_bool(field, False)
        if meta.get("mu_weights") is not None or meta.get("guard_eta") is not None:
            errors.append("metadata B1 semantic Guard fields must be not applicable")
    elif baseline_id == "Standard_MPC_CBF":
        if meta.get("semantic_mode") != "fixed" or meta.get("cbf_metric") != "distance":
            errors.append("metadata Standard_MPC_CBF must be fixed/distance")
        for field in (
            "guard_enable", "enable_rate_limit", "enable_available_projection",
            "enable_guard_fallback", "mpc_feasibility_guard_enabled", "front_adsm",
            "global_seesm_enable", "side_preference_enabled",
        ):
            require_bool(field, False)
    return meta, profile


def validate_b1_profile(run_dir, errors):
    headers = {}
    for file_name in B1_REQUIRED_LOGS:
        path = run_dir / file_name
        if not path.exists() or path.stat().st_size <= 1:
            errors.append(f"missing or empty B1 log: {file_name}")
            continue
        if file_name in FILE_FIELDS:
            header = read_header(path)
            headers[file_name] = header
            missing = sorted(FILE_FIELDS[file_name] - header)
            if missing:
                errors.append(f"{file_name}: missing fields {missing}")
            if not read_rows(path):
                errors.append(f"{file_name}: no data rows")
    robot_rows = read_rows(run_dir / "robot_log.csv") if (run_dir / "robot_log.csv").exists() else []
    obstacle_rows = read_rows(run_dir / "obstacle_log.csv") if (run_dir / "obstacle_log.csv").exists() else []
    if len(robot_rows) < 2:
        errors.append("robot_log.csv: B1 requires at least two samples")
    if len(obstacle_rows) < 2:
        errors.append("obstacle_log.csv: B1 requires at least two samples")
    meaningful_rows = 0
    for row in obstacle_rows:
        try:
            values = [
                float(row.get(field, "0"))
                for field in ("x", "y", "radius", "d_i", "rel_v", "h_EE")
            ]
        except (TypeError, ValueError):
            continue
        if all(math.isfinite(value) for value in values) and any(
            abs(value) > 1.0e-12 for value in values
        ):
            meaningful_rows += 1
    if meaningful_rows == 0:
        errors.append("obstacle_log.csv: B1 has only all-zero startup rows")
    event_rows = read_rows(run_dir / "event_log.csv") if (run_dir / "event_log.csv").exists() else []
    events = {row.get("event", "") for row in event_rows}
    if not {"start", "stop"}.issubset(events):
        errors.append("event_log.csv: B1 requires start and stop events")
    return headers


def validate_event_log(path, errors):
    rows = read_rows(path)
    events = {str(row.get("event", "")).strip() for row in rows}
    if not {"start", "stop"}.issubset(events):
        errors.append("event_log.csv: requires start and stop events")


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
                "h_eesm": "||l+tau*v_rel||-R_obs-R_robot",
                "h_seesm": "h_eesm-beta",
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


def validate_disabled_tau_rows(file_name, path, expected_mode, errors):
    header = read_header(path)
    if "tau" not in header:
        return
    for row_index, row in enumerate(read_rows(path), start=2):
        try:
            tau = float(row.get("tau", ""))
        except (TypeError, ValueError):
            errors.append(f"{file_name}:{row_index}: invalid disabled tau")
            continue
        if not math.isfinite(tau) or abs(tau) > 1.0e-12:
            errors.append(
                f"{file_name}:{row_index}: disabled dynamic tau must equal zero"
            )
        if "tau_mode" in header and row.get("tau_mode") != expected_mode:
            errors.append(
                f"{file_name}:{row_index}: disabled tau_mode does not match metadata"
            )
        active = str(row.get("tau_active", "")).strip().lower()
        if "tau_active" in header and active not in {"0", "false", "no"}:
            errors.append(
                f"{file_name}:{row_index}: disabled tau_active must be false"
            )
        reason = str(row.get("tau_reason", "")).strip()
        computed = str(row.get("tau_computed", "")).strip().lower()
        if "tau_reason" in header and reason != "disabled":
            errors.append(
                f"{file_name}:{row_index}: disabled tau_reason must equal 'disabled'"
            )
        if "tau_computed" in header and computed not in {"1", "true", "yes"}:
            errors.append(
                f"{file_name}:{row_index}: disabled tau must be marked computed"
            )
        if "tau_valid" in header and "tau_computed" in header:
            valid_alias = str(row.get("tau_valid", "")).strip().lower()
            if valid_alias != computed:
                errors.append(
                    f"{file_name}:{row_index}: tau_valid alias must match tau_computed"
                )


def validate_tau_stage_file(path, dynamic_contract, errors, required=False,
                            canonical_teacher=False):
    file_name = "tau_stage_log.csv"
    if not path.exists():
        if required:
            errors.append(f"missing file: {file_name}")
        return set()
    header = read_header(path)
    required_fields = TAU_STAGE_FIELDS
    if canonical_teacher:
        required_fields = required_fields | TEACHER_TAU_STAGE_EXTRA_FIELDS
    missing = sorted(required_fields - header)
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
        numeric_fields = TAU_STAGE_NUMERIC_FIELDS + (
            ("R_base",) if canonical_teacher else ()
        )
        for field in numeric_fields:
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
        tau_computed = (
            parse_bool(row, "tau_computed", row_index)
            if "tau_computed" in header
            else tau_valid
        )
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
            if tau_computed is not None and not tau_computed:
                errors.append(
                    f"{file_name}:{row_index}: tau_computed must be true for a "
                    "finite Teacher formula evaluation"
                )
            if tau_valid is not None and not tau_valid:
                errors.append(
                    f"{file_name}:{row_index}: deprecated tau_valid alias must "
                    "match tau_computed=true"
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
        if all(
            field in numeric_values
            for field in ("lx", "ly", "vrel_x", "vrel_y", "tau", "R_base", "h_eesm")
        ):
            expected_h_eesm = math.hypot(
                numeric_values["lx"] + numeric_values["tau"] * numeric_values["vrel_x"],
                numeric_values["ly"] + numeric_values["tau"] * numeric_values["vrel_y"],
            ) - numeric_values["R_base"]
            check_close(
                row_index, "h_eesm", numeric_values["h_eesm"], expected_h_eesm
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
    parser.add_argument(
        "--require-teacher-meta", action="store_true",
        help="require canonical Teacher-v1 metadata and log profile",
    )
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()

    headers = {}
    errors = []
    profile = ""
    teacher_meta = {}
    if args.require_teacher_meta:
        teacher_meta, profile = validate_teacher_metadata(args.run_dir, errors)
    dynamic_contract = dynamic_tau_contract(args.run_dir, errors)
    dynamic_enabled = dynamic_contract["enabled"]
    if profile == "legacy_b1_v1":
        if dynamic_enabled:
            errors.append("legacy_b1_v1 must have dynamic_tau.enabled=false")
        validate_b1_profile(args.run_dir, errors)
        if errors:
            for error in errors:
                print(f"ERROR: {error}", file=sys.stderr)
            return 1
        print(f"Legacy B1 CSV profile checks passed: {args.run_dir}")
        return 0

    if args.require_teacher_meta:
        for file_name in TEACHER_REQUIRED_LOGS:
            path = args.run_dir / file_name
            if not path.exists() or path.stat().st_size <= 1:
                errors.append(f"missing or empty required Teacher log: {file_name}")

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
        if args.require_teacher_meta:
            canonical_missing = sorted(
                TEACHER_CANONICAL_FILE_FIELDS.get(file_name, set()) - header
            )
            if canonical_missing:
                errors.append(
                    f"{file_name}: missing canonical Teacher fields {canonical_missing}"
                )
        if not missing and file_name in {"margin_guard_log.csv", "planner_log.csv"}:
            validate_tau_file(file_name, path, errors, required=dynamic_enabled)
        if args.require_teacher_meta and not missing and file_name == "event_log.csv":
            validate_event_log(path, errors)
        if args.require_teacher_meta and not dynamic_enabled:
            validate_disabled_tau_rows(
                file_name, path, dynamic_contract["mode"], errors
            )

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
            if args.require_teacher_meta:
                canonical_missing = sorted(
                    TEACHER_CANONICAL_FILE_FIELDS["global_seesm_log.csv"] - header
                )
                if canonical_missing:
                    errors.append(
                        f"{file_name}: missing canonical Teacher fields {canonical_missing}"
                    )
                if not dynamic_enabled:
                    validate_disabled_tau_rows(
                        file_name, path, dynamic_contract["mode"], errors
                    )
                try:
                    global_enabled = parse_meta_bool(
                        teacher_meta.get("global_seesm_enable", False),
                        "global_seesm_enable",
                    )
                except ValueError as exc:
                    errors.append(f"meta.yaml: {exc}")
                    global_enabled = False
                if global_enabled and not read_rows(path):
                    errors.append(
                        "global_seesm_log.csv: global_seesm_enable=true requires data rows"
                    )

    teacher_mode = dynamic_contract["mode"] in TEACHER_TAU_MODES
    tau_stage_path = args.run_dir / "tau_stage_log.csv"
    tau_stage_header = validate_tau_stage_file(
        tau_stage_path,
        dynamic_contract,
        errors,
        required=dynamic_enabled and teacher_mode,
        canonical_teacher=args.require_teacher_meta,
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
