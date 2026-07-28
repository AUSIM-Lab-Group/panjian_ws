#!/usr/bin/env python3
"""Run MPC-SECBF numerical simulation experiment matrix."""

import argparse
import copy
import csv
import datetime as dt
import hashlib
import math
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

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from reference_path_waypoints import generate_waypoints, initial_yaw


REQUIRED_TRIAL_LOGS = (
    "robot_log.csv",
    "obstacle_log.csv",
    "margin_guard_log.csv",
    "planner_log.csv",
    "timing_log.csv",
    "mpc_margin_log.csv",
    "event_log.csv",
    "tau_stage_log.csv",
    "guard_attempt_log.csv",
    "safety_recurrence_log.csv",
    "global_seesm_log.csv",
    "data_processor_summary.csv",
    "data_processor_distance.csv",
)
B1_REQUIRED_TRIAL_LOGS = (
    "robot_log.csv",
    "obstacle_log.csv",
    "event_log.csv",
    "data_processor_summary.csv",
    "data_processor_distance.csv",
)
LOG_PROFILES = {
    "teacher_seesm_v1": {
        "required_logs": REQUIRED_TRIAL_LOGS,
        "required_data_rows": (
            "robot_log.csv", "obstacle_log.csv", "margin_guard_log.csv",
            "planner_log.csv", "timing_log.csv", "mpc_margin_log.csv",
            "event_log.csv", "tau_stage_log.csv", "safety_recurrence_log.csv",
            "data_processor_summary.csv",
            "data_processor_distance.csv",
        ),
        "teacher_formula_applicable": True,
    },
    "teacher_distance_mpc_v1": {
        "required_logs": REQUIRED_TRIAL_LOGS,
        "required_data_rows": (
            "robot_log.csv", "obstacle_log.csv", "planner_log.csv",
            "timing_log.csv", "mpc_margin_log.csv",
            "event_log.csv", "data_processor_summary.csv",
            "data_processor_distance.csv",
        ),
        "teacher_formula_applicable": False,
    },
    "legacy_b1_v1": {
        "required_logs": B1_REQUIRED_TRIAL_LOGS,
        "required_data_rows": B1_REQUIRED_TRIAL_LOGS,
        "teacher_formula_applicable": False,
    },
}

EXPECTED_ROS_PACKAGE_PATHS = {
    "swarm_test": "swarm_test",
    "mpc_secbf": "planner/mpc_secbf",
    "semantic_guard": "planner/semantic_guard",
    "mpc_dcbf": "planner/mpc_dcbf",
    "traj_planner": "planner/vomp_planner/traj_planner",
}

# These are the algorithm-bearing executables selected by the two planner
# launch families.  Every real trial resolves and hashes its own subset, so a
# source-clean worktree cannot silently execute a stale Legacy overlay.
RUNTIME_NODE_SPECS = {
    "teacher_mpc": (
        "mpc_secbf", "mpc_secbf_node",
        ("planner/mpc_secbf", "planner/semantic_guard/include"),
    ),
    "teacher_guard_ground_truth": (
        "semantic_guard", "beta_ground_truth_node",
        ("planner/semantic_guard",),
    ),
    "global_planner": (
        "traj_planner", "globalFsm_by_adsm",
        ("planner/vomp_planner/traj_planner", "planner/semantic_guard/include"),
    ),
    "obstacle_manager": (
        "traj_planner", "obs_Manager_node",
        ("planner/vomp_planner/traj_planner", "planner/semantic_guard/include"),
    ),
    "phase5_logger": (
        "swarm_test", "phase5_csv_logger.py", ("swarm_test/scripts",),
    ),
    "legacy_acbf_mpc": (
        "mpc_dcbf", "mpc_node_c", ("planner/mpc_dcbf",),
    ),
}

ALGORITHM_VERSION = "teacher_v1"
FORMULA_VERSION = "teacher_v1_formula_001"
LOG_SCHEMA_VERSION = "teacher_v1_log_schema_002"
SEMANTIC_MARGIN_CONTRACT_VERSION = "teacher_v1_f05_f07_provisional_001"
TYPED_CYCLE_CONTRACT_VERSION = "teacher_v1_typed_cycle_001"
DEFAULT_PROTOCOL_ID = "teacher_v1_protocol_001"
TEACHER_WORKSPACE = SCRIPT_DIR.parents[2]
TEACHER_SEESM_REPO = TEACHER_WORKSPACE / "seesm_social_navigation"
TEACHER_MANUSCRIPT_PATH = (
    TEACHER_WORKSPACE
    / "老师发的实验设置/最新指示/draft_V7_071.tex"
)
TEACHER_MANUSCRIPT_SHA256 = (
    "c4482f2acda626162a830859db1745d12ee3afaf0c8820b47a3dd94b641ebdf5"
)
TEACHER_OUTPUT_ROOT = TEACHER_SEESM_REPO / "新计划实验输出目录"
PARAMETER_FREEZE_ROOT = SCRIPT_DIR.parent / "config/experiment_freezes"
DEFAULT_SMOKE_PARAMETER_FREEZE = (
    PARAMETER_FREEZE_ROOT / "main_smoke.yaml"
)
# Kept as a compatibility name for callers that import the former constant.
TEACHER_PARAMETER_FREEZE = DEFAULT_SMOKE_PARAMETER_FREEZE
COMMON_OFFLINE_EVALUATION_CONTRACT = (
    SCRIPT_DIR.parent / "config/common_offline_evaluation_v1.yaml"
)
RUN_COMPLETE_SENTINEL = "RUN_COMPLETE.txt"
RUN_INVALID_SENTINEL = "RUN_INVALID.txt"
GOAL_TOLERANCE_M = 0.55
DEADLOCK_SPEED_MPS = 0.05
DEADLOCK_HOLD_SEC = 2.0

PARAMETER_FREEZE_CATEGORIES = (
    "box", "adult", "pedestrian", "child", "child_like", "cyclist",
    "vehicle", "unknown",
)
COMMON_OFFLINE_EVALUATION_VERSION = "teacher_v1_common_offline_evaluation_001"
MAIN_MATRIX_SCENARIOS = (
    "head_on_context_bl", "head_on_context_int", "head_on_context_ext",
    "crossing_context_bl", "crossing_context_int", "crossing_context_ext",
    "local_crowding_context_bl", "local_crowding_context_int",
    "local_crowding_context_ext",
)
MAIN_MATRIX_METHODS = (
    "Standard_MPC_CBF", "EESM_MPC_ECBF", "SEESM_Without_FPU",
    "Proposed_MPC_SECBF",
)
ABLATION_MATRIX_SCENARIOS = (
    "head_on_context_int", "crossing_context_int", "local_crowding_context_int",
)
ABLATION_MATRIX_METHODS = (
    "No_semantic", "Category_only", "Unguarded_SEESM", "No_J_side",
    "SEESM_Ours",
)
STRESS_MATRIX_SCENARIOS = (
    "stress_high_candidate_margin", "stress_short_ttc",
    "stress_local_crowding",
)
STRESS_MATRIX_METHODS = ("Unguarded_SEESM", "SEESM_Ours")
RUNTIME_MATRIX_SCENARIOS = (
    "runtime_scaling_n1", "runtime_scaling_n2", "runtime_scaling_n4",
    "runtime_scaling_n6",
)
RUNTIME_MATRIX_METHODS = ("EESM_MPC_ECBF", "Proposed_MPC_SECBF")

CAMPAIGN_PROFILES = {
    "main": {
        "scenarios": MAIN_MATRIX_SCENARIOS,
        "methods": MAIN_MATRIX_METHODS,
        "smoke_trials": 72,
        "formal_trials": 1080,
        "scenario_semantic_overrides": False,
    },
    "ablation": {
        "scenarios": ABLATION_MATRIX_SCENARIOS,
        "methods": ABLATION_MATRIX_METHODS,
        "smoke_trials": 30,
        "formal_trials": 450,
        "scenario_semantic_overrides": False,
    },
    "stress": {
        "scenarios": STRESS_MATRIX_SCENARIOS,
        "methods": STRESS_MATRIX_METHODS,
        "smoke_trials": 12,
        "formal_trials": 180,
        "scenario_semantic_overrides": True,
    },
    "runtime": {
        "scenarios": RUNTIME_MATRIX_SCENARIOS,
        "methods": RUNTIME_MATRIX_METHODS,
        "smoke_trials": 16,
        "formal_trials": 240,
        "scenario_semantic_overrides": False,
    },
}


SCENARIO_INDEX = {
    "head_on_context_bl": 101,
    "head_on_context_int": 102,
    "head_on_context_ext": 103,
    "crossing_context_bl": 111,
    "crossing_context_int": 112,
    "crossing_context_ext": 113,
    "local_crowding_context_bl": 121,
    "local_crowding_context_int": 122,
    "local_crowding_context_ext": 123,
    "avocado_head_on_coop": 201,
    "avocado_head_on_noncoop": 202,
    "avocado_circle": 203,
    "drmpc_scene_1_circle": 211,
    "drmpc_scene_2_corridor": 212,
    "drmpc_scene_3_circle_dense": 213,
    "drmpc_scene_4_corridor_dense": 214,
    "drmpc_scene_4_corridor_spread6": 215,
    "drmpc_fig4_scene_1_arc": 216,
    "drmpc_fig4_scene_2_vertical": 217,
    "drmpc_fig4_scene_3_reverse_arc": 218,
    "drmpc_fig4_scene_4_reverse_vertical": 219,
    "Exp2_category_box": 21,
    "Exp2_category_adult": 22,
    "Exp2_category_child_like": 23,
    "Exp2_category_cyclist": 24,
    "Exp2_category_pedestrian": 25,
    "Exp2_category_vehicle": 26,
    "Exp3_context_static": 31,
    "Exp3_context_same_direction": 32,
    "Exp3_context_crossing": 33,
    "Exp3_context_frontal_approaching": 34,
    "guard_audit_pressure": 35,
    "stress_high_candidate_margin": 36,
    "stress_short_ttc": 37,
    "stress_local_crowding": 38,
    "runtime_scaling_n1": 41,
    "runtime_scaling_n2": 42,
    "runtime_scaling_n4": 44,
    "runtime_scaling_n6": 46,
}

BASELINES = {
    "B1_ACBF_fixed": {
        "planner": "acbf0_planner.launch",
        "controller_index": 4,
        "guard_enabled": None,
    },
    "Standard_MPC_CBF": {
        "planner": "secbf_planner.launch",
        "controller_index": 6,
        "guard_enabled": "false",
        "mpc_feasibility_guard_enabled": "false",
        "enable_rate_limit": "false",
        "enable_available_projection": "false",
        "enable_guard_fallback": "false",
        "experiment_label": "Standard_MPC_CBF",
        "semantic_mode": "fixed",
        "fixed_beta": 0.4,
        "cbf_metric": "distance",
        "front_adsm": "false",
        "global_seesm_enable": "false",
        "side_preference_enabled": "false",
    },
    "B2_SECBF_no_guard": {
        "planner": "secbf_planner.launch",
        "controller_index": 6,
        "guard_enabled": "false",
        "mpc_feasibility_guard_enabled": "false",
        "enable_rate_limit": "false",
        "enable_available_projection": "false",
        "enable_guard_fallback": "false",
        "experiment_label": "Unguarded_SEESM",
        "dynamic_tau_enabled": True,
        "cbf_metric": "seesm",
    },
    "B3_SECBF_with_guard": {
        "planner": "secbf_planner.launch",
        "controller_index": 6,
        "guard_enabled": "true",
        "experiment_label": "SEESM_Ours",
        "dynamic_tau_enabled": True,
        "cbf_metric": "seesm",
    },
    "Fixed_margin": {
        "planner": "secbf_planner.launch",
        "controller_index": 6,
        "guard_enabled": "true",
        "experiment_label": "Fixed_margin",
        "beta_bar": {
            "box": 0.4,
            "adult": 0.4,
            "pedestrian": 0.4,
            "child": 0.4,
            "child_like": 0.4,
            "cyclist": 0.4,
            "vehicle": 0.4,
            "unknown": 0.4,
        },
        "beta_max": {
            "box": 0.4,
            "adult": 0.4,
            "pedestrian": 0.4,
            "child": 0.4,
            "child_like": 0.4,
            "cyclist": 0.4,
            "vehicle": 0.4,
            "unknown": 0.4,
        },
        "mu_weights": {"bias": 1.0, "heading": 0.0, "ttc": 0.0, "density": 0.0},
        "beta_bar_unknown": 0.4,
        "semantic_mode": "fixed",
        "fixed_beta": 0.4,
    },
    "Category_only": {
        "planner": "secbf_planner.launch",
        "controller_index": 6,
        "guard_enabled": "true",
        "experiment_label": "Category_only",
        "mu_weights": {"bias": 1.0, "heading": 0.0, "ttc": 0.0, "density": 0.0},
        "semantic_mode": "category_only",
    },
    "Context_only": {
        "planner": "secbf_planner.launch",
        "controller_index": 6,
        "guard_enabled": "true",
        "experiment_label": "Context_only",
        "semantic_mode": "context_only",
    },
    "No_rate_limit": {
        "planner": "secbf_planner.launch",
        "controller_index": 6,
        "guard_enabled": "true",
        "experiment_label": "No_rate_limit",
        "enable_rate_limit": "false",
    },
    "No_projection": {
        "planner": "secbf_planner.launch",
        "controller_index": 6,
        "guard_enabled": "true",
        "experiment_label": "No_projection",
        "enable_available_projection": "false",
    },
    "No_mpc_guard": {
        "planner": "secbf_planner.launch",
        "controller_index": 6,
        "guard_enabled": "true",
        "experiment_label": "No_mpc_guard",
        "mpc_feasibility_guard_enabled": "false",
    },
    "No_semantic": {
        "planner": "secbf_planner.launch",
        "controller_index": 6,
        "guard_enabled": "false",
        "experiment_label": "No_semantic",
        "semantic_mode": "none",
        "dynamic_tau_enabled": True,
        "cbf_metric": "seesm",
        "fixed_beta": 0.0,
        "mpc_feasibility_guard_enabled": "false",
        "enable_rate_limit": "false",
        "enable_available_projection": "false",
        "enable_guard_fallback": "false",
        "side_preference_enabled": "false",
    },
    "Unguarded_SEESM": {
        "planner": "secbf_planner.launch",
        "controller_index": 6,
        "guard_enabled": "false",
        "experiment_label": "Unguarded_SEESM",
        "mpc_feasibility_guard_enabled": "false",
        "enable_rate_limit": "false",
        "enable_available_projection": "false",
        "enable_guard_fallback": "false",
        "dynamic_tau_enabled": True,
        "cbf_metric": "seesm",
    },
    "SEESM_Ours": {
        "planner": "secbf_planner.launch",
        "controller_index": 6,
        "guard_enabled": "true",
        "experiment_label": "SEESM_Ours",
        "dynamic_tau_enabled": True,
        "cbf_metric": "seesm",
    },
    "No_J_side": {
        "planner": "secbf_planner.launch",
        "controller_index": 6,
        "guard_enabled": "true",
        "experiment_label": "No_J_side",
        "dynamic_tau_enabled": True,
        "cbf_metric": "seesm",
        "side_preference_enabled": "false",
    },
    "SideWeight_005": {
        "planner": "secbf_planner.launch", "controller_index": 6,
        "guard_enabled": "true", "experiment_label": "SideWeight_005",
        "dynamic_tau_enabled": True, "cbf_metric": "seesm", "side_weight": 0.05,
    },
    "SideWeight_010": {
        "planner": "secbf_planner.launch", "controller_index": 6,
        "guard_enabled": "true", "experiment_label": "SideWeight_010",
        "dynamic_tau_enabled": True, "cbf_metric": "seesm", "side_weight": 0.10,
    },
    "SideWeight_020": {
        "planner": "secbf_planner.launch", "controller_index": 6,
        "guard_enabled": "true", "experiment_label": "SideWeight_020",
        "dynamic_tau_enabled": True, "cbf_metric": "seesm", "side_weight": 0.20,
    },
    "SideWeight_050": {
        "planner": "secbf_planner.launch", "controller_index": 6,
        "guard_enabled": "true", "experiment_label": "SideWeight_050",
        "dynamic_tau_enabled": True, "cbf_metric": "seesm", "side_weight": 0.50,
    },
}

PAPER_BASELINE_ALIASES = {
    "Standard_MPC_CBF": "Standard_MPC_CBF",
    "EESM_MPC_ECBF": "No_semantic",
    "SEESM_Without_FPU": "Unguarded_SEESM",
    "Proposed_MPC_SECBF": "SEESM_Ours",
}

SEED_MANIFEST_HEADER = [
    "trial_id", "seed", "scenario_id", "obstacle_id",
    "start_x_offset_m", "start_y_offset_m", "speed_scale", "start_delay_offset_s",
]
DEFAULT_BETA_BAR = {
    "box": 0.20,
    "adult": 0.75,
    "pedestrian": 0.75,
    "child": 1.05,
    "child_like": 1.05,
    "cyclist": 0.90,
    "vehicle": 0.80,
    "unknown": 0.75,
}

# The manuscript does not provide a separate numeric beta_max(c) table.
# Teacher-v1 therefore keeps a distinct parameter map while provisionally
# freezing its values equal to B_bar(c); an explicit beta_max override can
# replace either a baseline or scenario entry without changing B_bar(c).
DEFAULT_BETA_MAX = dict(DEFAULT_BETA_BAR)

DEFAULT_MU_WEIGHTS = {"bias": 0.6, "heading": 0.2, "ttc": 0.15, "density": 0.1}

DEFAULT_EXPERIMENT_SWITCHES = {
    "semantic_mode": "full",
    "enable_rate_limit": "true",
    "enable_available_projection": "true",
    "enable_guard_fallback": "true",
    "mpc_feasibility_guard_enabled": "true",
    "fixed_beta": 0.4,
    "epsilon_max": 0.05,
    "gamma": 0.35,
    "slack_weight": 1000.0,
    "qf_scale": 1.1,
    "delta_u_weight": 0.02,
    "delta_u_max": 0.4,
    "active_set_distance_m": 8.0,
    "graph_cache_enabled": "false",
    "solver_max_cpu_time_ms": 0.0,
    "safety_delta_bar": 0.10,
    "safety_delta_beta_bar": 0.30,
    "max_cbf_obstacles": 6,
    "guard_h_min": 0.10,
    "delta_beta_positive": 0.30,
    "cbf_metric": "seesm",
    "front_adsm": "true",
    "global_seesm_enable": "false",
    "dynamic_tau_enabled": False,
    "dynamic_tau_mode": "teacher_tca",
    "dynamic_tau_delta_tau": 1e-6,
    "dynamic_tau_ke": 0.30,
    "dynamic_tau_tmax": 2.0,
    "dynamic_tau_min_speed": 1e-6,
    "dynamic_tau_min_distance": 1e-6,
        "dynamic_tau_max_tau": 2.0,
    "guard_kappa": 0.5,
    "guard_max_backtracks": 6,
    "guard_time_budget_ms": 500.0,
    "guard_solver_max_cpu_time_ms": 70.0,
    # Opt-in diagnostic only.  The formal/default path keeps the teacher's
    # sequential q=0..Q search; enabling this probes q=0 first and then uses
    # binary search over the finite monotone candidate set after failure.
    "guard_binary_search": "false",
    "guard_bounded_midpoint_then_zero": "false",
    "emergency_cbf_enabled": "false",
    "emergency_cbf_alpha": 1.5,
    "emergency_cbf_extra_margin": 0.10,
    "emergency_cbf_v_max": 0.35,
    "emergency_cbf_turn_gain": 1.5,
    "emergency_cbf_activation_distance": 4.0,
    "emergency_cbf_progress_v": 0.20,
    "terminal_action": "safe_stop",
    "side_preference_enabled": "true",
    "side_weight": 0.05,
    "side_epsilon_n": 1e-3,
    "side_horizon": 20,
    "side_sign": 1.0,
    "side_min_obstacle_speed": 1e-3,
    "side_activation_distance": 3.0,
}

DYNAMIC_TAU_SWITCHES = (
    "dynamic_tau_enabled",
    "dynamic_tau_mode",
    "dynamic_tau_delta_tau",
    "dynamic_tau_ke",
    "dynamic_tau_tmax",
    "dynamic_tau_min_speed",
    "dynamic_tau_min_distance",
    "dynamic_tau_max_tau",
)

TEACHER_TAU_MODE = "teacher_tca"
TEACHER_KE_TAU_MODE = "teacher_ke_tca"
LEGACY_TAU_MODE = "legacy_gate"
TEACHER_TAU_FORMULA = (
    "tau=clip(-(l dot v_rel)/(||v_rel||^2+delta_tau),0,max_tau)"
)
TEACHER_KE_TAU_FORMULA = (
    "tau=clip(Ke*clip(-(l dot v_rel)/(||v_rel||^2+delta_tau),0,max_tau),0,max_tau)"
)
LEGACY_TAU_FORMULA = "tau=f_r*f_v*f_T*Ke*T_i"
RELATIVE_POSITION_CONVENTION = "l=p_robot-p_obstacle"
RELATIVE_VELOCITY_CONVENTION = "v_rel=v_robot-v_obstacle"
PREDICTION_SIGN_CONVENTION = "l(t+tau)=l+tau*v_rel"


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_write_text(path: Path, text: str) -> None:
    path = Path(path)
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    temporary.write_text(text, encoding="utf-8")
    os.replace(str(temporary), str(path))


def log_profile_for_baseline(baseline_id: str) -> str:
    if baseline_id == "B1_ACBF_fixed":
        return "legacy_b1_v1"
    if baseline_id == "Standard_MPC_CBF":
        return "teacher_distance_mpc_v1"
    return "teacher_seesm_v1"


def required_logs_for_baseline(baseline_id: str) -> tuple:
    return tuple(LOG_PROFILES[log_profile_for_baseline(baseline_id)]["required_logs"])


def ensure_teacher_output_root(output_path: Path) -> Path:
    """Resolve and constrain every real Teacher run to the frozen output root."""
    allowed_root = TEACHER_OUTPUT_ROOT.expanduser().resolve(strict=False)
    resolved = Path(output_path).expanduser().resolve(strict=False)
    if resolved != allowed_root and allowed_root not in resolved.parents:
        raise ValueError(
            f"Teacher-v1 output must be under {allowed_root}; got {resolved}"
        )
    return resolved


def _git_bytes(repository: Path, *arguments: str) -> bytes:
    result = subprocess.run(
        ["git", "-C", str(repository), *arguments],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.returncode != 0:
        message = result.stderr.decode("utf-8", errors="replace").strip()
        raise RuntimeError(
            f"git provenance failed for {repository}: {' '.join(arguments)}: {message}"
        )
    return result.stdout


def git_repo_provenance(repository: Path) -> dict:
    repository = Path(repository).resolve(strict=True)
    pathspec = ["."]  # keep a positive pathspec before exclusions
    excluded_paths = [
        "**/__pycache__/**",
        "**/*.pyc",
    ]
    if repository == TEACHER_SEESM_REPO.resolve(strict=True):
        # Experiment artifacts live in this repository by instruction.  They
        # must not make the source-state fingerprint change while the runner
        # itself is producing them.
        excluded_paths.append("新计划实验输出目录/**")
    pathspec.extend(
        f":(exclude,glob){relative_path}" for relative_path in excluded_paths
    )
    commit = _git_bytes(repository, "rev-parse", "HEAD").decode().strip()
    tree = _git_bytes(repository, "rev-parse", "HEAD^{tree}").decode().strip()
    branch = _git_bytes(repository, "branch", "--show-current").decode().strip()
    status = _git_bytes(
        repository, "status", "--porcelain=v1", "-z", "--untracked-files=all",
        "--", *pathspec,
    )
    tracked_patch = _git_bytes(
        repository, "diff", "--binary", "HEAD", "--", *pathspec
    )
    index_patch = _git_bytes(
        repository, "diff", "--cached", "--binary", "HEAD", "--", *pathspec
    )
    untracked_manifest = _git_bytes(
        repository, "ls-files", "--others", "--exclude-standard", "-z",
        "--", *pathspec,
    )
    state_payload = b"\0".join(
        (status, tracked_patch, index_patch, untracked_manifest)
    )
    return {
        "path": str(repository),
        "commit": commit,
        "tree": tree,
        "branch": branch,
        "dirty": bool(status),
        "status_sha256": sha256_bytes(status),
        "tracked_patch_sha256": sha256_bytes(tracked_patch),
        "index_patch_sha256": sha256_bytes(index_patch),
        "untracked_manifest_sha256": sha256_bytes(untracked_manifest),
        "working_tree_state_sha256": sha256_bytes(state_payload),
        "excluded_generated_paths": excluded_paths,
    }


def checked_file_hash(path: Path, expected_sha256=None) -> dict:
    path = Path(path).expanduser().resolve(strict=True)
    actual = sha256_file(path)
    if expected_sha256 is not None and actual != expected_sha256:
        raise RuntimeError(
            f"source hash mismatch for {path}: expected {expected_sha256}, got {actual}"
        )
    return {"path": str(path), "sha256": actual}


def resolve_ros_package(package: str) -> Path:
    result = subprocess.run(
        ["rospack", "find", package],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"rospack find {package} failed: {result.stderr.strip()}"
        )
    return Path(result.stdout.strip()).resolve(strict=True)


def command_version(command: list) -> str:
    result = subprocess.run(
        command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"toolchain provenance failed: {' '.join(command)}: "
            f"{result.stdout.strip()}"
        )
    return result.stdout.splitlines()[0].strip() if result.stdout else ""


def tracked_source_latest_mtime_ns(source_roots: tuple) -> tuple:
    root = repo_root().resolve(strict=True)
    relative_roots = [str(Path(value)) for value in source_roots]
    tracked = _git_bytes(root, "ls-files", "-z", "--", *relative_roots)
    paths = []
    for raw_name in tracked.split(b"\0"):
        if not raw_name:
            continue
        path = (root / raw_name.decode("utf-8")).resolve(strict=True)
        paths.append(path)
    if not paths:
        raise RuntimeError(
            f"no tracked source files found for runtime roots {relative_roots}"
        )
    return max(path.stat().st_mtime_ns for path in paths), len(paths)


def runtime_executable_record(label: str) -> dict:
    try:
        import roslib.packages
    except ImportError as exc:
        raise RuntimeError("roslib is required for runtime provenance") from exc

    package, executable, source_roots = RUNTIME_NODE_SPECS[label]
    candidates = roslib.packages.find_node(package, executable) or []
    resolved_candidates = sorted(
        {str(Path(candidate).resolve(strict=True)) for candidate in candidates}
    )
    if len(resolved_candidates) != 1:
        raise RuntimeError(
            f"runtime node {package}/{executable} must resolve uniquely; "
            f"got {resolved_candidates}"
        )
    path = Path(resolved_candidates[0])
    if not path.is_file() or not os.access(str(path), os.X_OK):
        raise RuntimeError(f"runtime node is not executable: {path}")

    package_path = resolve_ros_package(package)
    expected_package_path = (
        repo_root() / EXPECTED_ROS_PACKAGE_PATHS[package]
    ).resolve(strict=True)
    if package_path != expected_package_path:
        raise RuntimeError(
            f"ROS overlay mismatch for {package}: got {package_path}, "
            f"expected {expected_package_path}"
        )

    latest_source_mtime_ns, tracked_source_count = (
        tracked_source_latest_mtime_ns(source_roots)
    )
    source_tree = repo_root().resolve(strict=True)
    executes_source_directly = path == source_tree or source_tree in path.parents
    if not executes_source_directly and path.stat().st_mtime_ns < latest_source_mtime_ns:
        raise RuntimeError(
            f"stale runtime node {package}/{executable}: binary {path} predates "
            "its tracked Teacher-v1 sources; rebuild the isolated overlay"
        )

    build_prefix = ""
    for raw_prefix in os.environ.get("CMAKE_PREFIX_PATH", "").split(os.pathsep):
        if not raw_prefix:
            continue
        prefix = Path(raw_prefix).resolve(strict=False)
        if path == prefix or prefix in path.parents:
            build_prefix = str(prefix)
            break
    stat = path.stat()
    return {
        "label": label,
        "package": package,
        "executable": executable,
        "path": str(path),
        "sha256": sha256_file(path),
        "size_bytes": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
        "build_prefix": build_prefix,
        "package_source_path": str(package_path),
        "source_roots": list(source_roots),
        "tracked_source_count": tracked_source_count,
        "latest_source_mtime_ns": latest_source_mtime_ns,
        "executes_source_directly": executes_source_directly,
    }


def collect_trial_runtime_identity(baseline_id: str) -> dict:
    labels = (
        ("legacy_acbf_mpc", "global_planner", "obstacle_manager", "phase5_logger")
        if baseline_id == "B1_ACBF_fixed"
        else (
            "teacher_mpc", "teacher_guard_ground_truth", "global_planner",
            "obstacle_manager", "phase5_logger",
        )
    )
    return {
        "baseline_id": baseline_id,
        "executables": {
            label: runtime_executable_record(label) for label in labels
        },
    }


def verify_repository_context_unchanged(run_context: dict) -> None:
    repositories = (run_context or {}).get("repositories", {})
    expected_names = ("panjian_ws", "seesm_social_navigation")
    for name in expected_names:
        expected = repositories.get(name)
        if not isinstance(expected, dict) or not expected.get("path"):
            raise RuntimeError(f"missing captured repository context for {name}")
        current = git_repo_provenance(Path(expected["path"]))
        for field in (
            "commit", "tree", "branch", "dirty", "working_tree_state_sha256",
        ):
            if current.get(field) != expected.get(field):
                raise RuntimeError(
                    f"Teacher-v1 repository changed during batch: {name}.{field}"
                )

    inputs = (run_context or {}).get("inputs", {})
    for name, expected in inputs.items():
        if not isinstance(expected, dict) or not expected.get("path"):
            raise RuntimeError(f"missing captured input context for {name}")
        path = Path(expected["path"])
        if not path.is_file():
            raise RuntimeError(
                f"Teacher-v1 input changed during batch: {name}.missing"
            )
        current_sha256 = sha256_file(path)
        if current_sha256 != expected.get("sha256"):
            raise RuntimeError(
                f"Teacher-v1 input changed during batch: {name}.sha256"
            )


def collect_run_context(args) -> dict:
    panjian = git_repo_provenance(repo_root())
    seesm = git_repo_provenance(TEACHER_SEESM_REPO)
    for label, source in (("panjian_ws", panjian), ("seesm_social_navigation", seesm)):
        if source["branch"] != "teacher-v1":
            raise RuntimeError(
                f"{label} must be on teacher-v1, got {source['branch']!r}"
            )
        if source["dirty"] and getattr(args, "execution_tier", "formal") == "formal":
            raise RuntimeError(
                f"{label} Teacher-v1 source is dirty; commit or isolate changes before a real run"
            )

    resolved_packages = {}
    for package, relative_path in EXPECTED_ROS_PACKAGE_PATHS.items():
        resolved = resolve_ros_package(package)
        expected = (repo_root() / relative_path).resolve(strict=True)
        if resolved != expected:
            raise RuntimeError(
                f"ROS overlay mismatch: rospack resolves {package} to "
                f"{resolved}, expected {expected}"
            )
        resolved_packages[package] = str(resolved)

    freeze_path = Path(getattr(args, "parameter_freeze", TEACHER_PARAMETER_FREEZE))
    evaluation_path = Path(
        getattr(args, "evaluation_contract", COMMON_OFFLINE_EVALUATION_CONTRACT)
    )
    inputs = {
        "teacher_manuscript": checked_file_hash(
            TEACHER_MANUSCRIPT_PATH, TEACHER_MANUSCRIPT_SHA256
        ),
        "parameter_freeze": checked_file_hash(freeze_path),
        "common_offline_evaluation": checked_file_hash(evaluation_path),
        "scenario_config": checked_file_hash(Path(args.config)),
        "secbf_planner_launch": checked_file_hash(
            repo_root() / "swarm_test/launch/secbf_planner.launch"
        ),
        "acbf0_planner_launch": checked_file_hash(
            repo_root() / "swarm_test/launch/acbf0_planner.launch"
        ),
        "start_test_launch": checked_file_hash(
            repo_root() / "swarm_test/launch/start_test.launch"
        ),
    }
    if getattr(args, "seed_manifest", None):
        inputs["seed_manifest"] = checked_file_hash(Path(args.seed_manifest))
    return {
        "captured_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "repositories": {
            "panjian_ws": panjian,
            "seesm_social_navigation": seesm,
        },
        "inputs": inputs,
        "ros": {
            "package_paths": resolved_packages,
            "ros_package_path": os.environ.get("ROS_PACKAGE_PATH", ""),
            "cmake_prefix_path": os.environ.get("CMAKE_PREFIX_PATH", ""),
        },
        "build_environment": {
            "ros_distribution": command_version(["rosversion", "-d"]),
            "cmake": command_version(["cmake", "--version"]),
            "compiler": command_version(["g++", "--version"]),
            "python": sys.version.splitlines()[0],
        },
    }


def bool_switch(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def strict_bool_value(value, field: str) -> bool:
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"{field} must be a boolean")


def ros_bool(value) -> str:
    return "true" if bool_switch(value) else "false"


def baseline_beta_source(baseline_id: str) -> str:
    return {
        "Standard_MPC_CBF": "fixed_config",
        "No_semantic": "zero",
        "Unguarded_SEESM": "candidate",
        "SEESM_Ours": "beta_applied_final",
        "No_J_side": "beta_applied_final",
        "SideWeight_005": "beta_applied_final",
        "SideWeight_010": "beta_applied_final",
        "SideWeight_020": "beta_applied_final",
        "SideWeight_050": "beta_applied_final",
    }.get(baseline_id, "candidate")


def resolve_baseline_alias(value: str) -> str:
    baseline_id = PAPER_BASELINE_ALIASES.get(value, value)
    if baseline_id not in BASELINES:
        raise ValueError(f"Unknown baseline '{value}'")
    return baseline_id


def paper_baseline_label(baseline_id: str) -> str:
    for label, resolved in PAPER_BASELINE_ALIASES.items():
        if resolved == baseline_id:
            return label
    return baseline_id


def obstacle_id(obstacle: dict, index: int) -> str:
    return str(obstacle.get("obstacle_id", f"obs_{index + 1:03d}"))


def load_seed_manifest(path: Path) -> list:
    with Path(path).open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames != SEED_MANIFEST_HEADER:
            raise ValueError("seed manifest header must match the required contract")
        grouped = {}
        for row in reader:
            try:
                normalized = {
                    "trial_id": row["trial_id"], "seed": int(row["seed"]),
                    "scenario_id": row["scenario_id"], "obstacle_id": row["obstacle_id"],
                    "start_x_offset_m": float(row["start_x_offset_m"]),
                    "start_y_offset_m": float(row["start_y_offset_m"]),
                    "speed_scale": float(row["speed_scale"]),
                    "start_delay_offset_s": float(row["start_delay_offset_s"]),
                }
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError("invalid seed manifest row") from exc
            grouped.setdefault((normalized["scenario_id"], normalized["trial_id"], normalized["seed"]), []).append(normalized)
    return [{"scenario_id": key[0], "trial_id": key[1], "seed": key[2], "rows": rows}
            for key, rows in sorted(grouped.items())]


def materialize_trial(scene: dict, trial: dict):
    effective_scene = copy.deepcopy(scene)
    obstacles = effective_scene.get("obstacles", [])
    by_id = {obstacle_id(obstacle, index): obstacle for index, obstacle in enumerate(obstacles)}
    start_x, start_y = scenario_start_xy(effective_scene)
    for row in trial["rows"]:
        obstacle = by_id.get(row["obstacle_id"])
        if obstacle is None:
            raise ValueError(f"unknown obstacle ID: {row['obstacle_id']}")
        obstacle["obstacle_id"] = row["obstacle_id"]
        obstacle["x"] = float(obstacle.get("x", 0.0)) + row["start_x_offset_m"]
        obstacle["y"] = float(obstacle.get("y", 0.0)) + row["start_y_offset_m"]
        obstacle["slower"] = float(obstacle.get("slower", 1.0)) / row["speed_scale"]
        obstacle["start_delay"] = float(obstacle.get("start_delay", 0.0)) + row["start_delay_offset_s"]
        map_cfg = effective_scene.get("map", {})
        # Canonical scenarios use x as forward distance (start 0, goal near map.x),
        # while y remains centered about zero.
        if abs(obstacle["x"]) > float(map_cfg.get("x", 50.0)) or abs(obstacle["y"]) > float(map_cfg.get("y", 50.0)) / 2:
            raise ValueError("perturbed obstacle start is outside map")
        if math.hypot(obstacle["x"] - start_x, obstacle["y"] - start_y) < 0.8:
            raise ValueError("perturbed obstacle start is within 0.8 m of robot")
    for index, obstacle in enumerate(obstacles):
        obstacle.setdefault("obstacle_id", obstacle_id(obstacle, index))
    return effective_scene, obstacles


def write_trial_artifacts(run_dir: Path, scenario: dict, trial: dict, resolved_baseline_id: str) -> None:
    with (run_dir / "effective_obstacles.yaml").open("w", encoding="utf-8") as f:
        yaml.safe_dump({"obstacles": scenario.get("obstacles", [])}, f, sort_keys=False)
    with (run_dir / "trial_manifest_row.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=SEED_MANIFEST_HEADER)
        writer.writeheader()
        writer.writerows(trial["rows"])


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def load_scenarios(config_path: Path) -> dict:
    if yaml is None:
        raise RuntimeError("PyYAML is required: install python3-yaml or pip3 install pyyaml")
    with config_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data.get("scenarios", {})


class ParameterFreezeError(ValueError):
    """Raised when a requested Teacher-v1 execution freeze is incomplete."""


def _freeze_number(value, label: str, *, positive=False, nonnegative=False) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ParameterFreezeError(f"{label} must be a finite number") from exc
    if not math.isfinite(parsed):
        raise ParameterFreezeError(f"{label} must be a finite number")
    if positive and parsed <= 0.0:
        raise ParameterFreezeError(f"{label} must be positive")
    if nonnegative and parsed < 0.0:
        raise ParameterFreezeError(f"{label} must be nonnegative")
    return parsed


def _freeze_category_table(raw, label: str) -> dict:
    if not isinstance(raw, dict):
        raise ParameterFreezeError(f"{label} must be a mapping")
    missing = [key for key in PARAMETER_FREEZE_CATEGORIES if key not in raw]
    extra = sorted(set(raw) - set(PARAMETER_FREEZE_CATEGORIES))
    if missing or extra:
        raise ParameterFreezeError(
            f"{label} must contain exactly the eight Teacher categories; "
            f"missing={missing}, extra={extra}"
        )
    return {
        key: _freeze_number(raw[key], f"{label}.{key}", nonnegative=True)
        for key in PARAMETER_FREEZE_CATEGORIES
    }


def _freeze_mapping(raw, label: str) -> dict:
    if not isinstance(raw, dict):
        raise ParameterFreezeError(f"{label} must be a mapping")
    return raw


def load_parameter_freeze(path: Path, execution_tier: str,
                          campaign: str = "main") -> dict:
    """Load the parameters that the runner will actually pass to ROS.

    A hash alone is not a freeze: this loader converts the selected YAML into
    the per-run overrides below.  Formal execution additionally rejects any
    non-main matrix or context-dependent solver setting.
    """
    if yaml is None:
        raise ParameterFreezeError("PyYAML is required for parameter freezes")
    if execution_tier not in {"smoke", "formal"}:
        raise ParameterFreezeError("execution_tier must be smoke or formal")
    if campaign not in CAMPAIGN_PROFILES:
        raise ParameterFreezeError(
            f"unknown campaign {campaign!r}; expected one of "
            f"{sorted(CAMPAIGN_PROFILES)}"
        )
    resolved_path = Path(path).expanduser().resolve()
    if not resolved_path.exists():
        raise ParameterFreezeError(f"parameter freeze not found: {resolved_path}")
    try:
        with resolved_path.open("r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise ParameterFreezeError(
            f"cannot read parameter freeze {resolved_path}: {exc}"
        ) from exc
    if not isinstance(raw, dict):
        raise ParameterFreezeError("parameter freeze must be a YAML mapping")

    freeze_id = str(raw.get("freeze_id", "")).strip()
    if not freeze_id:
        raise ParameterFreezeError("parameter freeze freeze_id is required")
    status = str(raw.get("status", "")).strip()
    allowed_statuses = (
        {"smoke_frozen", "smoke_frozen_not_formal"}
        if execution_tier == "smoke" else {"formal_frozen"}
    )
    if status not in allowed_statuses:
        raise ParameterFreezeError(
            f"{execution_tier} execution requires status in {sorted(allowed_statuses)}, "
            f"got {status!r}"
        )
    declared_campaign = str(raw.get("campaign", "")).strip()
    if declared_campaign != campaign:
        raise ParameterFreezeError(
            f"parameter freeze campaign must be {campaign!r}, "
            f"got {declared_campaign!r}"
        )

    matrix = _freeze_mapping(raw.get("matrix"), "matrix")
    matrix_scenarios = matrix.get("scenarios")
    matrix_methods = matrix.get("requested_methods")
    profile = CAMPAIGN_PROFILES[campaign]
    if tuple(matrix_scenarios or ()) != tuple(profile["scenarios"]):
        raise ParameterFreezeError(
            f"{campaign} freeze must declare its exact scenario matrix"
        )
    if tuple(matrix_methods or ()) != tuple(profile["methods"]):
        raise ParameterFreezeError(
            f"{campaign} freeze must declare its exact method matrix"
        )
    expected_trials = profile[f"{execution_tier}_trials"]
    if int(matrix.get("expected_trials", 0)) != expected_trials:
        raise ParameterFreezeError(
            f"{campaign} {execution_tier} freeze expected_trials must be "
            f"{expected_trials}"
        )
    preserve_semantic_overrides = matrix.get(
        "preserve_scenario_semantic_overrides", False
    )
    if preserve_semantic_overrides is not profile["scenario_semantic_overrides"]:
        raise ParameterFreezeError(
            "matrix.preserve_scenario_semantic_overrides does not match the "
            f"frozen {campaign} campaign policy"
        )

    semantic = _freeze_mapping(raw.get("semantic_margin"), "semantic_margin")
    phi_status = str(semantic.get("phi_status", "")).strip()
    if phi_status not in {
        "explicit_linear_current_mapping_frozen_for_smoke",
        "explicit_linear_current_mapping_frozen_for_formal",
    }:
        raise ParameterFreezeError("semantic_margin.phi_status must freeze the explicit linear mapping")
    phi_raw = _freeze_mapping(semantic.get("phi_weights"), "semantic_margin.phi_weights")
    phi_key_map = {
        "bias": "bias", "head_on": "heading", "ttc_norm": "ttc",
        "density_norm": "density",
    }
    if set(phi_raw) != set(phi_key_map):
        raise ParameterFreezeError("semantic_margin.phi_weights keys must be bias/head_on/ttc_norm/density_norm")
    mu_weights = {
        target: _freeze_number(phi_raw[source], f"semantic_margin.phi_weights.{source}")
        for source, target in phi_key_map.items()
    }
    if any(mu_weights[name] < 0.0 for name in ("heading", "ttc", "density")):
        raise ParameterFreezeError("semantic interaction weights must be nonnegative")
    beta_bar = _freeze_category_table(semantic.get("beta_bar_m"), "semantic_margin.beta_bar_m")
    beta_max = _freeze_category_table(semantic.get("beta_max_m"), "semantic_margin.beta_max_m")
    h_min = _freeze_number(semantic.get("h_min_m"), "semantic_margin.h_min_m", nonnegative=True)
    delta_beta_positive = _freeze_number(
        semantic.get("delta_beta_positive_m_per_cycle"),
        "semantic_margin.delta_beta_positive_m_per_cycle", positive=True,
    )
    dynamic = _freeze_mapping(semantic.get("dynamic_tau"), "semantic_margin.dynamic_tau")
    if dynamic.get("mode") != TEACHER_TAU_MODE:
        raise ParameterFreezeError("semantic_margin.dynamic_tau.mode must be teacher_tca")
    dynamic_delta = _freeze_number(dynamic.get("delta_tau"), "dynamic_tau.delta_tau", positive=True)
    dynamic_max = _freeze_number(dynamic.get("max_tau_sec"), "dynamic_tau.max_tau_sec", nonnegative=True)

    mpc = _freeze_mapping(raw.get("robot_and_mpc"), "robot_and_mpc")
    gamma = _freeze_number(mpc.get("gamma"), "robot_and_mpc.gamma", positive=True)
    qf_scale = _freeze_number(mpc.get("qf_scale"), "robot_and_mpc.qf_scale", positive=True)
    delta_u_weight = _freeze_number(
        mpc.get("delta_u_weight"), "robot_and_mpc.delta_u_weight", nonnegative=True
    )
    delta_u_max = _freeze_number(mpc.get("delta_u_max"), "robot_and_mpc.delta_u_max", positive=True)
    max_cbf_obstacles = int(_freeze_number(
        mpc.get("max_cbf_obstacles"), "robot_and_mpc.max_cbf_obstacles", positive=True
    ))
    active_set_distance = _freeze_number(
        mpc.get("active_set_distance_m"), "robot_and_mpc.active_set_distance_m", positive=True
    )
    graph_cache = mpc.get("graph_cache_enabled")
    if not isinstance(graph_cache, bool):
        raise ParameterFreezeError("robot_and_mpc.graph_cache_enabled must be boolean")
    solver_max_cpu_time = _freeze_number(
        mpc.get("solver_max_cpu_time_ms", 0.0),
        "robot_and_mpc.solver_max_cpu_time_ms",
        nonnegative=True,
    )

    guard = _freeze_mapping(raw.get("guard_and_failure_policy"), "guard_and_failure_policy")
    guard_kappa = _freeze_number(guard.get("kappa"), "guard_and_failure_policy.kappa", positive=True)
    if guard_kappa > 1.0:
        raise ParameterFreezeError("guard_and_failure_policy.kappa must be <= 1")
    guard_backtracks = int(_freeze_number(
        guard.get("max_backtracks_q"), "guard_and_failure_policy.max_backtracks_q", nonnegative=True
    ))
    guard_budget = _freeze_number(
        guard.get("time_budget_ms"), "guard_and_failure_policy.time_budget_ms", positive=True
    )
    guard_solver_budget = _freeze_number(
        guard.get("recovery_solver_max_cpu_time_ms", 70.0),
        "guard_and_failure_policy.recovery_solver_max_cpu_time_ms", positive=True,
    )
    if guard_solver_budget >= guard_budget:
        raise ParameterFreezeError(
            "Guard recovery solver budget must be below total Guard budget"
        )
    guard_binary_search = guard.get("guard_binary_search", False)
    if not isinstance(guard_binary_search, bool):
        raise ParameterFreezeError("guard_and_failure_policy.guard_binary_search must be boolean")
    guard_bounded_midpoint_then_zero = guard.get(
        "bounded_midpoint_then_zero", False
    )
    if not isinstance(guard_bounded_midpoint_then_zero, bool):
        raise ParameterFreezeError(
            "guard_and_failure_policy.bounded_midpoint_then_zero must be boolean"
        )
    if execution_tier == "formal" and guard_binary_search:
        raise ParameterFreezeError("formal freeze must retain sequential teacher Guard search")
    terminal_action = guard.get("terminal_action")
    if terminal_action not in {"safe_stop", "emergency_cbf"}:
        raise ParameterFreezeError(
            "guard_and_failure_policy.terminal_action must be safe_stop or emergency_cbf"
        )
    emergency = guard.get("emergency_cbf", {})
    if not isinstance(emergency, dict):
        raise ParameterFreezeError("guard_and_failure_policy.emergency_cbf must be a mapping")
    emergency_enabled = terminal_action == "emergency_cbf"
    emergency_alpha = _freeze_number(emergency.get("alpha", 1.5), "emergency_cbf.alpha", positive=True)
    emergency_margin = _freeze_number(emergency.get("extra_margin_m", 0.10), "emergency_cbf.extra_margin_m", nonnegative=True)
    emergency_v_max = _freeze_number(emergency.get("v_max_mps", 0.35), "emergency_cbf.v_max_mps", positive=True)
    emergency_turn_gain = _freeze_number(emergency.get("turn_gain", 1.5), "emergency_cbf.turn_gain", positive=True)
    emergency_activation_distance = _freeze_number(
        emergency.get("activation_distance_m", 4.0),
        "emergency_cbf.activation_distance_m", positive=True,
    )
    emergency_progress_v = _freeze_number(
        emergency.get("progress_v_mps", 0.20),
        "emergency_cbf.progress_v_mps", nonnegative=True,
    )

    solver = _freeze_mapping(
        raw.get("scenario_specific_solver_settings"),
        "scenario_specific_solver_settings",
    )
    if solver.get("invariant_across_context_levels") is not True:
        raise ParameterFreezeError(
            "scenario_specific_solver_settings must be invariant across context levels"
        )
    uniform_solver = _freeze_mapping(
        solver.get("all_context_levels"),
        "scenario_specific_solver_settings.all_context_levels",
    )
    epsilon_max = _freeze_number(
        uniform_solver.get("epsilon_max"), "all_context_levels.epsilon_max", nonnegative=True
    )
    slack_weight = _freeze_number(
        uniform_solver.get("slack_weight"), "all_context_levels.slack_weight", positive=True
    )
    outcome = _freeze_mapping(raw.get("trial_outcome"), "trial_outcome")
    goal_tolerance = _freeze_number(
        outcome.get("goal_tolerance_m"), "trial_outcome.goal_tolerance_m",
        positive=True,
    )
    timeout_sec = _freeze_number(
        outcome.get("timeout_sec"), "trial_outcome.timeout_sec", positive=True,
    )
    deadlock_speed = _freeze_number(
        outcome.get("deadlock_speed_threshold_mps"),
        "trial_outcome.deadlock_speed_threshold_mps", positive=True,
    )
    deadlock_hold = _freeze_number(
        outcome.get("deadlock_hold_sec"),
        "trial_outcome.deadlock_hold_sec", positive=True,
    )
    if (
        abs(goal_tolerance - GOAL_TOLERANCE_M) > 1.0e-12
        or abs(deadlock_speed - DEADLOCK_SPEED_MPS) > 1.0e-12
        or abs(deadlock_hold - DEADLOCK_HOLD_SEC) > 1.0e-12
    ):
        raise ParameterFreezeError(
            "trial_outcome values must match the executable classifier constants"
        )
    stability = _freeze_mapping(raw.get("stability"), "stability")
    stability_contract = {
        "tracking_error_threshold_m": _freeze_number(
            stability.get("tracking_error_threshold_m"),
            "stability.tracking_error_threshold_m", positive=True,
        ),
        "tracking_hold_sec": _freeze_number(
            stability.get("tracking_hold_sec"),
            "stability.tracking_hold_sec", positive=True,
        ),
        "settling_deadline_sec": _freeze_number(
            stability.get("settling_deadline_sec"),
            "stability.settling_deadline_sec", positive=True,
        ),
        "final_goal_error_threshold_m": _freeze_number(
            stability.get("final_goal_error_threshold_m"),
            "stability.final_goal_error_threshold_m", positive=True,
        ),
        "persistent_slack_threshold": _freeze_number(
            stability.get("persistent_slack_threshold"),
            "stability.persistent_slack_threshold", positive=True,
        ),
        "persistent_slack_hold_sec": _freeze_number(
            stability.get("persistent_slack_hold_sec"),
            "stability.persistent_slack_hold_sec", positive=True,
        ),
        "runtime_control_period_ms": _freeze_number(
            stability.get("runtime_control_period_ms"),
            "stability.runtime_control_period_ms", positive=True,
        ),
    }

    return {
        "id": freeze_id,
        "status": status,
        "execution_tier": execution_tier,
        "campaign": campaign,
        "expected_trials": expected_trials,
        "preserve_scenario_semantic_overrides": preserve_semantic_overrides,
        "path": str(resolved_path),
        "sha256": sha256_file(resolved_path),
        "beta_bar": beta_bar,
        "beta_max": beta_max,
        "mu_weights": mu_weights,
        "semantic_phi_status": phi_status,
        "trial_outcome": {
            "goal_tolerance_m": goal_tolerance,
            "timeout_sec": timeout_sec,
            "deadlock_speed_threshold_mps": deadlock_speed,
            "deadlock_hold_sec": deadlock_hold,
        },
        "stability": stability_contract,
        "switches": {
            "guard_h_min": h_min,
            "delta_beta_positive": delta_beta_positive,
            "dynamic_tau_mode": TEACHER_TAU_MODE,
            "dynamic_tau_delta_tau": dynamic_delta,
            "dynamic_tau_tmax": dynamic_max,
            "dynamic_tau_max_tau": dynamic_max,
            "gamma": gamma,
            "qf_scale": qf_scale,
            "delta_u_weight": delta_u_weight,
            "delta_u_max": delta_u_max,
            "max_cbf_obstacles": max_cbf_obstacles,
            "active_set_distance_m": active_set_distance,
            "graph_cache_enabled": graph_cache,
            "solver_max_cpu_time_ms": solver_max_cpu_time,
            "guard_kappa": guard_kappa,
            "guard_max_backtracks": guard_backtracks,
            "guard_time_budget_ms": guard_budget,
            "guard_solver_max_cpu_time_ms": guard_solver_budget,
            "guard_binary_search": guard_binary_search,
            "guard_bounded_midpoint_then_zero": guard_bounded_midpoint_then_zero,
            "emergency_cbf_enabled": emergency_enabled,
            "emergency_cbf_alpha": emergency_alpha,
            "emergency_cbf_extra_margin": emergency_margin,
            "emergency_cbf_v_max": emergency_v_max,
            "emergency_cbf_turn_gain": emergency_turn_gain,
            "emergency_cbf_activation_distance": emergency_activation_distance,
            "emergency_cbf_progress_v": emergency_progress_v,
            "terminal_action": terminal_action,
            "epsilon_max": epsilon_max,
            "slack_weight": slack_weight,
        },
    }


def apply_parameter_freeze(scenario: dict, freeze=None) -> dict:
    """Return an execution scenario with all frozen, launch-bearing values set."""
    if not freeze:
        return scenario
    resolved = copy.deepcopy(scenario)
    scenario_beta_bar = dict(resolved.get("beta_bar", {}))
    scenario_beta_max = dict(resolved.get("beta_max", {}))
    scenario_mu_weights = dict(resolved.get("mu_weights", {}))
    resolved["beta_bar"] = dict(freeze["beta_bar"])
    resolved["beta_max"] = dict(freeze["beta_max"])
    resolved["mu_weights"] = dict(freeze["mu_weights"])
    if freeze.get("preserve_scenario_semantic_overrides"):
        resolved["beta_bar"].update(scenario_beta_bar)
        resolved["beta_max"].update(scenario_beta_max)
        resolved["mu_weights"].update(scenario_mu_weights)
    switches = dict(resolved.get("experiment_switches", {}))
    switches.update(freeze["switches"])
    resolved["experiment_switches"] = switches
    return resolved


def parameter_freeze_metadata(freeze):
    if not freeze:
        return None
    return {
        "id": freeze["id"],
        "status": freeze["status"],
        "execution_tier": freeze["execution_tier"],
        "campaign": freeze["campaign"],
        "expected_trials": freeze["expected_trials"],
        "path": freeze["path"],
        "sha256": freeze["sha256"],
        "phi_status": freeze["semantic_phi_status"],
    }


def load_common_offline_evaluation_contract(path: Path) -> dict:
    """Load only immutable provenance for the common post-hoc evaluator."""
    if yaml is None:
        raise ParameterFreezeError("PyYAML is required for common evaluation")
    resolved_path = Path(path).expanduser().resolve()
    if not resolved_path.exists():
        raise ParameterFreezeError(
            f"common offline evaluation contract not found: {resolved_path}"
        )
    try:
        with resolved_path.open("r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise ParameterFreezeError(
            f"cannot read common offline evaluation contract {resolved_path}: {exc}"
        ) from exc
    if not isinstance(raw, dict):
        raise ParameterFreezeError("common offline evaluation contract must be a mapping")
    contract_id = str(raw.get("id", "")).strip()
    if not contract_id:
        raise ParameterFreezeError("common offline evaluation contract id is required")
    if raw.get("version") != COMMON_OFFLINE_EVALUATION_VERSION:
        raise ParameterFreezeError("common offline evaluation contract version mismatch")
    status = str(raw.get("status", "")).strip()
    if status not in {"smoke_frozen", "formal_frozen"}:
        raise ParameterFreezeError(
            "common offline evaluation contract status must be smoke_frozen or formal_frozen"
        )
    return {
        "id": contract_id,
        "version": COMMON_OFFLINE_EVALUATION_VERSION,
        "status": status,
        "path": str(resolved_path),
        "sha256": sha256_file(resolved_path),
    }


def selected(values, requested):
    if requested == "all":
        return list(values)
    if "," in requested:
        result = []
        for item in requested.split(","):
            item = item.strip()
            if item not in values:
                raise ValueError(f"Unknown selection '{item}'. Valid values: {', '.join(values)} or all")
            result.append(item)
        return result
    if requested not in values:
        raise ValueError(f"Unknown selection '{requested}'. Valid values: {', '.join(values)} or all")
    return [requested]


def legacy_obstacle_for_simulator(obs: dict) -> dict:
    """Convert paper-style obstacle declarations to dynamic_simulator params."""
    if obs.get("motion_type") != "line":
        return {
            "x": float(obs.get("x", 0.0)),
            "y": float(obs.get("y", 0.0)),
            "z": float(obs.get("z", 0.75)),
            "offset": float(obs.get("offset", 0.0)),
            "slower": float(obs.get("slower", 999.0)),
            "start_delay": float(obs.get("start_delay", 0.0)),
            "scale_x": float(obs.get("scale_x", 0.0)),
            "scale_y": float(obs.get("scale_y", 0.0)),
            "scale_z": float(obs.get("scale_z", 0.0)),
        }

    start = obs.get("start", {})
    goal = obs.get("goal", {})
    sx = float(start.get("x", 0.0))
    sy = float(start.get("y", 0.0))
    gx = float(goal.get("x", sx))
    gy = float(goal.get("y", sy))
    z = float(start.get("z", obs.get("z", 0.75)))
    return {
        "x": 0.5 * (sx + gx),
        "y": 0.5 * (sy + gy),
        "z": z,
        "offset": float(obs.get("offset", 0.0)),
        "slower": float(obs.get("travel_time", obs.get("slower", 8.0))),
        "start_delay": float(obs.get("start_delay", 0.0)),
        "scale_x": 3.0 * (sx - gx),
        "scale_y": 2.5 * (sy - gy),
        "scale_z": float(obs.get("scale_z", 0.0)),
    }


def write_obstacle_params(run_dir: Path, obstacles: list) -> Path:
    param_path = run_dir / "obstacles_param.yaml"
    clean_obstacles = []
    for obs in obstacles:
        clean_obstacles.append(legacy_obstacle_for_simulator(obs))
    with param_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump({"obstacle_params": clean_obstacles}, f, sort_keys=False)
    return param_path


def baseline_beta_bar(baseline_id: str) -> dict:
    baseline = BASELINES[baseline_id]
    values = dict(DEFAULT_BETA_BAR)
    values.update(baseline.get("beta_bar", {}))
    return values


def scenario_beta_bar(baseline_id: str, scenario: dict) -> dict:
    values = baseline_beta_bar(baseline_id)
    values.update(scenario.get("beta_bar", {}))
    return values


def baseline_beta_max(baseline_id: str) -> dict:
    baseline = BASELINES[baseline_id]
    values = dict(DEFAULT_BETA_MAX)
    values.update(baseline.get("beta_max", {}))
    return values


def scenario_beta_max(baseline_id: str, scenario: dict) -> dict:
    values = baseline_beta_max(baseline_id)
    values.update(scenario.get("beta_max", {}))
    return values


def baseline_mu_weights(baseline_id: str) -> dict:
    baseline = BASELINES[baseline_id]
    values = dict(DEFAULT_MU_WEIGHTS)
    values.update(baseline.get("mu_weights", {}))
    return values


def scenario_mu_weights(baseline_id: str, scenario: dict) -> dict:
    values = baseline_mu_weights(baseline_id)
    values.update(scenario.get("mu_weights", {}))
    return values


def baseline_switches(baseline_id: str) -> dict:
    baseline = BASELINES[baseline_id]
    values = dict(DEFAULT_EXPERIMENT_SWITCHES)
    for key in values:
        if key in baseline:
            values[key] = baseline[key]
    # Teacher-v1 has exactly one production dynamic-time policy.  The legacy
    # gated/frozen policy remains readable by audit tools but is never emitted
    # by this runner.  The distance-only Standard baseline is deliberately
    # instantaneous and must never inherit dynamic tau from a scenario override.
    values["dynamic_tau_enabled"] = baseline_id not in {
        "B1_ACBF_fixed", "Standard_MPC_CBF",
    }
    values["dynamic_tau_mode"] = TEACHER_TAU_MODE
    if baseline_id == "Standard_MPC_CBF":
        values["cbf_metric"] = "distance"
    else:
        values["cbf_metric"] = "seesm"
    if baseline_id in {
        "Category_only", "Unguarded_SEESM", "SEESM_Ours", "No_J_side",
        "SideWeight_005", "SideWeight_010", "SideWeight_020", "SideWeight_050",
    }:
        values["global_seesm_enable"] = "true"
    else:
        values["global_seesm_enable"] = "false"
    return values


def scenario_switches(baseline_id: str, scenario: dict) -> dict:
    values = baseline_switches(baseline_id)
    values.update(scenario.get("experiment_switches", {}))
    if baseline_id in {"B1_ACBF_fixed", "Standard_MPC_CBF"}:
        values["dynamic_tau_enabled"] = False
        values["dynamic_tau_mode"] = TEACHER_TAU_MODE
    else:
        values["dynamic_tau_enabled"] = True
        if values["dynamic_tau_mode"] not in {TEACHER_TAU_MODE, TEACHER_KE_TAU_MODE}:
            raise ValueError(
                "Teacher-v1 runner only permits teacher_tca or explicit "
                "teacher_ke_tca diagnostic mode"
            )
    if baseline_id == "B1_ACBF_fixed":
        # B1 is the frozen legacy ACBF launch, not a Teacher SEESM method.
        # Record only its effective launch semantics instead of inheriting the
        # Teacher defaults into metadata.
        values["cbf_metric"] = "legacy_acbf"
        values["semantic_mode"] = "not_applicable"
        values["fixed_beta"] = 0.30
        values["guard_enabled"] = "false"
        values["enable_rate_limit"] = "false"
        values["enable_available_projection"] = "false"
        values["enable_guard_fallback"] = "false"
        values["mpc_feasibility_guard_enabled"] = "false"
        values["front_adsm"] = "true"
        values["global_seesm_enable"] = "false"
        values["side_preference_enabled"] = "false"
    elif baseline_id == "Standard_MPC_CBF":
        # Scenario YAML is allowed to tune experiment methods, but it must not
        # mutate the definition of the distance-only Standard control.  Re-lock
        # every semantic/Guard/global/side path after applying scenario values.
        values["cbf_metric"] = "distance"
        values["dynamic_tau_enabled"] = False
        values["dynamic_tau_mode"] = TEACHER_TAU_MODE
        values["semantic_mode"] = "fixed"
        values["fixed_beta"] = 0.4
        values["guard_enabled"] = "false"
        values["enable_rate_limit"] = "false"
        values["enable_available_projection"] = "false"
        values["enable_guard_fallback"] = "false"
        values["mpc_feasibility_guard_enabled"] = "false"
        values["front_adsm"] = "false"
        values["global_seesm_enable"] = "false"
        values["side_preference_enabled"] = "false"
    return values


def dynamic_tau_contract(switches: dict, beta_source: str) -> dict:
    enabled = bool_switch(switches["dynamic_tau_enabled"])
    mode = str(switches["dynamic_tau_mode"])
    if mode == TEACHER_TAU_MODE:
        formula = TEACHER_TAU_FORMULA
        mpc_stage_policy = "symbolic_stagewise" if enabled else "disabled"
    elif mode == TEACHER_KE_TAU_MODE:
        formula = TEACHER_KE_TAU_FORMULA
        mpc_stage_policy = "symbolic_stagewise" if enabled else "disabled"
    elif mode == LEGACY_TAU_MODE:
        formula = LEGACY_TAU_FORMULA
        mpc_stage_policy = "numeric_frozen_per_stage" if enabled else "disabled"
    else:
        raise ValueError(f"Unknown dynamic tau mode: {mode}")
    return {
        "enabled": enabled,
        "mode": mode,
        "delta_tau": float(switches["dynamic_tau_delta_tau"]),
        # Retained only so archived Legacy-v1 runs remain reconstructable.
        "Ke": float(switches["dynamic_tau_ke"]),
        "Tmax": float(switches["dynamic_tau_tmax"]),
        "min_speed": float(switches["dynamic_tau_min_speed"]),
        "min_distance": float(switches["dynamic_tau_min_distance"]),
        "max_tau": float(switches["dynamic_tau_max_tau"]),
        "formula": formula,
        "relative_position_convention": RELATIVE_POSITION_CONVENTION,
        "relative_velocity_convention": RELATIVE_VELOCITY_CONVENTION,
        "prediction_sign": PREDICTION_SIGN_CONVENTION,
        "mpc_stage_policy": mpc_stage_policy,
        "h_eesm": "||l+tau*v_rel||-R_obs-R_robot",
        "h_seesm": "h_eesm-beta",
        "beta_source": beta_source,
    }


def semantic_margin_contract(baseline_id: str, scenario: dict,
                             switches: dict):
    if baseline_id == "B1_ACBF_fixed":
        return None
    beta_bar = scenario_beta_bar(baseline_id, scenario)
    beta_max = scenario_beta_max(baseline_id, scenario)
    weights = scenario_mu_weights(baseline_id, scenario)
    guard_enabled = bool_switch(BASELINES[baseline_id]["guard_enabled"])
    semantic_mode = str(switches["semantic_mode"])
    return {
        "version": SEMANTIC_MARGIN_CONTRACT_VERSION,
        "status": "provisional",
        "applicable": (
            str(switches["cbf_metric"]) == "seesm" and semantic_mode != "none"
        ),
        "formula_ids": ["F05", "F06", "F07"],
        "phi": {
            "status": "provisional_missing_teacher_analytic_form",
            "formula": (
                "beta_tilde=beta_bar*clip(bias+head_on*f_head+"
                "ttc*TTC_norm+density*rho_norm,0,1)"
            ),
            "weights": {
                "bias": float(weights["bias"]),
                "head_on": float(weights["heading"]),
                "ttc": float(weights["ttc"]),
                "density": float(weights["density"]),
            },
            "multiplier_clip": [0.0, 1.0],
        },
        "beta_bar_m": {key: float(value) for key, value in beta_bar.items()},
        "beta_max_m": {key: float(value) for key, value in beta_max.items()},
        "beta_max_status": "provisional_separate_table_equal_to_beta_bar_default",
        "h_min_m": float(switches["guard_h_min"]),
        "h_min_status": "provisional_mapping_from_teacher_eta_0p10",
        "delta_beta_positive_m_per_cycle": float(
            switches["delta_beta_positive"]
        ),
        "delta_beta_positive_status": "provisional_component_value",
        "upper_bound_formula": (
            "min(beta_max(c),beta_previous+delta_beta_positive,"
            "max(h_eesm-h_min,0))"
        ),
        "pre_guard_formula": "min(beta_tilde,beta_upper_bound)",
        "enforcement": {
            "category_bound": guard_enabled,
            "positive_increment_bound": (
                guard_enabled and bool_switch(switches["enable_rate_limit"])
            ),
            "available_margin_bound": (
                guard_enabled and
                bool_switch(switches["enable_available_projection"])
            ),
        },
        "beta_previous_source": (
            "same_obstacle_id_previous_final_mpc_accepted_feedback"
        ),
        "first_seen_beta_previous_m": 0.0,
        "disappearance_policy": "erase_history_reappearance_is_rebirth_zero",
        "category_change_policy": (
            "preserve_same_id_history_apply_current_category_cap"
        ),
        "no_cbf_history_policy": "no_cbf_or_safe_stop_do_not_commit",
    }


def typed_cycle_contract(baseline_id: str, switches: dict,
                         num_obstacles: int):
    if baseline_id == "B1_ACBF_fixed":
        return None
    return {
        "version": TYPED_CYCLE_CONTRACT_VERSION,
        "applicable": str(switches["cbf_metric"]) == "seesm",
        "transport": "typed_ros_messages",
        "configured_obstacle_ids": [
            4000 + index for index in range(num_obstacles)
        ],
        "snapshot": {
            "topic": "/globalFsm_by_adsm/teacher_obstacle_snapshot",
            "message": "semantic_guard/PredictedObstacleArray",
            "cycle_field": "cycle_id",
        },
        "pre_guard": {
            "topic": "/safety_margin/beta_pre_guard",
            "message": "semantic_guard/PreGuardMarginArray",
            "cycle_field": "obstacle_cycle_id",
            "policy_fields": [
                "enforce_category_bound",
                "enforce_positive_increment_bound",
                "enforce_available_margin_bound",
            ],
        },
        "accepted_feedback": {
            "topic": "/safety_margin/beta_applied_final",
            "message": "semantic_guard/AppliedMarginArray",
            "cycle_field": "obstacle_cycle_id",
        },
        "join_key": ["obstacle_cycle_id", "obstacle_id"],
        "id_set_policy": "strict_unique_exact_match",
        "stale_cycle_policy": "reject_nonincreasing_or_unknown_cycle",
        "accepted_sources": [
            "candidate", "previous", "zero", "kappa", "no_cbf", "safe_stop", "emergency_cbf",
            "mpc_reprojected",
        ],
        "guard": {
            "finite_candidate_set": "beta_pre_guard*kappa^q for q=0..Q-1 plus explicit zero at q=Q",
            "kappa": float(switches["guard_kappa"]),
            "max_backtracks_q": int(switches["guard_max_backtracks"]),
            "time_budget_ms": float(switches["guard_time_budget_ms"]),
            "binary_search_enabled": bool_switch(switches["guard_binary_search"]),
            "bounded_midpoint_then_zero": bool_switch(
                switches["guard_bounded_midpoint_then_zero"]
            ),
            "search_mode": (
                "bounded_midpoint_then_zero"
                if bool_switch(switches["guard_bounded_midpoint_then_zero"])
                else (
                    "q0_then_binary_diagnostic"
                    if bool_switch(switches["guard_binary_search"])
                    else "sequential_q0_to_qmax"
                )
            ),
            "risk_proxy": "beta_tilde_descending_then_obstacle_id",
            "multi_obstacle_policy": "accepted_components_fixed_unprocessed_components_zero",
        },
        "history_commit_policy": "accepted_feedback_except_no_cbf_or_safe_stop",
    }


def point_xy(point: dict, default_x=0.0, default_y=0.0):
    return float(point.get("x", default_x)), float(point.get("y", default_y))


def scenario_start_xy(scenario: dict):
    return point_xy(scenario.get("start", {}))


def scenario_goal_xy(scenario: dict):
    return point_xy(scenario.get("goal", {}))


def is_reference_path_scene(scenario: dict) -> bool:
    return bool(scenario.get("reference_path"))


REFERENCE_GOAL_MODES = {"waypoints", "final_only"}


def reference_goal_mode(scenario: dict):
    reference_path = scenario.get("reference_path")
    if not reference_path:
        return None
    goal_mode = reference_path.get("goal_mode", "waypoints")
    if not isinstance(goal_mode, str) or goal_mode not in REFERENCE_GOAL_MODES:
        raise ValueError(f"Unknown reference_path goal_mode: {goal_mode}")
    if goal_mode == "final_only" and reference_path.get("type") != "line":
        raise ValueError("reference_path goal_mode final_only requires type line")
    return goal_mode


def uses_reference_waypoints(scenario: dict) -> bool:
    return reference_goal_mode(scenario) == "waypoints"


def scenario_planner_v_max(scenario: dict) -> float:
    return float(scenario.get("planner_v_max", 1.5))


def write_reference_path_config(run_dir: Path, scenario: dict):
    waypoints = generate_waypoints(scenario["reference_path"])
    payload = {
        "waypoints": [[x, y] for x, y in waypoints],
        "threshold": 0.35,
        "start_delay": 3.0,
        "planner_goal_min_distance": float(
            scenario["reference_path"].get("planner_goal_min_distance", 1.2)
        ),
        "final_goal": list(waypoints[-1]),
    }
    path = run_dir / "reference_path.yaml"
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return path, waypoints


def write_run_meta(run_dir: Path, scenario_id: str, baseline_id: str, scenario: dict,
                   classes_arg: str, num_obs: int, duration_sec: int,
                   reference_waypoints=None, trial=None, resolved_baseline_id=None,
                   requested_baseline_label=None, protocol_id=DEFAULT_PROTOCOL_ID,
                   run_context=None, artifact_hashes=None, freeze_contract=None,
                   common_evaluation_contract=None) -> Path:
    meta_path = run_dir / "meta.yaml"
    canonical_meta_path = run_dir / "run_meta.yaml"
    start_x, start_y = scenario_start_xy(scenario)
    goal_x, goal_y = scenario_goal_xy(scenario)
    map_cfg = scenario.get("map", {})
    baseline = BASELINES[baseline_id]
    switches = scenario_switches(baseline_id, scenario)
    is_legacy_b1 = baseline_id == "B1_ACBF_fixed"
    log_profile = log_profile_for_baseline(baseline_id)
    profile_contract = LOG_PROFILES[log_profile]
    random_seed = trial["seed"] if trial is not None else 1
    beta_bar = None if is_legacy_b1 else scenario_beta_bar(baseline_id, scenario)
    beta_max = None if is_legacy_b1 else scenario_beta_max(baseline_id, scenario)
    meta = {
        "algorithm_version": ALGORITHM_VERSION,
        "formula_version": FORMULA_VERSION,
        "log_schema_version": LOG_SCHEMA_VERSION,
        "protocol_id": protocol_id,
        "log_profile": log_profile,
        "execution_tier": (
            freeze_contract["execution_tier"] if freeze_contract else "legacy_unfrozen"
        ),
        "campaign": freeze_contract["campaign"] if freeze_contract else "legacy",
        "execution_freeze": parameter_freeze_metadata(freeze_contract),
        "common_offline_evaluation_contract": common_evaluation_contract,
        "run_state": "running",
        "created_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "teacher_manuscript_sha256": TEACHER_MANUSCRIPT_SHA256,
        "teacher_formula_applicable": profile_contract["teacher_formula_applicable"],
        "method_algorithm_version": (
            "legacy_v1_acbf_frozen"
            if baseline_id == "B1_ACBF_fixed"
            else ALGORITHM_VERSION
        ),
        "safety_bound_applicable": baseline_id != "B1_ACBF_fixed",
        "experiment_id": (
            "Exp2_category_aware" if scenario_id.startswith("Exp2_")
            else "Exp3_context_modulation" if scenario_id.startswith("Exp3_")
            else "Exp1_teacher_main_simulation"
        ),
        "run_id": run_dir.name,
        "scenario": scenario_id,
        "scenario_description": scenario.get("description", ""),
        "scenario_family": scenario.get("scenario_family", ""),
        "context_level": scenario.get("context_level", ""),
        "paper_role": scenario.get("paper_role", scenario.get("expected_role", "")),
        "scene_source": scenario.get("scene_source", "teacher_canonical"),
        "method": baseline.get("experiment_label", baseline_id),
        "baseline_id": baseline_id,
        "resolved_baseline_id": resolved_baseline_id or baseline_id,
        "requested_baseline_label": requested_baseline_label or paper_baseline_label(baseline_id),
        "paper_baseline_label": requested_baseline_label or paper_baseline_label(baseline_id),
        "map_name": (
            "corridor_12m_x_6m" if scenario_id.startswith(("Exp2_", "Exp3_"))
            else "paper_style_supplementary_2d" if scenario_id.startswith(("avocado_", "drmpc_"))
            else "teacher_canonical_2d"
        ),
        "start": [start_x, start_y, 0.0],
        "goal": [goal_x, goal_y, 0.0],
        "map_size": [
            float(map_cfg.get("x", 50.0)),
            float(map_cfg.get("y", 50.0)),
            float(map_cfg.get("z", 3.0)),
        ],
        "robot_radius": 0.4,
        "obstacle_radius": 0.4,
        "obstacle_count": num_obs,
        "obstacle_ids": [4000 + index for index in range(num_obs)],
        "obstacle_classes": classes_arg,
        "obstacles": scenario.get("obstacles", []),
        "beta_table": (
            {"legacy_fixed_clearance_m": 0.30}
            if is_legacy_b1 else beta_bar
        ),
        "beta_bar_table": beta_bar,
        "beta_max_table": beta_max,
        "mu_weights": None if is_legacy_b1 else scenario_mu_weights(baseline_id, scenario),
        # guard_eta is retained as a compatibility alias; Teacher F06 uses
        # h_min and the nested contract records the notation ambiguity.
        "guard_eta": None if is_legacy_b1 else float(switches["guard_h_min"]),
        "guard_h_min": None if is_legacy_b1 else float(switches["guard_h_min"]),
        "delta_beta_positive": (
            None if is_legacy_b1
            else float(switches["delta_beta_positive"])
        ),
        "semantic_margin_contract": semantic_margin_contract(
            baseline_id, scenario, switches
        ),
        "typed_cycle_contract": typed_cycle_contract(
            baseline_id, switches, num_obs
        ),
        "guard_enable": False if is_legacy_b1 else baseline["guard_enabled"],
        "semantic_mode": switches["semantic_mode"],
        "enable_rate_limit": switches["enable_rate_limit"],
        "enable_available_projection": switches["enable_available_projection"],
        "enable_guard_fallback": switches["enable_guard_fallback"],
        "mpc_feasibility_guard_enabled": switches["mpc_feasibility_guard_enabled"],
        "fixed_beta": switches["fixed_beta"],
        "gamma": switches["gamma"],
        "epsilon_max": switches["epsilon_max"],
        "slack_weight": switches["slack_weight"],
        "qf_scale": switches["qf_scale"],
        "delta_u_weight": switches["delta_u_weight"],
        "delta_u_max": switches["delta_u_max"],
        "safety_delta_bar": switches["safety_delta_bar"],
        "safety_delta_beta_bar": switches["safety_delta_beta_bar"],
        "mpc_objective_contract": {
            "version": "teacher_v1_t3_objective_001",
            "qf_scale": float(switches["qf_scale"]),
            "delta_u_weight": float(switches["delta_u_weight"]),
            "delta_u_max": float(switches["delta_u_max"]),
            "first_input_reference": "[cur_state.vx,0]",
            "delta_u_constraint": "[-delta_u_max,delta_u_max]",
        },
        "safety_recurrence_contract": {
            "version": "teacher_v1_theorem1_telemetry_001",
            "gamma": float(switches["gamma"]),
            "epsilon_max": float(switches["epsilon_max"]),
            "delta_bar": float(switches["safety_delta_bar"]),
            "delta_beta_bar": float(switches["safety_delta_beta_bar"]),
            "log": "safety_recurrence_log.csv",
            "theorem1_applicable_policy": (
                "cbf_executed and no backup/no_cbf/safe_stop/baseline_infeasible and "
                "epsilon_t<=epsilon_max and delta<=delta_bar and "
                "delta_beta_plus<=delta_beta_bar"
            ),
        },
        "terminal_fallback_contract": {
            "action": switches["terminal_action"],
            "trigger": "final_constrained_attempt_infeasible",
            "planner_status": ("infeasible_emergency_cbf" if switches["terminal_action"] == "emergency_cbf" else "infeasible_safe_stop"),
            "final_status": ("backup" if switches["terminal_action"] == "emergency_cbf" else "infeasible"),
            "accepted_beta_source": switches["terminal_action"],
            "command": ("analytic_current_state_cbf" if switches["terminal_action"] == "emergency_cbf" else "[0,0]"),
            "shared_across_methods": True,
        },
        "max_cbf_obstacles": switches["max_cbf_obstacles"],
        "active_set_distance_m": switches["active_set_distance_m"],
        "graph_cache_enabled": switches["graph_cache_enabled"],
        "solver_max_cpu_time_ms": switches["solver_max_cpu_time_ms"],
        "guard_binary_search": ros_bool(switches["guard_binary_search"]),
        "cbf_metric": switches["cbf_metric"],
        "front_adsm": switches["front_adsm"],
        "global_seesm_enable": switches["global_seesm_enable"],
        "side_preference_enabled": switches["side_preference_enabled"],
        "side_weight": switches["side_weight"],
        "side_epsilon_n": switches["side_epsilon_n"],
        "side_horizon": switches["side_horizon"],
        "side_sign": switches["side_sign"],
        "side_min_obstacle_speed": switches["side_min_obstacle_speed"],
        "side_activation_distance": switches["side_activation_distance"],
        "dynamic_tau": dynamic_tau_contract(
            switches, baseline_beta_source(baseline_id)
        ),
        "guard_tau_policy": (
            "dynamic_tau_contract"
            if bool_switch(switches["dynamic_tau_enabled"])
            else "disabled_zero"
        ),
        "mpc_horizon": 20,
        "control_period_sec": 0.10,
        "prediction_step_sec": 0.20,
        "planner_v_max": scenario_planner_v_max(scenario),
        "random_seed": random_seed,
        "duration_sec": duration_sec,
        "trial_outcome_contract": {
            "goal_tolerance_m": GOAL_TOLERANCE_M,
            "deadlock_speed_threshold_mps": DEADLOCK_SPEED_MPS,
            "deadlock_hold_sec": DEADLOCK_HOLD_SEC,
            "deadlock_requires_final_goal_distance_gt_tolerance": True,
            "collision_priority_over_goal": True,
        },
        "safety_contract": (
            {
                "applicable": False,
                "method": "legacy_acbf_controller_type_4",
                "fixed_clearance_m": 0.30,
                "tau_scale": 0.30,
            }
            if is_legacy_b1 else {
                "applicable": True,
                "h_phys": "||l||-R_obs-R_robot",
                "h_eesm": "||l+tau*v_rel||-R_obs-R_robot",
                "h_seesm": "h_eesm-beta_i",
                "guard_upper_bound": (
                    "min(beta_max(c),beta_previous+delta_beta_plus,"
                    "max(h_eesm-h_min,0))"
                ),
                "fixed_margin_baseline": "beta_i = d_safe",
            }
        ),
        "required_logs": list(profile_contract["required_logs"]),
        "provenance": run_context or {},
        "artifact_hashes": artifact_hashes or {},
    }
    if is_reference_path_scene(scenario):
        waypoints = reference_waypoints
        if waypoints is None:
            waypoints = generate_waypoints(scenario["reference_path"])
        meta["reference_path"] = scenario["reference_path"]
        meta["reference_waypoints"] = [[x, y] for x, y in waypoints]
        meta["corridor_width"] = float(scenario.get("corridor_width", 1.5))
        meta["planner_goal_min_distance"] = float(
            scenario["reference_path"].get("planner_goal_min_distance", 1.2)
        )
    if trial is not None:
        meta["trial_manifest"] = {"trial_id": trial["trial_id"], "seed": trial["seed"], "scenario_id": trial["scenario_id"]}
        meta["perturbations"] = trial["rows"]
    serialized = yaml.safe_dump(
        meta, sort_keys=False, allow_unicode=True
    )
    # run_meta.yaml is canonical. meta.yaml is a byte-identical compatibility
    # mirror for existing analysis scripts.
    atomic_write_text(canonical_meta_path, serialized)
    atomic_write_text(meta_path, serialized)
    return meta_path


def load_yaml_mapping(path: Path) -> dict:
    if yaml is None:
        raise RuntimeError("PyYAML is required for Teacher-v1 metadata")
    with Path(path).open("r", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream)
    if not isinstance(payload, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return payload


def validate_semantic_margin_metadata(meta: dict, baseline_id: str,
                                      errors: list) -> None:
    semantic = meta.get("semantic_margin_contract")
    typed = meta.get("typed_cycle_contract")
    if baseline_id == "B1_ACBF_fixed":
        for field in (
            "beta_bar_table", "beta_max_table", "guard_h_min",
            "delta_beta_positive", "semantic_margin_contract",
            "typed_cycle_contract",
        ):
            if meta.get(field) is not None:
                errors.append(f"{field} must be not applicable for B1_ACBF_fixed")
        return

    if not isinstance(semantic, dict):
        errors.append("semantic_margin_contract must be a mapping")
        semantic = {}
    if semantic.get("version") != SEMANTIC_MARGIN_CONTRACT_VERSION:
        errors.append("semantic_margin_contract.version mismatch")
    if semantic.get("status") != "provisional":
        errors.append("semantic_margin_contract.status must be provisional")
    if semantic.get("formula_ids") != ["F05", "F06", "F07"]:
        errors.append("semantic_margin_contract.formula_ids mismatch")
    expected_applicable = (
        meta.get("cbf_metric") == "seesm" and meta.get("semantic_mode") != "none"
    )
    if semantic.get("applicable") is not expected_applicable:
        errors.append("semantic_margin_contract.applicable mismatch")

    beta_bar = semantic.get("beta_bar_m")
    beta_max = semantic.get("beta_max_m")
    if not isinstance(beta_bar, dict) or beta_bar != meta.get("beta_bar_table"):
        errors.append("semantic_margin_contract.beta_bar_m mismatch")
    if not isinstance(beta_max, dict) or beta_max != meta.get("beta_max_table"):
        errors.append("semantic_margin_contract.beta_max_m mismatch")
    if meta.get("beta_table") != meta.get("beta_bar_table"):
        errors.append("beta_table compatibility alias must equal beta_bar_table")
    for table_name, table in (("beta_bar_m", beta_bar), ("beta_max_m", beta_max)):
        if not isinstance(table, dict):
            continue
        for category, raw_value in table.items():
            try:
                value = float(raw_value)
            except (TypeError, ValueError):
                errors.append(
                    f"semantic_margin_contract.{table_name}.{category} must be numeric"
                )
                continue
            if not math.isfinite(value) or value < 0.0:
                errors.append(
                    f"semantic_margin_contract.{table_name}.{category} "
                    "must be finite and nonnegative"
                )

    phi = semantic.get("phi")
    expected_phi_formula = (
        "beta_tilde=beta_bar*clip(bias+head_on*f_head+"
        "ttc*TTC_norm+density*rho_norm,0,1)"
    )
    if not isinstance(phi, dict):
        errors.append("semantic_margin_contract.phi must be a mapping")
    else:
        if phi.get("status") != "provisional_missing_teacher_analytic_form":
            errors.append("semantic_margin_contract.phi.status mismatch")
        if phi.get("formula") != expected_phi_formula:
            errors.append("semantic_margin_contract.phi.formula mismatch")
        if phi.get("multiplier_clip") != [0.0, 1.0]:
            errors.append("semantic_margin_contract.phi.multiplier_clip mismatch")
        weights = phi.get("weights")
        source_weights = meta.get("mu_weights")
        expected_weights = None
        if isinstance(source_weights, dict):
            try:
                expected_weights = {
                    "bias": float(source_weights["bias"]),
                    "head_on": float(source_weights["heading"]),
                    "ttc": float(source_weights["ttc"]),
                    "density": float(source_weights["density"]),
                }
            except (KeyError, TypeError, ValueError):
                errors.append("mu_weights must contain finite numeric weights")
        if weights != expected_weights:
            errors.append("semantic_margin_contract.phi.weights mismatch")

    for contract_key, top_level_key, strict_positive in (
        ("h_min_m", "guard_h_min", False),
        ("delta_beta_positive_m_per_cycle", "delta_beta_positive", True),
    ):
        try:
            value = float(semantic.get(contract_key))
            top_value = float(meta.get(top_level_key))
        except (TypeError, ValueError):
            errors.append(f"semantic_margin_contract.{contract_key} must be numeric")
            continue
        invalid = value <= 0.0 if strict_positive else value < 0.0
        if not math.isfinite(value) or invalid or value != top_value:
            errors.append(f"semantic_margin_contract.{contract_key} mismatch")

    if semantic.get("upper_bound_formula") != (
        "min(beta_max(c),beta_previous+delta_beta_positive,"
        "max(h_eesm-h_min,0))"
    ):
        errors.append("semantic_margin_contract.upper_bound_formula mismatch")
    if semantic.get("pre_guard_formula") != "min(beta_tilde,beta_upper_bound)":
        errors.append("semantic_margin_contract.pre_guard_formula mismatch")
    expected_enforcement = None
    try:
        guard_enabled = strict_bool_value(meta.get("guard_enable"), "guard_enable")
        expected_enforcement = {
            "category_bound": guard_enabled,
            "positive_increment_bound": guard_enabled and strict_bool_value(
                meta.get("enable_rate_limit"), "enable_rate_limit"
            ),
            "available_margin_bound": guard_enabled and strict_bool_value(
                meta.get("enable_available_projection"),
                "enable_available_projection",
            ),
        }
    except (TypeError, ValueError):
        pass
    if semantic.get("enforcement") != expected_enforcement:
        errors.append("semantic_margin_contract.enforcement mismatch")
    for field, expected in (
        ("beta_previous_source", "same_obstacle_id_previous_final_mpc_accepted_feedback"),
        ("first_seen_beta_previous_m", 0.0),
        ("disappearance_policy", "erase_history_reappearance_is_rebirth_zero"),
        ("category_change_policy", "preserve_same_id_history_apply_current_category_cap"),
        ("no_cbf_history_policy", "no_cbf_or_safe_stop_do_not_commit"),
    ):
        if semantic.get(field) != expected:
            errors.append(f"semantic_margin_contract.{field} mismatch")

    if not isinstance(typed, dict):
        errors.append("typed_cycle_contract must be a mapping")
        return
    if typed.get("version") != TYPED_CYCLE_CONTRACT_VERSION:
        errors.append("typed_cycle_contract.version mismatch")
    if typed.get("applicable") is not (meta.get("cbf_metric") == "seesm"):
        errors.append("typed_cycle_contract.applicable mismatch")
    if typed.get("transport") != "typed_ros_messages":
        errors.append("typed_cycle_contract.transport mismatch")
    if typed.get("configured_obstacle_ids") != meta.get("obstacle_ids"):
        errors.append("typed_cycle_contract.configured_obstacle_ids mismatch")
    expected_endpoints = {
        "snapshot": {
            "topic": "/globalFsm_by_adsm/teacher_obstacle_snapshot",
            "message": "semantic_guard/PredictedObstacleArray",
            "cycle_field": "cycle_id",
        },
        "pre_guard": {
            "topic": "/safety_margin/beta_pre_guard",
            "message": "semantic_guard/PreGuardMarginArray",
            "cycle_field": "obstacle_cycle_id",
            "policy_fields": [
                "enforce_category_bound",
                "enforce_positive_increment_bound",
                "enforce_available_margin_bound",
            ],
        },
        "accepted_feedback": {
            "topic": "/safety_margin/beta_applied_final",
            "message": "semantic_guard/AppliedMarginArray",
            "cycle_field": "obstacle_cycle_id",
        },
    }
    for endpoint, expected_endpoint in expected_endpoints.items():
        value = typed.get(endpoint)
        if not isinstance(value, dict) or value != expected_endpoint:
            errors.append(f"typed_cycle_contract.{endpoint} mismatch")
    if typed.get("join_key") != ["obstacle_cycle_id", "obstacle_id"]:
        errors.append("typed_cycle_contract.join_key mismatch")
    if typed.get("id_set_policy") != "strict_unique_exact_match":
        errors.append("typed_cycle_contract.id_set_policy mismatch")
    if typed.get("stale_cycle_policy") != "reject_nonincreasing_or_unknown_cycle":
        errors.append("typed_cycle_contract.stale_cycle_policy mismatch")
    if typed.get("accepted_sources") != [
        "candidate", "previous", "zero", "kappa", "no_cbf", "safe_stop",
        "emergency_cbf",
        "mpc_reprojected",
    ]:
        errors.append("typed_cycle_contract.accepted_sources mismatch")
    if typed.get("history_commit_policy") != "accepted_feedback_except_no_cbf_or_safe_stop":
        errors.append("typed_cycle_contract.history_commit_policy mismatch")


def validate_t3_objective_metadata(meta: dict, baseline_id: str,
                                   errors: list) -> None:
    """Validate the provenance-visible Teacher-v1 T3 objective contract."""
    if baseline_id == "B1_ACBF_fixed":
        # The legacy ACBF launch does not instantiate MPC-SECBF.  Its
        # compatibility metadata may still carry defaults, but no T3 contract
        # is applicable to that controller.
        return
    expected = {
        "version": "teacher_v1_t3_objective_001",
        "first_input_reference": "[cur_state.vx,0]",
        "delta_u_constraint": "[-delta_u_max,delta_u_max]",
    }
    contract = meta.get("mpc_objective_contract")
    if not isinstance(contract, dict):
        errors.append("mpc_objective_contract must be a mapping")
        return
    for field, value in expected.items():
        if contract.get(field) != value:
            errors.append(f"mpc_objective_contract.{field} mismatch")

    for field, lower, strict in (
        ("qf_scale", 0.0, True),
        ("delta_u_weight", 0.0, False),
        ("delta_u_max", 0.0, True),
    ):
        try:
            top_value = float(meta.get(field))
            contract_value = float(contract.get(field))
        except (TypeError, ValueError):
            errors.append(f"T3 {field} must be numeric")
            continue
        invalid = top_value <= lower if strict else top_value < lower
        if (not math.isfinite(top_value) or invalid or
                not math.isfinite(contract_value) or
                contract_value != top_value):
            errors.append(f"T3 {field} mismatch or invalid")


def validate_safety_recurrence_metadata(meta: dict, baseline_id: str,
                                        errors: list) -> None:
    """Keep Theorem-1 telemetry bounds and applicability policy provenance-visible."""
    if baseline_id == "B1_ACBF_fixed":
        return
    contract = meta.get("safety_recurrence_contract")
    if not isinstance(contract, dict):
        errors.append("safety_recurrence_contract must be a mapping")
        return
    if contract.get("version") != "teacher_v1_theorem1_telemetry_001":
        errors.append("safety_recurrence_contract.version mismatch")
    if contract.get("log") != "safety_recurrence_log.csv":
        errors.append("safety_recurrence_contract.log mismatch")
    expected_policy = (
        "cbf_executed and no backup/no_cbf/safe_stop/baseline_infeasible and "
        "epsilon_t<=epsilon_max and delta<=delta_bar and "
        "delta_beta_plus<=delta_beta_bar"
    )
    if contract.get("theorem1_applicable_policy") != expected_policy:
        errors.append("safety_recurrence_contract.theorem1_applicable_policy mismatch")
    for field, top_field, lower, strict in (
        ("gamma", "gamma", 0.0, True),
        ("epsilon_max", "epsilon_max", 0.0, False),
        ("delta_bar", "safety_delta_bar", 0.0, False),
        ("delta_beta_bar", "safety_delta_beta_bar", 0.0, False),
    ):
        try:
            top_value = float(meta[top_field])
            contract_value = float(contract[field])
        except (KeyError, TypeError, ValueError):
            errors.append(f"safety recurrence {field} must be numeric")
            continue
        invalid = top_value <= lower if strict else top_value < lower
        if (not math.isfinite(top_value) or invalid or
                not math.isfinite(contract_value) or
                abs(top_value - contract_value) > 1.0e-12):
            errors.append(f"safety recurrence {field} mismatch or invalid")


def validate_terminal_fallback_metadata(meta: dict, baseline_id: str,
                                        errors: list) -> None:
    """Require the same final infeasibility action for every comparison arm."""
    if baseline_id == "B1_ACBF_fixed":
        return
    contract = meta.get("terminal_fallback_contract")
    if not isinstance(contract, dict):
        errors.append("terminal_fallback_contract must be a mapping")
        return
    if contract.get("action") == "emergency_cbf":
        expected = {
            "action": "emergency_cbf",
            "trigger": "final_constrained_attempt_infeasible",
            "planner_status": "infeasible_emergency_cbf",
            "final_status": "backup",
            "accepted_beta_source": "emergency_cbf",
            "command": "analytic_current_state_cbf",
            "shared_across_methods": True,
        }
    else:
        expected = {
            "action": "safe_stop",
            "trigger": "final_constrained_attempt_infeasible",
            "planner_status": "infeasible_safe_stop",
            "final_status": "infeasible",
            "accepted_beta_source": "safe_stop",
            "command": "[0,0]",
            "shared_across_methods": True,
        }
    for field, value in expected.items():
        if contract.get(field) != value:
            errors.append(f"terminal_fallback_contract.{field} mismatch")


def validate_execution_freeze_metadata(meta: dict, errors: list) -> None:
    """Verify that the metadata names a selected, still-identical freeze."""
    tier = meta.get("execution_tier")
    record = meta.get("execution_freeze")
    if tier == "legacy_unfrozen":
        if record is not None:
            errors.append("legacy_unfrozen execution must not carry execution_freeze")
        return
    if tier not in {"smoke", "formal"}:
        errors.append("execution_tier must be smoke, formal, or legacy_unfrozen")
        return
    if not isinstance(record, dict):
        errors.append("execution_freeze must be a mapping")
        return
    try:
        campaign = str(record.get("campaign", "")).strip()
        loaded = load_parameter_freeze(
            Path(record.get("path", "")), tier, campaign=campaign
        )
    except (OSError, TypeError, ValueError) as exc:
        errors.append(f"execution_freeze invalid: {exc}")
        return
    for field in (
        "id", "status", "execution_tier", "campaign", "expected_trials",
        "path", "sha256", "phi_status",
    ):
        if record.get(field) != parameter_freeze_metadata(loaded).get(field):
            errors.append(f"execution_freeze.{field} mismatch")


def validate_common_offline_evaluation_metadata(meta: dict, errors: list) -> None:
    record = meta.get("common_offline_evaluation_contract")
    if meta.get("execution_tier") == "legacy_unfrozen" and record is None:
        return
    if not isinstance(record, dict):
        errors.append("common_offline_evaluation_contract must be a mapping")
        return
    try:
        loaded = load_common_offline_evaluation_contract(Path(record.get("path", "")))
    except (OSError, TypeError, ValueError) as exc:
        errors.append(f"common_offline_evaluation_contract invalid: {exc}")
        return
    for field in ("id", "version", "status", "path", "sha256"):
        if record.get(field) != loaded.get(field):
            errors.append(f"common_offline_evaluation_contract.{field} mismatch")
    if meta.get("execution_tier") == "formal" and record.get("status") != "formal_frozen":
        errors.append("formal execution requires formal_frozen common offline evaluation")


def validate_run_meta_contract(run_dir: Path, strict_provenance=True):
    errors = []
    run_dir = Path(run_dir)
    meta_path = run_dir / "meta.yaml"
    canonical_path = run_dir / "run_meta.yaml"
    if not meta_path.exists():
        errors.append("missing meta.yaml")
    if not canonical_path.exists():
        errors.append("missing run_meta.yaml")
    if errors:
        return False, errors
    try:
        meta_bytes = meta_path.read_bytes()
        canonical_bytes = canonical_path.read_bytes()
        if meta_bytes != canonical_bytes:
            errors.append("meta.yaml is not byte-identical to run_meta.yaml")
        meta = load_yaml_mapping(canonical_path)
    except (OSError, RuntimeError, TypeError, ValueError) as exc:
        return False, [f"invalid run metadata: {exc}"]
    except yaml.YAMLError as exc:
        return False, [f"invalid run metadata YAML: {exc}"]

    expected_values = {
        "algorithm_version": ALGORITHM_VERSION,
        "formula_version": FORMULA_VERSION,
        "log_schema_version": LOG_SCHEMA_VERSION,
        "teacher_manuscript_sha256": TEACHER_MANUSCRIPT_SHA256,
    }
    for key, expected in expected_values.items():
        if meta.get(key) != expected:
            errors.append(f"{key} must be {expected!r}")
    protocol_id = str(meta.get("protocol_id", "")).strip()
    if not protocol_id:
        errors.append("protocol_id must be nonempty")
    if meta.get("run_state") not in {"running", "complete", "invalid"}:
        errors.append("run_state must be running, complete, or invalid")

    baseline_id = str(meta.get("baseline_id", ""))
    if baseline_id not in BASELINES:
        expected_profile = ""
        errors.append(f"unknown baseline_id in metadata: {baseline_id!r}")
    else:
        expected_profile = log_profile_for_baseline(baseline_id)
    if meta.get("log_profile") != expected_profile:
        errors.append(
            f"log_profile {meta.get('log_profile')!r} does not match baseline "
            f"profile {expected_profile!r}"
        )
    if expected_profile:
        expected_logs = list(LOG_PROFILES[expected_profile]["required_logs"])
        if meta.get("required_logs") != expected_logs:
            errors.append("required_logs does not match the code-controlled log profile")
        expected_formula_applicable = LOG_PROFILES[expected_profile][
            "teacher_formula_applicable"
        ]
        if meta.get("teacher_formula_applicable") is not expected_formula_applicable:
            errors.append("teacher_formula_applicable does not match log profile")

    validate_semantic_margin_metadata(meta, baseline_id, errors)
    validate_t3_objective_metadata(meta, baseline_id, errors)
    validate_safety_recurrence_metadata(meta, baseline_id, errors)
    validate_terminal_fallback_metadata(meta, baseline_id, errors)
    validate_execution_freeze_metadata(meta, errors)
    validate_common_offline_evaluation_metadata(meta, errors)

    # T3 objective values are provenance-bearing parameters, not comments.
    objective = meta.get("mpc_objective_contract")
    if not isinstance(objective, dict):
        errors.append("mpc_objective_contract must be a mapping")
    else:
        if objective.get("version") != "teacher_v1_t3_objective_001":
            errors.append("mpc_objective_contract.version mismatch")
        for field in ("qf_scale", "delta_u_weight", "delta_u_max"):
            try:
                top_value = float(meta[field])
                contract_value = float(objective[field])
            except (KeyError, TypeError, ValueError):
                errors.append(f"mpc_objective_contract.{field} must be numeric")
                continue
            if (not math.isfinite(top_value) or not math.isfinite(contract_value) or
                    top_value <= 0.0 or
                    (field == "delta_u_weight" and top_value < 0.0) or
                    abs(top_value - contract_value) > 1.0e-12):
                errors.append(f"mpc_objective_contract.{field} mismatch or invalid")
        if objective.get("first_input_reference") != "[cur_state.vx,0]":
            errors.append("mpc_objective_contract.first_input_reference mismatch")
        if objective.get("delta_u_constraint") != "[-delta_u_max,delta_u_max]":
            errors.append("mpc_objective_contract.delta_u_constraint mismatch")

    def require_meta_bool(field, expected):
        try:
            actual = strict_bool_value(meta.get(field), field)
        except (TypeError, ValueError):
            errors.append(f"{field} must be a boolean")
            return
        if actual is not expected:
            errors.append(f"{field} must be {str(expected).lower()} for {baseline_id}")

    if baseline_id == "B1_ACBF_fixed":
        expected_b1_values = {
            "semantic_mode": "not_applicable",
            "cbf_metric": "legacy_acbf",
            "fixed_beta": 0.30,
            "front_adsm": "true",
        }
        for field, expected in expected_b1_values.items():
            if meta.get(field) != expected:
                errors.append(f"{field} must be {expected!r} for B1_ACBF_fixed")
        for field in (
            "guard_enable", "enable_rate_limit", "enable_available_projection",
            "enable_guard_fallback", "mpc_feasibility_guard_enabled",
            "global_seesm_enable", "side_preference_enabled",
        ):
            require_meta_bool(field, False)
        if meta.get("mu_weights") is not None or meta.get("guard_eta") is not None:
            errors.append("B1 semantic Guard metadata must be not applicable")
        safety_contract = meta.get("safety_contract", {})
        if not isinstance(safety_contract, dict) or safety_contract.get("applicable") is not False:
            errors.append("B1 safety_contract must declare applicable=false")
    elif baseline_id == "Standard_MPC_CBF":
        if meta.get("semantic_mode") != "fixed" or meta.get("cbf_metric") != "distance":
            errors.append("Standard_MPC_CBF must use fixed/distance metadata")
        for field in (
            "guard_enable", "enable_rate_limit", "enable_available_projection",
            "enable_guard_fallback", "mpc_feasibility_guard_enabled", "front_adsm",
            "global_seesm_enable", "side_preference_enabled",
        ):
            require_meta_bool(field, False)
        dynamic_meta = meta.get("dynamic_tau", {})
        if not isinstance(dynamic_meta, dict) or dynamic_meta.get("enabled") is not False:
            errors.append("Standard_MPC_CBF must disable dynamic_tau")
    elif expected_profile == "teacher_seesm_v1":
        if meta.get("cbf_metric") != "seesm":
            errors.append("Teacher SEESM profile must use cbf_metric='seesm'")
        dynamic_meta = meta.get("dynamic_tau", {})
        if not isinstance(dynamic_meta, dict) or dynamic_meta.get("enabled") is not True:
            errors.append("Teacher SEESM profile must enable dynamic_tau")

    try:
        ensure_teacher_output_root(run_dir)
    except ValueError as exc:
        if strict_provenance:
            errors.append(str(exc))

    provenance = meta.get("provenance")
    if strict_provenance:
        if not isinstance(provenance, dict):
            errors.append("provenance must be a mapping")
        else:
            repositories = provenance.get("repositories")
            inputs = provenance.get("inputs")
            if not isinstance(repositories, dict):
                errors.append("provenance.repositories must be a mapping")
            else:
                for repo_name in ("panjian_ws", "seesm_social_navigation"):
                    source = repositories.get(repo_name)
                    if not isinstance(source, dict):
                        errors.append(f"missing provenance repository {repo_name}")
                        continue
                    if not str(source.get("commit", "")).strip():
                        errors.append(f"{repo_name} provenance is missing commit")
                    if not str(source.get("tree", "")).strip():
                        errors.append(f"{repo_name} provenance is missing tree")
                    if source.get("branch") != "teacher-v1":
                        errors.append(f"{repo_name} provenance branch is not teacher-v1")
                    if (meta.get("execution_tier") != "smoke" and
                            source.get("dirty") is not False):
                        errors.append(f"{repo_name} provenance must be clean")
                    if not str(source.get("working_tree_state_sha256", "")).strip():
                        errors.append(f"{repo_name} provenance is missing state hash")
            if not isinstance(inputs, dict):
                errors.append("provenance.inputs must be a mapping")
            else:
                manuscript = inputs.get("teacher_manuscript", {})
                if manuscript.get("sha256") != TEACHER_MANUSCRIPT_SHA256:
                    errors.append("provenance teacher manuscript hash mismatch")
                for input_name in (
                    "parameter_freeze", "scenario_config", "secbf_planner_launch",
                    "acbf0_planner_launch", "start_test_launch",
                ):
                    record = inputs.get(input_name)
                    if not isinstance(record, dict) or not str(record.get("sha256", "")):
                        errors.append(f"missing provenance input hash {input_name}")
                if meta.get("execution_tier") in {"smoke", "formal"}:
                    evaluator = inputs.get("common_offline_evaluation")
                    if (not isinstance(evaluator, dict) or
                            not str(evaluator.get("sha256", "")).strip()):
                        errors.append("missing provenance input hash common_offline_evaluation")
                    freeze = meta.get("execution_freeze") or {}
                    freeze_input = inputs.get("parameter_freeze") or {}
                    if freeze.get("sha256") != freeze_input.get("sha256"):
                        errors.append("provenance parameter_freeze hash mismatch")
                    common = meta.get("common_offline_evaluation_contract") or {}
                    if common.get("sha256") != evaluator.get("sha256"):
                        errors.append("provenance common_offline_evaluation hash mismatch")

            ros_provenance = provenance.get("ros")
            if not isinstance(ros_provenance, dict):
                errors.append("provenance.ros must be a mapping")
            else:
                package_paths = ros_provenance.get("package_paths")
                if not isinstance(package_paths, dict):
                    errors.append("provenance.ros.package_paths must be a mapping")
                else:
                    for package, relative_path in EXPECTED_ROS_PACKAGE_PATHS.items():
                        expected_path = str(
                            (repo_root() / relative_path).resolve(strict=True)
                        )
                        if package_paths.get(package) != expected_path:
                            errors.append(
                                f"provenance ROS package path mismatch for {package}"
                            )

            build_environment = provenance.get("build_environment")
            if not isinstance(build_environment, dict):
                errors.append("provenance.build_environment must be a mapping")
            else:
                for tool in ("ros_distribution", "cmake", "compiler", "python"):
                    if not str(build_environment.get(tool, "")).strip():
                        errors.append(
                            f"provenance.build_environment.{tool} is required"
                        )

            runtime = provenance.get("runtime")
            if not isinstance(runtime, dict):
                errors.append("provenance.runtime must be a mapping")
            else:
                if runtime.get("baseline_id") != baseline_id:
                    errors.append("provenance.runtime baseline_id mismatch")
                records = runtime.get("executables")
                expected_labels = (
                    (
                        "legacy_acbf_mpc", "global_planner", "obstacle_manager",
                        "phase5_logger",
                    )
                    if baseline_id == "B1_ACBF_fixed"
                    else (
                        "teacher_mpc", "teacher_guard_ground_truth",
                        "global_planner", "obstacle_manager", "phase5_logger",
                    )
                )
                if not isinstance(records, dict):
                    errors.append("provenance.runtime.executables must be a mapping")
                else:
                    for label in expected_labels:
                        record = records.get(label)
                        if not isinstance(record, dict):
                            errors.append(f"missing runtime executable {label}")
                            continue
                        if not str(record.get("path", "")).strip():
                            errors.append(f"runtime executable {label} has no path")
                        digest = str(record.get("sha256", ""))
                        if len(digest) != 64:
                            errors.append(f"runtime executable {label} has invalid hash")
                        if not str(record.get("package_source_path", "")).strip():
                            errors.append(
                                f"runtime executable {label} has no package source path"
                            )
                        if (
                            not record.get("executes_source_directly")
                            and not str(record.get("build_prefix", "")).strip()
                        ):
                            errors.append(
                                f"compiled runtime executable {label} has no build prefix"
                            )

        artifacts = meta.get("artifact_hashes")
        if not isinstance(artifacts, dict) or not artifacts.get("obstacles_param.yaml"):
            errors.append("artifact_hashes.obstacles_param.yaml is required")
        elif isinstance(artifacts, dict):
            for artifact_name, expected_hash in artifacts.items():
                artifact_path = run_dir / artifact_name
                if not artifact_path.exists():
                    errors.append(f"missing hashed trial artifact {artifact_name}")
                elif sha256_file(artifact_path) != expected_hash:
                    errors.append(f"trial artifact hash mismatch: {artifact_name}")
    return not errors, errors


def update_run_meta_state(run_dir: Path, state: str, validation: dict) -> None:
    if state not in {"complete", "invalid"}:
        raise ValueError(f"invalid final run state: {state}")
    run_dir = Path(run_dir)
    meta = load_yaml_mapping(run_dir / "run_meta.yaml")
    meta["run_state"] = state
    meta["finalized_at"] = dt.datetime.now(dt.timezone.utc).isoformat()
    meta["validation"] = validation
    serialized = yaml.safe_dump(meta, sort_keys=False, allow_unicode=True)
    atomic_write_text(run_dir / "run_meta.yaml", serialized)
    atomic_write_text(run_dir / "meta.yaml", serialized)


def parse_sentinel(path: Path) -> dict:
    values = {}
    try:
        for line in Path(path).read_text(encoding="utf-8").splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                values[key.strip()] = value.strip()
    except OSError:
        return {}
    return values


def write_integrity_manifest(run_dir: Path) -> Path:
    run_dir = Path(run_dir)
    meta = load_yaml_mapping(run_dir / "run_meta.yaml")
    names = list(meta.get("required_logs", []))
    names.extend(meta.get("artifact_hashes", {}).keys())
    names.extend(
        [
            "run_meta.yaml", "meta.yaml", "summary.csv", "summary.md",
            "csv_contract_check.txt", "process_status.yaml",
            "verify_safety_bound.txt",
        ]
    )
    unique_names = []
    for name in names:
        name = str(name)
        if name not in unique_names and (run_dir / name).exists():
            unique_names.append(name)
    lines = [f"{sha256_file(run_dir / name)}  {name}" for name in unique_names]
    manifest = run_dir / "trial_integrity.sha256"
    atomic_write_text(manifest, "\n".join(lines) + "\n")
    return manifest


def verify_integrity_manifest(run_dir: Path) -> bool:
    run_dir = Path(run_dir)
    manifest = run_dir / "trial_integrity.sha256"
    if not manifest.exists() or manifest.stat().st_size <= 0:
        return False
    try:
        for line in manifest.read_text(encoding="utf-8").splitlines():
            expected, separator, relative_name = line.partition("  ")
            if not separator or not expected or not relative_name:
                return False
            relative = Path(relative_name)
            if relative.is_absolute() or ".." in relative.parts:
                return False
            target = run_dir / relative
            if not target.is_file() or sha256_file(target) != expected:
                return False
    except OSError:
        return False
    return True


def is_complete_run_dir(run_dir: Path) -> bool:
    run_dir = Path(run_dir)
    complete = run_dir / RUN_COMPLETE_SENTINEL
    invalid = run_dir / RUN_INVALID_SENTINEL
    if not complete.exists() or complete.stat().st_size <= 0 or invalid.exists():
        return False
    if not (run_dir / "summary.csv").exists() or not (run_dir / "run_meta.yaml").exists():
        return False
    sentinel = parse_sentinel(complete)
    expected_hash = sentinel.get("run_meta_sha256", "")
    if sentinel.get("trial_valid") != "true":
        return False
    if not expected_hash or expected_hash != sha256_file(run_dir / "run_meta.yaml"):
        return False
    manifest_hash = sentinel.get("integrity_manifest_sha256", "")
    manifest_path = run_dir / "trial_integrity.sha256"
    if (
        not manifest_hash
        or not manifest_path.exists()
        or sha256_file(manifest_path) != manifest_hash
        or not verify_integrity_manifest(run_dir)
    ):
        return False
    try:
        meta = load_yaml_mapping(run_dir / "run_meta.yaml")
    except (OSError, RuntimeError, TypeError, ValueError):
        return False
    if meta.get("run_state") != "complete":
        return False
    meta_ok, _ = validate_run_meta_contract(run_dir, strict_provenance=True)
    logs_ok, _ = audit_required_logs(run_dir, str(meta.get("baseline_id", "")))
    return meta_ok and logs_ok


def summary_declares_valid(run_dir: Path) -> tuple:
    path = Path(run_dir) / "summary.csv"
    if not path.exists():
        return False, ["missing summary.csv"]
    try:
        with path.open("r", newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
    except (OSError, csv.Error) as exc:
        return False, [f"invalid summary.csv: {exc}"]
    if len(rows) != 1:
        return False, ["summary.csv must contain exactly one data row"]
    row = rows[0]
    if str(row.get("trial_valid", "")).strip().lower() not in {
        "1", "true", "yes",
    }:
        return False, ["summary.csv does not declare trial_valid=true"]
    if not str(row.get("termination_reason", "")).strip():
        return False, ["summary.csv has no termination_reason"]
    return True, []


def validate_process_status(run_dir: Path) -> tuple:
    path = Path(run_dir) / "process_status.yaml"
    if not path.exists():
        return False, ["missing process_status.yaml"]
    try:
        status = load_yaml_mapping(path)
    except (OSError, RuntimeError, TypeError, ValueError, yaml.YAMLError) as exc:
        return False, [f"invalid process_status.yaml: {exc}"]
    errors = []
    if status.get("process_contract_passed") is not True:
        errors.append("process_status.yaml does not declare process_contract_passed=true")
    if status.get("unexpected_exits") not in ([], None):
        errors.append("process_status.yaml contains unexpected process exits")
    return not errors, errors


def seal_trial(run_dir: Path, valid: bool, validation: dict) -> Path:
    run_dir = Path(run_dir)
    validation = dict(validation)
    state = "complete" if valid else "invalid"
    update_run_meta_state(run_dir, state, validation)
    finalization_errors = []
    if valid:
        meta = load_yaml_mapping(run_dir / "run_meta.yaml")
        baseline_id = str(meta.get("baseline_id", ""))
        meta_ok, meta_errors = validate_run_meta_contract(
            run_dir, strict_provenance=True
        )
        logs_ok, log_errors = audit_required_logs(run_dir, baseline_id)
        csv_ok, _ = run_csv_contract_check(run_dir)
        summary_ok, summary_errors = summary_declares_valid(run_dir)
        process_ok, process_errors = validate_process_status(run_dir)
        finalization_errors.extend(meta_errors)
        finalization_errors.extend(log_errors)
        finalization_errors.extend(summary_errors)
        finalization_errors.extend(process_errors)
        if validation.get("trial_valid") is not True:
            finalization_errors.append("validation.trial_valid is not true")
        if validation.get("metadata_contract_passed") is not True or not meta_ok:
            finalization_errors.append("metadata contract did not pass")
        if validation.get("csv_contract_passed") is not True or not csv_ok:
            finalization_errors.append("CSV contract did not pass finalization")
        if validation.get("process_contract_passed") is not True or not process_ok:
            finalization_errors.append("process contract did not pass finalization")
        if not logs_ok:
            finalization_errors.append("required-log audit did not pass finalization")
        if not summary_ok:
            finalization_errors.append("summary audit did not pass finalization")
        finalization_errors = list(dict.fromkeys(finalization_errors))
        if finalization_errors:
            valid = False
            state = "invalid"
            validation["trial_valid"] = False
            validation["metadata_contract_passed"] = bool(meta_ok)
            validation["csv_contract_passed"] = bool(csv_ok)
            validation["process_contract_passed"] = bool(process_ok)
            validation["finalization_errors"] = finalization_errors
            validation["invalid_reasons"] = list(dict.fromkeys(
                list(validation.get("invalid_reasons", []))
                + finalization_errors
            ))
            update_run_meta_state(run_dir, state, validation)
    integrity_manifest = write_integrity_manifest(run_dir) if valid else None
    sentinel_name = RUN_COMPLETE_SENTINEL if valid else RUN_INVALID_SENTINEL
    opposite_name = RUN_INVALID_SENTINEL if valid else RUN_COMPLETE_SENTINEL
    if (run_dir / opposite_name).exists():
        raise RuntimeError(f"refusing to create dual trial sentinels in {run_dir}")
    reasons = validation.get("invalid_reasons", [])
    lines = [
        "TEACHER_V1_RUN_COMPLETE" if valid else "TEACHER_V1_RUN_INVALID",
        f"validated_at={dt.datetime.now(dt.timezone.utc).isoformat()}",
        f"trial_valid={'true' if valid else 'false'}",
        f"termination_reason={validation.get('termination_reason', '')}",
        f"run_meta_sha256={sha256_file(run_dir / 'run_meta.yaml')}",
        "integrity_manifest_sha256=" + (
            sha256_file(integrity_manifest) if integrity_manifest is not None else ""
        ),
        f"invalid_reasons={';'.join(str(reason) for reason in reasons)}",
    ]
    sentinel_path = run_dir / sentinel_name
    atomic_write_text(sentinel_path, "\n".join(lines) + "\n")
    return sentinel_path


def csv_has_data_rows(path: Path) -> bool:
    path = Path(path)
    if not path.exists() or path.stat().st_size <= 1:
        return False
    with path.open("r", newline="", encoding="utf-8", errors="replace") as stream:
        rows = [row for row in csv.reader(stream) if any(cell.strip() for cell in row)]
    if path.name.startswith("data_processor_"):
        return bool(rows)
    return len(rows) >= 2


def all_cycles_explicit_infeasible_safe_stop(run_dir: Path) -> bool:
    """Accept empty stage telemetry only with a complete safe-stop proof."""
    path = Path(run_dir) / "planner_log.csv"
    if not path.exists():
        return False
    try:
        with path.open(
            "r", newline="", encoding="utf-8", errors="replace"
        ) as stream:
            rows = list(csv.DictReader(stream))
        return bool(rows) and all(
            row.get("mpc_status") == "infeasible_safe_stop"
            and row.get("first_attempt_status") == "infeasible"
            and row.get("final_status") == "infeasible"
            and row.get("accepted_beta_source") == "safe_stop"
            and str(row.get("candidate_feasibility_checked", "")).strip().lower()
            in {"1", "true", "yes"}
            and abs(float(row.get("cmd_v", "nan"))) <= 1.0e-12
            and abs(float(row.get("cmd_w", "nan"))) <= 1.0e-12
            and row.get("tau_reason") == "stage_audit_unavailable"
            for row in rows
        )
    except (OSError, csv.Error, TypeError, ValueError):
        return False


def audit_required_logs(run_dir: Path, baseline_id: str):
    profile = LOG_PROFILES[log_profile_for_baseline(baseline_id)]
    run_dir = Path(run_dir)
    errors = []
    for name in profile["required_logs"]:
        path = run_dir / name
        if not path.exists() or path.stat().st_size <= 1:
            errors.append(f"missing or empty required log: {name}")
    explicit_infeasible_safe_stop = all_cycles_explicit_infeasible_safe_stop(
        run_dir
    )
    safe_stop_empty_logs = {
        "tau_stage_log.csv", "safety_recurrence_log.csv",
    }
    for name in profile["required_data_rows"]:
        if not csv_has_data_rows(run_dir / name):
            if explicit_infeasible_safe_stop and name in safe_stop_empty_logs:
                continue
            errors.append(f"required log has no data rows: {name}")

    def dict_rows(name):
        path = run_dir / name
        if not path.exists():
            return []
        try:
            with path.open("r", newline="", encoding="utf-8", errors="replace") as stream:
                return list(csv.DictReader(stream))
        except (OSError, csv.Error):
            return []

    meta_path = run_dir / "run_meta.yaml"
    if meta_path.exists():
        event_rows = dict_rows("event_log.csv")
        events = {str(row.get("event", "")).strip() for row in event_rows}
        if not {"start", "stop"}.issubset(events):
            errors.append("event_log.csv must contain start and stop events")

    if baseline_id == "B1_ACBF_fixed":
        if len(dict_rows("robot_log.csv")) < 2:
            errors.append("B1 robot_log.csv requires at least two samples")
        obstacle_rows = dict_rows("obstacle_log.csv")
        if len(obstacle_rows) < 2:
            errors.append("B1 obstacle_log.csv requires at least two samples")
        meaningful_obstacle_rows = 0
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
                meaningful_obstacle_rows += 1
        if meaningful_obstacle_rows == 0:
            errors.append("B1 obstacle_log.csv has only all-zero startup rows")

    if meta_path.exists():
        try:
            meta = load_yaml_mapping(meta_path)
        except (OSError, RuntimeError, TypeError, ValueError, yaml.YAMLError) as exc:
            errors.append(f"cannot audit log coverage without valid metadata: {exc}")
            meta = {}
        try:
            global_seesm_enabled = strict_bool_value(
                meta.get("global_seesm_enable", False),
                "global_seesm_enable",
            )
        except (TypeError, ValueError):
            global_seesm_enabled = None
            errors.append("global_seesm_enable metadata is not boolean")
        global_log_has_rows = csv_has_data_rows(
            run_dir / "global_seesm_log.csv"
        )
        if global_seesm_enabled is True and not global_log_has_rows:
            errors.append(
                "global_seesm_enable=true requires global_seesm_log.csv data rows"
            )
        if global_seesm_enabled is False and global_log_has_rows:
            errors.append(
                "global_seesm_enable=false requires global_seesm_log.csv "
                "to contain only its header"
            )

        try:
            duration_sec = float(meta.get("duration_sec", 0.0))
        except (TypeError, ValueError):
            duration_sec = 0.0
        minimum_span = max(0.5, duration_sec * 0.5)
        coverage_logs = ["robot_log.csv", "obstacle_log.csv"]
        if baseline_id != "B1_ACBF_fixed":
            coverage_logs.append("planner_log.csv")
        for name in coverage_logs:
            timestamps = []
            for row in dict_rows(name):
                try:
                    value = float(row.get("t", row.get("time", "")))
                except (TypeError, ValueError):
                    continue
                if math.isfinite(value):
                    timestamps.append(value)
            if len(timestamps) < 2:
                errors.append(f"{name} has fewer than two timestamped samples")
            elif max(timestamps) - min(timestamps) < minimum_span:
                errors.append(
                    f"{name} covers less than 50% of duration_sec"
                )
    return not errors, errors


def run_csv_contract_check(run_dir: Path):
    command = [
        sys.executable,
        str(repo_root() / "swarm_test/scripts/check_experiment_csv_fields.py"),
        "--require-teacher-meta",
        str(run_dir),
    ]
    result = subprocess.run(
        command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        check=False,
    )
    atomic_write_text(Path(run_dir) / "csv_contract_check.txt", result.stdout)
    return result.returncode == 0, "csv_contract_check.txt"


def obstacle_classes(obstacles: list) -> str:
    classes = [obs.get("semantic_class", "unknown") for obs in obstacles]
    return "[" + ",".join(classes) + "]"


def ground_truth_obstacle_ids(num_obstacles: int) -> str:
    if num_obstacles < 0:
        raise ValueError("num_obstacles must be nonnegative")
    return "[" + ",".join(str(4000 + index) for index in range(num_obstacles)) + "]"


def build_commands(scenario_id: str, baseline_id: str, run_dir: Path, obstacle_params: Path,
                   classes_arg: str, num_obs: int, scenario: dict,
                   global_path_ready_gate: bool = False,
                   global_path_start_gate_topic: str = "/teacher_v1/global_path_ready"):
    baseline = BASELINES[baseline_id]
    start_x, start_y = scenario_start_xy(scenario)
    goal_x, goal_y = scenario_goal_xy(scenario)
    map_cfg = scenario.get("map", {})
    beta_bar = scenario_beta_bar(baseline_id, scenario)
    beta_max = scenario_beta_max(baseline_id, scenario)
    mu_weights = scenario_mu_weights(baseline_id, scenario)
    switches = scenario_switches(baseline_id, scenario)
    obstacle_ids_arg = ground_truth_obstacle_ids(num_obs)
    reference_path = is_reference_path_scene(scenario)
    use_reference_waypoints = uses_reference_waypoints(scenario)
    reference_waypoints = generate_waypoints(scenario["reference_path"]) if reference_path else None
    init_yaw = initial_yaw(reference_waypoints) if reference_waypoints else 0.0
    reference_path_file = run_dir / "reference_path.yaml"
    common_start = [
        "roslaunch", "swarm_test", "start_test.launch",
        f"scenario_index:={SCENARIO_INDEX[scenario_id]}",
        f"controller_index:={baseline['controller_index']}",
        f"output_dir:={run_dir}",
        f"num_of_obs:={num_obs}",
        f"obstacle_params_file:={obstacle_params}",
        f"obstacle_classes:={classes_arg}",
        f"goal_x:={goal_x}",
        f"goal_y:={goal_y}",
        "record_data:=true",
        f"dynamic_tau_enabled:={ros_bool(switches['dynamic_tau_enabled'])}",
        f"dynamic_tau_mode:={switches['dynamic_tau_mode']}",
        f"dynamic_tau_delta_tau:={switches['dynamic_tau_delta_tau']}",
        f"dynamic_tau_ke:={switches['dynamic_tau_ke']}",
        f"dynamic_tau_tmax:={switches['dynamic_tau_tmax']}",
        f"dynamic_tau_min_speed:={switches['dynamic_tau_min_speed']}",
        f"dynamic_tau_min_distance:={switches['dynamic_tau_min_distance']}",
        f"dynamic_tau_max_tau:={switches['dynamic_tau_max_tau']}",
        f"wait_for_global_path_ready:={ros_bool(global_path_ready_gate)}",
        f"global_path_start_gate_topic:={global_path_start_gate_topic}",
    ]
    if use_reference_waypoints:
        common_start.extend([
            "use_reference_path:=true",
            f"reference_path_file:={reference_path_file}",
            f"final_goal_x:={reference_waypoints[-1][0]}",
            f"final_goal_y:={reference_waypoints[-1][1]}",
        ])

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
            f"semantic_mode:={switches['semantic_mode']}",
            f"enable_rate_limit:={switches['enable_rate_limit']}",
            f"enable_available_projection:={switches['enable_available_projection']}",
            f"enable_guard_fallback:={switches['enable_guard_fallback']}",
            f"mpc_feasibility_guard_enabled:={switches['mpc_feasibility_guard_enabled']}",
            f"guard_kappa:={switches['guard_kappa']}",
            f"guard_max_backtracks:={switches['guard_max_backtracks']}",
            f"guard_time_budget_ms:={switches['guard_time_budget_ms']}",
            f"guard_solver_max_cpu_time_ms:={switches['guard_solver_max_cpu_time_ms']}",
            f"guard_binary_search:={ros_bool(switches['guard_binary_search'])}",
            f"guard_bounded_midpoint_then_zero:={ros_bool(switches['guard_bounded_midpoint_then_zero'])}",
            f"emergency_cbf_enabled:={ros_bool(switches['emergency_cbf_enabled'])}",
            f"emergency_cbf_alpha:={switches['emergency_cbf_alpha']}",
            f"emergency_cbf_extra_margin:={switches['emergency_cbf_extra_margin']}",
            f"emergency_cbf_v_max:={switches['emergency_cbf_v_max']}",
            f"emergency_cbf_turn_gain:={switches['emergency_cbf_turn_gain']}",
            f"emergency_cbf_activation_distance:={switches['emergency_cbf_activation_distance']}",
            f"emergency_cbf_progress_v:={switches['emergency_cbf_progress_v']}",
            f"fixed_beta:={switches['fixed_beta']}",
            f"epsilon_max:={switches['epsilon_max']}",
            f"slack_weight:={switches['slack_weight']}",
            f"qf_scale:={switches['qf_scale']}",
            f"delta_u_weight:={switches['delta_u_weight']}",
            f"delta_u_max:={switches['delta_u_max']}",
            f"active_set_distance_m:={switches['active_set_distance_m']}",
            f"graph_cache_enabled:={ros_bool(switches['graph_cache_enabled'])}",
            f"solver_max_cpu_time_ms:={switches['solver_max_cpu_time_ms']}",
            f"safety_delta_bar:={switches['safety_delta_bar']}",
            f"safety_delta_beta_bar:={switches['safety_delta_beta_bar']}",
            f"max_cbf_obstacles:={switches['max_cbf_obstacles']}",
            f"cbf_metric:={switches['cbf_metric']}",
            f"front_adsm:={switches['front_adsm']}",
            f"global_seesm_enable:={switches['global_seesm_enable']}",
            f"side_preference_enabled:={switches['side_preference_enabled']}",
            f"side_weight:={switches['side_weight']}",
            f"side_epsilon_n:={switches['side_epsilon_n']}",
            f"side_horizon:={switches['side_horizon']}",
            f"side_sign:={switches['side_sign']}",
            f"side_min_obstacle_speed:={switches['side_min_obstacle_speed']}",
            f"side_activation_distance:={switches['side_activation_distance']}",
            f"dynamic_tau_enabled:={ros_bool(switches['dynamic_tau_enabled'])}",
            f"dynamic_tau_mode:={switches['dynamic_tau_mode']}",
            f"dynamic_tau_delta_tau:={switches['dynamic_tau_delta_tau']}",
            f"dynamic_tau_ke:={switches['dynamic_tau_ke']}",
            f"dynamic_tau_tmax:={switches['dynamic_tau_tmax']}",
            f"dynamic_tau_min_speed:={switches['dynamic_tau_min_speed']}",
            f"dynamic_tau_min_distance:={switches['dynamic_tau_min_distance']}",
            f"dynamic_tau_max_tau:={switches['dynamic_tau_max_tau']}",
            f"output_dir:={run_dir}",
            f"obstacle_classes:={classes_arg}",
            f"obstacle_ids:={obstacle_ids_arg}",
            f"map_size_x:={map_cfg.get('x', 50.0)}",
            f"map_size_y:={map_cfg.get('y', 50.0)}",
            f"map_size_z:={map_cfg.get('z', 3.0)}",
            f"goal_x:={goal_x}",
            f"goal_y:={goal_y}",
            f"init_x:={start_x}",
            f"init_y:={start_y}",
            f"init_yaw:={init_yaw}",
            f"beta_bar_box:={beta_bar['box']}",
            f"beta_bar_adult:={beta_bar['adult']}",
            f"beta_bar_pedestrian:={beta_bar['pedestrian']}",
            f"beta_bar_child:={beta_bar['child']}",
            f"beta_bar_child_like:={beta_bar['child_like']}",
            f"beta_bar_cyclist:={beta_bar['cyclist']}",
            f"beta_bar_vehicle:={beta_bar['vehicle']}",
            f"beta_bar_unknown:={beta_bar['unknown']}",
            f"beta_max_box:={beta_max['box']}",
            f"beta_max_adult:={beta_max['adult']}",
            f"beta_max_pedestrian:={beta_max['pedestrian']}",
            f"beta_max_child:={beta_max['child']}",
            f"beta_max_child_like:={beta_max['child_like']}",
            f"beta_max_cyclist:={beta_max['cyclist']}",
            f"beta_max_vehicle:={beta_max['vehicle']}",
            f"beta_max_unknown:={beta_max['unknown']}",
            f"guard_h_min:={switches['guard_h_min']}",
            f"delta_beta_positive:={switches['delta_beta_positive']}",
            f"mu_bias:={mu_weights['bias']}",
            f"mu_heading:={mu_weights['heading']}",
            f"mu_ttc:={mu_weights['ttc']}",
            f"mu_density:={mu_weights['density']}",
            f"v_max:={scenario_planner_v_max(scenario)}",
        ]

    return planner, common_start


def process_env(run_dir: Path):
    env = os.environ.copy()
    ros_home = run_dir / ".ros"
    ros_log = run_dir / "ros_log"
    ros_home.mkdir(parents=True, exist_ok=True)
    ros_log.mkdir(parents=True, exist_ok=True)
    env["ROS_HOME"] = str(ros_home)
    env["ROS_LOG_DIR"] = str(ros_log)
    env.setdefault("ROS_HOSTNAME", "localhost")
    return env


def start_process(cmd, log_path: Path, env=None):
    log_file = log_path.open("w", encoding="utf-8")
    proc = subprocess.Popen(
        cmd,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        text=True,
        env=env,
    )
    return proc, log_file


def wait_for_global_path_ready(topic: str, timeout_sec: float,
                               min_span_m: float, env=None):
    """Wait for a finite global Path or MPC reference with meaningful span."""
    deadline = time.time() + timeout_sec
    last_reason = "no message"
    while time.time() < deadline:
        remaining = max(0.2, min(2.0, deadline - time.time()))
        try:
            result = subprocess.run(
                ["rostopic", "echo", "-n", "1", topic],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=remaining,
                env=env,
                check=False,
            )
        except subprocess.TimeoutExpired:
            last_reason = "topic wait timed out"
            continue
        if result.returncode != 0:
            last_reason = result.stdout.strip() or f"rostopic exit {result.returncode}"
            time.sleep(0.1)
            continue
        try:
            documents = [
                document for document in yaml.safe_load_all(result.stdout)
                if isinstance(document, dict)
            ]
            message = documents[0] if documents else {}
        except (TypeError, ValueError, yaml.YAMLError) as exc:
            last_reason = f"invalid reference payload: {exc}"
            continue
        payload_length = 0
        if isinstance(message.get("poses"), list):
            points = []
            for pose_stamped in message["poses"]:
                position = ((pose_stamped or {}).get("pose") or {}).get("position") or {}
                points.append((float(position["x"]), float(position["y"])))
            if len(points) < 2:
                last_reason = f"global path point count={len(points)}, expected >=2"
                continue
            if not all(math.isfinite(value) for point in points for value in point):
                last_reason = "global path contains non-finite values"
                continue
            payload_length = len(points)
            span = math.hypot(points[-1][0] - points[0][0], points[-1][1] - points[0][1])
        else:
            try:
                values = [float(value) for value in message.get("data", [])]
            except (TypeError, ValueError) as exc:
                last_reason = f"invalid MPC reference payload: {exc}"
                continue
            if len(values) < 60 or len(values) % 3 != 0:
                last_reason = f"reference payload length={len(values)}, expected >=60 triplets"
                continue
            if not all(math.isfinite(value) for value in values):
                last_reason = "reference payload contains non-finite values"
                continue
            payload_length = len(values)
            span = math.hypot(values[-3] - values[0], values[-2] - values[1])
        if span < min_span_m:
            last_reason = f"reference span {span:.3f} m < {min_span_m:.3f} m"
            time.sleep(0.1)
            continue
        return {
            "ready": True,
            "topic": topic,
            "payload_length": payload_length,
            "reference_span_m": span,
            "reason": "valid reference received",
        }
    return {
        "ready": False,
        "topic": topic,
        "payload_length": 0,
        "reference_span_m": 0.0,
        "reason": last_reason,
    }


def release_global_path_start_gate(topic: str, env=None):
    result = subprocess.run(
        ["rostopic", "pub", "-1", topic, "std_msgs/Bool", "data: true"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=5.0,
        env=env,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"failed to publish global path start gate on {topic}: "
            f"{result.stdout.strip()}"
        )


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


def run_verify(run_dir: Path, epsilon_max=0.05, delta_bar_beta=0.3):
    guard_log = guard_log_path(run_dir)
    if not guard_log.exists():
        return False, "margin_guard_log.csv not found"
    cmd = [
        sys.executable,
        str(repo_root() / "swarm_test/scripts/verify_safety_bound.py"),
        "--csv", str(guard_log),
        "--gamma", "0.35",
        "--eps_max", str(epsilon_max),
        "--delta_bar_beta", str(delta_bar_beta),
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
        "delta_beta_max": "",
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
    delta_beta = floats("delta_beta")
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
    if delta_beta:
        metrics["delta_beta_max"] = f"{max(delta_beta):.6f}"
    elif beta_applied:
        max_delta = 0.0
        rows_by_obs = {}
        for row in rows:
            rows_by_obs.setdefault(row.get("obs_id", ""), []).append(row)
        for obs_rows in rows_by_obs.values():
            prev = None
            for row in obs_rows:
                try:
                    beta = float(row["beta_applied"])
                except (KeyError, TypeError, ValueError):
                    continue
                if prev is not None:
                    max_delta = max(max_delta, max(0.0, beta - prev))
                prev = beta
        metrics["delta_beta_max"] = f"{max_delta:.6f}"
    metrics["guard_rollback_count"] = guard_failed
    metrics["guard_rollback_rate"] = f"{guard_failed / len(rows):.6f}"
    return metrics


def summarize_planner_log(planner_log: Path) -> dict:
    metrics = {
        "planner_records": 0,
        "constrained_obs_count_mean": "",
        "constrained_obs_count_max": "",
        "first_infeasible_count": "",
        "first_infeasible_rate": "",
        "mpc_guard_used_count": "",
        "mpc_guard_used_rate": "",
        "no_cbf_fallback_count": "",
        "no_cbf_fallback_rate": "",
        "safe_stop_count": "",
        "safe_stop_rate": "",
        "emergency_cbf_count": "",
        "emergency_cbf_rate": "",
        "slack_max": "",
        "slack_mean": "",
        "solve_time_mean_ms": "",
        "solve_time_max_ms": "",
        "side_preference_enabled": "",
        "side_weight": "",
        "side_cost_mean": "",
        "side_cost_max": "",
        "tracking_rmse_m": "",
        "tracking_error_max_m": "",
    }
    if not planner_log.exists():
        return metrics
    with planner_log.open("r", newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return metrics

    def floats(name):
        out = []
        for row in rows:
            try:
                out.append(float(row[name]))
            except (KeyError, TypeError, ValueError):
                pass
        return out

    total = len(rows)
    first_infeasible = sum(1 for row in rows if row.get("first_attempt_status") == "infeasible")
    guard_used = sum(1 for row in rows if row.get("mpc_feasibility_guard_used") in {"1", "True", "true"})
    no_cbf = sum(
        1 for row in rows
        if row.get("accepted_beta_source") == "no_cbf"
    )
    safe_stop = sum(
        1 for row in rows
        if row.get("accepted_beta_source") == "safe_stop" or
        row.get("mpc_status") == "infeasible_safe_stop"
    )
    emergency_cbf = sum(
        1 for row in rows
        if row.get("accepted_beta_source") == "emergency_cbf" or
        row.get("mpc_status") == "infeasible_emergency_cbf"
    )
    constrained_obs_count = floats("constrained_obs_count")
    slack_max = floats("slack_max") or floats("slack")
    slack_mean = floats("slack_mean")
    solve_time = floats("solve_time_ms")
    side_enabled = floats("side_preference_enabled")
    side_weight = floats("side_weight")
    side_cost = floats("side_cost")
    tracking_error = floats("tracking_error")
    side_dynamic_obstacle_count = floats("side_dynamic_obstacle_count")
    side_candidate_count = floats("side_candidate_count")
    side_dominant_index = floats("side_dominant_obs_index")
    side_dominant_tau = floats("side_dominant_tau")
    side_dominant_h = floats("side_dominant_h")

    metrics["planner_records"] = total
    if constrained_obs_count:
        metrics["constrained_obs_count_mean"] = f"{sum(constrained_obs_count) / len(constrained_obs_count):.6f}"
        metrics["constrained_obs_count_max"] = f"{max(constrained_obs_count):.6f}"
    metrics["first_infeasible_count"] = first_infeasible
    metrics["first_infeasible_rate"] = f"{first_infeasible / total:.6f}"
    metrics["mpc_guard_used_count"] = guard_used
    metrics["mpc_guard_used_rate"] = f"{guard_used / total:.6f}"
    metrics["no_cbf_fallback_count"] = no_cbf
    metrics["no_cbf_fallback_rate"] = f"{no_cbf / total:.6f}"
    metrics["safe_stop_count"] = safe_stop
    metrics["safe_stop_rate"] = f"{safe_stop / total:.6f}"
    metrics["emergency_cbf_count"] = emergency_cbf
    metrics["emergency_cbf_rate"] = f"{emergency_cbf / total:.6f}"
    if slack_max:
        metrics["slack_max"] = f"{max(slack_max):.6f}"
    if slack_mean:
        metrics["slack_mean"] = f"{sum(slack_mean) / len(slack_mean):.6f}"
    if solve_time:
        metrics["solve_time_mean_ms"] = f"{sum(solve_time) / len(solve_time):.6f}"
        metrics["solve_time_max_ms"] = f"{max(solve_time):.6f}"
    if side_enabled:
        metrics["side_preference_enabled"] = int(max(side_enabled) > 0.5)
    if side_weight:
        metrics["side_weight"] = f"{max(side_weight):.6f}"
    if side_cost:
        metrics["side_cost_mean"] = f"{sum(side_cost) / len(side_cost):.6f}"
        metrics["side_cost_max"] = f"{max(side_cost):.6f}"
    if tracking_error:
        metrics["tracking_rmse_m"] = f"{math.sqrt(sum(value * value for value in tracking_error) / len(tracking_error)):.6f}"
        metrics["tracking_error_max_m"] = f"{max(tracking_error):.6f}"
    if side_candidate_count:
        metrics["side_candidate_count_mean"] = f"{sum(side_candidate_count) / len(side_candidate_count):.6f}"
        metrics["side_multi_candidate_rate"] = f"{sum(value > 1.0 for value in side_candidate_count) / len(side_candidate_count):.6f}"
    if side_dynamic_obstacle_count:
        metrics["side_dynamic_obstacle_count_mean"] = f"{sum(side_dynamic_obstacle_count) / len(side_dynamic_obstacle_count):.6f}"
        metrics["side_crowd_suppression_rate"] = f"{sum(value > 1.0 for value in side_dynamic_obstacle_count) / len(side_dynamic_obstacle_count):.6f}"
    active_dominant = [value for value in side_dominant_index if value >= 0.0]
    if side_dominant_index:
        metrics["side_dominant_active_count"] = len(active_dominant)
        metrics["side_dominant_active_rate"] = f"{len(active_dominant) / len(side_dominant_index):.6f}"
    if side_dominant_tau:
        metrics["side_dominant_tau_max"] = f"{max(side_dominant_tau):.6f}"
    finite_h = [value for value in side_dominant_h if value == value]
    if finite_h:
        metrics["side_dominant_h_min"] = f"{min(finite_h):.6f}"
    return metrics


TAU_COMPUTED_INACTIVE_REASONS = frozenset({
    "teacher_receding", "teacher_tangent",
})


def _tau_bool(value):
    normalized = str(value).strip().lower()
    if normalized in {"1", "true", "yes"}:
        return True
    if normalized in {"0", "false", "no"}:
        return False
    return None


def _summarize_tau_source(path: Path, required_fields: set, source_label: str) -> dict:
    summary = {
        "record_count": 0,
        "computational_valid_count": 0,
        "active_count": 0,
        "inactive_valid_count": 0,
        "invalid_count": 0,
        "mean": "",
        "max": "",
        "active_fraction": "",
        "reason_counts": "",
        "source": "",
    }
    if not path.exists():
        return summary
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if not required_fields.issubset(set(reader.fieldnames or ())):
            return summary
        rows = list(reader)
        header = set(reader.fieldnames or ())

    tau_values = []
    reason_counts = {}
    parsed_records = 0
    for row in rows:
        try:
            tau = float(row["tau"])
        except (KeyError, TypeError, ValueError):
            continue
        if not math.isfinite(tau) or tau < 0.0:
            continue
        computed_field = "tau_computed" if "tau_computed" in header else "tau_valid"
        computed = _tau_bool(row.get(computed_field, ""))
        reason = str(row.get("tau_reason", "")).strip()
        if computed is None or not reason:
            continue
        parsed_records += 1
        reason_counts[reason] = reason_counts.get(reason, 0) + 1

        # Legacy/current-state logs used tau_valid as an active flag.  The two
        # finite Teacher zero-horizon outcomes are nevertheless successful
        # computations and are recognized explicitly when reading such logs.
        if reason in TAU_COMPUTED_INACTIVE_REASONS:
            computed = True
        active = (
            _tau_bool(row.get("tau_active", ""))
            if "tau_active" in header
            else None
        )
        if active is None:
            active = computed and tau > 0.0

        if not computed:
            summary["invalid_count"] += 1
            continue
        tau_values.append(tau)
        summary["computational_valid_count"] += 1
        if active:
            summary["active_count"] += 1
        else:
            summary["inactive_valid_count"] += 1

    summary["record_count"] = parsed_records
    summary["reason_counts"] = ";".join(
        f"{reason}:{reason_counts[reason]}" for reason in sorted(reason_counts)
    )
    if parsed_records:
        summary["source"] = source_label
    if tau_values:
        summary["mean"] = f"{sum(tau_values) / len(tau_values):.6f}"
        summary["max"] = f"{max(tau_values):.6f}"
        summary["active_fraction"] = (
            f"{summary['active_count'] / len(tau_values):.6f}"
        )
    return summary


def summarize_tau_log(run_dir: Path) -> dict:
    # Stagewise MPC values and current-state Guard values are different audit
    # populations.  Keep both, and use the full MPC stage log as the canonical
    # top-level tau summary whenever it is present.
    stage = _summarize_tau_source(
        run_dir / "tau_stage_log.csv",
        {"tau", "tau_active", "tau_valid", "tau_reason"},
        "tau_stage_log.csv:mpc_stage",
    )
    guard = _summarize_tau_source(
        run_dir / "margin_guard_log.csv",
        {"tau", "tau_valid", "tau_reason"},
        "margin_guard_log.csv:guard_current_state",
    )
    planner = _summarize_tau_source(
        run_dir / "planner_log.csv",
        {"tau", "tau_valid", "tau_reason"},
        "planner_log.csv:mpc_stage_representative",
    )
    global_current = _summarize_tau_source(
        run_dir / "global_seesm_log.csv",
        {"tau", "tau_valid", "tau_reason"},
        "global_seesm_log.csv:global_current_state",
    )

    selected = next(
        (
            candidate for candidate in (stage, guard, planner, global_current)
            if candidate["computational_valid_count"] > 0
        ),
        next(
            (candidate for candidate in (stage, guard, planner, global_current)
             if candidate["record_count"] > 0),
            stage,
        ),
    )
    metrics = {
        "tau_record_count": selected["record_count"],
        "tau_computational_valid_count": selected["computational_valid_count"],
        "tau_active_count": selected["active_count"],
        "tau_inactive_valid_count": selected["inactive_valid_count"],
        "tau_mean": selected["mean"],
        "tau_max": selected["max"],
        "tau_active_fraction": selected["active_fraction"],
        "tau_invalid_count": selected["invalid_count"],
        "tau_reason_counts": selected["reason_counts"],
        "tau_source": selected["source"],
    }
    for prefix, source_summary in (("tau_mpc_stage", stage), ("tau_guard", guard)):
        for key, value in source_summary.items():
            metrics[f"{prefix}_{key}"] = value
    return metrics


def dynamic_tau_audit(run_dir: Path) -> dict:
    metrics = {
        "dynamic_tau_enabled": False,
        "dynamic_tau_mode": "",
        "dynamic_tau_delta_tau": "",
        "dynamic_tau_ke": "",
        "dynamic_tau_tmax": "",
        "dynamic_tau_min_speed": "",
        "dynamic_tau_min_distance": "",
        "dynamic_tau_max_tau": "",
        "dynamic_tau_formula": "",
        "dynamic_tau_h_eesm": "",
        "dynamic_tau_h_seesm": "",
        "dynamic_tau_beta_source": "",
        "dynamic_tau_relative_position_convention": "",
        "dynamic_tau_relative_velocity_convention": "",
        "dynamic_tau_prediction_sign": "",
        "dynamic_tau_mpc_stage_policy": "",
    }
    meta_path = run_dir / "meta.yaml"
    if yaml is None or not meta_path.exists():
        return metrics
    try:
        with meta_path.open("r", encoding="utf-8") as f:
            meta = yaml.safe_load(f) or {}
        dynamic = meta.get("dynamic_tau", {})
        if not isinstance(dynamic, dict):
            return metrics
        metrics["dynamic_tau_enabled"] = bool_switch(dynamic.get("enabled", False))
        for source_key, output_key in (
            ("delta_tau", "dynamic_tau_delta_tau"),
            ("Ke", "dynamic_tau_ke"),
            ("Tmax", "dynamic_tau_tmax"),
            ("min_speed", "dynamic_tau_min_speed"),
            ("min_distance", "dynamic_tau_min_distance"),
            ("max_tau", "dynamic_tau_max_tau"),
        ):
            if source_key in dynamic:
                metrics[output_key] = float(dynamic[source_key])
        for source_key, output_key in (
            ("mode", "dynamic_tau_mode"),
            ("formula", "dynamic_tau_formula"),
            ("h_eesm", "dynamic_tau_h_eesm"),
            ("h_seesm", "dynamic_tau_h_seesm"),
            ("beta_source", "dynamic_tau_beta_source"),
            ("relative_position_convention", "dynamic_tau_relative_position_convention"),
            ("relative_velocity_convention", "dynamic_tau_relative_velocity_convention"),
            ("prediction_sign", "dynamic_tau_prediction_sign"),
            ("mpc_stage_policy", "dynamic_tau_mpc_stage_policy"),
        ):
            metrics[output_key] = str(dynamic.get(source_key, ""))
    except (OSError, TypeError, ValueError):
        return metrics
    except yaml.YAMLError:
        return metrics
    return metrics


def summarize_data_processor(data_summary: Path, duration_sec: int) -> dict:
    metrics = {
        "nav_records": 0,
        "nav_finished_before_timeout": "",
        "nav_path_length_m": "",
        "nav_travel_time_s": "",
        "nav_mean_vel_ms": "",
        "nav_mean_ang_rads": "",
        "nav_var_vel": "",
        "nav_var_ang": "",
        "nav_collision_count": "",
        "nav_min_distance_m": "",
    }
    if not data_summary.exists():
        return metrics

    rows = []
    with data_summary.open("r", newline="", encoding="utf-8") as f:
        for row in csv.reader(f):
            clean = [cell.strip() for cell in row]
            if any(clean):
                rows.append(clean)
    if not rows:
        return metrics

    metrics["nav_records"] = len(rows)
    row = rows[-1]
    # data_processor.cpp writes:
    # empty, scenario, controller, path_length, travel_time, mean_vel,
    # mean_ang, var_vel, var_ang, collision_count, min_distance.
    if row and row[0] == "":
        row = row[1:]
    values = row[2:]
    names = [
        "nav_path_length_m",
        "nav_travel_time_s",
        "nav_mean_vel_ms",
        "nav_mean_ang_rads",
        "nav_var_vel",
        "nav_var_ang",
        "nav_collision_count",
        "nav_min_distance_m",
    ]
    for name, value in zip(names, values):
        metrics[name] = value

    try:
        travel_time = float(metrics["nav_travel_time_s"])
        # If the data processor kept updating until the runner timeout, this
        # is not a confirmed arrival. Leave a little shutdown margin.
        finished = travel_time > 0.0 and travel_time < max(0.0, duration_sec - 2.0)
        metrics["nav_finished_before_timeout"] = int(finished)
    except (TypeError, ValueError):
        pass

    return metrics


def summarize_phase5_logs(run_dir: Path) -> dict:
    metrics = {
        "success": "",
        "goal_reached": "",
        "robot_records": 0,
        "robot_path_length_m": "",
        "robot_travel_time_s": "",
        "robot_final_goal_distance_m": "",
        "robot_mean_abs_v": "",
        "robot_mean_abs_w": "",
        "robot_deadlock_low_speed_max_s": "",
        "robot_velocity_smoothness": "",
        "robot_control_effort": "",
        "log_min_distance_m": "",
        "log_min_h_ee": "",
        "log_invalid_obstacle_rows": 0,
    }

    meta_path = run_dir / "meta.yaml"
    goal = None
    robot_radius = 0.4
    if yaml is not None and meta_path.exists():
        try:
            with meta_path.open("r", encoding="utf-8") as f:
                meta = yaml.safe_load(f) or {}
            goal_raw = meta.get("goal", [])
            if len(goal_raw) >= 2:
                goal = (float(goal_raw[0]), float(goal_raw[1]))
            robot_radius = float(meta.get("robot_radius", robot_radius))
        except (TypeError, ValueError, yaml.YAMLError):
            goal = None

    robot_log = run_dir / "robot_log.csv"
    if robot_log.exists():
        with robot_log.open("r", newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        metrics["robot_records"] = len(rows)
        positions = []
        abs_v = []
        abs_w = []
        controls = []
        times = []
        for row in rows:
            try:
                x = float(row["x"])
                y = float(row["y"])
                positions.append((x, y))
                times.append(float(row["t"]))
                abs_v.append(abs(float(row.get("v", 0.0))))
                abs_w.append(abs(float(row.get("w", 0.0))))
                cmd_v = float(row.get("cmd_v", 0.0))
                cmd_w = float(row.get("cmd_w", 0.0))
                controls.append(cmd_v * cmd_v + cmd_w * cmd_w)
            except (KeyError, TypeError, ValueError):
                continue

        if len(positions) >= 2:
            path_length = 0.0
            step_lengths = []
            for idx in range(1, len(positions)):
                step = math.hypot(
                    positions[idx][0] - positions[idx - 1][0],
                    positions[idx][1] - positions[idx - 1][1],
                )
                path_length += step
                step_lengths.append(step)
            metrics["robot_path_length_m"] = f"{path_length:.6f}"
            if times:
                metrics["robot_travel_time_s"] = f"{max(times) - min(times):.6f}"
            if goal is not None:
                final_dist = math.hypot(positions[-1][0] - goal[0], positions[-1][1] - goal[1])
                metrics["robot_final_goal_distance_m"] = f"{final_dist:.6f}"
                metrics["goal_reached"] = int(final_dist <= GOAL_TOLERANCE_M)
                metrics["success"] = metrics["goal_reached"]
            if len(step_lengths) >= 2:
                diffs = [step_lengths[i] - step_lengths[i - 1] for i in range(1, len(step_lengths))]
                smoothness = math.sqrt(sum(diff * diff for diff in diffs) / len(diffs))
                metrics["robot_velocity_smoothness"] = f"{smoothness:.6f}"
        if abs_v:
            metrics["robot_mean_abs_v"] = f"{sum(abs_v) / len(abs_v):.6f}"
            longest_low_speed = 0.0
            episode_start = None
            previous_t = None
            for sample_t, sample_v in zip(times, abs_v):
                if sample_v < DEADLOCK_SPEED_MPS:
                    if (
                        episode_start is None
                        or previous_t is None
                        or sample_t - previous_t > 0.5
                    ):
                        episode_start = sample_t
                    longest_low_speed = max(
                        longest_low_speed, sample_t - episode_start
                    )
                    previous_t = sample_t
                else:
                    episode_start = None
                    previous_t = None
            metrics["robot_deadlock_low_speed_max_s"] = (
                f"{longest_low_speed:.6f}"
            )
        if abs_w:
            metrics["robot_mean_abs_w"] = f"{sum(abs_w) / len(abs_w):.6f}"
        if controls:
            metrics["robot_control_effort"] = f"{sum(controls) / len(controls):.6f}"

    obstacle_log = run_dir / "obstacle_log.csv"
    if obstacle_log.exists():
        min_distance = None
        min_h_ee = None
        with obstacle_log.open("r", newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                try:
                    d_i_raw = float(row["d_i"])
                    rel_v_raw = float(row["rel_v"])
                    ttc_raw = float(row["TTC"])
                    h_ee_raw = float(row["h_EE"])
                except (KeyError, TypeError, ValueError):
                    continue
                # The simulator can publish an all-zero prediction row while
                # obstacle ids are being initialized. It is not a clearance
                # measurement and must not become a false collision.
                if (
                    abs(d_i_raw) <= 1.0e-12
                    and abs(rel_v_raw) <= 1.0e-12
                    and abs(ttc_raw) <= 1.0e-12
                    and abs(h_ee_raw) <= 1.0e-12
                ):
                    metrics["log_invalid_obstacle_rows"] += 1
                    continue
                try:
                    d_i = float(row["d_i"])
                    radius = float(row["radius"])
                    distance = d_i - radius - robot_radius
                    min_distance = distance if min_distance is None else min(min_distance, distance)
                except (KeyError, TypeError, ValueError):
                    pass
                try:
                    h_ee = float(row["h_EE"])
                    min_h_ee = h_ee if min_h_ee is None else min(min_h_ee, h_ee)
                except (KeyError, TypeError, ValueError):
                    pass
        if min_distance is not None:
            metrics["log_min_distance_m"] = f"{min_distance:.6f}"
        if min_h_ee is not None:
            metrics["log_min_h_ee"] = f"{min_h_ee:.6f}"

    return metrics


def classify_termination_reason(run_dir: Path, nav_metrics: dict,
                                phase5_metrics: dict, planner_metrics: dict,
                                baseline_id="SEESM_Ours") -> str:
    """Classify one trial using only auditable logs and final metrics.

    Collision takes priority over goal arrival, because a trial that reaches the
    goal after crossing an obstacle is not a successful safety trial. A transient
    infeasible MPC solve is classified only when the trial does not eventually
    reach the goal; this preserves the distinction between recovery and failure.
    """
    logs_ok, _ = audit_required_logs(run_dir, baseline_id)
    planner_required = baseline_id != "B1_ACBF_fixed"
    if (
        not logs_ok
        or phase5_metrics.get("robot_records", 0) <= 0
        or (planner_required and planner_metrics.get("planner_records", 0) <= 0)
    ):
        return "invalid"

    def as_float(metrics, key):
        try:
            value = float(metrics.get(key, ""))
            return value if math.isfinite(value) else None
        except (TypeError, ValueError):
            return None

    collision_count = as_float(nav_metrics, "nav_collision_count")
    log_min_distance = as_float(phase5_metrics, "log_min_distance_m")
    if (collision_count is not None and collision_count > 0) or (
        log_min_distance is not None and log_min_distance <= 0.0
    ):
        return "collision"

    if str(phase5_metrics.get("goal_reached", "")).strip() in {"1", "true", "True"}:
        return "success"

    first_infeasible = as_float(planner_metrics, "first_infeasible_count")
    if first_infeasible is not None and first_infeasible > 0:
        return "infeasible"

    low_speed_duration = as_float(
        phase5_metrics, "robot_deadlock_low_speed_max_s"
    )
    final_goal_distance = as_float(phase5_metrics, "robot_final_goal_distance_m")
    if (
        low_speed_duration is not None
        and final_goal_distance is not None
        and final_goal_distance > GOAL_TOLERANCE_M
        and low_speed_duration >= DEADLOCK_HOLD_SEC
    ):
        return "deadlock"
    return "timeout"


def guard_log_path(run_dir: Path) -> Path:
    margin_log = run_dir / "margin_guard_log.csv"
    if margin_log.exists():
        return margin_log
    return run_dir / "guard_log.csv"


def write_summary(run_dir: Path, scenario_id: str, baseline_id: str, commands,
                  verify_passed, verify_note, duration_sec,
                  csv_contract_passed=None, csv_contract_note="",
                  metadata_contract_passed=None, process_contract_passed=True,
                  invalid_reasons=None):
    summary_md = run_dir / "summary.md"
    summary_csv = run_dir / "summary.csv"
    guard_log = guard_log_path(run_dir)
    data_summary = run_dir / "data_processor_summary.csv"
    data_distance = run_dir / "data_processor_distance.csv"
    planner_log = run_dir / "planner_log.csv"
    guard_metrics = summarize_guard_log(guard_log)
    planner_metrics = summarize_planner_log(planner_log)
    tau_metrics = summarize_tau_log(run_dir)
    tau_audit = dynamic_tau_audit(run_dir)
    nav_metrics = summarize_data_processor(data_summary, duration_sec)
    phase5_metrics = summarize_phase5_logs(run_dir)
    termination_reason = classify_termination_reason(
        run_dir, nav_metrics, phase5_metrics, planner_metrics, baseline_id
    )
    invalid_reasons = list(invalid_reasons or [])
    logs_ok, log_errors = audit_required_logs(run_dir, baseline_id)
    invalid_reasons.extend(log_errors)
    if termination_reason == "invalid":
        invalid_reasons.append("termination classification is invalid")
    if csv_contract_passed is not True:
        invalid_reasons.append("CSV contract failed")
    if metadata_contract_passed is not True:
        invalid_reasons.append("metadata contract failed")
    if process_contract_passed is not True:
        invalid_reasons.append("process contract failed")
    invalid_reasons = list(dict.fromkeys(invalid_reasons))
    trial_valid = (
        logs_ok
        and termination_reason != "invalid"
        and csv_contract_passed is True
        and metadata_contract_passed is True
        and process_contract_passed is True
    )
    phase5_metrics["success"] = (
        int(termination_reason == "success") if termination_reason != "invalid" else ""
    )
    phase5_metrics["termination_reason"] = termination_reason

    lines = [
        f"# {scenario_id} / {baseline_id}",
        "",
        f"- duration_sec: {duration_sec}",
        f"- margin_guard_log_exists: {guard_log.exists()}",
        f"- data_processor_summary_exists: {data_summary.exists()}",
        f"- data_processor_distance_exists: {data_distance.exists()}",
        f"- nav_finished_before_timeout: {nav_metrics['nav_finished_before_timeout']}",
        f"- nav_path_length_m: {nav_metrics['nav_path_length_m']}",
        f"- nav_travel_time_s: {nav_metrics['nav_travel_time_s']}",
        f"- nav_collision_count: {nav_metrics['nav_collision_count']}",
        f"- nav_min_distance_m: {nav_metrics['nav_min_distance_m']}",
        f"- termination_reason: {termination_reason}",
        f"- goal_reached: {phase5_metrics['goal_reached']}",
        f"- success: {phase5_metrics['success']}",
        f"- trial_valid: {trial_valid}",
        f"- log_profile: {log_profile_for_baseline(baseline_id)}",
        f"- csv_contract_passed: {csv_contract_passed}",
        f"- csv_contract_note: {csv_contract_note}",
        f"- metadata_contract_passed: {metadata_contract_passed}",
        f"- process_contract_passed: {process_contract_passed}",
        f"- invalid_reasons: {';'.join(invalid_reasons)}",
        f"- robot_path_length_m: {phase5_metrics['robot_path_length_m']}",
        f"- robot_final_goal_distance_m: {phase5_metrics['robot_final_goal_distance_m']}",
        f"- log_min_distance_m: {phase5_metrics['log_min_distance_m']}",
        f"- log_invalid_obstacle_rows: {phase5_metrics['log_invalid_obstacle_rows']}",
        f"- safety_bound_passed: {verify_passed}",
        f"- verification_note: {verify_note}",
        f"- guard_records: {guard_metrics['guard_records']}",
        f"- semantic_classes: {guard_metrics['semantic_classes']}",
        f"- beta_applied_mean: {guard_metrics['beta_applied_mean']}",
        f"- beta_applied_max: {guard_metrics['beta_applied_max']}",
        f"- h_ee_min: {guard_metrics['h_ee_min']}",
        f"- h_see_min: {guard_metrics['h_see_min']}",
        f"- delta_beta_max: {guard_metrics['delta_beta_max']}",
        f"- guard_rollback_count: {guard_metrics['guard_rollback_count']}",
        f"- guard_rollback_rate: {guard_metrics['guard_rollback_rate']}",
        f"- constrained_obs_count_mean: {planner_metrics['constrained_obs_count_mean']}",
        f"- constrained_obs_count_max: {planner_metrics['constrained_obs_count_max']}",
        f"- first_infeasible_rate: {planner_metrics['first_infeasible_rate']}",
        f"- mpc_guard_used_rate: {planner_metrics['mpc_guard_used_rate']}",
        f"- no_cbf_fallback_rate: {planner_metrics['no_cbf_fallback_rate']}",
        f"- safe_stop_rate: {planner_metrics['safe_stop_rate']}",
        f"- slack_max: {planner_metrics['slack_max']}",
        f"- slack_mean: {planner_metrics['slack_mean']}",
        f"- solve_time_mean_ms: {planner_metrics['solve_time_mean_ms']}",
        f"- tau_mean: {tau_metrics['tau_mean']}",
        f"- tau_max: {tau_metrics['tau_max']}",
        f"- tau_active_fraction: {tau_metrics['tau_active_fraction']}",
        f"- tau_computational_valid_count: {tau_metrics['tau_computational_valid_count']}",
        f"- tau_active_count: {tau_metrics['tau_active_count']}",
        f"- tau_inactive_valid_count: {tau_metrics['tau_inactive_valid_count']}",
        f"- tau_invalid_count: {tau_metrics['tau_invalid_count']}",
        f"- tau_reason_counts: {tau_metrics['tau_reason_counts']}",
        f"- tau_source: {tau_metrics['tau_source']}",
        f"- tau_mpc_stage_source: {tau_metrics['tau_mpc_stage_source']}",
        f"- tau_mpc_stage_record_count: {tau_metrics['tau_mpc_stage_record_count']}",
        f"- tau_mpc_stage_computational_valid_count: {tau_metrics['tau_mpc_stage_computational_valid_count']}",
        f"- tau_mpc_stage_active_count: {tau_metrics['tau_mpc_stage_active_count']}",
        f"- tau_mpc_stage_inactive_valid_count: {tau_metrics['tau_mpc_stage_inactive_valid_count']}",
        f"- tau_guard_source: {tau_metrics['tau_guard_source']}",
        f"- tau_guard_record_count: {tau_metrics['tau_guard_record_count']}",
        f"- tau_guard_computational_valid_count: {tau_metrics['tau_guard_computational_valid_count']}",
        f"- tau_guard_active_count: {tau_metrics['tau_guard_active_count']}",
        f"- tau_guard_inactive_valid_count: {tau_metrics['tau_guard_inactive_valid_count']}",
        "",
        "## Dynamic tau audit",
        f"- enabled: {tau_audit['dynamic_tau_enabled']}",
        f"- mode: {tau_audit['dynamic_tau_mode']}",
        f"- delta_tau: {tau_audit['dynamic_tau_delta_tau']}",
        f"- Ke: {tau_audit['dynamic_tau_ke']}",
        f"- Tmax: {tau_audit['dynamic_tau_tmax']}",
        f"- min_speed: {tau_audit['dynamic_tau_min_speed']}",
        f"- min_distance: {tau_audit['dynamic_tau_min_distance']}",
        f"- max_tau: {tau_audit['dynamic_tau_max_tau']}",
        f"- formula: {tau_audit['dynamic_tau_formula']}",
        f"- h_eesm: {tau_audit['dynamic_tau_h_eesm']}",
        f"- h_seesm: {tau_audit['dynamic_tau_h_seesm']}",
        f"- beta_source: {tau_audit['dynamic_tau_beta_source']}",
        f"- relative_position_convention: {tau_audit['dynamic_tau_relative_position_convention']}",
        f"- relative_velocity_convention: {tau_audit['dynamic_tau_relative_velocity_convention']}",
        f"- prediction_sign: {tau_audit['dynamic_tau_prediction_sign']}",
        f"- mpc_stage_policy: {tau_audit['dynamic_tau_mpc_stage_policy']}",
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
                "nav_records", "nav_finished_before_timeout",
                "nav_path_length_m", "nav_travel_time_s", "nav_mean_vel_ms",
                "nav_mean_ang_rads", "nav_var_vel", "nav_var_ang",
                "nav_collision_count", "nav_min_distance_m",
                "termination_reason", "goal_reached", "success", "trial_valid",
                "log_profile", "csv_contract_passed", "csv_contract_note",
                "metadata_contract_passed", "process_contract_passed",
                "invalid_reasons", "robot_records", "robot_path_length_m",
                "robot_travel_time_s", "robot_final_goal_distance_m",
                "robot_mean_abs_v", "robot_mean_abs_w",
                "robot_deadlock_low_speed_max_s",
                "robot_velocity_smoothness", "robot_control_effort",
                "log_min_distance_m", "log_min_h_ee", "log_invalid_obstacle_rows",
                "safety_bound_passed", "guard_records", "semantic_classes",
                "beta_applied_mean", "beta_applied_max", "h_ee_min", "h_see_min",
                "delta_beta_max", "guard_rollback_count", "guard_rollback_rate",
                "planner_records", "constrained_obs_count_mean", "constrained_obs_count_max",
                "first_infeasible_count", "first_infeasible_rate",
                "mpc_guard_used_count", "mpc_guard_used_rate", "no_cbf_fallback_count",
                "no_cbf_fallback_rate", "safe_stop_count", "safe_stop_rate",
                "emergency_cbf_count", "emergency_cbf_rate",
                "slack_max", "slack_mean",
                "solve_time_mean_ms", "solve_time_max_ms",
                "side_preference_enabled", "side_weight", "side_cost_mean", "side_cost_max",
                "tracking_rmse_m", "tracking_error_max_m",
                "side_candidate_count_mean", "side_multi_candidate_rate",
                "side_dynamic_obstacle_count_mean", "side_crowd_suppression_rate",
                "side_dominant_active_count", "side_dominant_active_rate",
                "side_dominant_tau_max", "side_dominant_h_min",
                "tau_record_count", "tau_computational_valid_count", "tau_active_count",
                "tau_inactive_valid_count", "tau_mean", "tau_max",
                "tau_active_fraction", "tau_invalid_count", "tau_reason_counts",
                "tau_source", "tau_mpc_stage_record_count",
                "tau_mpc_stage_computational_valid_count", "tau_mpc_stage_active_count",
                "tau_mpc_stage_inactive_valid_count", "tau_mpc_stage_invalid_count",
                "tau_mpc_stage_mean", "tau_mpc_stage_max",
                "tau_mpc_stage_active_fraction", "tau_mpc_stage_reason_counts",
                "tau_mpc_stage_source", "tau_guard_record_count",
                "tau_guard_computational_valid_count", "tau_guard_active_count",
                "tau_guard_inactive_valid_count", "tau_guard_invalid_count",
                "tau_guard_mean", "tau_guard_max", "tau_guard_active_fraction",
                "tau_guard_reason_counts", "tau_guard_source",
                "dynamic_tau_enabled", "dynamic_tau_mode", "dynamic_tau_delta_tau",
                "dynamic_tau_ke", "dynamic_tau_tmax",
                "dynamic_tau_min_speed", "dynamic_tau_min_distance", "dynamic_tau_max_tau",
                "dynamic_tau_formula", "dynamic_tau_h_eesm", "dynamic_tau_h_seesm",
                "dynamic_tau_beta_source", "dynamic_tau_relative_position_convention",
                "dynamic_tau_relative_velocity_convention", "dynamic_tau_prediction_sign",
                "dynamic_tau_mpc_stage_policy", "output_dir",
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
            "trial_valid": trial_valid,
            "log_profile": log_profile_for_baseline(baseline_id),
            "csv_contract_passed": csv_contract_passed,
            "csv_contract_note": csv_contract_note,
            "metadata_contract_passed": metadata_contract_passed,
            "process_contract_passed": process_contract_passed,
            "invalid_reasons": ";".join(invalid_reasons),
            **nav_metrics,
            **phase5_metrics,
            "safety_bound_passed": verify_passed,
            **guard_metrics,
            **planner_metrics,
            **tau_metrics,
            **tau_audit,
            "output_dir": str(run_dir),
        })
    return {
        "trial_valid": trial_valid,
        "termination_reason": termination_reason,
        "invalid_reasons": invalid_reasons,
    }


def write_aggregate_summary(output_root: Path, run_dirs: list):
    rows = []
    fieldnames = []
    fieldname_set = set()
    for run_dir in run_dirs:
        if not is_complete_run_dir(run_dir):
            continue
        summary_csv = run_dir / "summary.csv"
        if not summary_csv.exists():
            continue
        with summary_csv.open("r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for fieldname in reader.fieldnames or ():
                if fieldname not in fieldname_set:
                    fieldnames.append(fieldname)
                    fieldname_set.add(fieldname)
            try:
                rows.append(next(reader))
            except StopIteration:
                continue

    if not rows or not fieldnames:
        return None

    output_root.mkdir(parents=True, exist_ok=True)
    aggregate_csv = output_root / "summary.csv"
    with aggregate_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({fieldname: row.get(fieldname, "") for fieldname in fieldnames})
    return aggregate_csv


def quarantine_incomplete_run_dir(run_dir: Path, timestamp: str) -> Path:
    candidate = run_dir.parent / f".interrupted_{run_dir.name}_{timestamp}"
    suffix = 1
    while candidate.exists():
        candidate = run_dir.parent / f".interrupted_{run_dir.name}_{timestamp}_{suffix}"
        suffix += 1
    run_dir.rename(candidate)
    return candidate


def mark_exception_invalid(run_dir: Path, error: Exception) -> None:
    run_dir = Path(run_dir)
    if not run_dir.exists() or is_complete_run_dir(run_dir):
        return
    reason = f"runner exception: {type(error).__name__}: {error}"
    validation = {
        "trial_valid": False,
        "termination_reason": "invalid",
        "csv_contract_passed": False,
        "metadata_contract_passed": False,
        "process_contract_passed": False,
        "safety_bound_passed": None,
        "invalid_reasons": [reason],
    }
    if (run_dir / "run_meta.yaml").exists():
        seal_trial(run_dir, False, validation)
        return
    atomic_write_text(
        run_dir / RUN_INVALID_SENTINEL,
        "\n".join(
            [
                "TEACHER_V1_RUN_INVALID",
                f"validated_at={dt.datetime.now(dt.timezone.utc).isoformat()}",
                "trial_valid=false",
                "termination_reason=invalid",
                f"invalid_reasons={reason}",
            ]
        ) + "\n",
    )


def run_one(scenario_id: str, baseline_id: str, scenario: dict, args, timestamp: str,
            trial=None, requested_baseline_label=None):
    run_suffix = trial["trial_id"] if trial is not None else timestamp
    run_dir = Path(args.output_root) / f"{run_suffix}_{scenario_id}_{baseline_id}"
    if trial is not None:
        scenario, obstacles = materialize_trial(scenario, trial)
    else:
        obstacles = scenario.get("obstacles", [])
    scenario = apply_parameter_freeze(
        scenario, getattr(args, "execution_freeze", None)
    )
    if not obstacles:
        raise RuntimeError(f"Scenario {scenario_id} has no obstacles")

    classes_arg = obstacle_classes(obstacles)
    obstacle_params = run_dir / "obstacles_param.yaml"
    planner_cmd, start_cmd = build_commands(
        scenario_id, baseline_id, run_dir, obstacle_params, classes_arg, len(obstacles), scenario,
        global_path_ready_gate=getattr(args, "global_path_ready_gate", False),
        global_path_start_gate_topic=getattr(
            args, "global_path_start_gate_topic", "/teacher_v1/global_path_ready"
        ),
    )

    commands = [("planner", planner_cmd), ("start_test", start_cmd)]
    if args.dry_run:
        print(f"\n[{scenario_id} / {baseline_id}]")
        print(f"output_dir: {run_dir}")
        print(f"obstacle_classes: {classes_arg}")
        for name, cmd in commands:
            print(f"{name}: {' '.join(cmd)}")
        return None

    run_dir = ensure_teacher_output_root(run_dir)
    trial_run_context = copy.deepcopy(getattr(args, "run_context", None) or {})
    verify_repository_context_unchanged(trial_run_context)
    trial_run_context["runtime"] = collect_trial_runtime_identity(baseline_id)
    if run_dir.exists() and is_complete_run_dir(run_dir):
        if getattr(args, "skip_existing_complete", False):
            print(f"Skipped sealed complete {scenario_id} / {baseline_id}: {run_dir}")
            return run_dir
        raise RuntimeError(
            f"refusing to overwrite sealed complete trial without a new trial id: {run_dir}"
        )
    if run_dir.exists():
        quarantined = quarantine_incomplete_run_dir(run_dir, timestamp)
        print(f"Quarantined unsealed/invalid {scenario_id} / {baseline_id}: {quarantined}")

    run_dir.mkdir(parents=True, exist_ok=True)
    artifact_paths = []
    if trial is not None:
        write_trial_artifacts(run_dir, scenario, trial, baseline_id)
        artifact_paths.extend(
            [run_dir / "effective_obstacles.yaml", run_dir / "trial_manifest_row.csv"]
        )
    obstacle_params = write_obstacle_params(run_dir, obstacles)
    artifact_paths.append(obstacle_params)
    reference_waypoints = None
    if is_reference_path_scene(scenario):
        reference_path, reference_waypoints = write_reference_path_config(run_dir, scenario)
        artifact_paths.append(reference_path)
    artifact_hashes = {
        path.name: sha256_file(path) for path in artifact_paths if path.exists()
    }
    write_run_meta(
        run_dir,
        scenario_id,
        baseline_id,
        scenario,
        classes_arg,
        len(obstacles),
        args.duration_sec,
        reference_waypoints,
        trial=trial,
        resolved_baseline_id=baseline_id,
        requested_baseline_label=requested_baseline_label,
        protocol_id=getattr(args, "protocol_id", DEFAULT_PROTOCOL_ID),
        run_context=trial_run_context,
        artifact_hashes=artifact_hashes,
        freeze_contract=getattr(args, "execution_freeze", None),
        common_evaluation_contract=getattr(
            args, "common_evaluation_contract", None
        ),
    )
    planner_cmd, start_cmd = build_commands(
        scenario_id, baseline_id, run_dir, obstacle_params, classes_arg, len(obstacles), scenario,
        global_path_ready_gate=getattr(args, "global_path_ready_gate", False),
        global_path_start_gate_topic=getattr(
            args, "global_path_start_gate_topic", "/teacher_v1/global_path_ready"
        ),
    )

    processes = []
    unexpected_exits = []
    roscore = None
    roscore_log = None
    env = process_env(run_dir)
    try:
        if args.roscore == "auto":
            roscore, roscore_log = start_process(["roscore"], run_dir / "roscore.log", env=env)
            time.sleep(3.0)

        planner, planner_log = start_process(planner_cmd, run_dir / "planner.log", env=env)
        processes.append(("planner", planner, planner_log))
        time.sleep(3.0)

        starter, starter_log = start_process(start_cmd, run_dir / "start_test.log", env=env)
        processes.append(("start_test", starter, starter_log))

        gate_record = {
            "enabled": bool(getattr(args, "global_path_ready_gate", False)),
            "ready": True,
            "wait_sec": 0.0,
        }
        if gate_record["enabled"]:
            gate_started = time.time()
            gate_record.update(wait_for_global_path_ready(
                getattr(
                    args, "global_path_ready_topic",
                    "/global_path",
                ),
                getattr(args, "global_path_ready_timeout_sec", 15.0),
                getattr(args, "global_path_ready_min_span_m", 0.25),
                env=env,
            ))
            gate_record["wait_sec"] = time.time() - gate_started
            if gate_record["ready"]:
                release_global_path_start_gate(
                    getattr(
                        args, "global_path_start_gate_topic",
                        "/teacher_v1/global_path_ready",
                    ),
                    env=env,
                )
                gate_record["released_at"] = dt.datetime.now(
                    dt.timezone.utc
                ).isoformat()
            else:
                unexpected_exits.append({
                    "process": "global_path_ready_gate", "return_code": -1,
                })
            atomic_write_text(
                run_dir / "global_path_ready_gate.yaml",
                yaml.safe_dump(gate_record, sort_keys=False),
            )

        deadline = time.time() + args.duration_sec
        while time.time() < deadline:
            for name, proc, _ in processes:
                return_code = proc.poll()
                if return_code is not None:
                    unexpected_exits.append(
                        {"process": name, "return_code": int(return_code)}
                    )
            if roscore is not None and roscore.poll() is not None:
                unexpected_exits.append(
                    {"process": "roscore", "return_code": int(roscore.returncode)}
                )
            if unexpected_exits:
                break
            time.sleep(1.0)
    finally:
        for _, proc, log_file in reversed(processes):
            stop_process(proc)
            log_file.close()
        if roscore is not None:
            stop_process(roscore)
        if roscore_log is not None:
            roscore_log.close()

    process_contract_passed = not unexpected_exits
    atomic_write_text(
        run_dir / "process_status.yaml",
        yaml.safe_dump(
            {
                "unexpected_exits": unexpected_exits,
                "process_contract_passed": process_contract_passed,
            },
            sort_keys=False,
        ),
    )
    verify_repository_context_unchanged(trial_run_context)

    if baseline_id == "B1_ACBF_fixed":
        verify_passed, verify_note = None, "B1 has no Guard log"
    else:
        switches = scenario_switches(baseline_id, scenario)
        verify_passed, verify_note = run_verify(
            run_dir,
            epsilon_max=switches["epsilon_max"],
            delta_bar_beta=scenario.get("theory", {}).get("delta_bar_beta", 0.3),
        )

    metadata_contract_passed, metadata_errors = validate_run_meta_contract(
        run_dir, strict_provenance=True
    )
    csv_contract_passed, csv_contract_note = run_csv_contract_check(run_dir)
    pre_summary_invalid_reasons = list(metadata_errors)
    pre_summary_invalid_reasons.extend(
        f"unexpected process exit: {entry['process']}={entry['return_code']}"
        for entry in unexpected_exits
    )
    summary_result = write_summary(
        run_dir,
        scenario_id,
        baseline_id,
        [("planner", planner_cmd), ("start_test", start_cmd)],
        verify_passed,
        verify_note,
        args.duration_sec,
        csv_contract_passed=csv_contract_passed,
        csv_contract_note=csv_contract_note,
        metadata_contract_passed=metadata_contract_passed,
        process_contract_passed=process_contract_passed,
        invalid_reasons=pre_summary_invalid_reasons,
    )
    validation = {
        "trial_valid": summary_result["trial_valid"],
        "termination_reason": summary_result["termination_reason"],
        "csv_contract_passed": csv_contract_passed,
        "metadata_contract_passed": metadata_contract_passed,
        "process_contract_passed": process_contract_passed,
        "safety_bound_passed": verify_passed,
        "invalid_reasons": summary_result["invalid_reasons"],
    }
    sentinel = seal_trial(run_dir, summary_result["trial_valid"], validation)
    status = "COMPLETE" if sentinel.name == RUN_COMPLETE_SENTINEL else "INVALID"
    print(f"{status} {scenario_id} / {baseline_id}: {run_dir}")
    return run_dir


def main():
    root = repo_root()
    default_output = TEACHER_OUTPUT_ROOT
    parser = argparse.ArgumentParser(description="Run MPC-SECBF simulation experiments")
    parser.add_argument("--scenario", default="all")
    parser.add_argument("--baseline", default="all")
    parser.add_argument("--duration-sec", type=int, default=90)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--seed-manifest")
    parser.add_argument(
        "--campaign", choices=sorted(CAMPAIGN_PROFILES), default="main",
    )
    parser.add_argument("--execution-tier", choices=["smoke", "formal"], default="smoke")
    parser.add_argument(
        "--parameter-freeze",
        help="Campaign-specific frozen execution YAML.",
    )
    parser.add_argument(
        "--evaluation-contract",
        default=str(COMMON_OFFLINE_EVALUATION_CONTRACT),
        help="Method-independent Teacher offline evaluation contract.",
    )
    parser.add_argument("--output-root", default=str(default_output))
    parser.add_argument("--protocol-id")
    parser.add_argument("--roscore", choices=["auto", "external"], default="auto")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--skip-existing-complete", action="store_true")
    parser.add_argument(
        "--global-path-ready-gate", action="store_true",
        help="hold dynamic obstacles until a meaningful global MPC reference is published",
    )
    parser.add_argument(
        "--global-path-ready-topic",
        default="/global_path",
    )
    parser.add_argument(
        "--global-path-start-gate-topic",
        default="/teacher_v1/global_path_ready",
    )
    parser.add_argument("--global-path-ready-timeout-sec", type=float, default=15.0)
    parser.add_argument("--global-path-ready-min-span-m", type=float, default=0.25)
    parser.add_argument(
        "--config",
        default=str(root / "swarm_test/config/secbf_scenarios.yaml"),
    )
    args = parser.parse_args()
    args.output_root = str(Path(args.output_root).expanduser().resolve())
    args.config = str(Path(args.config).expanduser().resolve())
    if args.seed_manifest:
        args.seed_manifest = str(Path(args.seed_manifest).expanduser().resolve())
    if args.parameter_freeze:
        args.parameter_freeze = str(Path(args.parameter_freeze).expanduser().resolve())
    elif args.execution_tier == "smoke":
        args.parameter_freeze = str(
            PARAMETER_FREEZE_ROOT / f"{args.campaign}_smoke.yaml"
        )
    else:
        parser.error("--parameter-freeze is required for formal execution")
    args.evaluation_contract = str(
        Path(args.evaluation_contract).expanduser().resolve()
    )
    if args.duration_sec <= 0:
        parser.error("--duration-sec must be positive")
    if args.repeat <= 0:
        parser.error("--repeat must be positive")
    if args.global_path_ready_timeout_sec <= 0.0:
        parser.error("--global-path-ready-timeout-sec must be positive")
    if args.global_path_ready_min_span_m <= 0.0:
        parser.error("--global-path-ready-min-span-m must be positive")
    try:
        args.output_root = str(ensure_teacher_output_root(Path(args.output_root)))
    except ValueError as exc:
        parser.error(str(exc))
    try:
        args.execution_freeze = load_parameter_freeze(
            Path(args.parameter_freeze), args.execution_tier,
            campaign=args.campaign,
        )
        args.common_evaluation_contract = load_common_offline_evaluation_contract(
            Path(args.evaluation_contract)
        )
    except ParameterFreezeError as exc:
        parser.error(f"Teacher-v1 freeze preflight failed: {exc}")
    if not args.dry_run:
        if not str(args.protocol_id or "").strip():
            parser.error("--protocol-id is required for every non-dry Teacher-v1 run")
        try:
            args.run_context = collect_run_context(args)
        except (OSError, RuntimeError, ValueError) as exc:
            parser.error(f"Teacher-v1 provenance preflight failed: {exc}")
    else:
        args.protocol_id = args.protocol_id or "teacher_v1_dry_run"
        args.run_context = None

    scenarios = load_scenarios(Path(args.config))
    scenario_ids = selected(SCENARIO_INDEX.keys(), args.scenario)
    requested_baselines = selected(list(BASELINES) + list(PAPER_BASELINE_ALIASES), args.baseline)
    baseline_ids = [resolve_baseline_alias(value) for value in requested_baselines]
    if args.execution_tier == "formal":
        profile = CAMPAIGN_PROFILES[args.campaign]
        if tuple(scenario_ids) != tuple(profile["scenarios"]):
            parser.error(
                f"formal {args.campaign} execution must select its exact "
                "frozen scenario matrix"
            )
        if tuple(requested_baselines) != tuple(profile["methods"]):
            parser.error(
                f"formal {args.campaign} execution must select its exact "
                "frozen method matrix"
            )
    if args.seed_manifest and args.repeat != 1:
        parser.error("--seed-manifest cannot be combined with --repeat != 1")
    trials = load_seed_manifest(Path(args.seed_manifest)) if args.seed_manifest else []
    if args.seed_manifest:
        manifest_scenarios = {trial["scenario_id"] for trial in trials}
        missing_scenarios = sorted(set(scenario_ids) - manifest_scenarios)
        if missing_scenarios:
            parser.error(
                "--seed-manifest has no trials for selected scenario(s): "
                + ", ".join(missing_scenarios)
            )
    base_timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    completed_runs = []
    invalid_runs = []

    for repeat_idx in range(args.repeat):
        timestamp = base_timestamp if args.repeat == 1 else f"{base_timestamp}_r{repeat_idx + 1:02d}"
        for scenario_id in scenario_ids:
            scenario_trials = (
                [trial for trial in trials if trial["scenario_id"] == scenario_id]
                if args.seed_manifest else [None]
            )
            for trial in scenario_trials:
                for requested, baseline_id in zip(requested_baselines, baseline_ids):
                    run_suffix = trial["trial_id"] if trial is not None else timestamp
                    expected_run_dir = (
                        Path(args.output_root)
                        / f"{run_suffix}_{scenario_id}_{baseline_id}"
                    )
                    try:
                        run_dir = run_one(
                            scenario_id, baseline_id, scenarios[scenario_id], args,
                            timestamp, trial=trial,
                            requested_baseline_label=requested,
                        )
                    except Exception as exc:
                        if not args.dry_run:
                            try:
                                mark_exception_invalid(expected_run_dir, exc)
                            except Exception as sentinel_exc:
                                print(
                                    f"ERROR could not seal invalid trial {expected_run_dir}: "
                                    f"{sentinel_exc}",
                                    file=sys.stderr,
                                )
                        print(
                            f"ERROR {scenario_id} / {baseline_id}: {exc}",
                            file=sys.stderr,
                        )
                        invalid_runs.append(
                            f"{scenario_id}/{baseline_id}: {exc}"
                        )
                        continue
                    if run_dir is not None:
                        if is_complete_run_dir(run_dir):
                            completed_runs.append(run_dir)
                        else:
                            invalid_runs.append(str(run_dir))

    if completed_runs:
        aggregate_csv = write_aggregate_summary(Path(args.output_root), completed_runs)
        if aggregate_csv is not None:
            print(f"Wrote aggregate summary: {aggregate_csv}")
    if invalid_runs:
        print(
            f"Teacher-v1 batch invalid: {len(invalid_runs)} trial(s)",
            file=sys.stderr,
        )
        for invalid in invalid_runs:
            print(f"  - {invalid}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
