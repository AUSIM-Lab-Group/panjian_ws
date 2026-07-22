#!/usr/bin/env python3
"""Run MPC-SECBF numerical simulation experiment matrix."""

import argparse
import copy
import csv
import datetime as dt
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
    "event_log.csv",
)
GOAL_TOLERANCE_M = 0.55
DEADLOCK_MEAN_ABS_V_MPS = 0.05


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

DEFAULT_MU_WEIGHTS = {"bias": 0.6, "heading": 0.2, "ttc": 0.15, "density": 0.1}

DEFAULT_EXPERIMENT_SWITCHES = {
    "semantic_mode": "full",
    "enable_rate_limit": "true",
    "enable_available_projection": "true",
    "enable_guard_fallback": "true",
    "mpc_feasibility_guard_enabled": "true",
    "fixed_beta": 0.4,
    "epsilon_max": 0.05,
    "slack_weight": 1000.0,
    "max_cbf_obstacles": 6,
    "cbf_metric": "seesm",
    "front_adsm": "true",
    "global_seesm_enable": "false",
    "dynamic_tau_enabled": False,
    "dynamic_tau_ke": 0.30,
    "dynamic_tau_tmax": 2.0,
    "dynamic_tau_min_speed": 1e-6,
    "dynamic_tau_min_distance": 1e-6,
    "dynamic_tau_max_tau": 2.0,
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
    "dynamic_tau_ke",
    "dynamic_tau_tmax",
    "dynamic_tau_min_speed",
    "dynamic_tau_min_distance",
    "dynamic_tau_max_tau",
)


def bool_switch(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


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
    if baseline_id in {
        "No_semantic", "Fixed_margin", "Category_only",
        "Unguarded_SEESM", "SEESM_Ours",
    }:
        values["dynamic_tau_enabled"] = True
    elif baseline_id in {
        "No_J_side", "SideWeight_005", "SideWeight_010",
        "SideWeight_020", "SideWeight_050",
    }:
        values["dynamic_tau_enabled"] = True
    else:
        values["dynamic_tau_enabled"] = False
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
    return values


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
                   requested_baseline_label=None) -> Path:
    meta_path = run_dir / "meta.yaml"
    start_x, start_y = scenario_start_xy(scenario)
    goal_x, goal_y = scenario_goal_xy(scenario)
    map_cfg = scenario.get("map", {})
    baseline = BASELINES[baseline_id]
    switches = scenario_switches(baseline_id, scenario)
    meta = {
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
        "obstacle_classes": classes_arg,
        "obstacles": scenario.get("obstacles", []),
        "beta_table": scenario_beta_bar(baseline_id, scenario),
        "mu_weights": scenario_mu_weights(baseline_id, scenario),
        "guard_eta": 0.10,
        "guard_enable": baseline["guard_enabled"],
        "semantic_mode": switches["semantic_mode"],
        "enable_rate_limit": switches["enable_rate_limit"],
        "enable_available_projection": switches["enable_available_projection"],
        "enable_guard_fallback": switches["enable_guard_fallback"],
        "mpc_feasibility_guard_enabled": switches["mpc_feasibility_guard_enabled"],
        "fixed_beta": switches["fixed_beta"],
        "epsilon_max": switches["epsilon_max"],
        "slack_weight": switches["slack_weight"],
        "max_cbf_obstacles": switches["max_cbf_obstacles"],
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
        "dynamic_tau": {
            "enabled": bool_switch(switches["dynamic_tau_enabled"]),
            "Ke": float(switches["dynamic_tau_ke"]),
            "Tmax": float(switches["dynamic_tau_tmax"]),
            "min_speed": float(switches["dynamic_tau_min_speed"]),
            "min_distance": float(switches["dynamic_tau_min_distance"]),
            "max_tau": float(switches["dynamic_tau_max_tau"]),
            "formula": "tau=f_r*f_v*f_T*Ke*T_i",
            "h_ee": "||l+tau*v||-R_obs-R_robot",
            "h_see": "h_ee-beta",
            "beta_source": baseline_beta_source(baseline_id),
        },
        "guard_tau": 0.20,
        "mpc_horizon": 20,
        "dt": 0.10,
        "planner_v_max": scenario_planner_v_max(scenario),
        "random_seed": 1,
        "duration_sec": duration_sec,
        "safety_contract": {
            "h_see": "||p_rel + tau v_rel|| - R_obs - R_robot - beta_i",
            "guard_upper_bound": "min(beta_bar_i, max(0, h_EE - eta))",
            "fixed_margin_baseline": "beta_i = d_safe",
        },
        "required_logs": [
            "robot_log.csv",
            "obstacle_log.csv",
            "margin_guard_log.csv",
            "planner_log.csv",
            "timing_log.csv",
            "event_log.csv",
        ],
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
    with meta_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(meta, f, sort_keys=False, allow_unicode=True)
    return meta_path


def obstacle_classes(obstacles: list) -> str:
    classes = [obs.get("semantic_class", "unknown") for obs in obstacles]
    return "[" + ",".join(classes) + "]"


def build_commands(scenario_id: str, baseline_id: str, run_dir: Path, obstacle_params: Path,
                   classes_arg: str, num_obs: int, scenario: dict):
    baseline = BASELINES[baseline_id]
    start_x, start_y = scenario_start_xy(scenario)
    goal_x, goal_y = scenario_goal_xy(scenario)
    map_cfg = scenario.get("map", {})
    beta_bar = scenario_beta_bar(baseline_id, scenario)
    mu_weights = scenario_mu_weights(baseline_id, scenario)
    switches = scenario_switches(baseline_id, scenario)
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
            f"fixed_beta:={switches['fixed_beta']}",
            f"epsilon_max:={switches['epsilon_max']}",
            f"slack_weight:={switches['slack_weight']}",
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
            f"dynamic_tau_ke:={switches['dynamic_tau_ke']}",
            f"dynamic_tau_tmax:={switches['dynamic_tau_tmax']}",
            f"dynamic_tau_min_speed:={switches['dynamic_tau_min_speed']}",
            f"dynamic_tau_min_distance:={switches['dynamic_tau_min_distance']}",
            f"dynamic_tau_max_tau:={switches['dynamic_tau_max_tau']}",
            f"output_dir:={run_dir}",
            f"obstacle_classes:={classes_arg}",
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
    no_cbf = sum(1 for row in rows if row.get("accepted_beta_source") == "no_cbf" or row.get("used_fallback") in {"1", "True", "true"})
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


def summarize_tau_log(run_dir: Path) -> dict:
    metrics = {
        "tau_mean": "",
        "tau_max": "",
        "tau_active_fraction": "",
        "tau_invalid_count": 0,
        "tau_reason_counts": "",
        "tau_source": "",
    }
    tau_fields = {"tau", "T_i", "f_r", "f_v", "f_T", "tau_valid", "tau_reason"}
    candidates = (
        run_dir / "margin_guard_log.csv",
        run_dir / "planner_log.csv",
        run_dir / "global_seesm_log.csv",
    )
    for source in candidates:
        if not source.exists():
            continue
        with source.open("r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            header = set(reader.fieldnames or ())
            if not tau_fields.issubset(header):
                continue
            rows = list(reader)

        tau_values = []
        active_count = 0
        invalid_count = 0
        reason_counts = {}
        for row in rows:
            try:
                values = [float(row[field]) for field in ("tau", "T_i", "f_r", "f_v", "f_T")]
            except (KeyError, TypeError, ValueError):
                continue
            if not all(math.isfinite(value) for value in values):
                continue
            valid = str(row.get("tau_valid", "")).strip().lower()
            reason = str(row.get("tau_reason", "")).strip()
            if valid not in {"0", "1", "true", "false", "yes", "no"} or not reason:
                continue
            tau = values[0]
            reason_counts[reason] = reason_counts.get(reason, 0) + 1
            if valid in {"0", "false", "no"}:
                invalid_count += 1
                continue
            tau_values.append(tau)
            active_count += int(tau > 0.0)

        if not tau_values:
            continue
        metrics["tau_mean"] = f"{sum(tau_values) / len(tau_values):.6f}"
        metrics["tau_max"] = f"{max(tau_values):.6f}"
        metrics["tau_active_fraction"] = f"{active_count / len(tau_values):.6f}"
        metrics["tau_invalid_count"] = invalid_count
        metrics["tau_reason_counts"] = ";".join(
            f"{reason}:{reason_counts[reason]}" for reason in sorted(reason_counts)
        )
        metrics["tau_source"] = source.name
        break
    return metrics


def dynamic_tau_audit(run_dir: Path) -> dict:
    metrics = {
        "dynamic_tau_enabled": False,
        "dynamic_tau_ke": "",
        "dynamic_tau_tmax": "",
        "dynamic_tau_min_speed": "",
        "dynamic_tau_min_distance": "",
        "dynamic_tau_max_tau": "",
        "dynamic_tau_formula": "",
        "dynamic_tau_h_ee": "",
        "dynamic_tau_h_see": "",
        "dynamic_tau_beta_source": "",
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
            ("Ke", "dynamic_tau_ke"),
            ("Tmax", "dynamic_tau_tmax"),
            ("min_speed", "dynamic_tau_min_speed"),
            ("min_distance", "dynamic_tau_min_distance"),
            ("max_tau", "dynamic_tau_max_tau"),
        ):
            if source_key in dynamic:
                metrics[output_key] = float(dynamic[source_key])
        for source_key, output_key in (
            ("formula", "dynamic_tau_formula"),
            ("h_ee", "dynamic_tau_h_ee"),
            ("h_see", "dynamic_tau_h_see"),
            ("beta_source", "dynamic_tau_beta_source"),
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
                                phase5_metrics: dict, planner_metrics: dict) -> str:
    """Classify one trial using only auditable logs and final metrics.

    Collision takes priority over goal arrival, because a trial that reaches the
    goal after crossing an obstacle is not a successful safety trial. A transient
    infeasible MPC solve is classified only when the trial does not eventually
    reach the goal; this preserves the distinction between recovery and failure.
    """
    missing = [
        name for name in REQUIRED_TRIAL_LOGS
        if not (run_dir / name).exists() or (run_dir / name).stat().st_size <= 1
    ]
    if missing or phase5_metrics.get("robot_records", 0) <= 0 or planner_metrics.get("planner_records", 0) <= 0:
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

    mean_abs_v = as_float(phase5_metrics, "robot_mean_abs_v")
    final_goal_distance = as_float(phase5_metrics, "robot_final_goal_distance_m")
    if (
        mean_abs_v is not None
        and final_goal_distance is not None
        and final_goal_distance > GOAL_TOLERANCE_M
        and mean_abs_v < DEADLOCK_MEAN_ABS_V_MPS
    ):
        return "deadlock"
    return "timeout"


def guard_log_path(run_dir: Path) -> Path:
    margin_log = run_dir / "margin_guard_log.csv"
    if margin_log.exists():
        return margin_log
    return run_dir / "guard_log.csv"


def write_summary(run_dir: Path, scenario_id: str, baseline_id: str, commands,
                  verify_passed, verify_note, duration_sec):
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
        run_dir, nav_metrics, phase5_metrics, planner_metrics
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
        f"- slack_max: {planner_metrics['slack_max']}",
        f"- slack_mean: {planner_metrics['slack_mean']}",
        f"- solve_time_mean_ms: {planner_metrics['solve_time_mean_ms']}",
        f"- tau_mean: {tau_metrics['tau_mean']}",
        f"- tau_max: {tau_metrics['tau_max']}",
        f"- tau_active_fraction: {tau_metrics['tau_active_fraction']}",
        f"- tau_invalid_count: {tau_metrics['tau_invalid_count']}",
        f"- tau_reason_counts: {tau_metrics['tau_reason_counts']}",
        f"- tau_source: {tau_metrics['tau_source']}",
        "",
        "## Dynamic tau audit",
        f"- enabled: {tau_audit['dynamic_tau_enabled']}",
        f"- Ke: {tau_audit['dynamic_tau_ke']}",
        f"- Tmax: {tau_audit['dynamic_tau_tmax']}",
        f"- min_speed: {tau_audit['dynamic_tau_min_speed']}",
        f"- min_distance: {tau_audit['dynamic_tau_min_distance']}",
        f"- max_tau: {tau_audit['dynamic_tau_max_tau']}",
        f"- formula: {tau_audit['dynamic_tau_formula']}",
        f"- h_ee: {tau_audit['dynamic_tau_h_ee']}",
        f"- h_see: {tau_audit['dynamic_tau_h_see']}",
        f"- beta_source: {tau_audit['dynamic_tau_beta_source']}",
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
                "termination_reason", "goal_reached", "success", "robot_records", "robot_path_length_m",
                "robot_travel_time_s", "robot_final_goal_distance_m",
                "robot_mean_abs_v", "robot_mean_abs_w",
                "robot_velocity_smoothness", "robot_control_effort",
                "log_min_distance_m", "log_min_h_ee", "log_invalid_obstacle_rows",
                "safety_bound_passed", "guard_records", "semantic_classes",
                "beta_applied_mean", "beta_applied_max", "h_ee_min", "h_see_min",
                "delta_beta_max", "guard_rollback_count", "guard_rollback_rate",
                "planner_records", "constrained_obs_count_mean", "constrained_obs_count_max",
                "first_infeasible_count", "first_infeasible_rate",
                "mpc_guard_used_count", "mpc_guard_used_rate", "no_cbf_fallback_count",
                "no_cbf_fallback_rate", "slack_max", "slack_mean",
                "solve_time_mean_ms", "solve_time_max_ms",
                "side_preference_enabled", "side_weight", "side_cost_mean", "side_cost_max",
                "tracking_rmse_m", "tracking_error_max_m",
                "side_candidate_count_mean", "side_multi_candidate_rate",
                "side_dynamic_obstacle_count_mean", "side_crowd_suppression_rate",
                "side_dominant_active_count", "side_dominant_active_rate",
                "side_dominant_tau_max", "side_dominant_h_min",
                "tau_mean", "tau_max", "tau_active_fraction", "tau_invalid_count",
                "tau_reason_counts", "tau_source",
                "dynamic_tau_enabled", "dynamic_tau_ke", "dynamic_tau_tmax",
                "dynamic_tau_min_speed", "dynamic_tau_min_distance", "dynamic_tau_max_tau",
                "dynamic_tau_formula", "dynamic_tau_h_ee", "dynamic_tau_h_see",
                "dynamic_tau_beta_source", "output_dir",
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
            **nav_metrics,
            **phase5_metrics,
            "safety_bound_passed": verify_passed,
            **guard_metrics,
            **planner_metrics,
            **tau_metrics,
            **tau_audit,
            "output_dir": str(run_dir),
        })


def write_aggregate_summary(output_root: Path, run_dirs: list):
    rows = []
    fieldnames = []
    fieldname_set = set()
    for run_dir in run_dirs:
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


def run_one(scenario_id: str, baseline_id: str, scenario: dict, args, timestamp: str,
            trial=None, requested_baseline_label=None):
    run_suffix = trial["trial_id"] if trial is not None else timestamp
    run_dir = Path(args.output_root) / f"{run_suffix}_{scenario_id}_{baseline_id}"
    summary_path = run_dir / "summary.csv"
    if getattr(args, "skip_existing_complete", False) and summary_path.exists() and summary_path.stat().st_size > 0:
        print(f"Skipped complete {scenario_id} / {baseline_id}: {run_dir}")
        return run_dir
    if run_dir.exists() and not summary_path.exists():
        quarantined = quarantine_incomplete_run_dir(run_dir, timestamp)
        print(f"Quarantined incomplete {scenario_id} / {baseline_id}: {quarantined}")
    if trial is not None:
        scenario, obstacles = materialize_trial(scenario, trial)
    else:
        obstacles = scenario.get("obstacles", [])
    if not obstacles:
        raise RuntimeError(f"Scenario {scenario_id} has no obstacles")

    classes_arg = obstacle_classes(obstacles)
    obstacle_params = run_dir / "obstacles_param.yaml"
    planner_cmd, start_cmd = build_commands(
        scenario_id, baseline_id, run_dir, obstacle_params, classes_arg, len(obstacles), scenario
    )

    commands = [("planner", planner_cmd), ("start_test", start_cmd)]
    if args.dry_run:
        print(f"\n[{scenario_id} / {baseline_id}]")
        print(f"output_dir: {run_dir}")
        print(f"obstacle_classes: {classes_arg}")
        for name, cmd in commands:
            print(f"{name}: {' '.join(cmd)}")
        return None

    run_dir.mkdir(parents=True, exist_ok=True)
    if trial is not None:
        write_trial_artifacts(run_dir, scenario, trial, baseline_id)
    obstacle_params = write_obstacle_params(run_dir, obstacles)
    reference_waypoints = None
    if is_reference_path_scene(scenario):
        _, reference_waypoints = write_reference_path_config(run_dir, scenario)
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
    )
    planner_cmd, start_cmd = build_commands(
        scenario_id, baseline_id, run_dir, obstacle_params, classes_arg, len(obstacles), scenario
    )

    processes = []
    roscore = None
    roscore_log = None
    env = process_env(run_dir)
    try:
        if args.roscore == "auto":
            roscore, roscore_log = start_process(["roscore"], run_dir / "roscore.log", env=env)
            time.sleep(3.0)

        planner, planner_log = start_process(planner_cmd, run_dir / "planner.log", env=env)
        processes.append((planner, planner_log))
        time.sleep(3.0)

        starter, starter_log = start_process(start_cmd, run_dir / "start_test.log", env=env)
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
        switches = scenario_switches(baseline_id, scenario)
        verify_passed, verify_note = run_verify(
            run_dir,
            epsilon_max=switches["epsilon_max"],
            delta_bar_beta=scenario.get("theory", {}).get("delta_bar_beta", 0.3),
        )

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
    return run_dir


def main():
    root = repo_root()
    default_output = root / "swarm_test/output/secbf_runs"
    parser = argparse.ArgumentParser(description="Run MPC-SECBF simulation experiments")
    parser.add_argument("--scenario", default="all")
    parser.add_argument("--baseline", default="all")
    parser.add_argument("--duration-sec", type=int, default=90)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--seed-manifest")
    parser.add_argument("--output-root", default=str(default_output))
    parser.add_argument("--roscore", choices=["auto", "external"], default="auto")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--skip-existing-complete", action="store_true")
    parser.add_argument(
        "--config",
        default=str(root / "swarm_test/config/secbf_scenarios.yaml"),
    )
    args = parser.parse_args()
    args.output_root = str(Path(args.output_root).expanduser().resolve())
    args.config = str(Path(args.config).expanduser().resolve())

    scenarios = load_scenarios(Path(args.config))
    scenario_ids = selected(SCENARIO_INDEX.keys(), args.scenario)
    requested_baselines = selected(list(BASELINES) + list(PAPER_BASELINE_ALIASES), args.baseline)
    baseline_ids = [resolve_baseline_alias(value) for value in requested_baselines]
    if args.seed_manifest and args.repeat != 1:
        parser.error("--seed-manifest cannot be combined with --repeat != 1")
    trials = load_seed_manifest(Path(args.seed_manifest)) if args.seed_manifest else []
    base_timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    completed_runs = []

    for repeat_idx in range(args.repeat):
        timestamp = base_timestamp if args.repeat == 1 else f"{base_timestamp}_r{repeat_idx + 1:02d}"
        for scenario_id in scenario_ids:
            scenario_trials = [trial for trial in trials if trial["scenario_id"] == scenario_id] or [None]
            for trial in scenario_trials:
                for requested, baseline_id in zip(requested_baselines, baseline_ids):
                    run_dir = run_one(scenario_id, baseline_id, scenarios[scenario_id], args, timestamp,
                                      trial=trial, requested_baseline_label=requested)
                    if run_dir is not None:
                        completed_runs.append(run_dir)

    if completed_runs:
        aggregate_csv = write_aggregate_summary(Path(args.output_root), completed_runs)
        if aggregate_csv is not None:
            print(f"Wrote aggregate summary: {aggregate_csv}")


if __name__ == "__main__":
    main()
