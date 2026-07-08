#!/usr/bin/env python3
"""Run MPC-SECBF numerical simulation experiment matrix."""

import argparse
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
    "Exp2_category_box": 21,
    "Exp2_category_adult": 22,
    "Exp2_category_child_like": 23,
    "Exp2_category_cyclist": 24,
    "Exp3_context_static": 31,
    "Exp3_context_same_direction": 32,
    "Exp3_context_crossing": 33,
    "Exp3_context_frontal_approaching": 34,
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
        "mpc_feasibility_guard_enabled": "false",
        "enable_rate_limit": "false",
        "enable_available_projection": "false",
        "enable_guard_fallback": "false",
        "experiment_label": "Unguarded_SEESM",
    },
    "B3_SECBF_with_guard": {
        "planner": "secbf_planner.launch",
        "controller_index": 6,
        "guard_enabled": "true",
        "experiment_label": "SEESM_Ours",
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
        "guard_enabled": "true",
        "experiment_label": "No_semantic",
        "semantic_mode": "none",
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
    },
    "SEESM_Ours": {
        "planner": "secbf_planner.launch",
        "controller_index": 6,
        "guard_enabled": "true",
        "experiment_label": "SEESM_Ours",
    },
}

DEFAULT_BETA_BAR = {
    "box": 0.1,
    "adult": 0.4,
    "pedestrian": 0.4,
    "child": 0.7,
    "child_like": 0.7,
    "cyclist": 0.6,
    "vehicle": 0.5,
    "unknown": 0.4,
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
    return values


def scenario_switches(baseline_id: str, scenario: dict) -> dict:
    values = baseline_switches(baseline_id)
    values.update(scenario.get("experiment_switches", {}))
    return values


def write_run_meta(run_dir: Path, scenario_id: str, baseline_id: str, scenario: dict,
                   classes_arg: str, num_obs: int, duration_sec: int) -> Path:
    meta_path = run_dir / "meta.yaml"
    goal = scenario.get("goal", {})
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
        "map_name": (
            "corridor_12m_x_6m" if scenario_id.startswith(("Exp2_", "Exp3_"))
            else "paper_style_supplementary_2d" if scenario_id.startswith(("avocado_", "drmpc_"))
            else "teacher_canonical_2d"
        ),
        "start": [0.0, 0.0, 0.0],
        "goal": [float(goal.get("x", 0.0)), float(goal.get("y", 0.0)), 0.0],
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
        "guard_tau": 0.20,
        "mpc_horizon": 20,
        "dt": 0.10,
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
    with meta_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(meta, f, sort_keys=False, allow_unicode=True)
    return meta_path


def obstacle_classes(obstacles: list) -> str:
    classes = [obs.get("semantic_class", "unknown") for obs in obstacles]
    return "[" + ",".join(classes) + "]"


def build_commands(scenario_id: str, baseline_id: str, run_dir: Path, obstacle_params: Path,
                   classes_arg: str, num_obs: int, scenario: dict):
    baseline = BASELINES[baseline_id]
    goal = scenario.get("goal", {})
    map_cfg = scenario.get("map", {})
    beta_bar = scenario_beta_bar(baseline_id, scenario)
    mu_weights = scenario_mu_weights(baseline_id, scenario)
    switches = scenario_switches(baseline_id, scenario)
    common_start = [
        "roslaunch", "swarm_test", "start_test.launch",
        f"scenario_index:={SCENARIO_INDEX[scenario_id]}",
        f"controller_index:={baseline['controller_index']}",
        f"output_dir:={run_dir}",
        f"num_of_obs:={num_obs}",
        f"obstacle_params_file:={obstacle_params}",
        f"obstacle_classes:={classes_arg}",
        f"goal_x:={goal.get('x', 21.0)}",
        f"goal_y:={goal.get('y', 0.0)}",
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
            f"semantic_mode:={switches['semantic_mode']}",
            f"enable_rate_limit:={switches['enable_rate_limit']}",
            f"enable_available_projection:={switches['enable_available_projection']}",
            f"enable_guard_fallback:={switches['enable_guard_fallback']}",
            f"mpc_feasibility_guard_enabled:={switches['mpc_feasibility_guard_enabled']}",
            f"fixed_beta:={switches['fixed_beta']}",
            f"epsilon_max:={switches['epsilon_max']}",
            f"slack_weight:={switches['slack_weight']}",
            f"output_dir:={run_dir}",
            f"obstacle_classes:={classes_arg}",
            f"map_size_x:={map_cfg.get('x', 50.0)}",
            f"map_size_y:={map_cfg.get('y', 50.0)}",
            f"map_size_z:={map_cfg.get('z', 3.0)}",
            f"goal_x:={goal.get('x', 10.0)}",
            f"goal_y:={goal.get('y', 0.0)}",
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
    slack_max = floats("slack_max") or floats("slack")
    slack_mean = floats("slack_mean")
    solve_time = floats("solve_time_ms")

    metrics["planner_records"] = total
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
                metrics["success"] = int(final_dist <= 0.55)
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
    nav_metrics = summarize_data_processor(data_summary, duration_sec)
    phase5_metrics = summarize_phase5_logs(run_dir)

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
        f"- success: {phase5_metrics['success']}",
        f"- robot_path_length_m: {phase5_metrics['robot_path_length_m']}",
        f"- robot_final_goal_distance_m: {phase5_metrics['robot_final_goal_distance_m']}",
        f"- log_min_distance_m: {phase5_metrics['log_min_distance_m']}",
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
        f"- first_infeasible_rate: {planner_metrics['first_infeasible_rate']}",
        f"- mpc_guard_used_rate: {planner_metrics['mpc_guard_used_rate']}",
        f"- no_cbf_fallback_rate: {planner_metrics['no_cbf_fallback_rate']}",
        f"- slack_max: {planner_metrics['slack_max']}",
        f"- slack_mean: {planner_metrics['slack_mean']}",
        f"- solve_time_mean_ms: {planner_metrics['solve_time_mean_ms']}",
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
                "success", "robot_records", "robot_path_length_m",
                "robot_travel_time_s", "robot_final_goal_distance_m",
                "robot_mean_abs_v", "robot_mean_abs_w",
                "robot_velocity_smoothness", "robot_control_effort",
                "log_min_distance_m", "log_min_h_ee",
                "safety_bound_passed", "guard_records", "semantic_classes",
                "beta_applied_mean", "beta_applied_max", "h_ee_min", "h_see_min",
                "delta_beta_max", "guard_rollback_count", "guard_rollback_rate",
                "planner_records", "first_infeasible_count", "first_infeasible_rate",
                "mpc_guard_used_count", "mpc_guard_used_rate", "no_cbf_fallback_count",
                "no_cbf_fallback_rate", "slack_max", "slack_mean",
                "solve_time_mean_ms", "solve_time_max_ms", "output_dir",
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
            "output_dir": str(run_dir),
        })


def write_aggregate_summary(output_root: Path, run_dirs: list):
    rows = []
    fieldnames = None
    for run_dir in run_dirs:
        summary_csv = run_dir / "summary.csv"
        if not summary_csv.exists():
            continue
        with summary_csv.open("r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            if fieldnames is None:
                fieldnames = reader.fieldnames
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
        writer.writerows(rows)
    return aggregate_csv


def run_one(scenario_id: str, baseline_id: str, scenario: dict, args, timestamp: str):
    run_dir = Path(args.output_root) / f"{timestamp}_{scenario_id}_{baseline_id}"
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
    obstacle_params = write_obstacle_params(run_dir, obstacles)
    write_run_meta(run_dir, scenario_id, baseline_id, scenario, classes_arg, len(obstacles), args.duration_sec)
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
    parser.add_argument("--output-root", default=str(default_output))
    parser.add_argument("--roscore", choices=["auto", "external"], default="auto")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--config",
        default=str(root / "swarm_test/config/secbf_scenarios.yaml"),
    )
    args = parser.parse_args()
    args.output_root = str(Path(args.output_root).expanduser().resolve())
    args.config = str(Path(args.config).expanduser().resolve())

    scenarios = load_scenarios(Path(args.config))
    scenario_ids = selected(SCENARIO_INDEX.keys(), args.scenario)
    baseline_ids = selected(BASELINES.keys(), args.baseline)
    base_timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    completed_runs = []

    for repeat_idx in range(args.repeat):
        timestamp = base_timestamp if args.repeat == 1 else f"{base_timestamp}_r{repeat_idx + 1:02d}"
        for scenario_id in scenario_ids:
            for baseline_id in baseline_ids:
                run_dir = run_one(scenario_id, baseline_id, scenarios[scenario_id], args, timestamp)
                if run_dir is not None:
                    completed_runs.append(run_dir)

    if completed_runs:
        aggregate_csv = write_aggregate_summary(Path(args.output_root), completed_runs)
        if aggregate_csv is not None:
            print(f"Wrote aggregate summary: {aggregate_csv}")


if __name__ == "__main__":
    main()
