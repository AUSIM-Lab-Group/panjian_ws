import csv
import importlib.util
import math
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = REPO_ROOT / "swarm_test/scripts/postprocess_teacher_canonical_runs.py"
SPEC = importlib.util.spec_from_file_location("postprocess_teacher_canonical_runs", MODULE_PATH)
postprocess = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(postprocess)


def test_global_log_metrics_count_semantic_and_stale_rows(tmp_path):
    rows = [
        {
            "beta_applied": "0.4", "primitive_rejected": "1", "shot_rejected": "0",
            "reason": "semantic", "global_replan_ms": "3.0",
        },
        {
            "beta_applied": "0.0", "primitive_rejected": "0", "shot_rejected": "0",
            "reason": "stale", "global_replan_ms": "5.0",
        },
        {
            "beta_applied": "0.0", "primitive_rejected": "0", "shot_rejected": "0",
            "reason": "missing", "global_replan_ms": "4.0",
        },
    ]
    path = tmp_path / "global_seesm_log.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)

    metrics = postprocess.global_seesm_metrics(tmp_path)

    assert metrics["global_beta_applied_max"] == 0.4
    assert metrics["global_semantic_rejection_count"] == 1
    assert metrics["global_stale_margin_count"] == 1
    assert metrics["global_missing_margin_count"] == 1
    assert metrics["global_replan_mean_ms"] == 4.0


def test_paper_success_requires_goal_and_zero_collisions():
    assert postprocess.paper_outcome({
        "goal_reached": "1", "success": "1", "nav_collision_count": "0",
        "log_min_distance_m": "0.01",
    }) == (1, 1, 0)
    assert postprocess.paper_outcome({"goal_reached": "1", "success": "0", "nav_collision_count": "22"}) == (0, 1, 22)
    assert postprocess.paper_outcome({"goal_reached": "0", "success": "0", "nav_collision_count": "0"}) == (0, 0, 0)


def test_paper_success_rejects_collision_seen_only_in_geometric_log():
    assert postprocess.paper_outcome({
        "goal_reached": "1", "success": "1", "nav_collision_count": "0",
        "log_min_distance_m": "-0.001",
    }) == (0, 1, 1)


def test_paper_outcome_falls_back_to_legacy_success_field():
    assert postprocess.paper_outcome({"success": "1", "nav_collision_count": "0"}) == (1, 1, 0)


def test_paper_travel_time_prefers_navigation_completion_time():
    summary = {"nav_travel_time_s": "10.5", "robot_travel_time_s": "27.3"}

    assert postprocess.paper_travel_time(summary) == 10.5


def test_common_eval_reconstructs_dynamic_eesm_independently_of_controller_margin(tmp_path):
    (tmp_path / "meta.yaml").write_text(
        "dynamic_tau:\n"
        "  enabled: true\n"
        "  mode: legacy_gate\n"
        "  Ke: 0.3\n"
        "  Tmax: 2.0\n"
        "  min_speed: 1.0e-6\n"
        "  min_distance: 1.0e-6\n"
        "  max_tau: 2.0\n",
        encoding="utf-8",
    )
    rows = [
        {
            "time": "0.1",
            "h_see": "0.84",
            "h_ee": "0.84",
            "d_i": "2.0",
            "rel_v_norm": "1.0",
            "cos_delta": "-1.0",
            "R_base": "0.8",
            "beta_bar": "0.75",
            "mu": "0.8",
        }
    ]
    path = tmp_path / "margin_guard_log.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)

    metrics = postprocess.semantic_violation_metrics(tmp_path)

    # T_i=(2.0-0.8)/1.0=1.2 s, tau=0.3*T_i=0.36 s,
    # h_EESM=|2.0-0.36|-0.8=0.84 m and beta_eval=0.75*0.8=0.60 m.
    assert math.isclose(metrics["min_h_eval"], 0.24, abs_tol=1.0e-12)
    assert metrics["semantic_violation_eval_ratio"] == 0.0
    assert metrics["eval_records"] == 1


def test_common_eval_recomputes_teacher_tca_from_explicit_meta_mode(tmp_path):
    (tmp_path / "meta.yaml").write_text(
        "dynamic_tau:\n"
        "  enabled: true\n"
        "  mode: teacher_tca\n"
        "  delta_tau: 0.01\n"
        "  max_tau: 2.0\n",
        encoding="utf-8",
    )
    rows = [{
        "time": "0.1", "h_see": "-1.38", "h_ee": "-0.78",
        "d_i": "2.0", "rel_v_norm": "1.0", "cos_delta": "-1.0",
        "R_base": "0.8", "beta_bar": "0.75", "mu": "0.8",
    }]
    with (tmp_path / "margin_guard_log.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)

    metrics = postprocess.semantic_violation_metrics(tmp_path)

    tau = 2.0 / 1.01
    expected = abs(2.0 - tau) - 0.8 - 0.75 * 0.8
    assert math.isclose(metrics["min_h_eval"], expected, abs_tol=1.0e-12)


def test_common_eval_prefers_valid_logged_tau_over_reconstruction(tmp_path):
    (tmp_path / "meta.yaml").write_text(
        "dynamic_tau:\n"
        "  enabled: true\n"
        "  mode: teacher_tca\n"
        "  delta_tau: 1.0e-6\n"
        "  max_tau: 2.0\n",
        encoding="utf-8",
    )
    rows = [{
        "time": "0.1", "h_see": "0.1", "h_ee": "0.7",
        "d_i": "2.0", "rel_v_norm": "1.0", "cos_delta": "-1.0",
        "R_base": "0.8", "beta_bar": "0.75", "mu": "0.8",
        "tau": "0.5", "tau_valid": "1",
    }]
    with (tmp_path / "margin_guard_log.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)

    metrics = postprocess.semantic_violation_metrics(tmp_path)

    # Logged tau=0.5 gives |2-0.5|-0.8-0.6=0.1. Reconstructing pure TCA
    # would give a negative value, so this assertion fixes source priority.
    assert math.isclose(metrics["min_h_eval"], 0.1, abs_tol=1.0e-12)


def test_common_eval_prefers_tau_computed_and_final_applied_margin(tmp_path):
    (tmp_path / "meta.yaml").write_text(
        "dynamic_tau:\n"
        "  enabled: true\n"
        "  mode: teacher_tca\n"
        "  delta_tau: 1.0e-6\n"
        "  max_tau: 2.0\n",
        encoding="utf-8",
    )
    rows = [{
        "time": "0.1", "h_seesm": "1.1", "h_eesm": "1.2",
        "d_i": "2.0", "rel_v_norm": "1.0", "cos_delta": "-1.0",
        "R_base": "0.8", "beta_bar": "0.75", "mu": "0.8",
        "beta_applied": "0.1", "tau": "0.0", "tau_computed": "1",
        "tau_active": "0", "tau_valid": "0",
    }]
    with (tmp_path / "margin_guard_log.csv").open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)

    metrics = postprocess.semantic_violation_metrics(tmp_path)

    # tau_computed=true makes the valid inactive tau=0 authoritative even
    # though the deprecated tau_valid field says false. The final applied
    # margin is 0.1, not the requested beta_bar*mu=0.6.
    assert math.isclose(metrics["min_h_eval"], 1.1, abs_tol=1.0e-12)
    assert math.isclose(metrics["min_h_seesm_from_log"], 1.1, abs_tol=1.0e-12)


def test_mpc_feasibility_excludes_no_cbf_emergency_fallback(tmp_path):
    rows = [
        {
            "mpc_status": "success", "first_attempt_status": "success",
            "final_status": "success", "accepted_beta_source": "candidate",
            "solve_time_ms": "10.0",
        },
        {
            "mpc_status": "no_cbf_fallback", "first_attempt_status": "infeasible",
            "final_status": "success", "accepted_beta_source": "no_cbf",
            "solve_time_ms": "11.0",
        },
        {
            "mpc_status": "guard_previous", "first_attempt_status": "infeasible",
            "final_status": "success", "accepted_beta_source": "previous",
            "solve_time_ms": "12.0",
        },
        {
            "mpc_status": "guard_zero", "first_attempt_status": "infeasible",
            "final_status": "success", "accepted_beta_source": "zero",
            "solve_time_ms": "13.0",
        },
    ]
    path = tmp_path / "planner_log.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)

    metrics = postprocess.planner_metrics(tmp_path, {})

    assert metrics["mpc_feasibility_rate"] == 0.75
    assert metrics["mpc_first_attempt_feasibility_rate"] == 0.25


def test_standard_mpc_cbf_uses_paper_standard_label():
    assert postprocess.METHOD_LABELS["Standard_MPC_CBF"] == "Standard MPC-CBF"


def test_collision_episode_metrics_groups_negative_clearance_per_obstacle(tmp_path):
    rows = [
        {"t": "0.00", "id": "a", "radius": "0.4", "d_i": "0.70"},
        {"t": "0.10", "id": "a", "radius": "0.4", "d_i": "0.70"},
        {"t": "0.20", "id": "a", "radius": "0.4", "d_i": "0.90"},
        {"t": "0.30", "id": "a", "radius": "0.4", "d_i": "0.70"},
        {"t": "0.30", "id": "b", "radius": "0.4", "d_i": "0.70"},
        {"t": "0.40", "id": "b", "radius": "0.4", "d_i": "0.90"},
    ]
    path = tmp_path / "obstacle_log.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)

    assert postprocess.collision_episode_metrics(tmp_path)["collision_episode_count"] == 3


def test_collision_episode_metrics_ignores_duplicate_prediction_bursts(tmp_path):
    rows = [
        {"t": "0.0000", "id": "a", "radius": "0.4", "d_i": "0.70"},
        {"t": "0.0002", "id": "a", "radius": "0.4", "d_i": "0.70"},
        {"t": "0.0004", "id": "a", "radius": "0.4", "d_i": "0.70"},
        {"t": "0.0006", "id": "a", "radius": "0.4", "d_i": "0.70"},
        {"t": "0.0008", "id": "a", "radius": "0.4", "d_i": "0.70"},
        {"t": "0.0500", "id": "a", "radius": "0.4", "d_i": "0.70"},
        {"t": "0.0502", "id": "a", "radius": "0.4", "d_i": "0.70"},
        {"t": "0.0504", "id": "a", "radius": "0.4", "d_i": "0.70"},
        {"t": "0.0506", "id": "a", "radius": "0.4", "d_i": "0.70"},
        {"t": "0.0508", "id": "a", "radius": "0.4", "d_i": "0.70"},
        {"t": "0.1000", "id": "a", "radius": "0.4", "d_i": "0.70"},
    ]
    path = tmp_path / "obstacle_log.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)

    assert postprocess.collision_episode_metrics(tmp_path)["collision_episode_count"] == 1


def test_collision_episode_metrics_ignores_zero_initialization_placeholder(tmp_path):
    rows = [
        {
            "t": "0.00", "id": "a", "radius": "0.4", "d_i": "0.0",
            "rel_v": "0.0", "TTC": "0.0", "h_EE": "0.0",
        },
        {
            "t": "0.10", "id": "a", "radius": "0.4", "d_i": "0.90",
            "rel_v": "0.1", "TTC": "1.0", "h_EE": "0.1",
        },
    ]
    path = tmp_path / "obstacle_log.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)

    assert postprocess.collision_episode_metrics(tmp_path)["collision_episode_count"] == 0


def test_aggregate_rows_reports_mean_collision_episode_count():
    rows = [
        {
            "scenario": "head_on_context_int",
            "scenario_family": "Head-on",
            "context_level": "Context-Int",
            "baseline": "Standard_MPC_CBF",
            "method_label": "Standard MPC-CBF",
            "success": 0,
            "goal_reached": 1,
            "collision_count": 10,
            "collision_episode_count": 1,
        },
        {
            "scenario": "head_on_context_int",
            "scenario_family": "Head-on",
            "context_level": "Context-Int",
            "baseline": "Standard_MPC_CBF",
            "method_label": "Standard MPC-CBF",
            "success": 0,
            "goal_reached": 1,
            "collision_count": 20,
            "collision_episode_count": 3,
        },
    ]

    summary = postprocess.aggregate_rows(rows)

    assert summary[0]["collision_episode_mean"] == "2.000000"
