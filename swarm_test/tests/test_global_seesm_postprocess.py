import csv
import importlib.util
import math
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = REPO_ROOT / "swarm_test/scripts/postprocess_teacher_canonical_runs.py"
SPEC = importlib.util.spec_from_file_location("postprocess_teacher_canonical_runs", MODULE_PATH)
postprocess = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(postprocess)

COMMON_EVALUATION_PATH = (
    REPO_ROOT / "swarm_test/config/common_offline_evaluation_v1.yaml"
)


def common_evaluation_contract():
    return postprocess.load_common_evaluation_contract(COMMON_EVALUATION_PATH)


def write_margin_rows(run_dir: Path, rows) -> None:
    path = run_dir / "margin_guard_log.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)


def write_sealed_summary(output_root: Path, run_name: str, run_id: str) -> Path:
    run_dir = output_root / run_name
    run_dir.mkdir(parents=True)
    (run_dir / "run_meta.yaml").write_text("run_state: complete\n", encoding="utf-8")
    with (run_dir / "summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["run_id", "scenario", "baseline"])
        writer.writeheader()
        writer.writerow({"run_id": run_id, "scenario": "head_on_context_bl", "baseline": "SEESM_Ours"})
    manifest = run_dir / "trial_integrity.sha256"
    manifest.write_text(
        "\n".join(
            f"{postprocess.sha256_file(run_dir / name)}  {name}"
            for name in ("run_meta.yaml", "summary.csv")
        ) + "\n",
        encoding="utf-8",
    )
    (run_dir / "RUN_COMPLETE.txt").write_text(
        "\n".join([
            "TEACHER_V1_RUN_COMPLETE",
            "trial_valid=true",
            f"run_meta_sha256={postprocess.sha256_file(run_dir / 'run_meta.yaml')}",
            f"integrity_manifest_sha256={postprocess.sha256_file(manifest)}",
        ]) + "\n",
        encoding="utf-8",
    )
    return run_dir


def test_collect_summary_rows_uses_only_sealed_complete_trials(tmp_path):
    valid = write_sealed_summary(tmp_path, "valid", "valid_run")
    invalid = write_sealed_summary(tmp_path, "invalid", "invalid_run")
    (invalid / "RUN_INVALID.txt").write_text("trial_valid=false\n", encoding="utf-8")
    unsealed = tmp_path / "unsealed"
    unsealed.mkdir()
    (unsealed / "summary.csv").write_text(
        "run_id,scenario,baseline\nunsealed_run,head_on_context_bl,SEESM_Ours\n",
        encoding="utf-8",
    )
    # A stale aggregate must not bypass the per-trial seal and integrity checks.
    (tmp_path / "summary.csv").write_text(
        "run_id,scenario,baseline\npoisoned_root,head_on_context_bl,SEESM_Ours\n",
        encoding="utf-8",
    )

    rows = postprocess.collect_summary_rows(tmp_path)

    assert rows == [{"run_id": "valid_run", "scenario": "head_on_context_bl", "baseline": "SEESM_Ours"}]
    assert postprocess.is_sealed_complete_run_dir(valid)


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


def test_common_eval_reconstructs_teacher_tca_and_phi_independently(tmp_path):
    rows = [{
        "time": "0.1", "obstacle_cycle_id": "7", "obs_id": "4000",
        "class": "adult", "h_seesm": "1.1", "h_eesm": "1.2",
        "d_i": "2.0", "rel_v_norm": "1.0", "cos_delta": "-1.0",
        "R_base": "0.8",
        # These controller fields intentionally disagree with the frozen
        # evaluator and must not affect H_eval.
        "beta_requested": "0.0", "beta_applied": "0.0", "tau": "0.0",
    }]
    write_margin_rows(tmp_path, rows)

    contract = common_evaluation_contract()
    metrics = postprocess.semantic_violation_metrics(tmp_path, contract)

    tau = 2.0 / (1.0 + 1.0e-6)
    ttc_norm = 1.0 - 2.0 / 5.0
    mu = 0.6 + 0.2 + 0.15 * ttc_norm
    expected = abs(2.0 - tau) - 0.8 - 0.75 * mu
    assert math.isclose(metrics["min_h_eval"], expected, abs_tol=1.0e-12)
    assert metrics["common_eval_status"] == "ok"
    assert metrics["semantic_violation_eval_ratio"] == 1.0
    assert metrics["eval_records"] == 1


def test_common_eval_ignores_logged_tau_and_controller_margin(tmp_path):
    base = {
        "time": "0.1", "obstacle_cycle_id": "8", "obs_id": "4000",
        "class": "adult", "h_seesm": "0.0", "h_eesm": "0.0",
        "d_i": "3.0", "rel_v_norm": "1.5", "cos_delta": "-0.5",
        "R_base": "0.8",
    }
    reference_dir = tmp_path / "reference"
    changed_dir = tmp_path / "changed"
    reference_dir.mkdir()
    changed_dir.mkdir()
    write_margin_rows(reference_dir, [{
        **base, "beta_requested": "0.0", "beta_applied": "0.0", "tau": "0.0",
    }])
    write_margin_rows(changed_dir, [{
        **base, "beta_requested": "99.0", "beta_applied": "0.75", "tau": "1.7",
    }])

    contract = common_evaluation_contract()
    reference = postprocess.semantic_violation_metrics(reference_dir, contract)
    changed = postprocess.semantic_violation_metrics(changed_dir, contract)
    assert math.isclose(reference["min_h_eval"], changed["min_h_eval"], abs_tol=1.0e-12)
    assert reference["common_eval_status"] == changed["common_eval_status"] == "ok"


def test_common_eval_caps_phi_with_frozen_beta_max(tmp_path):
    rows = [{
        "time": "0.1", "obstacle_cycle_id": "9", "obs_id": "4000",
        "class": "adult", "h_see": "0.0", "h_ee": "0.0",
        "d_i": "5.0", "rel_v_norm": "1.0", "cos_delta": "-1.0",
        "R_base": "0.8", "beta_requested": "0.0", "beta_applied": "0.0",
    }]
    write_margin_rows(tmp_path, rows)
    contract = common_evaluation_contract()
    contract["beta_max_m"] = dict(contract["beta_max_m"])
    contract["beta_max_m"]["adult"] = 0.50

    metrics = postprocess.semantic_violation_metrics(tmp_path, contract)
    tau = 2.0
    expected = abs(5.0 - tau) - 0.8 - 0.50
    assert math.isclose(metrics["min_h_eval"], expected, abs_tol=1.0e-12)


def test_common_eval_recomputes_density_per_obstacle_cycle(tmp_path):
    common = {
        "time": "0.1", "obstacle_cycle_id": "10", "class": "adult",
        "h_see": "0.0", "h_ee": "0.0", "d_i": "5.0",
        "rel_v_norm": "0.0", "cos_delta": "0.0", "R_base": "0.8",
        "beta_requested": "0.0", "beta_applied": "0.0",
    }
    write_margin_rows(tmp_path, [
        {**common, "obs_id": "4000", "rho_norm": "0.0"},
        {**common, "obs_id": "4001", "rho_norm": "0.0"},
    ])
    metrics = postprocess.semantic_violation_metrics(
        tmp_path, common_evaluation_contract()
    )
    # rho=(2-1)/5=0.2, so the frozen evaluator uses
    # beta=0.75*(0.6+0.1*0.2)=0.465, not the forged rho_norm log value.
    assert math.isclose(metrics["min_h_eval"], 5.0 - 0.8 - 0.465, abs_tol=1.0e-12)


def test_common_eval_fails_closed_for_unknown_category(tmp_path):
    write_margin_rows(tmp_path, [{
        "time": "0.1", "obstacle_cycle_id": "11", "obs_id": "4000",
        "class": "not_a_teacher_category", "h_see": "0.0", "h_ee": "0.0",
        "d_i": "2.0", "rel_v_norm": "1.0", "cos_delta": "-1.0",
        "R_base": "0.8",
    }])
    metrics = postprocess.semantic_violation_metrics(
        tmp_path, common_evaluation_contract()
    )
    assert metrics["common_eval_status"] == "invalid_raw_input"
    assert metrics["min_h_eval"] is None


def test_mpc_feasibility_excludes_terminal_safe_stop(tmp_path):
    rows = [
        {
            "mpc_status": "success", "first_attempt_status": "success",
            "final_status": "success", "accepted_beta_source": "candidate",
            "solve_time_ms": "10.0",
        },
        {
            "mpc_status": "infeasible_safe_stop", "first_attempt_status": "infeasible",
            "final_status": "infeasible", "accepted_beta_source": "safe_stop",
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
