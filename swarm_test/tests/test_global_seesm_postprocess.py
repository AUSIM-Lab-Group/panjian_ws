import csv
import importlib.util
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
    assert postprocess.paper_outcome({"success": "1", "nav_collision_count": "0"}) == (1, 1, 0)
    assert postprocess.paper_outcome({"success": "1", "nav_collision_count": "22"}) == (0, 1, 22)
    assert postprocess.paper_outcome({"success": "0", "nav_collision_count": "0"}) == (0, 0, 0)


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
