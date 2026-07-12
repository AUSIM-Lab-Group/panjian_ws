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
