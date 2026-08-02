import csv
import importlib.util
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
FREEZE = REPO_ROOT / "swarm_test/config/experiment_freezes/runtime_formal_current.yaml"
MANIFEST = REPO_ROOT / "swarm_test/config/seed_manifests/20260721_runtime_stability_formal30.csv"
RUN_SCRIPT = REPO_ROOT / "swarm_test/scripts/run_20260802_current_runtime_stability_formal30.sh"
ANALYZER = REPO_ROOT / "swarm_test/scripts/analyze_runtime_stability_thresholds.py"


def load_analyzer():
    spec = importlib.util.spec_from_file_location("runtime_thresholds", ANALYZER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_csv(path, fieldnames, rows):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def test_current_runtime_freeze_declares_current_formal_matrix():
    freeze = yaml.safe_load(FREEZE.read_text(encoding="utf-8"))
    assert freeze["freeze_id"] == "teacher_v1_runtime_stability_formal_current_20260802"
    assert freeze["campaign"] == "runtime"
    assert freeze["matrix"]["expected_trials"] == 240
    assert freeze["matrix"]["paired_seeds_per_condition"] == 30
    assert freeze["robot_and_mpc"]["max_cbf_obstacles"] == 6
    contract = freeze["validated_recovery_implementation_contract"]
    assert contract["source_release_branch"] == "formal/logging-repair-v4-20260729"
    assert contract["build_overlay"] == "panjian_ws/devel_current"
    assert contract["seesm_analysis_source_commit"] == "2f2f3bd56d741dc64c2707af6ce5a3cca2ca8dfb"
    assert contract["teacher_mpc_binary_sha256"].startswith("0a07dc1e")


def test_runtime_manifest_has_four_complete_thirty_seed_cells():
    with MANIFEST.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    grouped = {}
    for row in rows:
        grouped.setdefault((row["scenario_id"], row["trial_id"], row["seed"]), set()).add(
            row["obstacle_id"]
        )
    expected_obstacles = {
        "runtime_scaling_n1": 1,
        "runtime_scaling_n2": 2,
        "runtime_scaling_n4": 4,
        "runtime_scaling_n6": 6,
    }
    for scenario, obstacle_count in expected_obstacles.items():
        trials = [ids for (name, _, _), ids in grouped.items() if name == scenario]
        assert len(trials) == 30
        assert all(len(ids) == obstacle_count for ids in trials)


def test_runtime_script_uses_current_provenance_and_auto_roscore():
    text = RUN_SCRIPT.read_text(encoding="utf-8")
    assert "runtime_stability_formal_current_20260802" in text
    assert "runtime_formal_current.yaml" in text
    assert "devel_current" in text
    assert "--roscore auto" in text
    assert "Expected 240 runtime run directories" in text
    assert "120/120 paired hashes" in text
    assert "RUNTIME_STABILITY_FORMAL_AUDIT_PASS.txt" in text


def test_settling_evaluable_distinguishes_missing_post_interaction_window(tmp_path):
    analyzer = load_analyzer()
    run = tmp_path / "run"
    run.mkdir()
    obstacle_rows = [
        {"t": 0.0, "id": "4000", "vx": 1.0, "vy": 0.0, "d_i": 2.0},
        {"t": 1.0, "id": "4000", "vx": 1.0, "vy": 0.0, "d_i": 1.0},
        {"t": 2.0, "id": "4000", "vx": 1.0, "vy": 0.0, "d_i": 2.0},
    ]
    planner_rows = []
    for index in range(83):
        t = index * 0.1
        error = 0.5 if t < 2.5 else 0.1
        planner_rows.append({"t": t, "tracking_error": error, "slack_max": 0.0})
    write_csv(run / "planner_log.csv", ["t", "tracking_error", "slack_max"], planner_rows)
    write_csv(run / "obstacle_log.csv", ["t", "id", "vx", "vy", "d_i"], obstacle_rows)
    result = analyzer.analyze_trial(run)
    assert result["settle_0.30_1.0_evaluable"] == 1
    assert result["settle_0.30_1.0_within5"] == 1

    short_run = tmp_path / "short"
    short_run.mkdir()
    write_csv(
        short_run / "planner_log.csv",
        ["t", "tracking_error", "slack_max"],
        [row for row in planner_rows if float(row["t"]) < 2.0],
    )
    write_csv(
        short_run / "obstacle_log.csv",
        ["t", "id", "vx", "vy", "d_i"],
        obstacle_rows,
    )
    short_result = analyzer.analyze_trial(short_run)
    assert short_result["settle_0.30_1.0_evaluable"] == 0
    assert short_result["settle_0.30_1.0_within5"] == ""
