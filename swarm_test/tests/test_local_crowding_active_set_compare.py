import importlib.util
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
ANALYZER = REPO_ROOT / "swarm_test/scripts/analyze_local_crowding_active_set_compare.py"
CONFIG = REPO_ROOT / "swarm_test/config/active_set_comparisons/local_crowding_max3_pilot.yaml"
MANIFEST = REPO_ROOT / "swarm_test/config/seed_manifests/20260801_local_crowding_max3_pilot5.csv"
RUN_SCRIPT = REPO_ROOT / "swarm_test/scripts/run_20260801_local_crowding_max3_pilot5.sh"


def load_analyzer():
    spec = importlib.util.spec_from_file_location("active_set_compare", ANALYZER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_comparison_config_keeps_default_reference_and_distance_ordering():
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    assert config["scenario"] == "stress_local_crowding"
    assert config["baseline"] == "SEESM_Ours"
    assert config["variants"]["reference"]["max_cbf_obstacles"] == 6
    assert config["variants"]["candidate"]["max_cbf_obstacles"] == 3
    assert config["selection_policy"]["ordering"] == "current_distance_ascending"
    assert config["acceptance"]["expected_pairs"] == 5


def test_manifest_has_five_complete_five_obstacle_trials():
    import csv

    with MANIFEST.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    grouped = {}
    for row in rows:
        grouped.setdefault((row["trial_id"], row["seed"]), set()).add(
            row["obstacle_id"]
        )
        assert row["scenario_id"] == "stress_local_crowding"
    assert len(grouped) == 5
    assert all(ids == {f"obs_{index:03d}" for index in range(1, 6)} for ids in grouped.values())


def test_acceptance_gates_pass_and_fail_expected_cases():
    analyzer = load_analyzer()
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    acceptance = config["acceptance"]
    passing = [
        {
            "executed_collision_added": 0,
            "success_regression": 0,
            "delta_d_min_m": -0.01,
            "delta_semantic_violation": 0.001,
        }
        for _ in range(5)
    ]
    gates, passed = analyzer.evaluate_gates(passing, acceptance)
    assert passed
    assert all(gate["passed"] for gate in gates)

    failing = [dict(row) for row in passing]
    failing[0]["executed_collision_added"] = 1
    gates, passed = analyzer.evaluate_gates(failing, acceptance)
    assert not passed
    assert next(gate for gate in gates if gate["gate"] == "added_executed_collision_pairs")["passed"] == 0


def test_run_script_uses_both_active_set_limits_and_dedicated_roots():
    text = RUN_SCRIPT.read_text(encoding="utf-8")
    assert 'run_variant max6 6 "$REFERENCE_ROOT"' in text
    assert 'run_variant max3 3 "$CANDIDATE_ROOT"' in text
    assert "local_crowding_active_set_max3_pilot_20260801" in text
    assert "--skip-existing-complete" in text
