import importlib.util
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
ANALYZER = REPO_ROOT / "swarm_test/scripts/analyze_local_crowding_active_set_compare.py"
CONFIG = REPO_ROOT / "swarm_test/config/active_set_comparisons/local_crowding_max3_pilot.yaml"
FORMAL30_CONFIG = REPO_ROOT / "swarm_test/config/active_set_comparisons/local_crowding_max3_formal30.yaml"
MANIFEST = REPO_ROOT / "swarm_test/config/seed_manifests/20260801_local_crowding_max3_pilot5.csv"
FORMAL30_MANIFEST = REPO_ROOT / "swarm_test/config/seed_manifests/20260720_feasibility_stress_formal30.csv"
RUN_SCRIPT = REPO_ROOT / "swarm_test/scripts/run_20260801_local_crowding_max3_pilot5.sh"
FORMAL30_RUN_SCRIPT = REPO_ROOT / "swarm_test/scripts/run_20260801_local_crowding_max3_formal30.sh"
FREEZE = REPO_ROOT / "swarm_test/config/experiment_freezes/active_set_formal.yaml"
FORMAL30_FREEZE = REPO_ROOT / "swarm_test/config/experiment_freezes/active_set_formal30.yaml"


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
    assert config["parameter_freeze"].endswith("active_set_formal.yaml")


def test_active_set_formal_freeze_matches_dedicated_matrix():
    config = yaml.safe_load(FREEZE.read_text(encoding="utf-8"))
    assert config["status"] == "formal_frozen"
    assert config["campaign"] == "active_set"
    assert config["matrix"]["scenarios"] == ["stress_local_crowding"]
    assert config["matrix"]["requested_methods"] == ["SEESM_Ours"]
    assert config["matrix"]["expected_trials"] == 5


def test_active_set_formal30_freeze_and_config_match_dedicated_matrix():
    freeze = yaml.safe_load(FORMAL30_FREEZE.read_text(encoding="utf-8"))
    config = yaml.safe_load(FORMAL30_CONFIG.read_text(encoding="utf-8"))
    assert freeze["status"] == "formal_frozen"
    assert freeze["campaign"] == "active_set_formal30"
    assert freeze["matrix"]["scenarios"] == ["stress_local_crowding"]
    assert freeze["matrix"]["requested_methods"] == ["SEESM_Ours"]
    assert freeze["matrix"]["expected_trials"] == 30
    assert config["acceptance"]["expected_pairs"] == 30
    assert config["variants"]["reference"]["max_cbf_obstacles"] == 6
    assert config["variants"]["candidate"]["max_cbf_obstacles"] == 3
    assert config["diagnostics"]["severe_d_min_drop_threshold_m"] == 0.10
    assert config["artifacts"]["pass_sentinel"] == "ACTIVE_SET_MAX3_FORMAL30_PASS.txt"


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


def test_formal_manifest_has_thirty_local_crowding_five_obstacle_trials():
    import csv

    with FORMAL30_MANIFEST.open(newline="", encoding="utf-8") as handle:
        rows = [
            row for row in csv.DictReader(handle)
            if row["scenario_id"] == "stress_local_crowding"
        ]
    grouped = {}
    for row in rows:
        grouped.setdefault((row["trial_id"], row["seed"]), set()).add(
            row["obstacle_id"]
        )
    assert len(grouped) == 30
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
    assert "--campaign active_set" in text
    assert "--execution-tier formal" in text


def test_formal30_run_script_uses_dedicated_campaign_and_formal_root():
    text = FORMAL30_RUN_SCRIPT.read_text(encoding="utf-8")
    assert 'run_variant max6 6 "$REFERENCE_ROOT"' in text
    assert 'run_variant max3 3 "$CANDIDATE_ROOT"' in text
    assert "--campaign active_set_formal30" in text
    assert "active_set_formal30.yaml" in text
    assert "20260720_feasibility_stress_formal30.csv" in text
    assert "04_formal/local_crowding_active_set_max3_formal30_20260801" in text
    assert "audit_variant \"$REFERENCE_ROOT\" 6" in text
    assert "audit_variant \"$CANDIDATE_ROOT\" 3" in text
