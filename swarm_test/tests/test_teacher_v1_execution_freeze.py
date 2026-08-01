import importlib.util
from pathlib import Path
import sys

import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER_PATH = REPO_ROOT / "swarm_test/scripts/run_secbf_sim_experiments.py"


def load_runner():
    script_dir = str(RUNNER_PATH.parent)
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)
    spec = importlib.util.spec_from_file_location("teacher_v1_execution_freeze", RUNNER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def runner():
    return load_runner()


def launch_args(command):
    return {
        token.split(":=", 1)[0]: token.split(":=", 1)[1]
        for token in command if ":=" in token
    }


def test_execution_tier_selects_expected_source_branch(runner):
    assert runner.required_source_branch("smoke") == "teacher-v1"
    assert runner.required_source_branch("formal") == (
        "formal/logging-repair-v4-20260729"
    )


def test_smoke_freeze_is_complete_and_drives_all_main_launches(tmp_path, runner):
    freeze = runner.load_parameter_freeze(
        runner.DEFAULT_SMOKE_PARAMETER_FREEZE, "smoke"
    )
    assert set(freeze["beta_bar"]) == set(runner.PARAMETER_FREEZE_CATEGORIES)
    assert set(freeze["beta_max"]) == set(runner.PARAMETER_FREEZE_CATEGORIES)

    scenarios = runner.load_scenarios(
        REPO_ROOT / "swarm_test/config/secbf_scenarios.yaml"
    )
    for scenario_id in runner.MAIN_MATRIX_SCENARIOS:
        frozen_scenario = runner.apply_parameter_freeze(scenarios[scenario_id], freeze)
        for baseline_id in (
            "Standard_MPC_CBF", "No_semantic", "Unguarded_SEESM", "SEESM_Ours",
        ):
            switches = runner.scenario_switches(baseline_id, frozen_scenario)
            assert switches["epsilon_max"] == pytest.approx(0.05)
            assert switches["slack_weight"] == pytest.approx(1000.0)
            planner, _ = runner.build_commands(
                scenario_id,
                baseline_id,
                tmp_path / f"{scenario_id}_{baseline_id}",
                tmp_path / "obstacles_param.yaml",
                "[adult]",
                1,
                frozen_scenario,
            )
            args = launch_args(planner)
            assert float(args["epsilon_max"]) == pytest.approx(0.05)
            assert float(args["slack_weight"]) == pytest.approx(1000.0)


def test_formal_tier_rejects_smoke_freeze(runner):
    with pytest.raises(runner.ParameterFreezeError, match="formal execution requires"):
        runner.load_parameter_freeze(runner.DEFAULT_SMOKE_PARAMETER_FREEZE, "formal")


@pytest.mark.parametrize("campaign", ("main", "ablation", "stress", "runtime"))
@pytest.mark.parametrize("tier", ("smoke", "formal"))
def test_each_campaign_freeze_matches_its_matrix(runner, campaign, tier):
    path = runner.PARAMETER_FREEZE_ROOT / f"{campaign}_{tier}.yaml"
    freeze = runner.load_parameter_freeze(path, tier, campaign=campaign)
    profile = runner.CAMPAIGN_PROFILES[campaign]
    assert freeze["campaign"] == campaign
    assert freeze["expected_trials"] == profile[f"{tier}_trials"]
    assert freeze["trial_outcome"]["deadlock_hold_sec"] == pytest.approx(
        runner.DEADLOCK_HOLD_SEC
    )


@pytest.mark.parametrize("campaign", ("main", "ablation", "stress", "runtime"))
def test_formal_freezes_use_regression_validated_recovery_profile(runner, campaign):
    path = runner.PARAMETER_FREEZE_ROOT / f"{campaign}_formal.yaml"
    freeze = runner.load_parameter_freeze(path, "formal", campaign=campaign)
    switches = freeze["switches"]

    expected_freeze_ids = {
        "main": "teacher_v1_main_formal_v3_clean_20260728",
        "ablation": "teacher_v1_ablation_formal_current_20260729",
        "stress": "teacher_v1_stress_formal_current_20260731",
        "runtime": "teacher_v1_runtime_formal_v3_clean_20260728",
    }
    assert freeze["id"] == expected_freeze_ids[campaign]
    assert switches["graph_cache_enabled"] is True
    assert switches["solver_max_cpu_time_ms"] == pytest.approx(120.0)
    assert switches["guard_time_budget_ms"] == pytest.approx(200.0)
    assert switches["guard_solver_max_cpu_time_ms"] == pytest.approx(70.0)
    assert switches["guard_bounded_midpoint_then_zero"] is True
    assert switches["terminal_action"] == "emergency_cbf"
    assert switches["emergency_cbf_activation_distance"] == pytest.approx(4.0)
    assert switches["emergency_cbf_progress_v"] == pytest.approx(0.2)


def test_active_set_formal30_campaign_loads_thirty_trial_freeze(runner):
    path = runner.PARAMETER_FREEZE_ROOT / "active_set_formal30.yaml"
    freeze = runner.load_parameter_freeze(
        path, "formal", campaign="active_set_formal30"
    )
    assert freeze["campaign"] == "active_set_formal30"
    assert freeze["expected_trials"] == 30
    assert freeze["switches"]["max_cbf_obstacles"] == 6
    assert freeze["switches"]["active_set_distance_m"] == pytest.approx(8.0)


def test_main_method_side_preference_definition(runner):
    assert runner.baseline_switches("Standard_MPC_CBF")[
        "side_preference_enabled"
    ] == "false"
    assert runner.baseline_switches("No_semantic")[
        "side_preference_enabled"
    ] == "false"
    assert runner.baseline_switches("Unguarded_SEESM")[
        "side_preference_enabled"
    ] == "true"
    assert runner.baseline_switches("SEESM_Ours")[
        "side_preference_enabled"
    ] == "true"


def test_crossing_context_ext_is_single_dynamic_obstacle_without_static_box(runner):
    scenarios = runner.load_scenarios(
        REPO_ROOT / "swarm_test/config/secbf_scenarios.yaml"
    )
    scenario = scenarios["crossing_context_ext"]

    assert len(scenario["obstacles"]) == 1
    obstacle = scenario["obstacles"][0]
    assert obstacle["semantic_class"] == "child_like"
    assert float(obstacle["scale_y"]) > 0.0
    assert "narrow" not in scenario["description"].lower()
    assert "narrow" not in scenario["expected_role"].lower()


@pytest.mark.parametrize(
    "manifest_name",
    (
        "20260714_nine_condition_pilot2.csv",
        "20260712_nine_condition_pilot5.csv",
        "20260712_nine_condition_formal30.csv",
    ),
)
def test_main_manifests_match_single_crossing_ext_obstacle(runner, manifest_name):
    trials = runner.load_seed_manifest(
        REPO_ROOT / "swarm_test/config/seed_manifests" / manifest_name
    )
    crossing_trials = [
        trial for trial in trials
        if trial["scenario_id"] == "crossing_context_ext"
    ]

    assert crossing_trials
    assert all(
        {row["obstacle_id"] for row in trial["rows"]} == {"obs_001"}
        for trial in crossing_trials
    )


def test_execution_freeze_metadata_is_tamper_evident(tmp_path, runner):
    freeze = runner.load_parameter_freeze(
        runner.DEFAULT_SMOKE_PARAMETER_FREEZE, "smoke"
    )
    common = runner.load_common_offline_evaluation_contract(
        runner.COMMON_OFFLINE_EVALUATION_CONTRACT
    )
    scenario = {
        "description": "freeze fixture",
        "start": {"x": 0.0, "y": 0.0},
        "goal": {"x": 2.0, "y": 0.0},
        "map": {"x": 10.0, "y": 8.0, "z": 3.0},
        "obstacles": [{"semantic_class": "adult"}],
    }
    scenario = runner.apply_parameter_freeze(scenario, freeze)
    run_dir = tmp_path / "run"
    obstacle_params = run_dir / "obstacles_param.yaml"
    run_dir.mkdir()
    obstacle_params.write_text("obstacle_params: []\n", encoding="utf-8")
    runner.write_run_meta(
        run_dir,
        "head_on_context_bl",
        "SEESM_Ours",
        scenario,
        "[adult]",
        1,
        30,
        protocol_id="teacher_v1_execution_freeze_test",
        freeze_contract=freeze,
        common_evaluation_contract=common,
    )
    passed, errors = runner.validate_run_meta_contract(run_dir, strict_provenance=False)
    assert passed, errors

    meta_path = run_dir / "run_meta.yaml"
    meta = yaml.safe_load(meta_path.read_text(encoding="utf-8"))
    meta["execution_freeze"]["sha256"] = "0" * 64
    payload = yaml.safe_dump(meta, sort_keys=False, allow_unicode=True)
    meta_path.write_text(payload, encoding="utf-8")
    (run_dir / "meta.yaml").write_text(payload, encoding="utf-8")
    passed, errors = runner.validate_run_meta_contract(run_dir, strict_provenance=False)
    assert not passed
    assert any("execution_freeze.sha256" in error for error in errors)
