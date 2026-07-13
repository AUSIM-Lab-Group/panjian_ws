import importlib.util
import sys
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER_PATH = REPO_ROOT / "swarm_test/scripts/run_secbf_sim_experiments.py"
DYNAMIC_TAU_SWITCHES = (
    "dynamic_tau_enabled",
    "dynamic_tau_ke",
    "dynamic_tau_tmax",
    "dynamic_tau_min_speed",
    "dynamic_tau_min_distance",
    "dynamic_tau_max_tau",
)
DYNAMIC_TAU_METADATA = ("Ke", "Tmax", "min_speed", "min_distance", "max_tau")
METADATA_TO_SWITCH = {
    "Ke": "dynamic_tau_ke",
    "Tmax": "dynamic_tau_tmax",
    "min_speed": "dynamic_tau_min_speed",
    "min_distance": "dynamic_tau_min_distance",
    "max_tau": "dynamic_tau_max_tau",
}
TAU_FIELDS = ("tau", "T_i", "f_r", "f_v", "f_T", "tau_valid", "tau_reason")


def read(relative):
    return (REPO_ROOT / relative).read_text(encoding="utf-8")


def runner_module():
    script_dir = str(RUNNER_PATH.parent)
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)
    spec = importlib.util.spec_from_file_location("dynamic_tau_runner", RUNNER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def minimal_scenario():
    return {
        "start": {"x": 0.0, "y": 0.0},
        "goal": {"x": 12.0, "y": 0.0},
        "map": {"x": 12.0, "y": 6.0, "z": 3.0},
        "obstacles": [{"x": 6.0, "y": 0.0, "z": 0.5, "semantic_class": "adult"}],
    }


def launch_args(command):
    return {
        token.split(":=", 1)[0]: token.split(":=", 1)[1]
        for token in command
        if ":=" in token
    }


def write_meta(runner, tmp_path, baseline_id):
    run_dir = tmp_path / baseline_id
    run_dir.mkdir()
    return runner.write_run_meta(
        run_dir,
        "head_on_context_bl",
        baseline_id,
        minimal_scenario(),
        "[adult]",
        1,
        1,
    )


def csv_regions(source, stream, header_prefix, header_end_marker, row_marker, row_end_marker):
    header_start = source.index(stream + ' << "' + header_prefix)
    header_end = source.index(header_end_marker, header_start)
    row_start = source.index(row_marker, header_end)
    row_end = source.index(row_end_marker, row_start)
    return source[header_start:header_end], source[row_start:row_end]


def assert_tau_csv_contract(
    source, stream, header_prefix, header_end_marker, row_marker, row_end_marker, row_tokens
):
    header, row = csv_regions(
        source, stream, header_prefix, header_end_marker, row_marker, row_end_marker
    )
    for field in TAU_FIELDS:
        assert field in header
    for token in row_tokens:
        assert token in row


def test_shared_policy_is_the_numeric_source_of_truth():
    header = read("planner/semantic_guard/include/semantic_guard/dynamic_tau.hpp")
    assert "struct DynamicTauParams" in header
    assert "struct DynamicTauResult" in header
    assert "computeDynamicTau" in header
    assert "f_r" in header and "f_v" in header and "f_T" in header
    assert "max_tau" in header


def test_runner_baseline_switches_are_parsed_from_real_configuration():
    runner = runner_module()

    for baseline_id in ("No_semantic", "Unguarded_SEESM", "SEESM_Ours"):
        resolved = runner.resolve_baseline_alias(baseline_id)
        switches = runner.baseline_switches(resolved)
        assert switches.get("dynamic_tau_enabled") is True
        assert switches["cbf_metric"] == "seesm"
        for key in DYNAMIC_TAU_SWITCHES:
            assert key in switches

    standard = runner.baseline_switches(runner.resolve_baseline_alias("Standard_MPC_CBF"))
    assert standard.get("dynamic_tau_enabled") is False
    assert standard["cbf_metric"] == "distance"
    for key in DYNAMIC_TAU_SWITCHES:
        assert key in standard


def test_runner_generates_dynamic_switches_and_metadata_at_write_sites(tmp_path):
    runner = runner_module()
    scenario = minimal_scenario()
    obstacle_params = tmp_path / "obstacles_param.yaml"
    dynamic_cmd, _ = runner.build_commands(
        "head_on_context_bl", "SEESM_Ours", tmp_path, obstacle_params, "[adult]", 1, scenario
    )
    standard_cmd, _ = runner.build_commands(
        "head_on_context_bl", "Standard_MPC_CBF", tmp_path, obstacle_params, "[adult]", 1, scenario
    )

    dynamic_args = launch_args(dynamic_cmd)
    standard_args = launch_args(standard_cmd)
    dynamic_switches = runner.baseline_switches("SEESM_Ours")
    standard_switches = runner.baseline_switches("Standard_MPC_CBF")
    for key in DYNAMIC_TAU_SWITCHES:
        assert key in dynamic_args
        assert key in standard_args
        if key != "dynamic_tau_enabled":
            assert float(dynamic_args[key]) == float(dynamic_switches[key])
            assert float(standard_args[key]) == float(standard_switches[key])
    assert dynamic_args["dynamic_tau_enabled"].lower() == "true"
    assert standard_args["dynamic_tau_enabled"].lower() == "false"

    meta_path = write_meta(runner, tmp_path, "SEESM_Ours")
    meta = yaml.safe_load(meta_path.read_text(encoding="utf-8"))
    dynamic_tau = meta["dynamic_tau"]
    assert dynamic_tau["enabled"] is True
    assert dynamic_tau["beta_source"] == "beta_applied_final"
    for field in DYNAMIC_TAU_METADATA:
        assert field in dynamic_tau
        assert float(dynamic_tau[field]) == float(dynamic_switches[METADATA_TO_SWITCH[field]])

    standard_meta_path = write_meta(runner, tmp_path, "Standard_MPC_CBF")
    standard_meta = yaml.safe_load(standard_meta_path.read_text(encoding="utf-8"))
    assert standard_meta["dynamic_tau"]["enabled"] is False


def test_final_beta_and_audit_fields_are_at_their_actual_writer_paths():
    mpc = read("planner/mpc_secbf/src/mpc_secbf_node.cpp")
    publisher_region = mpc[mpc.index("// Publishers"):mpc.index("// Timers")]
    publish_region = mpc[mpc.index("void publishAcceptedMargins"):mpc.index("void chooseGoalState")]
    assert '"/safety_margin/beta_applied_final"' in publisher_region
    assert "out.beta_applied.assign(beta.begin(), beta.end())" in publish_region
    assert "pub_beta_applied_final_.publish(out)" in publish_region

    obs_manager = read("planner/vomp_planner/traj_planner/include/obs_manager/obs_manager.hpp")
    subscription_region = obs_manager[obs_manager.index("if (global_seesm_enable_)"):obs_manager.index("private:")]
    unsafe_region = obs_manager[obs_manager.index("bool is_SEESM_unsafe"):obs_manager.index("private:")]
    assert 'subscribe("/safety_margin/beta_applied_final"' in subscription_region
    assert "beta_applied = margin_entry.beta_applied" in unsafe_region
    assert "- beta_applied" in unsafe_region

    guard = read("planner/semantic_guard/src/beta_guard_node.cpp")
    guard_row_tokens = (
        "tau_result.tau", "tau_result.T_i", "tau_result.f_r", "tau_result.f_v",
        "tau_result.f_T", "tau_result.valid", "tau_result.reason",
    )
    assert_tau_csv_contract(
        guard,
        "csv_file_",
        "time,",
        "// Subscribers",
        "// CSV log",
        "log_msg.total_rollbacks",
        guard_row_tokens,
    )

    ground_truth = read("planner/semantic_guard/src/beta_ground_truth_node.cpp")
    assert_tau_csv_contract(
        ground_truth,
        "csv_file_",
        "time,",
        "// Subscribers",
        "if (csv_file_.is_open())",
        "log_msg.total_rollbacks",
        guard_row_tokens,
    )

    global_writer = obs_manager[obs_manager.index("void prepareGlobalSeesmLog"):obs_manager.index("bool isObstacleActiveAt")]
    for field in TAU_FIELDS:
        assert field in global_writer

    planner = read("planner/mpc_secbf/src/mpc_secbf_node.cpp")
    planner_writer = planner[planner.index("openCsv(planner_csv_"):planner.index("openCsv(timing_csv_")]
    planner_row = planner[planner.index("void writePlannerCsv"):planner.index("bool validateObstacleContractLocked")]
    for field in TAU_FIELDS:
        assert field in planner_writer
    for token in ("tau", "T_i", "f_r", "f_v", "f_T", "tau_valid", "tau_reason"):
        assert token in planner_row


def test_standard_mpc_cbf_and_legacy_acbf_are_separate_paths():
    runner = runner_module()
    standard = runner.BASELINES["Standard_MPC_CBF"]
    legacy = runner.BASELINES["B1_ACBF_fixed"]
    assert runner.resolve_baseline_alias("Standard_MPC_CBF") == "Standard_MPC_CBF"
    assert standard["planner"] == "secbf_planner.launch"
    assert standard["cbf_metric"] == "distance"
    assert standard["fixed_beta"] == 0.4
    assert legacy["planner"] == "acbf0_planner.launch"
    assert legacy["controller_index"] == 4

    standard_cmd, _ = runner.build_commands(
        "head_on_context_bl", "Standard_MPC_CBF", Path("/tmp"), Path("/tmp/obstacles.yaml"), "[adult]", 1, minimal_scenario()
    )
    legacy_cmd, _ = runner.build_commands(
        "head_on_context_bl", "B1_ACBF_fixed", Path("/tmp"), Path("/tmp/obstacles.yaml"), "[adult]", 1, minimal_scenario()
    )
    assert "cbf_metric:=distance" in standard_cmd
    assert all(not token.startswith("dynamic_tau_") for token in legacy_cmd)
    assert all(not token.startswith("cbf_metric:=") for token in legacy_cmd)

    legacy_source = read("planner/mpc_dcbf/src/mpc_cbf.cpp")
    assert "set_tau_value" in legacy_source
