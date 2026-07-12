import pathlib
import importlib.util


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]


def test_distance_metric_freezes_obstacle_position_for_the_horizon():
    source = (REPO_ROOT / "planner/mpc_secbf/src/mpc_secbf.cpp").read_text(encoding="utf-8")
    header = (REPO_ROOT / "planner/mpc_secbf/include/mpc_secbf/mpc_secbf.h").read_text(encoding="utf-8")

    assert "cbf_metric" in header
    assert 'cbf_metric_ == "distance"' in source
    assert "obs_matrix->col(original_idx * N_)" in source
    assert "h_cbf" in source


def test_mpc_launch_exposes_seesm_default_metric():
    launch = (REPO_ROOT / "planner/mpc_secbf/launch/mpc_secbf.launch").read_text(encoding="utf-8")

    assert '<arg name="cbf_metric" default="seesm"/>' in launch
    assert '<param name="mpc/cbf_metric" value="$(arg cbf_metric)"/>' in launch


def _runner():
    path = REPO_ROOT / "swarm_test/scripts/run_secbf_sim_experiments.py"
    spec = importlib.util.spec_from_file_location("runner", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_standard_mpc_cbf_alias_uses_common_stack():
    runner = _runner()
    baseline = runner.BASELINES["Standard_MPC_CBF"]
    switches = runner.baseline_switches("Standard_MPC_CBF")

    assert runner.resolve_baseline_alias("Standard_MPC_CBF") == "Standard_MPC_CBF"
    assert baseline["planner"] == "secbf_planner.launch"
    assert switches["semantic_mode"] == "fixed"
    assert switches["fixed_beta"] == 0.4
    assert switches["cbf_metric"] == "distance"
    assert switches["front_adsm"] == "false"
    assert switches["global_seesm_enable"] == "false"


def test_top_level_launch_forwards_distance_metric_and_adsm():
    launch = (REPO_ROOT / "swarm_test/launch/secbf_planner.launch").read_text(encoding="utf-8")

    assert '<arg name="cbf_metric" default="seesm"/>' in launch
    assert '<arg name="front_adsm" default="true"/>' in launch
    assert '<arg name="cbf_metric" value="$(arg cbf_metric)"/>' in launch
    assert '<arg name="used_adsm_" value="$(arg front_adsm)"/>' in launch
