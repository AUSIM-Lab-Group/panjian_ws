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


def test_standard_and_dynamic_kernel_barrier_contracts():
    source = (REPO_ROOT / "planner/mpc_secbf/src/mpc_secbf.cpp").read_text(encoding="utf-8")
    header = (REPO_ROOT / "planner/mpc_secbf/include/mpc_secbf/mpc_secbf.h").read_text(encoding="utf-8")
    node = (REPO_ROOT / "planner/mpc_secbf/src/mpc_secbf_node.cpp").read_text(encoding="utf-8")
    launch = (REPO_ROOT / "planner/mpc_secbf/launch/mpc_secbf.launch").read_text(encoding="utf-8")
    cmake = (REPO_ROOT / "planner/mpc_secbf/CMakeLists.txt").read_text(encoding="utf-8")
    package = (REPO_ROOT / "planner/mpc_secbf/package.xml").read_text(encoding="utf-8")

    # Standard MPC-CBF is instantaneous distance with fixed beta=0.4 selected by the runner.
    assert 'dynamic_tau_enabled = false' in header
    assert "if (!dynamic_tau_enabled_)" in source
    assert (
        "return casadi::MX::sqrt(lx * lx + ly * ly) - obs_radius - robot_radius_ - beta_i;"
        in source
    )
    assert "R_safe" not in source and "R_safe" not in header
    assert "switches[\"fixed_beta\"]" in (
        REPO_ROOT / "swarm_test/scripts/run_secbf_sim_experiments.py"
    ).read_text(encoding="utf-8")
    assert "0.4" in (
        REPO_ROOT / "swarm_test/scripts/run_secbf_sim_experiments.py"
    ).read_text(encoding="utf-8")

    # Dynamic methods use the CasADi kernel and the shared semantic_guard parameter contract.
    assert "semantic_guard/dynamic_tau.hpp" in header
    assert "dynamicTauCasadi" in header and "dynamicTauCasadi" in source
    assert '<arg name="dynamic_tau_enabled" default="false"/>' in launch
    assert "obs(5) - curpos(3)" in source
    assert "obs(6) - curpos(4)" in source
    assert "lx + tau * vx" in source and "ly + tau * vy" in source
    assert (
        "return casadi::MX::sqrt(lookahead_x * lookahead_x + lookahead_y * lookahead_y)\n"
        "         - obs_radius - robot_radius_ - beta_i;"
        in source
    )
    assert "dynamic_tau_enabled" in node
    for param in (
        "dynamic_tau/Ke", "dynamic_tau/Tmax", "dynamic_tau/min_speed",
        "dynamic_tau/min_distance", "dynamic_tau/max_tau",
    ):
        assert param in launch and 'type="double"' in launch
        assert param in node
    assert "semantic_guard" in cmake
    assert "${semantic_guard_INCLUDE_DIRS}" in cmake
    assert "<build_export_depend>semantic_guard</build_export_depend>" in package
    for field in (
        "dynamic_tau_enabled", "tau", "T_i", "f_r", "f_v", "f_T",
        "tau_valid", "tau_reason",
    ):
        assert field in node
    assert 'tau_result.reason = "no_constrained_obstacle"' in node


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


def test_top_level_launch_forwards_dynamic_tau_to_mpc():
    launch = (REPO_ROOT / "swarm_test/launch/secbf_planner.launch").read_text(encoding="utf-8")
    mpc_include = launch.split('<include file="$(find mpc_secbf)/launch/mpc_secbf.launch">', 1)[1].split(
        "</include>", 1
    )[0]

    dynamic_args = (
        ("dynamic_tau_enabled", "false"),
        ("dynamic_tau_ke", "0.30"),
        ("dynamic_tau_tmax", "2.0"),
        ("dynamic_tau_min_speed", "1e-6"),
        ("dynamic_tau_min_distance", "1e-6"),
        ("dynamic_tau_max_tau", "2.0"),
    )
    for name, default in dynamic_args:
        assert f'<arg name="{name}" default="{default}"/>' in launch
        assert f'<arg name="{name}" value="$(arg {name})"/>' in mpc_include

    # The nested MPC launch owns the typed ROS params; the top-level launch preserves
    # the existing Guard/global forwarding while passing these args through.
    mpc_launch = (REPO_ROOT / "planner/mpc_secbf/launch/mpc_secbf.launch").read_text(encoding="utf-8")
    for name in (
        "dynamic_tau/Ke", "dynamic_tau/Tmax", "dynamic_tau/min_speed",
        "dynamic_tau/min_distance", "dynamic_tau/max_tau",
    ):
        assert f'<param name="{name}"' in mpc_launch
        assert f'<param name="{name}"' in mpc_launch and 'type="double"' in mpc_launch


def test_runner_metadata_records_distance_metric_and_adsm_switch():
    runner = _runner()
    source = (REPO_ROOT / "swarm_test/scripts/run_secbf_sim_experiments.py").read_text(encoding="utf-8")

    assert '"cbf_metric": switches["cbf_metric"]' in source
    assert '"front_adsm": switches["front_adsm"]' in source
