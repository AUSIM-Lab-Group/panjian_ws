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
    assert '<arg name="dynamic_tau_enabled" default="true"/>' in launch
    assert '<arg name="dynamic_tau_mode" default="teacher_tca"/>' in launch
    assert "curpos(3) - obs(5)" in source
    assert "curpos(4) - obs(6)" in source
    assert "lx + tau * vx" in source and "ly + tau * vy" in source
    assert "if (!config_valid)" in source
    assert "return casadi::MX(0.0);" in source
    assert (
        "return casadi::MX::sqrt(lookahead_x * lookahead_x + lookahead_y * lookahead_y)\n"
        "         - obs_radius - robot_radius_ - beta_i;"
        in source
    )
    assert "dynamic_tau_enabled" in node
    for param in (
        "dynamic_tau/delta_tau", "dynamic_tau/Ke", "dynamic_tau/Tmax", "dynamic_tau/min_speed",
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
    assert "const bool obstacle_contract_valid = validateObstacleContractLocked();" in node
    assert "const int constrained_obs_count = obstacle_contract_valid" in node
    assert "!obstacle_contract_valid" in node
    assert "resetAuditMetrics" in header and "resetAuditMetrics" in node
    assert "std::isfinite" in node
    assert "accepted_beta_by_id_" in node
    assert "accepted_beta_by_id_" in node
    assert "retainAcceptedMarginsForActiveIds" in node
    assert "obstacle_cycle_id" in node


def test_mpc_launch_exposes_seesm_default_metric():
    launch = (REPO_ROOT / "planner/mpc_secbf/launch/mpc_secbf.launch").read_text(encoding="utf-8")

    assert '<arg name="cbf_metric" default="seesm"/>' in launch
    assert '<param name="mpc/cbf_metric" value="$(arg cbf_metric)"/>' in launch


def test_teacher_t3_independent_terminal_cost_and_input_increment_contract():
    """Freeze the executable T3 objective/actuator contract.

    Q_f and the delta-u penalty/bound are deliberately separate parameters;
    they must be visible in the solver, loaded by the ROS node, and forwarded
    by both launch layers.  This catches the easy-to-miss failure mode where
    the objective is changed in ``mpc_secbf.cpp`` but every experiment still
    runs the old hard-coded defaults.
    """
    source = (REPO_ROOT / "planner/mpc_secbf/src/mpc_secbf.cpp").read_text(encoding="utf-8")
    header = (REPO_ROOT / "planner/mpc_secbf/include/mpc_secbf/mpc_secbf.h").read_text(encoding="utf-8")
    node = (REPO_ROOT / "planner/mpc_secbf/src/mpc_secbf_node.cpp").read_text(encoding="utf-8")
    mpc_launch = (REPO_ROOT / "planner/mpc_secbf/launch/mpc_secbf.launch").read_text(encoding="utf-8")
    planner_launch = (REPO_ROOT / "swarm_test/launch/secbf_planner.launch").read_text(encoding="utf-8")

    # Solver API/state and the independent terminal Q_f expression.
    assert "double qf_scale" in header
    assert "double delta_u_weight" in header
    assert "double delta_u_max" in header
    assert "qf_scale_" in header and "delta_u_weight_" in header and "delta_u_max_" in header
    assert "Qf_mat" in source
    assert "qf_scale_ * Q_[0]" in source
    assert "qf_scale_ * Q_[1]" in source
    assert "qf_scale_ * Q_[2]" in source

    # Delta-u is both penalized and constrained, including the first input
    # increment against the measured state (the documented provisional
    # omega=0 convention is part of the auditable implementation).
    assert "delta_u_weight_ * casadi::MX::sumsqr(delta_u)" in source
    assert "prob.subject_to(prob.bounded(-delta_u_max_, delta_u, delta_u_max_))" in source
    assert "casadi::MX previous_u = casadi::MX::vertcat({(*cur_state)(3), 0.0})" in source
    assert "last_delta_u_max" in source

    # ROS parameter plumbing must not silently leave the solver at defaults.
    for name, ros_name, default in (
        ("qf_scale", "mpc/qf_scale", "1.1"),
        ("delta_u_weight", "mpc/delta_u_weight", "0.02"),
        ("delta_u_max", "mpc/delta_u_max", "0.4"),
    ):
        assert f'<arg name="{name}" default="{default}"/>' in mpc_launch
        assert f'<param name="{ros_name}" value="$(arg {name})" type="double"/>' in mpc_launch
        assert f'<arg name="{name}" default="{default}"/>' in planner_launch
        assert f'<arg name="{name}" value="$(arg {name})"/>' in planner_launch
        assert f'"{ros_name}"' in node
        assert f"{name}" in node
        # The values are also emitted in planner/mpc-margin CSV audit rows;
        # copy the validated ROS values into the node's member state instead
        # of silently logging the member defaults for custom runs.
        assert f"{name}_ = {name};" in node

    # The call site must pass the loaded values as the final three init_solver
    # arguments, rather than relying on header defaults.
    assert "solver_.init_solver" in node
    init_call = node.split("solver_.init_solver", 1)[1].split(");", 1)[0]
    assert "qf_scale" in init_call
    assert "delta_u_weight" in init_call
    assert "delta_u_max" in init_call


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
    assert switches["dynamic_tau_enabled"] is False
    assert switches["dynamic_tau_mode"] == "teacher_tca"
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
        ("dynamic_tau_enabled", "true"),
        ("dynamic_tau_mode", "teacher_tca"),
        ("dynamic_tau_delta_tau", "1e-6"),
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
        "dynamic_tau/delta_tau", "dynamic_tau/Ke", "dynamic_tau/Tmax", "dynamic_tau/min_speed",
        "dynamic_tau/min_distance", "dynamic_tau/max_tau",
    ):
        assert f'<param name="{name}"' in mpc_launch
        assert f'<param name="{name}"' in mpc_launch and 'type="double"' in mpc_launch


def test_runner_metadata_records_distance_metric_and_adsm_switch():
    runner = _runner()
    source = (REPO_ROOT / "swarm_test/scripts/run_secbf_sim_experiments.py").read_text(encoding="utf-8")

    assert '"cbf_metric": switches["cbf_metric"]' in source
    assert '"front_adsm": switches["front_adsm"]' in source
