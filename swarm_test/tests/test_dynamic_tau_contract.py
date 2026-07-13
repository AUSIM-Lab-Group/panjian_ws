import ast
from contextlib import contextmanager
import importlib.util
import sys
from pathlib import Path

import pytest
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
BASELINE_CONTRACT = (
    ("Standard_MPC_CBF", False, "distance", False, "fixed_config"),
    ("No_semantic", True, "seesm", False, "zero"),
    ("Unguarded_SEESM", True, "seesm", True, "candidate"),
    ("SEESM_Ours", True, "seesm", True, "beta_applied_final"),
)


def read(relative):
    return (REPO_ROOT / relative).read_text(encoding="utf-8")


@contextmanager
def runner_module():
    original_sys_path = list(sys.path)
    module_name = "dynamic_tau_runner"
    sentinel = object()
    saved_modules = {
        name: sys.modules.get(name, sentinel)
        for name in (module_name, "reference_path_waypoints")
    }
    try:
        script_dir = str(RUNNER_PATH.parent)
        if script_dir not in sys.path:
            sys.path.insert(0, script_dir)
        spec = importlib.util.spec_from_file_location(module_name, RUNNER_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        yield module
    finally:
        sys.path[:] = original_sys_path
        for name, previous in saved_modules.items():
            if previous is sentinel:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous
        assert sys.path == original_sys_path


def runner_tree():
    return ast.parse(read("swarm_test/scripts/run_secbf_sim_experiments.py"))


def top_level_function(tree, name):
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name:
            return node
    raise AssertionError(f"runner function not found: {name}")


def literal_assignment(tree, name):
    for node in tree.body:
        targets = []
        if isinstance(node, ast.Assign):
            targets = node.targets
        elif isinstance(node, ast.AnnAssign):
            targets = [node.target]
        if any(isinstance(target, ast.Name) and target.id == name for target in targets):
            return ast.literal_eval(node.value)
    raise AssertionError(f"runner assignment not found: {name}")


def subscript_assignments(function_node):
    assignments = {}
    for node in ast.walk(function_node):
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if not isinstance(target, ast.Subscript):
                continue
            if not isinstance(target.value, ast.Name):
                continue
            slice_node = target.slice
            if isinstance(slice_node, ast.Index):
                slice_node = slice_node.value
            try:
                key = ast.literal_eval(slice_node)
            except (ValueError, TypeError):
                continue
            assignments.setdefault(key, []).append(node.value)
    return assignments


def literal_values(nodes):
    values = []
    for node in nodes:
        try:
            values.append(ast.literal_eval(node))
        except (ValueError, TypeError):
            pass
    return values


def string_literals(node):
    return [
        child.value
        for child in ast.walk(node)
        if isinstance(child, ast.Constant) and isinstance(child.value, str)
    ]


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


def cpp_function_body(source, signature):
    signature_start = source.find(signature)
    assert signature_start >= 0, f"C++ function not found: {signature}"
    opening_brace = source.find("{", signature_start)
    assert opening_brace >= 0, f"C++ function body not found: {signature}"
    depth = 0
    for index in range(opening_brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[opening_brace + 1:index]
    raise AssertionError(f"unterminated C++ function: {signature}")


def assert_tau_csv_contract(header_body, row_body, row_tokens):
    for field in TAU_FIELDS:
        assert field in header_body
    for token in row_tokens:
        assert token in row_body


def bool_value(value):
    if isinstance(value, bool):
        return value
    return str(value).lower() == "true"


def test_shared_policy_is_the_numeric_source_of_truth():
    header = read("planner/semantic_guard/include/semantic_guard/dynamic_tau.hpp")
    assert "struct DynamicTauParams" in header
    assert "struct DynamicTauResult" in header
    assert "computeDynamicTau" in header
    assert "f_r" in header and "f_v" in header and "f_T" in header
    assert "max_tau" in header


def test_runner_source_has_structured_baseline_and_writer_assignments():
    tree = runner_tree()
    baselines = literal_assignment(tree, "BASELINES")
    assert baselines["Standard_MPC_CBF"]["planner"] == "secbf_planner.launch"
    assert baselines["B1_ACBF_fixed"]["planner"] == "acbf0_planner.launch"

    switches_fn = top_level_function(tree, "baseline_switches")
    assignments = subscript_assignments(switches_fn)
    for key in DYNAMIC_TAU_SWITCHES:
        assert key in assignments
    assert True in literal_values(assignments["dynamic_tau_enabled"])
    assert False in literal_values(assignments["dynamic_tau_enabled"])
    assert "seesm" in literal_values(assignments["cbf_metric"])
    assert "distance" in literal_values(assignments["cbf_metric"])
    assert "global_seesm_enable" in assignments

    build_fn = top_level_function(tree, "build_commands")
    write_meta_fn = top_level_function(tree, "write_run_meta")
    build_literals = string_literals(build_fn)
    metadata_literals = string_literals(write_meta_fn)
    for key in DYNAMIC_TAU_SWITCHES:
        assert any(key in value for value in build_literals)
    for key in ("dynamic_tau", "beta_source") + DYNAMIC_TAU_METADATA:
        assert key in metadata_literals


@pytest.mark.parametrize(
    ("baseline_id", "dynamic_enabled", "cbf_metric", "global_enabled", "beta_source"),
    BASELINE_CONTRACT,
)
def test_runner_generates_all_baseline_contracts_at_write_sites(
    tmp_path, baseline_id, dynamic_enabled, cbf_metric, global_enabled, beta_source
):
    original_sys_path = list(sys.path)
    try:
        with runner_module() as runner:
            scenario = minimal_scenario()
            obstacle_params = tmp_path / "obstacles_param.yaml"
            planner_cmd, _ = runner.build_commands(
                "head_on_context_bl", baseline_id, tmp_path, obstacle_params, "[adult]", 1, scenario
            )

            args = launch_args(planner_cmd)
            switches = runner.baseline_switches(runner.resolve_baseline_alias(baseline_id))
            for key in DYNAMIC_TAU_SWITCHES:
                assert key in args
                assert key in switches
                if key != "dynamic_tau_enabled":
                    assert float(args[key]) == float(switches[key])
            assert args["dynamic_tau_enabled"].lower() == str(dynamic_enabled).lower()
            assert args["cbf_metric"] == cbf_metric
            assert args["global_seesm_enable"].lower() == str(global_enabled).lower()

            meta_path = write_meta(runner, tmp_path, baseline_id)
            meta = yaml.safe_load(meta_path.read_text(encoding="utf-8"))
            dynamic_tau = meta["dynamic_tau"]
            assert dynamic_tau["enabled"] is dynamic_enabled
            assert dynamic_tau["beta_source"] == beta_source
            assert meta["cbf_metric"] == cbf_metric
            assert bool_value(meta["global_seesm_enable"]) is global_enabled
            for field in DYNAMIC_TAU_METADATA:
                assert field in dynamic_tau
                assert float(dynamic_tau[field]) == float(switches[METADATA_TO_SWITCH[field]])
    finally:
        assert sys.path == original_sys_path


def test_final_beta_and_audit_fields_are_at_their_actual_writer_paths():
    mpc = read("planner/mpc_secbf/src/mpc_secbf_node.cpp")
    mpc_constructor = cpp_function_body(mpc, "MpcSecbfNode(ros::NodeHandle& nh)")
    publish_region = cpp_function_body(mpc, "void publishAcceptedMargins")
    planner_row = cpp_function_body(mpc, "void writePlannerCsv")
    assert '"/safety_margin/beta_applied_final"' in mpc_constructor
    assert "out.beta_applied.assign(beta.begin(), beta.end())" in publish_region
    assert "pub_beta_applied_final_.publish(out)" in publish_region
    for field in TAU_FIELDS:
        assert field in mpc_constructor
    for token in ("tau", "T_i", "f_r", "f_v", "f_T", "tau_valid", "tau_reason"):
        assert token in planner_row

    obs_manager = read("planner/vomp_planner/traj_planner/include/obs_manager/obs_manager.hpp")
    obs_init = cpp_function_body(obs_manager, "void init(ros::NodeHandle &nh)")
    unsafe_region = cpp_function_body(obs_manager, "bool is_SEESM_unsafe")
    global_header = cpp_function_body(obs_manager, "void prepareGlobalSeesmLog")
    global_row = cpp_function_body(obs_manager, "void writeGlobalSeesmLog")
    assert 'subscribe("/safety_margin/beta_applied_final"' in obs_init
    assert "beta_applied = margin_entry.beta_applied" in unsafe_region
    assert "- beta_applied" in unsafe_region
    for field in TAU_FIELDS:
        assert field in global_header
        assert field in global_row

    guard = read("planner/semantic_guard/src/beta_guard_node.cpp")
    guard_header = cpp_function_body(guard, "BetaGuardNode(ros::NodeHandle& nh)")
    guard_row = cpp_function_body(
        guard, "void semanticCb(const semantic_fusion::SemanticObstacleArrayConstPtr& msg)"
    )
    row_tokens = (
        "tau_result.tau", "tau_result.T_i", "tau_result.f_r", "tau_result.f_v",
        "tau_result.f_T", "tau_result.valid", "tau_result.reason",
    )
    assert_tau_csv_contract(guard_header, guard_row, row_tokens)

    ground_truth = read("planner/semantic_guard/src/beta_ground_truth_node.cpp")
    ground_header = cpp_function_body(ground_truth, "BetaGroundTruthNode(ros::NodeHandle& nh)")
    ground_row = cpp_function_body(
        ground_truth, "void obsCb(const std_msgs::Float32MultiArrayConstPtr& msg)"
    )
    assert_tau_csv_contract(ground_header, ground_row, row_tokens)


def test_standard_mpc_cbf_and_legacy_acbf_are_separate_paths(tmp_path):
    tree = runner_tree()
    baselines = literal_assignment(tree, "BASELINES")
    assert baselines["Standard_MPC_CBF"]["planner"] == "secbf_planner.launch"
    assert baselines["B1_ACBF_fixed"]["planner"] == "acbf0_planner.launch"

    original_sys_path = list(sys.path)
    try:
        with runner_module() as runner:
            standard_cmd, _ = runner.build_commands(
                "head_on_context_bl", "Standard_MPC_CBF", tmp_path,
                tmp_path / "standard_obstacles.yaml", "[adult]", 1, minimal_scenario()
            )
            legacy_cmd, _ = runner.build_commands(
                "head_on_context_bl", "B1_ACBF_fixed", tmp_path,
                tmp_path / "legacy_obstacles.yaml", "[adult]", 1, minimal_scenario()
            )
            standard_args = launch_args(standard_cmd)
            assert standard_args["cbf_metric"] == "distance"
            assert all(not token.startswith("dynamic_tau_") for token in legacy_cmd)
            assert all(not token.startswith("cbf_metric:=") for token in legacy_cmd)
    finally:
        assert sys.path == original_sys_path

    legacy = read("planner/mpc_dcbf/src/mpc_cbf.cpp")
    legacy_safety = cpp_function_body(
        legacy,
        "void MPC_SOLVE::set_safety_st(std::string& smetric, casadi::Opti& opt, int index)",
    )
    legacy_tau = cpp_function_body(
        legacy, "double MPC_SOLVE::set_tau_value(Eigen::VectorXd _rob, Eigen::VectorXd _obs)"
    )
    assert 'if(smetric == "ACBF")' in legacy_safety
    assert "set_tau_value" in legacy_safety
    assert "dynamic_tau_enabled" not in legacy_safety
    assert "double tau_max" in legacy_tau
