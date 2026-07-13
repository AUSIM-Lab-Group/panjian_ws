import ast
import csv
from contextlib import contextmanager
import importlib.util
import re
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
MPC_TAU_FIELDS = ("dynamic_tau_enabled",) + TAU_FIELDS
EXPECTED_DYNAMIC_TAU_METADATA = {
    "formula": "tau=f_r*f_v*f_T*Ke*T_i",
    "h_ee": "||l+tau*v||-R_obs-R_robot",
    "h_see": "h_ee-beta",
}
BASELINE_CONTRACT = (
    ("Standard_MPC_CBF", False, "distance", False, "fixed", "fixed_config", 0.4),
    ("No_semantic", True, "seesm", False, "none", "zero", 0.0),
    ("Unguarded_SEESM", True, "seesm", True, "full", "candidate", None),
    ("SEESM_Ours", True, "seesm", True, "full", "beta_applied_final", None),
)


def read(relative):
    return (REPO_ROOT / relative).read_text(encoding="utf-8")


@contextmanager
def runner_module():
    original_sys_path = list(sys.path)
    original_sys_modules = dict(sys.modules)
    module_name = "dynamic_tau_runner"
    try:
        script_dir = str(RUNNER_PATH.parent)
        if script_dir not in sys.path:
            sys.path.insert(0, script_dir)
        spec = importlib.util.spec_from_file_location(module_name, RUNNER_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        yield module
    finally:
        for name in list(sys.modules):
            if name not in original_sys_modules:
                del sys.modules[name]
        for name, module in original_sys_modules.items():
            sys.modules[name] = module
        sys.path[:] = original_sys_path
        assert set(sys.modules) == set(original_sys_modules)
        assert all(
            sys.modules[name] is module
            for name, module in original_sys_modules.items()
        )
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


def assigned_values(function_node, name):
    values = []
    for node in ast.walk(function_node):
        if not isinstance(node, ast.Assign):
            continue
        if any(isinstance(target, ast.Name) and target.id == name for target in node.targets):
            values.append(node.value)
    return values


def dict_key(node):
    if node is None:
        return None
    try:
        value = ast.literal_eval(node)
    except (ValueError, TypeError):
        return None
    return value if isinstance(value, str) else None


def dict_value(node, key):
    if not isinstance(node, ast.Dict):
        return None
    for key_node, value_node in zip(node.keys, node.values):
        if dict_key(key_node) == key:
            return value_node
    return None


def dict_keys(node):
    if not isinstance(node, ast.Dict):
        return set()
    return {key for key in (dict_key(item) for item in node.keys) if key is not None}


def membership_sets(function_node):
    sets = []
    for node in ast.walk(function_node):
        if not isinstance(node, ast.If) or not isinstance(node.test, ast.Compare):
            continue
        if len(node.test.ops) != 1 or not isinstance(node.test.ops[0], ast.In):
            continue
        if not isinstance(node.test.left, ast.Name) or node.test.left.id != "baseline_id":
            continue
        try:
            value = ast.literal_eval(node.test.comparators[0])
        except (ValueError, TypeError):
            continue
        if isinstance(value, (set, frozenset, tuple, list)):
            sets.append(set(value))
    return sets


def ast_contains_text(node, text):
    for child in ast.walk(node):
        if isinstance(child, ast.Constant) and isinstance(child.value, str) and text in child.value:
            return True
    return False


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


def strip_cpp_non_code(source):
    masked = list(source)
    state = "code"
    index = 0

    def mask(position):
        if masked[position] != "\n":
            masked[position] = " "

    while index < len(source):
        if state == "code":
            if source.startswith("//", index):
                mask(index)
                mask(index + 1)
                index += 2
                state = "line_comment"
            elif source.startswith("/*", index):
                mask(index)
                mask(index + 1)
                index += 2
                state = "block_comment"
            elif source[index] == '"':
                mask(index)
                index += 1
                state = "string"
            elif source[index] == "'":
                mask(index)
                index += 1
                state = "char"
            else:
                index += 1
        elif state == "line_comment":
            if source[index] == "\n":
                state = "code"
            else:
                mask(index)
            index += 1
        elif state == "block_comment":
            if source.startswith("*/", index):
                mask(index)
                mask(index + 1)
                index += 2
                state = "code"
            else:
                mask(index)
                index += 1
        else:
            if source[index] == "\\":
                mask(index)
                index += 1
                if index < len(source):
                    mask(index)
                    index += 1
            elif source[index] == ("\"" if state == "string" else "'"):
                mask(index)
                index += 1
                state = "code"
            else:
                mask(index)
                index += 1
    return "".join(masked)


def cpp_function_bounds(source, signature):
    clean_source = strip_cpp_non_code(source)
    signature_start = clean_source.find(signature)
    assert signature_start >= 0, f"C++ function not found: {signature}"
    opening_brace = clean_source.find("{", signature_start)
    assert opening_brace >= 0, f"C++ function body not found: {signature}"
    depth = 0
    for index in range(opening_brace, len(clean_source)):
        if clean_source[index] == "{":
            depth += 1
        elif clean_source[index] == "}":
            depth -= 1
            if depth == 0:
                return opening_brace + 1, index
    raise AssertionError(f"unterminated C++ function: {signature}")


def cpp_function_body(source, signature):
    start, end = cpp_function_bounds(source, signature)
    return strip_cpp_non_code(source)[start:end]


def cpp_function_body_raw(source, signature):
    start, end = cpp_function_bounds(source, signature)
    return source[start:end]


def cpp_string_literals(source):
    literals = []
    state = "code"
    buffer = []
    index = 0
    while index < len(source):
        if state == "code":
            if source.startswith("//", index):
                state = "line_comment"
                index += 2
            elif source.startswith("/*", index):
                state = "block_comment"
                index += 2
            elif source[index] == '"':
                buffer = []
                state = "string"
                index += 1
            elif source[index] == "'":
                state = "char"
                index += 1
            else:
                index += 1
        elif state == "line_comment":
            if source[index] == "\n":
                state = "code"
            index += 1
        elif state == "block_comment":
            if source.startswith("*/", index):
                state = "code"
                index += 2
            else:
                index += 1
        else:
            terminator = '"' if state == "string" else "'"
            if source[index] == "\\":
                if state == "string" and index + 1 < len(source):
                    escaped = source[index + 1]
                    buffer.append({
                        "n": "\n",
                        "r": "\r",
                        "t": "\t",
                        "\\": "\\",
                        '"': '"',
                    }.get(escaped, escaped))
                index += 2
            elif source[index] == terminator:
                if state == "string":
                    literals.append("".join(buffer))
                state = "code"
                index += 1
            else:
                if state == "string":
                    buffer.append(source[index])
                index += 1
    return literals


def csv_header_fields(source, stream_prefix, end_marker=";"):
    start = source.find(stream_prefix)
    assert start >= 0, f"CSV header writer not found: {stream_prefix}"
    end = source.find(end_marker, start)
    assert end >= 0, f"CSV header writer terminator not found: {stream_prefix}"
    header = "".join(cpp_string_literals(source[start:end])).rstrip("\r\n")
    rows = list(csv.reader([header]))
    assert len(rows) == 1, f"CSV header is not one record: {stream_prefix}"
    return rows[0]


def assert_csv_fields(header_fields, required_fields):
    assert len(header_fields) == len(set(header_fields))
    assert set(required_fields).issubset(set(header_fields))


def cpp_stream_operands(source):
    return [
        re.sub(r"\s+", "", operand)
        for operand in source.split("<<")[1:]
    ]


def assert_stream_writes(writer_body, expressions):
    operands = cpp_stream_operands(writer_body)
    for expression in expressions:
        normalized = re.sub(r"\s+", "", expression)
        assert any(normalized in operand for operand in operands), expression


def bool_value(value):
    if isinstance(value, bool):
        return value
    return str(value).lower() == "true"


def assert_runtime_snapshot(path_snapshot, module_snapshot):
    assert sys.path == path_snapshot
    assert set(sys.modules) == set(module_snapshot)
    assert all(
        sys.modules[name] is module
        for name, module in module_snapshot.items()
    )


def test_shared_policy_is_the_numeric_source_of_truth():
    header = read("planner/semantic_guard/include/semantic_guard/dynamic_tau.hpp")
    assert "struct DynamicTauParams" in header
    assert "struct DynamicTauResult" in header
    assert "computeDynamicTau" in header
    assert "f_r" in header and "f_v" in header and "f_T" in header
    assert "max_tau" in header


def test_runner_import_context_is_reentrant_and_isolated():
    path_snapshot = list(sys.path)
    module_snapshot = dict(sys.modules)
    for _ in range(2):
        with runner_module() as runner:
            assert runner.resolve_baseline_alias("Standard_MPC_CBF") == "Standard_MPC_CBF"
        assert_runtime_snapshot(path_snapshot, module_snapshot)


def test_runner_source_has_structured_baseline_and_writer_assignments():
    tree = runner_tree()
    defaults = literal_assignment(tree, "DEFAULT_EXPERIMENT_SWITCHES")
    for key in DYNAMIC_TAU_SWITCHES:
        assert key in defaults

    baselines = literal_assignment(tree, "BASELINES")
    for baseline_id, _, expected_metric, _, _, _, _ in BASELINE_CONTRACT:
        assert baseline_id in baselines
        assert baselines[baseline_id]["planner"] == "secbf_planner.launch"
        if baseline_id == "Standard_MPC_CBF":
            assert baselines[baseline_id]["cbf_metric"] == expected_metric
    assert baselines["B1_ACBF_fixed"]["planner"] == "acbf0_planner.launch"

    switches_fn = top_level_function(tree, "baseline_switches")
    baseline_sets = membership_sets(switches_fn)
    assert {"No_semantic", "Unguarded_SEESM", "SEESM_Ours"} in baseline_sets
    assert {"Unguarded_SEESM", "SEESM_Ours"} in baseline_sets
    assignments = subscript_assignments(switches_fn)
    assert "dynamic_tau_enabled" in assignments
    assert True in literal_values(assignments["dynamic_tau_enabled"])
    assert False in literal_values(assignments["dynamic_tau_enabled"])
    assert "seesm" in literal_values(assignments["cbf_metric"])
    assert "distance" in literal_values(assignments["cbf_metric"])
    assert "global_seesm_enable" in assignments

    build_fn = top_level_function(tree, "build_commands")
    write_meta_fn = top_level_function(tree, "write_run_meta")
    planner_lists = assigned_values(build_fn, "planner")
    assert any(
        all(ast_contains_text(planner, key) for key in DYNAMIC_TAU_SWITCHES)
        for planner in planner_lists
    )
    meta_dicts = [node for node in assigned_values(write_meta_fn, "meta") if isinstance(node, ast.Dict)]
    assert meta_dicts
    dynamic_tau_dict = dict_value(meta_dicts[0], "dynamic_tau")
    assert isinstance(dynamic_tau_dict, ast.Dict)
    assert dict_keys(dynamic_tau_dict) >= {
        "enabled", "Ke", "Tmax", "min_speed", "min_distance", "max_tau",
        "formula", "h_ee", "h_see", "beta_source",
    }


@pytest.mark.parametrize(
    (
        "baseline_id", "dynamic_enabled", "cbf_metric", "global_enabled",
        "semantic_mode", "beta_source", "fixed_beta",
    ),
    BASELINE_CONTRACT,
)
def test_runner_generates_all_baseline_contracts_at_write_sites(
    tmp_path, baseline_id, dynamic_enabled, cbf_metric, global_enabled,
    semantic_mode, beta_source, fixed_beta
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
            assert switches["semantic_mode"] == semantic_mode
            assert bool_value(switches["dynamic_tau_enabled"]) is dynamic_enabled
            assert switches["cbf_metric"] == cbf_metric
            assert bool_value(switches["global_seesm_enable"]) is global_enabled
            if fixed_beta is not None:
                assert float(switches["fixed_beta"]) == fixed_beta
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
            for key, expected in EXPECTED_DYNAMIC_TAU_METADATA.items():
                assert dynamic_tau[key] == expected
            assert meta["cbf_metric"] == cbf_metric
            assert bool_value(meta["global_seesm_enable"]) is global_enabled
            assert meta["semantic_mode"] == semantic_mode
            if fixed_beta is not None:
                assert float(meta["fixed_beta"]) == fixed_beta
            for field in DYNAMIC_TAU_METADATA:
                assert field in dynamic_tau
                assert float(dynamic_tau[field]) == float(switches[METADATA_TO_SWITCH[field]])
    finally:
        assert sys.path == original_sys_path


def test_final_beta_and_audit_fields_are_at_their_actual_writer_paths():
    mpc = read("planner/mpc_secbf/src/mpc_secbf_node.cpp")
    mpc_constructor_raw = cpp_function_body_raw(mpc, "MpcSecbfNode(ros::NodeHandle& nh)")
    open_csv = cpp_function_body(
        mpc,
        "void openCsv(std::ofstream& file, const std::string& path, const std::string& header)",
    )
    publish_region = cpp_function_body(mpc, "void publishAcceptedMargins")
    planner_row_if = cpp_function_body(mpc, "if (planner_csv_.is_open())")
    planner_header_start = mpc_constructor_raw.index("openCsv(planner_csv_")
    planner_header_end = mpc_constructor_raw.index("openCsv(timing_csv_", planner_header_start)
    planner_header_fields = csv_header_fields(
        mpc_constructor_raw[planner_header_start:planner_header_end],
        '"t,mpc_status,',
    )
    assert "if (file.is_open())" in open_csv
    assert "file << header" in open_csv
    assert "/safety_margin/beta_applied_final" in cpp_string_literals(mpc_constructor_raw)
    assert "out.beta_applied.assign(beta.begin(), beta.end())" in publish_region
    assert "pub_beta_applied_final_.publish(out)" in publish_region
    assert_csv_fields(planner_header_fields, MPC_TAU_FIELDS)
    assert "planner_csv_" in planner_row_if
    assert_stream_writes(
        planner_row_if,
        (
            "dynamic_tau_enabled",
            "tau_result.tau",
            "tau_result.T_i",
            "tau_result.f_r",
            "tau_result.f_v",
            "tau_result.f_T",
            "tau_result.valid",
            "tau_result.reason",
        ),
    )

    obs_manager = read("planner/vomp_planner/traj_planner/include/obs_manager/obs_manager.hpp")
    obs_init_raw = cpp_function_body_raw(obs_manager, "void init(ros::NodeHandle &nh)")
    unsafe_region = cpp_function_body(obs_manager, "bool is_SEESM_unsafe")
    global_header_raw = cpp_function_body_raw(obs_manager, "void prepareGlobalSeesmLog")
    global_row = cpp_function_body(obs_manager, "void writeGlobalSeesmLog")
    global_header_clean = cpp_function_body(obs_manager, "void prepareGlobalSeesmLog")
    global_header_fields = csv_header_fields(global_header_raw, '"t,replan_id,')
    assert "/safety_margin/beta_applied_final" in cpp_string_literals(obs_init_raw)
    assert "beta_applied = margin_entry.beta_applied" in unsafe_region
    assert "- beta_applied" in unsafe_region
    assert "global_seesm_log_stream_" in global_header_clean
    assert_csv_fields(global_header_fields, TAU_FIELDS)
    assert "global_seesm_log_stream_" in global_row
    assert_stream_writes(
        global_row,
        (
            "tau_result.tau",
            "tau_result.T_i",
            "tau_result.f_r",
            "tau_result.f_v",
            "tau_result.f_T",
            "tau_result.valid",
            "tau_result.reason",
        ),
    )

    guard = read("planner/semantic_guard/src/beta_guard_node.cpp")
    guard_header_raw = cpp_function_body_raw(guard, "BetaGuardNode(ros::NodeHandle& nh)")
    guard_header_if_raw = cpp_function_body_raw(guard_header_raw, "if (csv_file_.is_open())")
    guard_header_if = cpp_function_body(guard_header_raw, "if (csv_file_.is_open())")
    guard_callback = cpp_function_body_raw(
        guard,
        "void semanticCb(const semantic_fusion::SemanticObstacleArrayConstPtr& msg)",
    )
    guard_row_if = cpp_function_body(guard_callback, "if (csv_file_.is_open())")
    guard_header_fields = csv_header_fields(guard_header_if_raw, 'csv_file_ << "time,')
    assert "csv_file_" in guard_header_if
    assert_csv_fields(guard_header_fields, TAU_FIELDS)
    assert "csv_file_" in guard_row_if
    assert_stream_writes(
        guard_row_if,
        (
            "tau_result.tau",
            "tau_result.T_i",
            "tau_result.f_r",
            "tau_result.f_v",
            "tau_result.f_T",
            "tau_result.valid",
            "tau_result.reason",
        ),
    )

    ground_truth = read("planner/semantic_guard/src/beta_ground_truth_node.cpp")
    ground_header_raw = cpp_function_body_raw(ground_truth, "BetaGroundTruthNode(ros::NodeHandle& nh)")
    ground_header_if_raw = cpp_function_body_raw(ground_header_raw, "if (csv_file_.is_open())")
    ground_header_if = cpp_function_body(ground_header_raw, "if (csv_file_.is_open())")
    ground_callback = cpp_function_body_raw(
        ground_truth,
        "void obsCb(const std_msgs::Float32MultiArrayConstPtr& msg)",
    )
    ground_row_if = cpp_function_body(ground_callback, "if (csv_file_.is_open())")
    ground_header_fields = csv_header_fields(ground_header_if_raw, 'csv_file_ << "time,')
    assert "csv_file_" in ground_header_if
    assert_csv_fields(ground_header_fields, TAU_FIELDS)
    assert "csv_file_" in ground_row_if
    assert_stream_writes(
        ground_row_if,
        (
            "tau_result.tau",
            "tau_result.T_i",
            "tau_result.f_r",
            "tau_result.f_v",
            "tau_result.f_T",
            "tau_result.valid",
            "tau_result.reason",
        ),
    )


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
    legacy_safety_raw = cpp_function_body_raw(
        legacy,
        "void MPC_SOLVE::set_safety_st(std::string& smetric, casadi::Opti& opt, int index)",
    )
    legacy_tau = cpp_function_body(
        legacy, "double MPC_SOLVE::set_tau_value(Eigen::VectorXd _rob, Eigen::VectorXd _obs)"
    )
    assert "if(smetric ==" in legacy_safety
    assert "ACBF" in cpp_string_literals(legacy_safety_raw)
    assert "set_tau_value" in legacy_safety
    assert "dynamic_tau_enabled" not in legacy_safety
    assert "double tau_max" in legacy_tau
