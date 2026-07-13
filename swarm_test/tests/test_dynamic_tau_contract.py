import ast
import csv
from contextlib import contextmanager
import importlib.util
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml


CPP_RAW_STRING_START = re.compile(r'(?:u8|u|U|L)?R"([^ ()\\]*)\(')
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
DYNAMIC_TAU_NUMERIC_PARAMS = tuple(f"dynamic_tau/{field}" for field in DYNAMIC_TAU_METADATA)
METADATA_TO_SWITCH = {
    "Ke": "dynamic_tau_ke",
    "Tmax": "dynamic_tau_tmax",
    "min_speed": "dynamic_tau_min_speed",
    "min_distance": "dynamic_tau_min_distance",
    "max_tau": "dynamic_tau_max_tau",
}
EXPECTED_DYNAMIC_TAU_METADATA = {
    "formula": "tau=f_r*f_v*f_T*Ke*T_i",
    "h_ee": "||l+tau*v||-R_obs-R_robot",
    "h_see": "h_ee-beta",
}
BASELINE_CONTRACT = (
    ("Standard_MPC_CBF", False, "distance", False, "fixed", "fixed_config", 0.4, "false", 6),
    ("No_semantic", True, "seesm", False, "none", "zero", 0.0, "true", 6),
    ("Unguarded_SEESM", True, "seesm", True, "full", "candidate", None, "false", 6),
    ("SEESM_Ours", True, "seesm", True, "full", "beta_applied_final", None, "true", 6),
)
ALIAS_CONTRACT = {
    "Standard_MPC_CBF": "Standard_MPC_CBF",
    "EESM_MPC_ECBF": "No_semantic",
    "SEESM_Without_FPU": "Unguarded_SEESM",
    "Proposed_MPC_SECBF": "SEESM_Ours",
}


def read(relative):
    return (REPO_ROOT / relative).read_text(encoding="utf-8")


@contextmanager
def runner_module():
    original_sys_path = list(sys.path)
    original_path_importer_cache = dict(sys.path_importer_cache)
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
        sys.path_importer_cache.clear()
        sys.path_importer_cache.update(original_path_importer_cache)
        assert set(sys.modules) == set(original_sys_modules)
        assert all(
            sys.modules[name] is module
            for name, module in original_sys_modules.items()
        )
        assert sys.path == original_sys_path
        assert sys.path_importer_cache == original_path_importer_cache


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


def strip_cpp_comments_only(source):
    masked = list(source)
    state = "code"
    raw_delimiter = None
    index = 0

    def mask(position):
        if masked[position] != "\n":
            masked[position] = " "

    while index < len(source):
        if state == "code":
            raw_match = CPP_RAW_STRING_START.match(source, index)
            if raw_match:
                raw_delimiter = raw_match.group(1)
                index = raw_match.end()
                state = "raw"
            elif source.startswith("//", index):
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
                index += 1
                state = "string"
            elif source[index] == "'":
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
        elif state == "raw":
            terminator = ")" + raw_delimiter + '"'
            end = source.find(terminator, index)
            if end < 0:
                index = len(source)
            else:
                index = end + len(terminator)
                raw_delimiter = None
                state = "code"
        else:
            if source[index] == "\\":
                index += 2
            elif source[index] == ("\"" if state == "string" else "'"):
                index += 1
                state = "code"
            else:
                index += 1
    return "".join(masked)


def cpp_code_positions(source, start=0):
    state = "code"
    raw_delimiter = None
    index = start
    while index < len(source):
        if state == "code":
            raw_match = CPP_RAW_STRING_START.match(source, index)
            if raw_match:
                raw_delimiter = raw_match.group(1)
                index = raw_match.end()
                state = "raw"
            elif source[index] == '"':
                index += 1
                state = "string"
            elif source[index] == "'":
                index += 1
                state = "char"
            else:
                yield index
                index += 1
        elif state == "raw":
            terminator = ")" + raw_delimiter + '"'
            end = source.find(terminator, index)
            if end < 0:
                index = len(source)
            else:
                index = end + len(terminator)
                raw_delimiter = None
                state = "code"
        else:
            if source[index] == "\\":
                index += 2
            elif source[index] == ('"' if state == "string" else "'"):
                index += 1
                state = "code"
            else:
                index += 1


def cpp_function_bounds(source, signature):
    clean_source = strip_cpp_comments_only(source)
    signature_start = next(
        (index for index in cpp_code_positions(clean_source)
         if clean_source.startswith(signature, index)),
        None,
    )
    assert signature_start is not None, f"C++ function not found: {signature}"
    opening_brace = next(
        (index for index in cpp_code_positions(clean_source, signature_start)
         if clean_source[index] == "{"),
        None,
    )
    assert opening_brace is not None, f"C++ function body not found: {signature}"
    depth = 0
    for index in cpp_code_positions(clean_source, opening_brace):
        if clean_source[index] == "{":
            depth += 1
        elif clean_source[index] == "}":
            depth -= 1
            if depth == 0:
                return opening_brace + 1, index
    raise AssertionError(f"unterminated C++ function: {signature}")


def cpp_function_body(source, signature):
    start, end = cpp_function_bounds(source, signature)
    return strip_cpp_comments_only(source)[start:end]


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


def cpp_string_spans(source):
    clean_source = strip_cpp_comments_only(source)
    spans = []
    state = "code"
    raw_delimiter = None
    start = None
    index = 0
    while index < len(clean_source):
        if state == "code":
            raw_match = CPP_RAW_STRING_START.match(clean_source, index)
            if raw_match:
                raw_delimiter = raw_match.group(1)
                index = raw_match.end()
                state = "raw"
            elif clean_source[index] == '"':
                start = index
                index += 1
                state = "string"
            elif clean_source[index] == "'":
                index += 1
                state = "char"
            else:
                index += 1
        elif state == "raw":
            terminator = ")" + raw_delimiter + '"'
            end = clean_source.find(terminator, index)
            if end < 0:
                index = len(clean_source)
            else:
                index = end + len(terminator)
                raw_delimiter = None
                state = "code"
        else:
            if clean_source[index] == "\\":
                index += 2
            elif clean_source[index] == '"':
                spans.append((start, index + 1))
                start = None
                index += 1
                state = "code"
            else:
                index += 1
    return spans


def cpp_statement_from_prefix(source, stream_prefix, end_marker=";"):
    clean_source = strip_cpp_comments_only(source)
    if stream_prefix.startswith('"'):
        starts = (
            start
            for start, _ in cpp_string_spans(clean_source)
            if clean_source.startswith(stream_prefix, start)
        )
    else:
        starts = (
            index
            for index in cpp_code_positions(clean_source)
            if clean_source.startswith(stream_prefix, index)
        )
    start = next(starts, None)
    assert start is not None, f"CSV header writer not found: {stream_prefix}"
    end = next(
        (
            index
            for index in cpp_code_positions(clean_source, start)
            if clean_source.startswith(end_marker, index)
        ),
        None,
    )
    assert end is not None, f"CSV header writer terminator not found: {stream_prefix}"
    return clean_source[start:end]


def csv_header_fields(source, stream_prefix, end_marker=";"):
    statement = cpp_statement_from_prefix(source, stream_prefix, end_marker)
    header = "".join(cpp_string_literals(statement)).rstrip("\r\n")
    rows = list(csv.reader([header]))
    assert len(rows) == 1, f"CSV header is not one record: {stream_prefix}"
    return rows[0]


def cpp_stream_operands(source):
    clean_source = strip_cpp_comments_only(source)
    shifts = []
    state = "code"
    raw_delimiter = None
    index = 0
    while index < len(clean_source):
        if state == "code":
            if clean_source.startswith("<<", index):
                shifts.append(index)
                index += 2
            else:
                raw_match = CPP_RAW_STRING_START.match(clean_source, index)
                if raw_match:
                    raw_delimiter = raw_match.group(1)
                    index = raw_match.end()
                    state = "raw"
                elif clean_source[index] == '"':
                    index += 1
                    state = "string"
                elif clean_source[index] == "'":
                    index += 1
                    state = "char"
                else:
                    index += 1
        elif state == "raw":
            terminator = ")" + raw_delimiter + '"'
            end = clean_source.find(terminator, index)
            if end < 0:
                index = len(clean_source)
            else:
                index = end + len(terminator)
                raw_delimiter = None
                state = "code"
        else:
            if clean_source[index] == "\\":
                index += 2
            elif clean_source[index] == ('"' if state == "string" else "'"):
                index += 1
                state = "code"
            else:
                index += 1
    assert shifts, "CSV writer has no stream insertion expressions"
    return [
        clean_source[start + 2:end]
        for start, end in zip(shifts, shifts[1:] + [len(clean_source)])
    ]


def csv_value_expressions(writer_block):
    values = []
    for operand in cpp_stream_operands(writer_block):
        literals = cpp_string_literals(operand)
        if len(literals) == 1:
            literal = literals[0]
            if literal == "," or literal == "\n":
                continue
            if literal.startswith(",") and literal.endswith(","):
                values.extend(
                    fragment
                    for fragment in literal.split(",")[1:-1]
                    if fragment
                )
                continue
        values.append(re.sub(r"\s+", "", operand).rstrip(";").rstrip())
    return values


def assert_csv_writer_contract(header_fields, writer_block, expected_columns):
    expected_fields = [field for field, _ in expected_columns]
    expected_expressions = [
        re.sub(r"\s+", "", expression)
        for _, expression in expected_columns
    ]
    assert header_fields == expected_fields
    assert csv_value_expressions(writer_block) == expected_expressions


def assert_cpp_include(source, header):
    pattern = re.compile(r'^\s*#\s*include\s+"' + re.escape(header) + r'"\s*$')
    state = "code"
    raw_delimiter = None
    found = False
    for line in strip_cpp_comments_only(source).splitlines():
        if state == "code" and pattern.fullmatch(line):
            found = True
        index = 0
        while index < len(line):
            if state == "code":
                match = CPP_RAW_STRING_START.match(line, index)
                if match:
                    raw_delimiter = match.group(1)
                    state = "raw"
                    index = match.end()
                elif line[index] == '"':
                    state = "string"
                    index += 1
                elif line[index] == "'":
                    state = "char"
                    index += 1
                else:
                    index += 1
            elif state == "raw":
                terminator = ")" + raw_delimiter + '"'
                end = line.find(terminator, index)
                if end < 0:
                    index = len(line)
                else:
                    state = "code"
                    raw_delimiter = None
                    index = end + len(terminator)
            else:
                if line[index] == "\\":
                    index += 2
                elif line[index] == ('"' if state == "string" else "'"):
                    state = "code"
                    index += 1
                else:
                    index += 1
    assert found


def test_cpp_parser_preserves_writer_literals_and_real_include():
    source = r'''
// #include "semantic_guard/dynamic_tau.hpp"
const char* fake = "#include \"semantic_guard/dynamic_tau.hpp\"";
const char* braces = "{ not code }";
R"raw(
#include "semantic_guard/dynamic_tau.hpp"
)raw";
#include "semantic_guard/dynamic_tau.hpp"

void writer() {
  const char brace = '}';
  stream << "," << "tau" << "\n";
}
'''
    stripped = strip_cpp_comments_only(source)
    assert 'const char* fake = "#include \\\"semantic_guard/dynamic_tau.hpp\\\"";' in stripped
    body = cpp_function_body_raw(source, "void writer()")
    assert [cpp_string_literals(operand) for operand in cpp_stream_operands(body)] == [
        [","],
        ["tau"],
        ["\n"],
    ]
    assert_cpp_include(source, "semantic_guard/dynamic_tau.hpp")

    header_source = r'''
// csv_file_ << "fake,comment\n";
R"raw(
csv_file_ << "fake,raw\n";
)raw";
if (csv_file_.is_open()) {
  csv_file_ << "real,header" << "\n";
}
'''
    assert csv_header_fields(header_source, 'csv_file_ << "real,') == [
        "real",
        "header",
    ]


def test_cpp_parser_rejects_comment_and_string_pseudo_declarations():
    source = r'''
// #include "semantic_guard/dynamic_tau.hpp"
const char* fake = "#include \"semantic_guard/dynamic_tau.hpp\"";
R"raw(
#include "semantic_guard/dynamic_tau.hpp"
)raw";
'''
    with pytest.raises(AssertionError):
        assert_cpp_include(source, "semantic_guard/dynamic_tau.hpp")


def test_csv_header_parser_rejects_comment_and_raw_string_pseudo_headers():
    source = r'''
// csv_file_ << "fake,comment\n";
R"raw(
csv_file_ << "fake,raw\n";
)raw";
'''
    with pytest.raises(AssertionError):
        csv_header_fields(source, 'csv_file_ << "fake,')


def bool_value(value):
    if isinstance(value, bool):
        return value
    return str(value).lower() == "true"


def assert_runtime_snapshot(path_snapshot, importer_cache_snapshot, module_snapshot):
    assert sys.path == path_snapshot
    assert sys.path_importer_cache == importer_cache_snapshot
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


def test_margin_launch_dynamic_tau_numeric_params_are_explicit_doubles():
    launch_files = (
        "planner/semantic_guard/launch/beta_guard.launch",
        "planner/semantic_guard/launch/beta_ground_truth.launch",
    )
    for relative_path in launch_files:
        root = ET.parse(REPO_ROOT / relative_path).getroot()
        params = {
            param.attrib["name"]: param
            for param in root.iter("param")
            if "name" in param.attrib
        }
        for name in DYNAMIC_TAU_NUMERIC_PARAMS:
            assert name in params, f"missing dynamic tau param: {relative_path}: {name}"
            assert params[name].attrib.get("type") == "double", (
                f"dynamic tau param must be type=double: {relative_path}: {name}"
            )


def test_runner_import_context_is_reentrant_and_isolated():
    path_snapshot = list(sys.path)
    importer_cache_snapshot = dict(sys.path_importer_cache)
    module_snapshot = dict(sys.modules)
    for _ in range(3):
        with runner_module() as runner:
            assert runner.resolve_baseline_alias("Standard_MPC_CBF") == "Standard_MPC_CBF"
        assert_runtime_snapshot(path_snapshot, importer_cache_snapshot, module_snapshot)


def test_runner_alias_mapping_is_explicit_and_resolved():
    with runner_module() as runner:
        for requested, resolved in ALIAS_CONTRACT.items():
            assert runner.resolve_baseline_alias(requested) == resolved


def test_runner_source_has_structured_baseline_and_writer_assignments():
    tree = runner_tree()
    defaults = literal_assignment(tree, "DEFAULT_EXPERIMENT_SWITCHES")
    for key in DYNAMIC_TAU_SWITCHES:
        assert key in defaults

    baselines = literal_assignment(tree, "BASELINES")
    for baseline_id, _, expected_metric, _, _, _, _, _, _ in BASELINE_CONTRACT:
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
        "semantic_mode", "beta_source", "fixed_beta", "guard_enabled", "controller_index",
    ),
    BASELINE_CONTRACT,
)
def test_runner_generates_all_baseline_contracts_at_write_sites(
    tmp_path, baseline_id, dynamic_enabled, cbf_metric, global_enabled,
    semantic_mode, beta_source, fixed_beta, guard_enabled, controller_index
):
    original_sys_path = list(sys.path)
    original_importer_cache = dict(sys.path_importer_cache)
    original_sys_modules = dict(sys.modules)
    try:
        with runner_module() as runner:
            scenario = minimal_scenario()
            obstacle_params = tmp_path / "obstacles_param.yaml"
            planner_cmd, start_cmd = runner.build_commands(
                "head_on_context_bl", baseline_id, tmp_path, obstacle_params, "[adult]", 1, scenario
            )

            args = launch_args(planner_cmd)
            start_args = launch_args(start_cmd)
            assert args["semantic_mode"] == semantic_mode
            assert args["guard_enabled"] == guard_enabled
            assert bool_value(args["dynamic_tau_enabled"]) is dynamic_enabled
            assert args["cbf_metric"] == cbf_metric
            assert bool_value(args["global_seesm_enable"]) is global_enabled
            assert start_args["controller_index"] == str(controller_index)
            assert start_args["record_data"] == "true"
            assert start_args["scenario_index"] == str(runner.SCENARIO_INDEX["head_on_context_bl"])
            assert runner.resolve_baseline_alias(baseline_id) == baseline_id
            if fixed_beta is not None:
                assert float(args["fixed_beta"]) == fixed_beta
            else:
                assert "fixed_beta" in args
            for key in DYNAMIC_TAU_SWITCHES:
                assert key in args

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
            assert meta["guard_enable"] == guard_enabled
            assert float(meta["fixed_beta"]) == float(args["fixed_beta"])
            if fixed_beta is not None:
                assert float(meta["fixed_beta"]) == fixed_beta
            for field in DYNAMIC_TAU_METADATA:
                assert field in dynamic_tau
                assert float(dynamic_tau[field]) == float(args[METADATA_TO_SWITCH[field]])
    finally:
        assert_runtime_snapshot(original_sys_path, original_importer_cache, original_sys_modules)


def test_final_beta_and_audit_fields_are_at_their_actual_writer_paths():
    mpc = read("planner/mpc_secbf/src/mpc_secbf_node.cpp")
    assert_cpp_include(mpc, "semantic_guard/dynamic_tau.hpp")
    mpc_constructor_raw = cpp_function_body_raw(mpc, "MpcSecbfNode(ros::NodeHandle& nh)")
    open_csv = cpp_function_body(
        mpc,
        "void openCsv(std::ofstream& file, const std::string& path, const std::string& header)",
    )
    publish_region = cpp_function_body(mpc, "void publishAcceptedMargins")
    planner_row_if = cpp_function_body(mpc, "if (planner_csv_.is_open()) {")
    mpc_columns = (
        ("t", "t"),
        ("mpc_status", "mpc_status"),
        ("first_attempt_status", "first_attempt_status"),
        ("final_status", "final_status"),
        ("accepted_beta_source", "accepted_beta_source"),
        ("cmd_v", "cmd_vel_.linear.x"),
        ("cmd_w", "cmd_vel_.angular.z"),
        ("obs_count", "obs_count"),
        ("constrained_obs_count", "constrained_obs_count"),
        ("beta_count", "beta_list_.size()"),
        ("used_fallback", "(used_fallback ? 1 : 0)"),
        ("mpc_feasibility_guard_used", "(mpc_guard_used ? 1 : 0)"),
        ("slack", "solver_.last_slack_max"),
        ("slack_sum", "solver_.last_slack_sum"),
        ("slack_mean", "solver_.last_slack_mean"),
        ("slack_max", "solver_.last_slack_max"),
        ("solve_time_ms", "solve_time_ms"),
        ("dynamic_tau_enabled", "dynamic_tau_enabled"),
        ("tau", "tau_result.tau"),
        ("T_i", "tau_result.T_i"),
        ("f_r", "tau_result.f_r"),
        ("f_v", "tau_result.f_v"),
        ("f_T", "tau_result.f_T"),
        ("tau_valid", "tau_result.valid"),
        ("tau_reason", "tau_result.reason"),
    )
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
    assert_csv_writer_contract(planner_header_fields, planner_row_if, mpc_columns)

    obs_manager = read("planner/vomp_planner/traj_planner/include/obs_manager/obs_manager.hpp")
    obs_init_raw = cpp_function_body_raw(obs_manager, "void init(ros::NodeHandle &nh)")
    unsafe_region = cpp_function_body(obs_manager, "bool is_SEESM_unsafe")
    global_header_raw = cpp_function_body_raw(obs_manager, "void prepareGlobalSeesmLog")
    global_row_raw = cpp_function_body_raw(obs_manager, "void writeGlobalSeesmLog")
    global_row = cpp_function_body(obs_manager, "void writeGlobalSeesmLog")
    global_columns = (
        ("t", "t"),
        ("replan_id", "current_global_replan_id_"),
        ("global_seesm_enable", "(global_seesm_enable_ ? 1 : 0)"),
        ("obs_id", "obs_id"),
        ("beta_applied", "beta_applied"),
        ("accepted_source", "sanitizeCsvField(accepted_source)"),
        ("margin_age_ms", "margin_age_ms"),
        ("h_ee", "h_ee"),
        ("h_see", "h_see"),
        ("primitive_rejected", "(primitive_rejected ? 1 : 0)"),
        ("shot_rejected", "(shot_rejected ? 1 : 0)"),
        ("reason", "sanitizeCsvField(reason)"),
        ("global_replan_ms", "global_replan_ms"),
        ("tau", "tau_result.tau"),
        ("T_i", "tau_result.T_i"),
        ("f_r", "tau_result.f_r"),
        ("f_v", "tau_result.f_v"),
        ("f_T", "tau_result.f_T"),
        ("tau_valid", "tau_result.valid"),
        ("tau_reason", "tau_result.reason"),
    )
    global_header_clean = cpp_function_body(obs_manager, "void prepareGlobalSeesmLog")
    global_open_guard = cpp_function_body(
        global_row_raw, "if (!global_seesm_log_stream_.is_open()) {"
    )
    global_header_fields = csv_header_fields(global_header_raw, '"t,replan_id,')
    assert "/safety_margin/beta_applied_final" in cpp_string_literals(obs_init_raw)
    assert "beta_applied = margin_entry.beta_applied" in unsafe_region
    assert "- beta_applied" in unsafe_region
    assert "global_seesm_log_stream_" in global_header_clean
    assert "return;" in global_open_guard
    assert "global_seesm_log_stream_" in global_row
    assert_csv_writer_contract(global_header_fields, global_row, global_columns)

    guard = read("planner/semantic_guard/src/beta_guard_node.cpp")
    guard_header_raw = cpp_function_body_raw(guard, "BetaGuardNode(ros::NodeHandle& nh)")
    guard_header_if_raw = cpp_function_body_raw(guard_header_raw, "if (csv_file_.is_open()) {")
    guard_header_if = cpp_function_body(guard_header_raw, "if (csv_file_.is_open()) {")
    guard_callback = cpp_function_body_raw(
        guard,
        "void semanticCb(const semantic_fusion::SemanticObstacleArrayConstPtr& msg)",
    )
    guard_row_if = cpp_function_body(guard_callback, "if (csv_file_.is_open()) {")
    guard_columns = (
        ("time", "ros::Time::now().toSec()"),
        ("obs_id", "obs.id"),
        ("class", "cls"),
        ("beta_bar", "beta_bar_val"),
        ("mu", "mu"),
        ("beta_requested", "beta_hat"),
        ("beta_applied", "beta_final"),
        ("guard_upper_bound", "guard_upper_bound"),
        ("guard_passed", "(guard_pass ? 1 : 0)"),
        ("guard_status", "guard_status"),
        ("semantic_mode", "semantic_mode_"),
        ("delta_beta", "delta_beta"),
        ("rate_limit_active", "(rate_limit_active ? 1 : 0)"),
        ("projection_active", "(projection_active ? 1 : 0)"),
        ("d_i", "d_i"),
        ("rel_v_norm", "rel_v_norm"),
        ("ttc", "(std::isfinite(ttc) ? ttc : -1.0)"),
        ("ttc_norm", "ttc_norm"),
        ("inv_ttc", "inv_ttc"),
        ("cos_delta", "cos_delta"),
        ("rho_i", "obs.density_norm"),
        ("rho_norm", "rho_norm"),
        ("group_flag", "0"),
        ("h_ee", "h_ee"),
        ("h_see", "h_see"),
        ("R_base", "r_base"),
        ("R_sem", "r_sem"),
        ("tau", "tau_result.tau"),
        ("T_i", "tau_result.T_i"),
        ("f_r", "tau_result.f_r"),
        ("f_v", "tau_result.f_v"),
        ("f_T", "tau_result.f_T"),
        ("tau_valid", "tau_result.valid"),
        ("tau_reason", "sanitizeCsvField(tau_result.reason)"),
    )
    guard_header_fields = csv_header_fields(guard_header_if_raw, 'csv_file_ << "time,')
    assert "csv_file_" in guard_header_if
    assert_csv_writer_contract(guard_header_fields, guard_row_if, guard_columns)

    ground_truth = read("planner/semantic_guard/src/beta_ground_truth_node.cpp")
    ground_header_raw = cpp_function_body_raw(ground_truth, "BetaGroundTruthNode(ros::NodeHandle& nh)")
    ground_header_if_raw = cpp_function_body_raw(ground_header_raw, "if (csv_file_.is_open()) {")
    ground_header_if = cpp_function_body(ground_header_raw, "if (csv_file_.is_open()) {")
    ground_callback = cpp_function_body_raw(
        ground_truth,
        "void obsCb(const std_msgs::Float32MultiArrayConstPtr& msg)",
    )
    ground_row_if = cpp_function_body(ground_callback, "if (csv_file_.is_open()) {")
    ground_columns = tuple(
        (
            field,
            "obstacle_id" if field == "obs_id" else
            "density_norm" if field in {"rho_i", "rho_norm"} else expression,
        )
        for field, expression in guard_columns
    )
    ground_header_fields = csv_header_fields(ground_header_if_raw, 'csv_file_ << "time,')
    assert "csv_file_" in ground_header_if
    assert_csv_writer_contract(ground_header_fields, ground_row_if, ground_columns)


def test_standard_mpc_cbf_and_legacy_acbf_are_separate_paths(tmp_path):
    tree = runner_tree()
    baselines = literal_assignment(tree, "BASELINES")
    assert baselines["Standard_MPC_CBF"]["planner"] == "secbf_planner.launch"
    assert baselines["B1_ACBF_fixed"]["planner"] == "acbf0_planner.launch"

    original_sys_path = list(sys.path)
    original_importer_cache = dict(sys.path_importer_cache)
    original_sys_modules = dict(sys.modules)
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
        assert_runtime_snapshot(original_sys_path, original_importer_cache, original_sys_modules)

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
