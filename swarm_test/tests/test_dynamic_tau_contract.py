import ast
import csv
from contextlib import contextmanager
import importlib.util
import math
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml


CPP_RAW_STRING_START = re.compile(r'(?:u8|u|U|L)?R"([^ ()\\]*)\(')
REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER_PATH = REPO_ROOT / "swarm_test/scripts/run_secbf_sim_experiments.py"
CSV_CHECKER_PATH = REPO_ROOT / "swarm_test/scripts/check_experiment_csv_fields.py"
DYNAMIC_TAU_SWITCHES = (
    "dynamic_tau_enabled",
    "dynamic_tau_mode",
    "dynamic_tau_delta_tau",
    "dynamic_tau_ke",
    "dynamic_tau_tmax",
    "dynamic_tau_min_speed",
    "dynamic_tau_min_distance",
    "dynamic_tau_max_tau",
)
DYNAMIC_TAU_METADATA = (
    "delta_tau", "Ke", "Tmax", "min_speed", "min_distance", "max_tau",
)
DYNAMIC_TAU_NUMERIC_PARAMS = tuple(f"dynamic_tau/{field}" for field in DYNAMIC_TAU_METADATA)
METADATA_TO_SWITCH = {
    "delta_tau": "dynamic_tau_delta_tau",
    "Ke": "dynamic_tau_ke",
    "Tmax": "dynamic_tau_tmax",
    "min_speed": "dynamic_tau_min_speed",
    "min_distance": "dynamic_tau_min_distance",
    "max_tau": "dynamic_tau_max_tau",
}
EXPECTED_DYNAMIC_TAU_METADATA = {
    "mode": "teacher_tca",
    "formula": "tau=clip(-(l dot v_rel)/(||v_rel||^2+delta_tau),0,max_tau)",
    "relative_position_convention": "l=p_robot-p_obstacle",
    "relative_velocity_convention": "v_rel=v_robot-v_obstacle",
    "prediction_sign": "l(t+tau)=l+tau*v_rel",
    "h_eesm": "||l+tau*v_rel||-R_obs-R_robot",
    "h_seesm": "h_eesm-beta",
}
BASELINE_CONTRACT = (
    ("Standard_MPC_CBF", False, "distance", False, "fixed", "fixed_config", 0.4, "false", 6),
    ("No_semantic", True, "seesm", False, "none", "zero", 0.0, "false", 6),
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


CHECKER_HEADERS = {
    "robot_log.csv": ["t", "x", "y", "yaw", "v", "w", "cmd_v", "cmd_w"],
    "obstacle_log.csv": [
        "t", "id", "class", "x", "y", "radius", "vx", "vy", "d_i", "rel_v", "TTC", "h_EE"
    ],
    "margin_guard_log.csv": [
        "time", "obstacle_cycle_id", "obs_id", "class", "d_i", "rel_v_norm",
        "ttc", "ttc_norm", "cos_delta", "rho_norm", "mu", "beta_bar",
        "beta_max", "beta_requested", "beta_previous", "beta_pre_guard",
        "beta_applied", "available_margin", "positive_increment_bound",
        "guard_upper_bound", "guard_passed", "accepted_source", "h_ee", "h_see",
        "guard_status",
        "semantic_mode", "delta_beta", "rate_limit_active", "projection_active",
    ],
    "planner_log.csv": [
        "t", "obstacle_cycle_id", "mpc_status", "first_attempt_status",
        "final_status", "accepted_beta_source",
        "cmd_v", "cmd_w", "slack", "slack_sum", "slack_mean", "slack_max",
        "solve_time_ms", "mpc_feasibility_guard_enabled", "candidate_feasibility_checked",
        "mpc_feasibility_guard_used",
    ],
    "timing_log.csv": ["t", "mpc_secbf_ms", "total_loop_time_ms"],
    "event_log.csv": ["t", "event", "detail"],
}
TAU_HEADERS = ["tau", "T_i", "f_r", "f_v", "f_T", "tau_valid", "tau_reason"]
TAU_STAGE_HEADERS = [
    "t", "obstacle_cycle_id", "obs_id", "stage", "tau_mode", "lx", "ly",
    "vrel_x", "vrel_y",
    "tca_raw", "tca_clipped", "tau", "tau_computed", "tau_active",
    "R_base", "beta", "h_eesm", "h_seesm", "tau_valid", "tau_reason",
]


def teacher_tau_metadata(enabled=True, mode="teacher_tca", policy=None):
    if policy is None:
        policy = "symbolic_stagewise" if enabled else "disabled"
    formula = (
        "tau=clip(Ke*clip(-(l dot v_rel)/(||v_rel||^2+delta_tau),0,max_tau),"
        "0,max_tau)"
        if mode == "teacher_ke_tca"
        else "tau=clip(-(l dot v_rel)/(||v_rel||^2+delta_tau),0,max_tau)"
    )
    return {
        "dynamic_tau": {
            "enabled": enabled,
            "mode": mode,
            "delta_tau": 1.0e-6,
            "Ke": 0.3,
            "Tmax": 2.0,
            "min_speed": 1.0e-6,
            "min_distance": 1.0e-6,
            "max_tau": 2.0,
            "formula": formula,
            "relative_position_convention": "l=p_robot-p_obstacle",
            "relative_velocity_convention": "v_rel=v_robot-v_obstacle",
            "prediction_sign": "l(t+tau)=l+tau*v_rel",
            "mpc_stage_policy": policy,
            "h_eesm": "||l+tau*v_rel||-R_obs-R_robot",
            "h_seesm": "h_eesm-beta",
        }
    }


def teacher_tau_stage_row(
    mode="teacher_tca", *, lx=2.0, ly=0.0, vrel_x=-1.0, vrel_y=0.0,
    delta_tau=1.0e-6, max_tau=2.0, ke=0.3, overrides=None,
):
    relative_dot = lx * vrel_x + ly * vrel_y
    speed_squared = vrel_x * vrel_x + vrel_y * vrel_y
    tca_raw = -relative_dot / (speed_squared + delta_tau)
    tca_clipped = min(max(tca_raw, 0.0), max_tau)
    tau_unclipped = ke * tca_clipped if mode == "teacher_ke_tca" else tca_clipped
    tau = min(tau_unclipped, max_tau)
    r_base = 0.8
    beta = 0.4
    h_eesm = math.hypot(lx + tau * vrel_x, ly + tau * vrel_y) - r_base
    active = tau > 0.0
    if tca_clipped <= 0.0:
        reason = "teacher_receding" if relative_dot > 0.0 else "teacher_tangent"
    elif mode == "teacher_ke_tca":
        reason = (
            "teacher_ke_tca_clipped"
            if tau_unclipped > max_tau
            else "teacher_ke_tca_active"
        )
    else:
        reason = "teacher_tca_clipped" if tca_raw > max_tau else "teacher_tca_active"
    row = {
        "t": "0.1",
        "obstacle_cycle_id": "1",
        "obs_id": "7",
        "stage": "0",
        "tau_mode": mode,
        "lx": f"{lx:.12g}",
        "ly": f"{ly:.12g}",
        "vrel_x": f"{vrel_x:.12g}",
        "vrel_y": f"{vrel_y:.12g}",
        "tca_raw": f"{tca_raw:.12g}",
        "tca_clipped": f"{tca_clipped:.12g}",
        "tau": f"{tau:.12g}",
        "tau_computed": "1",
        "tau_active": "1" if active else "0",
        "R_base": f"{r_base:.12g}",
        "beta": f"{beta:.12g}",
        "h_eesm": f"{h_eesm:.12g}",
        "h_seesm": f"{h_eesm - beta:.12g}",
        "tau_valid": "1",
        "tau_reason": reason,
    }
    row.update(overrides or {})
    return row


def write_tau_stage_fixture(run_dir, mode="teacher_tca", missing=(), rows=None):
    headers = [field for field in TAU_STAGE_HEADERS if field not in set(missing)]
    with (run_dir / "tau_stage_log.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        writer.writeheader()
        for row in rows if rows is not None else [teacher_tau_stage_row(mode)]:
            writer.writerow({field: row[field] for field in headers})


def write_checker_fixture(run_dir, tau_files=(), metadata_enabled=None, partial_tau=False):
    run_dir.mkdir(parents=True, exist_ok=True)
    for file_name, headers in CHECKER_HEADERS.items():
        rows = []
        output_headers = list(headers)
        if file_name in tau_files:
            output_headers.extend(TAU_HEADERS[:1] if partial_tau else TAU_HEADERS)
            rows = [{field: ("1" if field != "tau_reason" else "active") for field in output_headers}]
        with (run_dir / file_name).open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=output_headers)
            writer.writeheader()
            if rows:
                writer.writerows(rows)
    if metadata_enabled is not None:
        (run_dir / "meta.yaml").write_text(
            "dynamic_tau:\n  enabled: " + ("true" if metadata_enabled else "false") + "\n",
            encoding="utf-8",
        )


def run_csv_checker(run_dir):
    return subprocess.run(
        [sys.executable, str(CSV_CHECKER_PATH), str(run_dir)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )


@contextmanager
def checker_module():
    module_name = "dynamic_tau_csv_checker"
    original_sys_modules = dict(sys.modules)
    try:
        spec = importlib.util.spec_from_file_location(module_name, CSV_CHECKER_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        yield module
    finally:
        for name in list(sys.modules):
            if name not in original_sys_modules:
                del sys.modules[name]
        for name, module in original_sys_modules.items():
            sys.modules[name] = module


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
        "[pedestrian]",
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


def test_mpc_teacher_tau_is_symbolic_stagewise_and_only_legacy_is_frozen():
    source = read("planner/mpc_secbf/src/mpc_secbf.cpp")
    header = read("planner/mpc_secbf/include/mpc_secbf/mpc_secbf.h")
    solve = cpp_function_body_raw(source, "bool MPC_SECBF_SOLVE::solve(")
    h_cbf = cpp_function_body_raw(source, "casadi::MX MPC_SECBF_SOLVE::h_cbf(")

    assert "const bool freeze_legacy_tau" in solve
    assert "DynamicTauMode::kLegacyGate" in solve
    assert "computeFrozenStageTau(obs_k, *cur_state)" in solve
    assert "computeFrozenStageTau(obs_k1, *cur_state)" in solve
    assert "h_cbf(X_cur, obs_k, beta_i, tau_k)" in solve
    assert "h_cbf(X_nxt, obs_k1, beta_i, tau_k1)" in solve

    assert "double stage_tau" in header
    assert "if (dynamic_tau_params_.mode ==" in h_cbf
    assert "DynamicTauMode::kLegacyGate" in h_cbf
    assert "const double finite_stage_tau" in h_cbf
    assert "tau = finite_stage_tau" in h_cbf
    assert "tau = dynamicTauCasadi(lx, ly, vx, vy" in h_cbf
    assert "lx + tau * vx" in h_cbf
    assert "ly + tau * vy" in h_cbf
    assert "last_tau_stage_audit" in header
    assert "state_sol(0, stage)" in solve
    assert "state_sol(3, stage)" in solve


def test_mpc_teacher_symbolic_tau_is_the_production_constraint_path():
    source = read("planner/mpc_secbf/src/mpc_secbf.cpp")
    h_cbf = cpp_function_body_raw(source, "casadi::MX MPC_SECBF_SOLVE::h_cbf(")
    helper = cpp_function_body_raw(
        source, "casadi::MX MPC_SECBF_SOLVE::dynamicTauCasadi("
    )

    assert "dynamicTauCasadi" in h_cbf
    assert "Teacher-v1" in h_cbf
    assert "raw_tca = -dot / (speed_sq + dynamic_tau_params_.delta_tau)" in helper
    teacher_branch = helper.split("// Legacy-v1 algebraic reference", 1)[0]
    assert "inflated_radius" not in strip_cpp_comments_only(teacher_branch)


def test_mpc_teacher_symbolic_tau_clips_tca_then_applies_explicit_ke_mode():
    source = read("planner/mpc_secbf/src/mpc_secbf.cpp")
    helper = cpp_function_body_raw(
        source, "casadi::MX MPC_SECBF_SOLVE::dynamicTauCasadi("
    )

    assert "casadi::MX raw_tca =" in helper
    assert "casadi::MX clipped_tca =" in helper
    assert "DynamicTauMode::kTeacherKeTca" in helper
    assert "dynamic_tau_params_.ke * clipped_tca" in helper
    assert "return clipped_tca" in helper
    assert "lookahead_x" not in helper
    assert "lookahead_y" not in helper


def test_mpc_standard_distance_keeps_instantaneous_path():
    source = read("planner/mpc_secbf/src/mpc_secbf.cpp")
    h_cbf = cpp_function_body_raw(source, "casadi::MX MPC_SECBF_SOLVE::h_cbf(")

    assert "if (!dynamic_tau_enabled_)" in h_cbf
    assert (
        "return casadi::MX::sqrt(lx * lx + ly * ly) - obs_radius - robot_radius_ - beta_i;"
        in h_cbf
    )
    assert "R_safe" not in source
    assert "epsilon" in source


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
        assert "dynamic_tau/mode" in params


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


def test_csv_checker_accepts_legacy_tau_free_logs(tmp_path):
    run_dir = tmp_path / "legacy"
    write_checker_fixture(run_dir)
    result = run_csv_checker(run_dir)
    assert result.returncode == 0, result.stdout


def test_csv_checker_rejects_partial_tau_fields_even_for_legacy_logs(tmp_path):
    run_dir = tmp_path / "partial"
    write_checker_fixture(run_dir, tau_files={"margin_guard_log.csv"}, partial_tau=True)
    result = run_csv_checker(run_dir)
    assert result.returncode != 0
    assert "incomplete tau field group" in result.stdout


def test_csv_checker_requires_tau_fields_when_metadata_enables_dynamic_tau(tmp_path):
    run_dir = tmp_path / "dynamic_missing"
    write_checker_fixture(run_dir, metadata_enabled=True)
    result = run_csv_checker(run_dir)
    assert result.returncode != 0
    assert "missing complete tau field group" in result.stdout


@pytest.mark.parametrize(
    "metadata",
    (
        "dynamic_tau: [\n",
        "dynamic_tau: false\n",
        "dynamic_tau:\n  enabled: maybe\n",
        "dynamic_tau:\n",
    ),
)
def test_csv_checker_rejects_invalid_dynamic_tau_metadata(tmp_path, metadata):
    run_dir = tmp_path / "invalid_metadata"
    write_checker_fixture(run_dir)
    (run_dir / "meta.yaml").write_text(metadata, encoding="utf-8")
    result = run_csv_checker(run_dir)
    assert result.returncode != 0
    assert "meta.yaml" in result.stdout


def test_csv_checker_fails_closed_when_pyyaml_is_unavailable(tmp_path, monkeypatch):
    run_dir = tmp_path / "no_yaml"
    write_checker_fixture(run_dir, metadata_enabled=True)
    with checker_module() as checker:
        monkeypatch.setattr(checker, "yaml", None)
        monkeypatch.setattr(sys, "argv", [str(CSV_CHECKER_PATH), str(run_dir)])
        assert checker.main() == 1


def test_csv_checker_accepts_complete_tau_fields_when_metadata_enables_dynamic_tau(tmp_path):
    run_dir = tmp_path / "dynamic_complete"
    write_checker_fixture(
        run_dir,
        tau_files={"margin_guard_log.csv", "planner_log.csv"},
        metadata_enabled=True,
    )
    result = run_csv_checker(run_dir)
    assert result.returncode == 0, result.stdout


def test_csv_checker_requires_stagewise_tau_log_for_teacher_mode(tmp_path):
    run_dir = tmp_path / "teacher_missing_stage_log"
    write_checker_fixture(
        run_dir,
        tau_files={"margin_guard_log.csv", "planner_log.csv"},
    )
    (run_dir / "meta.yaml").write_text(
        yaml.safe_dump(teacher_tau_metadata()), encoding="utf-8"
    )

    result = run_csv_checker(run_dir)

    assert result.returncode != 0
    assert "missing file: tau_stage_log.csv" in result.stdout


def test_csv_checker_accepts_teacher_stagewise_tau_contract(tmp_path):
    run_dir = tmp_path / "teacher_complete"
    write_checker_fixture(
        run_dir,
        tau_files={"margin_guard_log.csv", "planner_log.csv"},
    )
    (run_dir / "meta.yaml").write_text(
        yaml.safe_dump(teacher_tau_metadata()), encoding="utf-8"
    )
    write_tau_stage_fixture(run_dir)

    result = run_csv_checker(run_dir)

    assert result.returncode == 0, result.stdout


def test_csv_checker_rejects_empty_teacher_stagewise_tau_log(tmp_path):
    run_dir = tmp_path / "teacher_empty_stage"
    write_checker_fixture(
        run_dir,
        tau_files={"margin_guard_log.csv", "planner_log.csv"},
    )
    (run_dir / "meta.yaml").write_text(
        yaml.safe_dump(teacher_tau_metadata()), encoding="utf-8"
    )
    write_tau_stage_fixture(run_dir, rows=[])

    result = run_csv_checker(run_dir)

    assert result.returncode != 0
    assert "no data rows for enabled Teacher dynamic tau" in result.stdout


@pytest.mark.parametrize("missing", ("lx", "ly", "vrel_x", "vrel_y", "tau_active"))
def test_csv_checker_requires_teacher_stage_replay_fields(tmp_path, missing):
    run_dir = tmp_path / f"teacher_missing_{missing}"
    write_checker_fixture(
        run_dir,
        tau_files={"margin_guard_log.csv", "planner_log.csv"},
    )
    (run_dir / "meta.yaml").write_text(
        yaml.safe_dump(teacher_tau_metadata()), encoding="utf-8"
    )
    write_tau_stage_fixture(run_dir, missing={missing})

    result = run_csv_checker(run_dir)

    assert result.returncode != 0
    assert missing in result.stdout


@pytest.mark.parametrize(
    ("field", "wrong_value"),
    (("tca_raw", "-2.0"), ("tca_clipped", "0.0"), ("tau", "0.6")),
)
def test_csv_checker_recomputes_teacher_stage_formula(tmp_path, field, wrong_value):
    run_dir = tmp_path / f"teacher_wrong_{field}"
    write_checker_fixture(
        run_dir,
        tau_files={"margin_guard_log.csv", "planner_log.csv"},
    )
    (run_dir / "meta.yaml").write_text(
        yaml.safe_dump(teacher_tau_metadata()), encoding="utf-8"
    )
    row = teacher_tau_stage_row(overrides={field: wrong_value})
    write_tau_stage_fixture(run_dir, rows=[row])

    result = run_csv_checker(run_dir)

    assert result.returncode != 0
    assert f"{field}=" in result.stdout
    assert "does not match recomputed" in result.stdout


def test_csv_checker_accepts_computed_but_inactive_teacher_zero_tau(tmp_path):
    run_dir = tmp_path / "teacher_receding"
    write_checker_fixture(
        run_dir,
        tau_files={"margin_guard_log.csv", "planner_log.csv"},
    )
    (run_dir / "meta.yaml").write_text(
        yaml.safe_dump(teacher_tau_metadata()), encoding="utf-8"
    )
    row = teacher_tau_stage_row(lx=1.0, vrel_x=1.0)
    assert row["tau"] == "0"
    assert row["tau_valid"] == "1"
    assert row["tau_active"] == "0"
    assert row["tau_reason"] == "teacher_receding"
    write_tau_stage_fixture(run_dir, rows=[row])

    result = run_csv_checker(run_dir)

    assert result.returncode == 0, result.stdout


@pytest.mark.parametrize(
    ("overrides", "expected_error"),
    (
        ({"tau_valid": "0"}, "tau_valid alias must match tau_computed=true"),
        ({"tau_active": "1"}, "does not match recomputed active=False"),
        ({"tau_reason": "teacher_tangent"}, "does not match recomputed 'teacher_receding'"),
    ),
)
def test_csv_checker_rejects_inconsistent_teacher_zero_tau_state(
    tmp_path, overrides, expected_error
):
    run_dir = tmp_path / "teacher_bad_zero_state"
    write_checker_fixture(
        run_dir,
        tau_files={"margin_guard_log.csv", "planner_log.csv"},
    )
    (run_dir / "meta.yaml").write_text(
        yaml.safe_dump(teacher_tau_metadata()), encoding="utf-8"
    )
    row = teacher_tau_stage_row(lx=1.0, vrel_x=1.0, overrides=overrides)
    write_tau_stage_fixture(run_dir, rows=[row])

    result = run_csv_checker(run_dir)

    assert result.returncode != 0
    assert expected_error in result.stdout


def test_csv_checker_recomputes_explicit_ke_teacher_diagnostic(tmp_path):
    run_dir = tmp_path / "teacher_ke"
    write_checker_fixture(
        run_dir,
        tau_files={"margin_guard_log.csv", "planner_log.csv"},
    )
    (run_dir / "meta.yaml").write_text(
        yaml.safe_dump(teacher_tau_metadata(mode="teacher_ke_tca")),
        encoding="utf-8",
    )
    write_tau_stage_fixture(run_dir, mode="teacher_ke_tca")

    result = run_csv_checker(run_dir)

    assert result.returncode == 0, result.stdout


def test_csv_checker_knows_every_teacher_runtime_reason():
    with checker_module() as checker:
        assert {
            "arithmetic_invalid",
            "invalid_mode",
            "stage_audit_unavailable",
            "teacher_receding",
            "teacher_tangent",
            "teacher_tca_active",
            "teacher_tca_clipped",
            "teacher_ke_tca_active",
            "teacher_ke_tca_clipped",
        }.issubset(checker.KNOWN_TAU_REASONS)


def test_csv_checker_rejects_teacher_frozen_tau_policy(tmp_path):
    run_dir = tmp_path / "teacher_frozen"
    write_checker_fixture(
        run_dir,
        tau_files={"margin_guard_log.csv", "planner_log.csv"},
    )
    (run_dir / "meta.yaml").write_text(
        yaml.safe_dump(
            teacher_tau_metadata(policy="numeric_frozen_per_stage")
        ),
        encoding="utf-8",
    )
    write_tau_stage_fixture(run_dir)

    result = run_csv_checker(run_dir)

    assert result.returncode != 0
    assert "must use mpc_stage_policy='symbolic_stagewise'" in result.stdout


def test_csv_checker_rejects_stage_mode_mismatch(tmp_path):
    run_dir = tmp_path / "teacher_mode_mismatch"
    write_checker_fixture(
        run_dir,
        tau_files={"margin_guard_log.csv", "planner_log.csv"},
    )
    (run_dir / "meta.yaml").write_text(
        yaml.safe_dump(teacher_tau_metadata()), encoding="utf-8"
    )
    write_tau_stage_fixture(run_dir, mode="teacher_ke_tca")

    result = run_csv_checker(run_dir)

    assert result.returncode != 0
    assert "does not match meta mode" in result.stdout


def test_standard_scenario_override_cannot_enable_dynamic_tau():
    with runner_module() as runner:
        switches = runner.scenario_switches(
            "Standard_MPC_CBF",
            {
                "experiment_switches": {
                    "dynamic_tau_enabled": True,
                    "dynamic_tau_mode": "legacy_gate",
                    "cbf_metric": "seesm",
                    "semantic_mode": "full",
                    "fixed_beta": 9.0,
                    "guard_enabled": "true",
                    "enable_rate_limit": "true",
                    "enable_available_projection": "true",
                    "enable_guard_fallback": "true",
                    "mpc_feasibility_guard_enabled": "true",
                    "front_adsm": "true",
                    "global_seesm_enable": "true",
                    "side_preference_enabled": "true",
                }
            },
        )

    assert switches["dynamic_tau_enabled"] is False
    assert switches["dynamic_tau_mode"] == "teacher_tca"
    assert switches["cbf_metric"] == "distance"
    assert switches["semantic_mode"] == "fixed"
    assert switches["fixed_beta"] == 0.4
    assert switches["guard_enabled"] == "false"
    assert switches["enable_rate_limit"] == "false"
    assert switches["enable_available_projection"] == "false"
    assert switches["enable_guard_fallback"] == "false"
    assert switches["mpc_feasibility_guard_enabled"] == "false"
    assert switches["front_adsm"] == "false"
    assert switches["global_seesm_enable"] == "false"
    assert switches["side_preference_enabled"] == "false"


def test_runner_tau_summary_falls_back_and_writes_audit_block(tmp_path):
    with runner_module() as runner:
        run_dir = tmp_path / "summary"
        run_dir.mkdir()
        (run_dir / "meta.yaml").write_text(
            "dynamic_tau:\n"
            "  enabled: true\n"
            "  mode: teacher_tca\n"
            "  delta_tau: 1.0e-6\n"
            "  Ke: 0.3\n"
            "  Tmax: 2.0\n"
            "  min_speed: 1.0e-6\n"
            "  min_distance: 1.0e-6\n"
            "  max_tau: 2.0\n"
            "  formula: 'tau=clip(-(l dot v_rel)/(||v_rel||^2+delta_tau),0,max_tau)'\n"
            "  relative_position_convention: l=p_robot-p_obstacle\n"
            "  relative_velocity_convention: v_rel=v_robot-v_obstacle\n"
            "  prediction_sign: l(t+tau)=l+tau*v_rel\n"
            "  mpc_stage_policy: symbolic_stagewise\n"
            "  h_eesm: '||l+tau*v_rel||-R_obs-R_robot'\n"
            "  h_seesm: 'h_eesm-beta'\n"
            "  beta_source: beta_applied_final\n",
            encoding="utf-8",
        )
        (run_dir / "margin_guard_log.csv").write_text("time,obs_id\n", encoding="utf-8")
        planner_headers = [
            "t", "mpc_status", "first_attempt_status", "final_status", "accepted_beta_source",
            "cmd_v", "cmd_w", "slack", "slack_sum", "slack_mean", "slack_max",
            "solve_time_ms", "mpc_feasibility_guard_enabled", "candidate_feasibility_checked",
            "mpc_feasibility_guard_used", *TAU_HEADERS,
        ]
        with (run_dir / "planner_log.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=planner_headers)
            writer.writeheader()
            writer.writerow({
                **{field: "0" for field in planner_headers},
                "tau": "0.25", "T_i": "1.0", "f_r": "1.0", "f_v": "1.0",
                "f_T": "1.0", "tau_valid": "1", "tau_reason": "active",
            })

        metrics = runner.summarize_tau_log(run_dir)
        assert metrics["tau_source"] == "planner_log.csv:mpc_stage_representative"
        assert metrics["tau_mean"] == "0.250000"
        runner.write_summary(run_dir, "head_on_context_bl", "SEESM_Ours", [], True, "ok", 1)

        summary_md = (run_dir / "summary.md").read_text(encoding="utf-8")
        for field in (
            "enabled", "mode", "delta_tau", "Ke", "Tmax", "min_speed",
            "min_distance", "max_tau", "formula", "h_eesm", "h_seesm",
            "beta_source", "relative_position_convention",
            "relative_velocity_convention", "prediction_sign", "mpc_stage_policy",
        ):
            assert f"- {field}:" in summary_md
        with (run_dir / "summary.csv").open("r", newline="", encoding="utf-8") as f:
            summary = next(csv.DictReader(f))
        assert summary["tau_source"] == "planner_log.csv:mpc_stage_representative"
        assert summary["dynamic_tau_enabled"] == "True"
        assert summary["dynamic_tau_mode"] == "teacher_tca"
        assert summary["dynamic_tau_mpc_stage_policy"] == "symbolic_stagewise"
        assert summary["dynamic_tau_beta_source"] == "beta_applied_final"


def test_runner_tau_summary_ignores_invalid_rows_and_falls_back(tmp_path):
    with runner_module() as runner:
        run_dir = tmp_path / "tau_fallback"
        run_dir.mkdir()
        headers = [*TAU_HEADERS]

        def write_tau_log(path, rows):
            with path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=headers)
                writer.writeheader()
                writer.writerows(rows)

        invalid_row = {
            "tau": "9.0", "T_i": "1.0", "f_r": "1.0", "f_v": "1.0",
            "f_T": "1.0", "tau_valid": "false", "tau_reason": "tau_invalid",
        }
        valid_row = {
            "tau": "0.4", "T_i": "1.0", "f_r": "1.0", "f_v": "1.0",
            "f_T": "1.0", "tau_valid": "true", "tau_reason": "active",
        }
        write_tau_log(run_dir / "margin_guard_log.csv", [invalid_row])
        write_tau_log(run_dir / "planner_log.csv", [valid_row])
        metrics = runner.summarize_tau_log(run_dir)
        assert metrics["tau_source"] == "planner_log.csv:mpc_stage_representative"
        assert metrics["tau_mean"] == "0.400000"
        assert metrics["tau_max"] == "0.400000"
        assert metrics["tau_active_fraction"] == "1.000000"
        assert metrics["tau_invalid_count"] == 0

        write_tau_log(run_dir / "margin_guard_log.csv", [valid_row, invalid_row])
        metrics = runner.summarize_tau_log(run_dir)
        assert metrics["tau_source"] == "margin_guard_log.csv:guard_current_state"
        assert metrics["tau_mean"] == "0.400000"
        assert metrics["tau_max"] == "0.400000"
        assert metrics["tau_active_fraction"] == "1.000000"
        assert metrics["tau_invalid_count"] == 1
        assert metrics["tau_reason_counts"] == "active:1;tau_invalid:1"


def test_runner_tau_summary_prioritizes_mpc_stage_and_splits_guard_population(tmp_path):
    with runner_module() as runner:
        run_dir = tmp_path / "tau_populations"
        run_dir.mkdir()
        active_stage = teacher_tau_stage_row()
        inactive_stage = teacher_tau_stage_row(lx=1.0, vrel_x=1.0)
        write_tau_stage_fixture(run_dir, rows=[active_stage, inactive_stage])

        with (run_dir / "margin_guard_log.csv").open(
            "w", newline="", encoding="utf-8"
        ) as f:
            writer = csv.DictWriter(f, fieldnames=TAU_HEADERS)
            writer.writeheader()
            writer.writerow({
                "tau": "0.4", "T_i": "0.4", "f_r": "1", "f_v": "1",
                "f_T": "1", "tau_valid": "1", "tau_reason": "active",
            })
        with (run_dir / "planner_log.csv").open(
            "w", newline="", encoding="utf-8"
        ) as f:
            writer = csv.DictWriter(f, fieldnames=TAU_HEADERS)
            writer.writeheader()
            writer.writerow({
                "tau": "0.8", "T_i": "0.8", "f_r": "1", "f_v": "1",
                "f_T": "1", "tau_valid": "1", "tau_reason": "active",
            })

        metrics = runner.summarize_tau_log(run_dir)

        expected_stage_tau = float(active_stage["tau"])
        assert metrics["tau_source"] == "tau_stage_log.csv:mpc_stage"
        assert metrics["tau_mean"] == f"{expected_stage_tau / 2.0:.6f}"
        assert metrics["tau_computational_valid_count"] == 2
        assert metrics["tau_active_count"] == 1
        assert metrics["tau_inactive_valid_count"] == 1
        assert metrics["tau_invalid_count"] == 0
        assert metrics["tau_active_fraction"] == "0.500000"
        assert metrics["tau_mpc_stage_record_count"] == 2
        assert metrics["tau_mpc_stage_source"] == "tau_stage_log.csv:mpc_stage"
        assert metrics["tau_guard_record_count"] == 1
        assert metrics["tau_guard_mean"] == "0.400000"
        assert metrics["tau_guard_source"] == "margin_guard_log.csv:guard_current_state"


def test_runner_classifies_auditable_termination_reasons(tmp_path):
    with runner_module() as runner:
        run_dir = tmp_path / "termination"
        run_dir.mkdir()
        for file_name in runner.REQUIRED_TRIAL_LOGS:
            (run_dir / file_name).write_text("header\nrow\n", encoding="utf-8")

        nav = {"nav_collision_count": "0"}
        planner = {"planner_records": 1, "first_infeasible_count": 0}
        phase5 = {
            "robot_records": 1,
            "goal_reached": 1,
            "robot_final_goal_distance_m": "0.10",
            "robot_mean_abs_v": "0.30",
            "log_min_distance_m": "0.40",
        }
        assert runner.classify_termination_reason(run_dir, nav, phase5, planner) == "success"

        phase5["log_min_distance_m"] = "-0.01"
        assert runner.classify_termination_reason(run_dir, nav, phase5, planner) == "collision"

        phase5.update({
            "goal_reached": 0,
            "robot_final_goal_distance_m": "4.0",
            "log_min_distance_m": "0.40",
        })
        planner["first_infeasible_count"] = 1
        assert runner.classify_termination_reason(run_dir, nav, phase5, planner) == "infeasible"

        planner["first_infeasible_count"] = 0
        phase5["robot_mean_abs_v"] = "0.01"
        assert runner.classify_termination_reason(run_dir, nav, phase5, planner) == "deadlock"

        phase5["robot_mean_abs_v"] = "0.20"
        assert runner.classify_termination_reason(run_dir, nav, phase5, planner) == "timeout"


def test_runner_ignores_all_zero_obstacle_startup_rows(tmp_path):
    with runner_module() as runner:
        run_dir = tmp_path / "zero_obstacle_row"
        run_dir.mkdir()
        (run_dir / "meta.yaml").write_text(
            "goal: [1.0, 0.0, 0.0]\nrobot_radius: 0.4\n", encoding="utf-8"
        )
        (run_dir / "robot_log.csv").write_text(
            "t,x,y,yaw,v,w,cmd_v,cmd_w\n0,0,0,0,0,0,0,0\n1,1,0,0,0,0,0,0\n",
            encoding="utf-8",
        )
        (run_dir / "obstacle_log.csv").write_text(
            "t,id,class,x,y,radius,vx,vy,d_i,rel_v,TTC,h_EE\n"
            "0,0,box,0,0,0.4,0,0,0,0,0,0\n"
            "1,0,box,0,0,0.4,0,0,1.0,0.2,1.0,0.2\n",
            encoding="utf-8",
        )
        for file_name, header in (
            ("margin_guard_log.csv", "time\n"),
            ("planner_log.csv", "t\nrow\n"),
            ("timing_log.csv", "t\nrow\n"),
            ("event_log.csv", "t\nrow\n"),
        ):
            (run_dir / file_name).write_text(header, encoding="utf-8")

        metrics = runner.summarize_phase5_logs(run_dir)
        assert metrics["log_invalid_obstacle_rows"] == 1
        assert metrics["log_min_distance_m"] == "0.200000"


def test_runner_aggregate_summary_unions_only_sealed_complete_columns(
    tmp_path, monkeypatch
):
    with runner_module() as runner:
        monkeypatch.setattr(runner, "is_complete_run_dir", lambda _: True)
        old_dir = tmp_path / "old"
        new_dir = tmp_path / "new"
        old_dir.mkdir()
        new_dir.mkdir()
        with (old_dir / "summary.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=["scenario", "baseline", "success"])
            writer.writeheader()
            writer.writerow({"scenario": "old", "baseline": "Standard", "success": "1"})
        with (new_dir / "summary.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(
                f, fieldnames=["scenario", "baseline", "success", "tau_source"]
            )
            writer.writeheader()
            writer.writerow({
                "scenario": "new", "baseline": "SEESM_Ours", "success": "1",
                "tau_source": "planner_log.csv",
            })

        aggregate = runner.write_aggregate_summary(tmp_path / "aggregate", [old_dir, new_dir])
        with aggregate.open("r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            assert reader.fieldnames == ["scenario", "baseline", "success", "tau_source"]
            rows = list(reader)
        assert rows[0]["tau_source"] == ""
        assert rows[1]["tau_source"] == "planner_log.csv"


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
    assignments = subscript_assignments(switches_fn)
    assert "dynamic_tau_enabled" in assignments
    assert "dynamic_tau_mode" in assignments
    assert ast_contains_text(switches_fn, "Standard_MPC_CBF")
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
    dynamic_tau_value = dict_value(meta_dicts[0], "dynamic_tau")
    assert isinstance(dynamic_tau_value, ast.Call)
    assert isinstance(dynamic_tau_value.func, ast.Name)
    assert dynamic_tau_value.func.id == "dynamic_tau_contract"
    contract_fn = top_level_function(tree, "dynamic_tau_contract")
    dynamic_tau_dict = next(
        node.value
        for node in ast.walk(contract_fn)
        if isinstance(node, ast.Return) and isinstance(node.value, ast.Dict)
    )
    assert isinstance(dynamic_tau_dict, ast.Dict)
    assert dict_keys(dynamic_tau_dict) >= {
        "enabled", "mode", "delta_tau", "Ke", "Tmax", "min_speed",
        "min_distance", "max_tau", "formula", "relative_position_convention",
        "relative_velocity_convention", "prediction_sign", "mpc_stage_policy",
        "h_eesm", "h_seesm", "beta_source",
    }


@pytest.mark.parametrize("baseline_id", ("Fixed_margin", "Category_only"))
def test_component_baselines_keep_dynamic_tau_enabled(baseline_id):
    with runner_module() as runner:
        switches = runner.baseline_switches(baseline_id)

    assert switches["dynamic_tau_enabled"] is True


def test_runner_uses_frozen_category_prior_table():
    with runner_module() as runner:
        assert runner.DEFAULT_BETA_BAR == {
            "box": 0.20,
            "adult": 0.75,
            "pedestrian": 0.75,
            "child": 1.05,
            "child_like": 1.05,
            "cyclist": 0.90,
            "vehicle": 0.80,
            "unknown": 0.75,
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
            assert args["dynamic_tau_mode"] == "teacher_tca"
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
                assert key in start_args
                assert start_args[key] == args[key]

            meta_path = write_meta(runner, tmp_path, baseline_id)
            meta = yaml.safe_load(meta_path.read_text(encoding="utf-8"))
            dynamic_tau = meta["dynamic_tau"]
            assert dynamic_tau["enabled"] is dynamic_enabled
            assert dynamic_tau["mode"] == "teacher_tca"
            assert dynamic_tau["mpc_stage_policy"] == (
                "symbolic_stagewise" if dynamic_enabled else "disabled"
            )
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
        ("obstacle_cycle_id", "active_obstacle_cycle_id_"),
        ("mpc_status", "mpc_status"),
        ("first_attempt_status", "first_attempt_status"),
        ("final_status", "final_status"),
        ("accepted_beta_source", "accepted_beta_source"),
        ("cmd_v", "cmd_vel_.linear.x"),
        ("cmd_w", "cmd_vel_.angular.z"),
        ("ref_x", "ref_x"),
        ("ref_y", "ref_y"),
        ("tracking_error", "tracking_error"),
        ("obs_count", "obs_count"),
        ("constrained_obs_count", "constrained_obs_count"),
        ("beta_count", "beta_list_.size()"),
        ("used_fallback", "(used_fallback ? 1 : 0)"),
        ("mpc_feasibility_guard_enabled", "(mpc_feasibility_guard_enabled_ ? 1 : 0)"),
        ("candidate_feasibility_checked", "((first_attempt_status == \"success\" || first_attempt_status == \"infeasible\") ? 1 : 0)"),
        ("mpc_feasibility_guard_used", "(mpc_guard_used ? 1 : 0)"),
        ("slack", "solver_.last_slack_max"),
            ("slack_sum", "solver_.last_slack_sum"),
            ("slack_mean", "solver_.last_slack_mean"),
            ("slack_max", "solver_.last_slack_max"),
            ("side_preference_enabled", "(side_preference_enabled_ ? 1 : 0)"),
            ("side_weight", "side_weight_"),
            ("side_cost", "solver_.last_side_cost"),
            ("side_dynamic_obstacle_count", "solver_.last_side_dynamic_obstacle_count"),
            ("side_candidate_count", "solver_.last_side_candidate_count"),
            ("side_dominant_obs_index", "solver_.last_side_dominant_obs_index"),
            ("side_dominant_stage", "solver_.last_side_dominant_stage"),
            ("side_dominant_tau", "solver_.last_side_dominant_tau"),
            ("side_dominant_h", "solver_.last_side_dominant_h"),
        ("solve_time_ms", "solve_time_ms"),
        ("dynamic_tau_enabled", "dynamic_tau_enabled"),
        ("tau_mode", "semantic_guard::dynamicTauModeName(tau_result.mode)"),
        ("tau", "tau_result.tau"),
        ("tca_raw", "tau_result.t_ca_raw"),
        ("tca_clipped", "tau_result.t_ca_clipped"),
        ("tau_scale", "(tau_result.ke_scaled ? dynamic_tau_params_.ke : 1.0)"),
        ("tau_computed", "tau_result.computed"),
        ("tau_active", "tau_result.valid"),
        ("tau_clipped_low", "tau_result.lower_clipped"),
        ("tau_clipped_high", "tau_result.upper_clipped"),
        ("T_i", "tau_result.T_i"),
        ("f_r", "tau_result.f_r"),
        ("f_v", "tau_result.f_v"),
        ("f_T", "tau_result.f_T"),
            ("tau_valid", "tau_result.computed"),
        ("tau_reason", "tau_result.reason"),
    )
    planner_header_start = mpc_constructor_raw.index("openCsv(planner_csv_")
    planner_header_end = mpc_constructor_raw.index("openCsv(timing_csv_", planner_header_start)
    planner_header_fields = csv_header_fields(
        mpc_constructor_raw[planner_header_start:planner_header_end],
        '"t,obstacle_cycle_id,mpc_status,',
    )
    assert "if (file.is_open())" in open_csv
    assert "file << header" in open_csv
    assert "/safety_margin/beta_applied_final" in cpp_string_literals(mpc_constructor_raw)
    assert "out.beta_applied.assign(beta.begin(), beta.end())" in publish_region
    assert "pub_beta_applied_final_.publish(out)" in publish_region
    assert_csv_writer_contract(planner_header_fields, planner_row_if, mpc_columns)

    margin_writer = cpp_function_body(mpc, "void writeMpcMarginCsv")
    assert "beta_list_[i]" in margin_writer
    assert "final_beta_values[i]" in margin_writer
    assert "accepted_beta_source" in margin_writer
    assert "candidate_checked" in margin_writer

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
        ("h_phys", "h_phys"),
        ("h_eesm", "h_eesm"),
        ("h_seesm", "h_seesm"),
        ("primitive_rejected", "(primitive_rejected ? 1 : 0)"),
        ("shot_rejected", "(shot_rejected ? 1 : 0)"),
        ("reason", "sanitizeCsvField(reason)"),
        ("global_replan_ms", "global_replan_ms"),
        ("tau", "tau_result.tau"),
        ("tau_mode", "semantic_guard::dynamicTauModeName(dynamic_tau_params_.mode)"),
        ("delta_tau", "dynamic_tau_params_.delta_tau"),
        ("relative_dot", "tau_result.relative_dot"),
        ("speed_squared", "tau_result.speed_squared"),
        ("denominator", "tau_result.denominator"),
        ("tca_raw", "tau_result.t_ca_raw"),
        ("tca_clipped", "tau_result.t_ca_clipped"),
        ("tau_unclipped", "tau_result.tau_unclipped"),
        ("lower_clipped", "tau_result.lower_clipped"),
        ("upper_clipped", "tau_result.upper_clipped"),
        ("ke_scaled", "tau_result.ke_scaled"),
        ("T_i", "tau_result.T_i"),
        ("f_r", "tau_result.f_r"),
        ("f_v", "tau_result.f_v"),
        ("f_T", "tau_result.f_T"),
        ("tau_computed", "tau_result.computed"),
        ("tau_active", "tau_result.valid"),
        ("tau_valid", "tau_result.computed"),
        ("tau_reason", "sanitizeCsvField(tau_result.reason)"),
    )
    global_header_clean = cpp_function_body(obs_manager, "void prepareGlobalSeesmLog")
    global_open_guard = cpp_function_body(
        global_row_raw, "if (!global_seesm_log_stream_.is_open()) {"
    )
    global_header_fields = csv_header_fields(global_header_raw, '"t,replan_id,')
    assert "/safety_margin/beta_applied_final" in cpp_string_literals(obs_init_raw)
    prepare_call = obs_init_raw.index("prepareGlobalSeesmLog();")
    enable_guard = obs_init_raw.index("if (global_seesm_enable_)")
    assert prepare_call < enable_guard
    enabled_body = cpp_function_body(obs_init_raw, "if (global_seesm_enable_)")
    assert "/safety_margin/beta_applied_final" in cpp_string_literals(enabled_body)
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
        ("beta_pre_guard", "beta_before_projection"),
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
        ("tau_mode", "semantic_guard::dynamicTauModeName(dynamic_tau_params_.mode)"),
        ("delta_tau", "dynamic_tau_params_.delta_tau"),
        ("relative_dot", "tau_result.relative_dot"),
        ("speed_squared", "tau_result.speed_squared"),
        ("denominator", "tau_result.denominator"),
        ("tca_raw", "tau_result.t_ca_raw"),
        ("tca_clipped", "tau_result.t_ca_clipped"),
        ("tau_unclipped", "tau_result.tau_unclipped"),
        ("lower_clipped", "tau_result.lower_clipped"),
        ("upper_clipped", "tau_result.upper_clipped"),
        ("ke_scaled", "tau_result.ke_scaled"),
        ("T_i", "tau_result.T_i"),
        ("f_r", "tau_result.f_r"),
        ("f_v", "tau_result.f_v"),
        ("f_T", "tau_result.f_T"),
        ("tau_valid", "tau_computed"),
        ("tau_reason", "sanitizeCsvField(tau_result.reason)"),
        ("h_phys", "h_phys"),
        ("h_eesm", "h_ee"),
        ("h_seesm", "h_see"),
        ("tau_computed", "tau_computed"),
        ("tau_active", "tau_active"),
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
        "void appliedMarginCb(",
    )
    ground_row_if = cpp_function_body(ground_callback, "if (csv_file_.is_open()) {")
    ground_columns = (
        ("time", "ros::Time::now().toSec()"),
        ("obstacle_cycle_id", "msg->obstacle_cycle_id"),
        ("obs_id", "audit.obstacle_id"),
        ("class", "audit.semantic_class"),
        ("beta_bar", "audit.beta_bar"),
        ("beta_max", "audit.beta_max"),
        ("mu", "audit.mu"),
        ("beta_requested", "audit.beta_tilde"),
        ("beta_previous", "audit.beta_previous"),
        ("beta_pre_guard", "audit.beta_pre"),
        ("beta_applied", "beta_applied"),
        ("available_margin", "audit.available_margin"),
        ("positive_increment_bound", "audit.positive_increment_bound"),
        ("guard_upper_bound", "audit.beta_upper_bound"),
        ("guard_passed", "(candidate_accepted ? 1 : 0)"),
        ("guard_status", "guard_status"),
        ("accepted_source", "sanitizeCsvField(accepted_source)"),
        ("semantic_mode", "semantic_mode_"),
        ("delta_beta", "delta_beta"),
        ("rate_limit_active", "(audit.rate_limit_active ? 1 : 0)"),
        ("projection_active", "(audit.projection_active ? 1 : 0)"),
        ("d_i", "audit.d_i"),
        ("rel_v_norm", "audit.rel_v_norm"),
        ("ttc", "(std::isfinite(audit.ttc) ? audit.ttc : -1.0)"),
        ("ttc_norm", "audit.ttc_norm"),
        ("inv_ttc", "audit.inv_ttc"),
        ("cos_delta", "audit.cos_delta"),
        ("rho_i", "audit.density_norm"),
        ("rho_norm", "audit.density_norm"),
        ("group_flag", "0"),
        ("h_ee", "audit.h_eesm"),
        ("h_see", "h_seesm"),
        ("R_base", "audit.r_base"),
        ("R_sem", "r_sem"),
        ("tau", "audit.tau_result.tau"),
        ("tau_mode", "semantic_guard::dynamicTauModeName(dynamic_tau_params_.mode)"),
        ("delta_tau", "dynamic_tau_params_.delta_tau"),
        ("relative_dot", "audit.tau_result.relative_dot"),
        ("speed_squared", "audit.tau_result.speed_squared"),
        ("denominator", "audit.tau_result.denominator"),
        ("tca_raw", "audit.tau_result.t_ca_raw"),
        ("tca_clipped", "audit.tau_result.t_ca_clipped"),
        ("tau_unclipped", "audit.tau_result.tau_unclipped"),
        ("lower_clipped", "audit.tau_result.lower_clipped"),
        ("upper_clipped", "audit.tau_result.upper_clipped"),
        ("ke_scaled", "audit.tau_result.ke_scaled"),
        ("T_i", "audit.tau_result.T_i"),
        ("f_r", "audit.tau_result.f_r"),
        ("f_v", "audit.tau_result.f_v"),
        ("f_T", "audit.tau_result.f_T"),
        ("tau_valid", "tau_computed"),
        ("tau_reason", "sanitizeCsvField(audit.tau_result.reason)"),
        ("h_phys", "audit.h_phys"),
        ("h_eesm", "audit.h_eesm"),
        ("h_seesm", "h_seesm"),
        ("tau_computed", "tau_computed"),
        ("tau_active", "tau_active"),
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
