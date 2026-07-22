import csv
import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER_PATH = REPO_ROOT / "swarm_test/scripts/run_secbf_sim_experiments.py"
CHECKER_PATH = REPO_ROOT / "swarm_test/scripts/check_experiment_csv_fields.py"


def load_runner():
    script_dir = str(RUNNER_PATH.parent)
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)
    spec = importlib.util.spec_from_file_location("teacher_v1_runner_gate", RUNNER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_checker():
    spec = importlib.util.spec_from_file_location(
        "teacher_v1_csv_gate", CHECKER_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def runner():
    return load_runner()


def scenario():
    return {
        "description": "gate fixture",
        "start": {"x": 0.0, "y": 0.0},
        "goal": {"x": 2.0, "y": 0.0},
        "map": {"x": 10.0, "y": 8.0, "z": 3.0},
        "obstacles": [
            {
                "obstacle_id": "obs_001",
                "semantic_class": "adult",
                "x": 1.0,
                "y": 1.0,
                "slower": 8.0,
            }
        ],
    }


def provenance(runner, baseline="SEESM_Ours"):
    clean_repo = {
        "commit": "a" * 40,
        "tree": "d" * 40,
        "branch": "teacher-v1",
        "dirty": False,
        "working_tree_state_sha256": "b" * 64,
    }
    hashed = {"path": "/fixture", "sha256": "c" * 64}
    payload = {
        "repositories": {
            "panjian_ws": dict(clean_repo),
            "seesm_social_navigation": dict(clean_repo),
        },
        "inputs": {
            "teacher_manuscript": {
                "path": "/teacher.tex",
                "sha256": runner.TEACHER_MANUSCRIPT_SHA256,
            },
            "parameter_freeze": dict(hashed),
            "scenario_config": dict(hashed),
            "secbf_planner_launch": dict(hashed),
            "acbf0_planner_launch": dict(hashed),
            "start_test_launch": dict(hashed),
        },
        "ros": {
            "package_paths": {
                package: str((REPO_ROOT / relative).resolve())
                for package, relative in runner.EXPECTED_ROS_PACKAGE_PATHS.items()
            },
            "ros_package_path": "/fixture",
            "cmake_prefix_path": "/fixture/devel",
        },
        "build_environment": {
            "ros_distribution": "noetic",
            "cmake": "cmake version fixture",
            "compiler": "g++ fixture",
            "python": "Python fixture",
        },
    }

    labels = (
        ("legacy_acbf_mpc", "global_planner", "obstacle_manager", "phase5_logger")
        if baseline == "B1_ACBF_fixed"
        else (
            "teacher_mpc", "teacher_guard_ground_truth", "global_planner",
            "obstacle_manager", "phase5_logger",
        )
    )
    payload["runtime"] = {
        "baseline_id": baseline,
        "executables": {
            label: {
                "path": f"/fixture/devel/lib/{label}",
                "sha256": "e" * 64,
                "package_source_path": "/fixture/source",
                "build_prefix": "/fixture/devel",
                "executes_source_directly": label == "phase5_logger",
            }
            for label in labels
        },
    }
    return payload


def write_meta(runner, run_dir, baseline="SEESM_Ours", seed=17):
    run_dir.mkdir(parents=True, exist_ok=True)
    obstacle_artifact = run_dir / "obstacles_param.yaml"
    obstacle_artifact.write_text("obstacle_params: []\n", encoding="utf-8")
    trial = {
        "trial_id": "trial_001",
        "seed": seed,
        "scenario_id": "head_on_context_bl",
        "rows": [],
    }
    return runner.write_run_meta(
        run_dir,
        "head_on_context_bl",
        baseline,
        scenario(),
        "[adult]",
        1,
        5,
        trial=trial,
        protocol_id="teacher_v1_gate_test",
        run_context=provenance(runner, baseline),
        artifact_hashes={
            "obstacles_param.yaml": runner.sha256_file(obstacle_artifact)
        },
    )


def test_output_root_rejects_prefix_and_symlink_escape(tmp_path, monkeypatch, runner):
    allowed = tmp_path / "teacher_outputs"
    allowed.mkdir()
    monkeypatch.setattr(runner, "TEACHER_OUTPUT_ROOT", allowed)

    assert runner.ensure_teacher_output_root(allowed / "pilot") == allowed / "pilot"
    with pytest.raises(ValueError):
        runner.ensure_teacher_output_root(tmp_path / "teacher_outputs_evil" / "pilot")

    outside = tmp_path / "outside"
    outside.mkdir()
    escape = allowed / "escape"
    escape.symlink_to(outside, target_is_directory=True)
    with pytest.raises(ValueError):
        runner.ensure_teacher_output_root(escape / "trial")


def test_run_meta_is_canonical_versioned_and_uses_trial_seed(tmp_path, runner):
    run_dir = tmp_path / "run"
    meta_path = write_meta(runner, run_dir, seed=73)

    assert meta_path.read_bytes() == (run_dir / "run_meta.yaml").read_bytes()
    meta = yaml.safe_load(meta_path.read_text(encoding="utf-8"))
    assert meta["algorithm_version"] == "teacher_v1"
    assert meta["formula_version"] == "teacher_v1_formula_001"
    assert meta["log_schema_version"] == "teacher_v1_log_schema_001"
    assert meta["protocol_id"] == "teacher_v1_gate_test"
    assert meta["log_profile"] == "teacher_seesm_v1"
    assert meta["run_state"] == "running"
    assert meta["random_seed"] == 73
    assert meta["control_period_sec"] == pytest.approx(0.1)
    assert meta["prediction_step_sec"] == pytest.approx(0.2)
    assert meta["dynamic_tau"]["h_eesm"].startswith("||l+tau")
    assert "h_ee" not in meta["dynamic_tau"]
    assert "data_processor_summary.csv" in meta["required_logs"]
    assert "data_processor_distance.csv" in meta["required_logs"]


def test_per_trial_repository_state_is_rechecked(monkeypatch, runner):
    expected = {
        "path": "/fixture/repo",
        "commit": "a" * 40,
        "tree": "b" * 40,
        "branch": "teacher-v1",
        "dirty": False,
        "working_tree_state_sha256": "c" * 64,
    }
    context = {
        "repositories": {
            "panjian_ws": dict(expected),
            "seesm_social_navigation": dict(expected),
        }
    }
    monkeypatch.setattr(
        runner, "git_repo_provenance", lambda _path: dict(expected)
    )
    runner.verify_repository_context_unchanged(context)

    changed = dict(expected)
    changed["dirty"] = True
    monkeypatch.setattr(runner, "git_repo_provenance", lambda _path: changed)
    with pytest.raises(RuntimeError, match="changed during batch"):
        runner.verify_repository_context_unchanged(context)


def test_meta_contract_fails_if_compatibility_mirror_diverges(
    tmp_path, monkeypatch, runner
):
    allowed = tmp_path / "allowed"
    run_dir = allowed / "trial"
    monkeypatch.setattr(runner, "TEACHER_OUTPUT_ROOT", allowed)
    write_meta(runner, run_dir)

    passed, errors = runner.validate_run_meta_contract(run_dir, strict_provenance=True)
    assert passed, errors

    (run_dir / "meta.yaml").write_text("corrupt: true\n", encoding="utf-8")
    passed, errors = runner.validate_run_meta_contract(run_dir, strict_provenance=True)
    assert not passed
    assert any("byte-identical" in error for error in errors)


def test_complete_sentinel_is_hash_bound_and_negative_outcome_can_be_valid(
    tmp_path, monkeypatch, runner
):
    allowed = tmp_path / "allowed"
    run_dir = allowed / "collision_trial"
    monkeypatch.setattr(runner, "TEACHER_OUTPUT_ROOT", allowed)
    monkeypatch.setattr(
        runner, "audit_required_logs", lambda *_args, **_kwargs: (True, [])
    )
    monkeypatch.setattr(
        runner, "run_csv_contract_check", lambda *_args, **_kwargs: (True, "fixture")
    )
    write_meta(runner, run_dir)
    (run_dir / "summary.csv").write_text(
        "termination_reason,trial_valid\ncollision,True\n", encoding="utf-8"
    )
    (run_dir / "process_status.yaml").write_text(
        "unexpected_exits: []\nprocess_contract_passed: true\n", encoding="utf-8"
    )
    validation = {
        "trial_valid": True,
        "termination_reason": "collision",
        "csv_contract_passed": True,
        "metadata_contract_passed": True,
        "process_contract_passed": True,
        "safety_bound_passed": False,
        "invalid_reasons": [],
    }

    sentinel = runner.seal_trial(run_dir, True, validation)
    assert sentinel.name == runner.RUN_COMPLETE_SENTINEL
    assert runner.is_complete_run_dir(run_dir)

    with (run_dir / "run_meta.yaml").open("a", encoding="utf-8") as stream:
        stream.write("tampered: true\n")
    assert not runner.is_complete_run_dir(run_dir)


def test_seal_trial_downgrades_missing_logs_to_invalid(
    tmp_path, monkeypatch, runner
):
    allowed = tmp_path / "allowed"
    run_dir = allowed / "missing_logs"
    monkeypatch.setattr(runner, "TEACHER_OUTPUT_ROOT", allowed)
    write_meta(runner, run_dir)
    (run_dir / "summary.csv").write_text(
        "termination_reason,trial_valid\ntimeout,True\n", encoding="utf-8"
    )
    (run_dir / "process_status.yaml").write_text(
        "unexpected_exits: []\nprocess_contract_passed: true\n", encoding="utf-8"
    )
    validation = {
        "trial_valid": True,
        "termination_reason": "timeout",
        "csv_contract_passed": True,
        "metadata_contract_passed": True,
        "process_contract_passed": True,
        "safety_bound_passed": False,
        "invalid_reasons": [],
    }

    sentinel = runner.seal_trial(run_dir, True, validation)
    assert sentinel.name == runner.RUN_INVALID_SENTINEL
    assert not (run_dir / runner.RUN_COMPLETE_SENTINEL).exists()
    assert not runner.is_complete_run_dir(run_dir)


def test_summary_only_and_invalid_trials_are_not_complete(tmp_path, runner):
    summary_only = tmp_path / "summary_only"
    summary_only.mkdir()
    (summary_only / "summary.csv").write_text("success\n1\n", encoding="utf-8")
    assert not runner.is_complete_run_dir(summary_only)

    invalid = tmp_path / "invalid"
    invalid.mkdir()
    (invalid / runner.RUN_INVALID_SENTINEL).write_text(
        "TEACHER_V1_RUN_INVALID\n", encoding="utf-8"
    )
    assert not runner.is_complete_run_dir(invalid)


def _write_csv(path, fields, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def test_legacy_b1_profile_passes_without_teacher_guard_logs(tmp_path, runner):
    run_dir = tmp_path / "b1"
    write_meta(runner, run_dir, baseline="B1_ACBF_fixed")
    _write_csv(
        run_dir / "robot_log.csv",
        ["t", "x", "y", "yaw", "v", "w", "cmd_v", "cmd_w"],
        [
            {"t": 0, "x": 0, "y": 0, "yaw": 0, "v": 0, "w": 0, "cmd_v": 0, "cmd_w": 0},
            {"t": 1, "x": 0.1, "y": 0, "yaw": 0, "v": 0.1, "w": 0, "cmd_v": 0.1, "cmd_w": 0},
        ],
    )
    obstacle_fields = [
        "t", "id", "class", "x", "y", "radius", "vx", "vy", "d_i",
        "rel_v", "TTC", "h_EE",
    ]
    _write_csv(
        run_dir / "obstacle_log.csv",
        obstacle_fields,
        [
            {**{field: 0 for field in obstacle_fields}, "t": 0, "radius": 0.4, "d_i": 1.0, "h_EE": 0.2},
            {**{field: 0 for field in obstacle_fields}, "t": 1, "radius": 0.4, "d_i": 0.9, "h_EE": 0.1},
        ],
    )
    _write_csv(
        run_dir / "event_log.csv",
        ["t", "event", "detail"],
        [
            {"t": 0, "event": "start", "detail": "fixture"},
            {"t": 1, "event": "stop", "detail": "fixture"},
        ],
    )
    (run_dir / "data_processor_summary.csv").write_text(
        ",101,4,1.0,1.0,1.0,0,0,0,0,1.0\n", encoding="utf-8"
    )
    (run_dir / "data_processor_distance.csv").write_text(
        "0,1.0\n", encoding="utf-8"
    )

    passed, _ = runner.run_csv_contract_check(run_dir)
    assert passed, (run_dir / "csv_contract_check.txt").read_text(encoding="utf-8")

    (run_dir / "robot_log.csv").write_text(
        "t,x,y,yaw,v,w,cmd_v,cmd_w\n", encoding="utf-8"
    )
    passed, _ = runner.run_csv_contract_check(run_dir)
    assert not passed


def test_teacher_profile_missing_runtime_logs_fails_closed(tmp_path, runner):
    run_dir = tmp_path / "teacher_missing"
    write_meta(runner, run_dir)

    passed, _ = runner.run_csv_contract_check(run_dir)
    assert not passed
    output = (run_dir / "csv_contract_check.txt").read_text(encoding="utf-8")
    assert "missing file" in output


def test_disabled_tau_contract_requires_zero_inactive_computed_rows(tmp_path):
    checker = load_checker()
    path = tmp_path / "planner_log.csv"
    fields = [
        "tau", "tau_mode", "tau_computed", "tau_active", "tau_valid",
        "tau_reason",
    ]
    _write_csv(
        path,
        fields,
        [{
            "tau": "0", "tau_mode": "teacher_tca", "tau_computed": "1",
            "tau_active": "0", "tau_valid": "1", "tau_reason": "disabled",
        }],
    )
    errors = []
    checker.validate_disabled_tau_rows(
        path.name, path, "teacher_tca", errors
    )
    assert errors == []

    rows = read_csv_rows(path)
    rows[0]["tau"] = "0.2"
    _write_csv(path, fields, rows)
    errors = []
    checker.validate_disabled_tau_rows(
        path.name, path, "teacher_tca", errors
    )
    assert any("must equal zero" in error for error in errors)

    for field, bad_value, expected_error in (
        ("tau_computed", "0", "must be marked computed"),
        ("tau_active", "1", "must be false"),
        ("tau_reason", "no_constrained_obstacle", "must equal 'disabled'"),
    ):
        invalid_rows = [{
            "tau": "0", "tau_mode": "teacher_tca", "tau_computed": "1",
            "tau_active": "0", "tau_valid": "1", "tau_reason": "disabled",
        }]
        invalid_rows[0][field] = bad_value
        if field == "tau_computed":
            invalid_rows[0]["tau_valid"] = bad_value
        _write_csv(path, fields, invalid_rows)
        errors = []
        checker.validate_disabled_tau_rows(
            path.name, path, "teacher_tca", errors
        )
        assert any(expected_error in error for error in errors)


def test_cli_dry_run_cannot_escape_teacher_output_root(tmp_path):
    result = subprocess.run(
        [
            sys.executable, str(RUNNER_PATH), "--dry-run",
            "--scenario", "head_on_context_bl",
            "--baseline", "Standard_MPC_CBF",
            "--output-root", str(tmp_path / "outside"),
        ],
        cwd=REPO_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    assert result.returncode != 0
    assert "Teacher-v1 output must be under" in result.stdout


def test_seed_manifest_missing_selected_scenario_fails_closed(tmp_path, runner):
    manifest = tmp_path / "seed.csv"
    _write_csv(
        manifest,
        runner.SEED_MANIFEST_HEADER,
        [{
            "trial_id": "trial_001", "seed": 1,
            "scenario_id": "crossing_context_bl", "obstacle_id": "obs_001",
            "start_x_offset_m": 0, "start_y_offset_m": 0,
            "speed_scale": 1, "start_delay_offset_s": 0,
        }],
    )
    result = subprocess.run(
        [
            sys.executable, str(RUNNER_PATH), "--dry-run",
            "--scenario", "head_on_context_bl",
            "--baseline", "Standard_MPC_CBF",
            "--seed-manifest", str(manifest),
            "--output-root", str(runner.TEACHER_OUTPUT_ROOT / "99_cli_test"),
        ],
        cwd=REPO_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    assert result.returncode != 0
    assert "has no trials for selected scenario" in result.stdout


def read_csv_rows(path):
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))
