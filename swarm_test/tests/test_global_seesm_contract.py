import pathlib
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
TRAJ_PLANNER_DIR = REPO_ROOT / "planner/vomp_planner/traj_planner"
SCRIPT_DIR = REPO_ROOT / "swarm_test/scripts"
CONFIG_PATH = REPO_ROOT / "swarm_test/config/secbf_scenarios.yaml"
GENERATOR_PATH = SCRIPT_DIR / "generate_seed_manifest.py"

if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import run_secbf_sim_experiments as runner  # noqa: E402


def include_args(launch_text, include_suffix):
    root = ET.fromstring(launch_text)
    for include in root.findall("include"):
        if include.attrib.get("file", "").endswith(include_suffix):
            return {arg.attrib["name"]: arg.attrib.get("value") for arg in include.findall("arg")}
    raise AssertionError(f"include not found: {include_suffix}")


def test_acbf_launch_supplies_required_initial_yaw():
    launch_text = (REPO_ROOT / "swarm_test/launch/acbf0_planner.launch").read_text(encoding="utf-8")
    args = include_args(launch_text, "scout_simulator.xml")
    assert args["init_yaw_"] == "0.0"


def test_quarantine_incomplete_run_dir_preserves_corrupt_logs(tmp_path):
    run_dir = tmp_path / "formal_002_head_on_context_bl_SEESM_Ours"
    run_dir.mkdir()
    corrupt_log = run_dir / "margin_guard_log.csv"
    corrupt_log.write_bytes(b"header\n\x00\x00\x00")

    quarantined = runner.quarantine_incomplete_run_dir(run_dir, "20260713_120000")

    assert not run_dir.exists()
    assert quarantined.name == ".interrupted_formal_002_head_on_context_bl_SEESM_Ours_20260713_120000"
    assert (quarantined / "margin_guard_log.csv").read_bytes() == b"header\n\x00\x00\x00"


def test_applied_margin_message_contract():
    path = REPO_ROOT / "planner/semantic_guard/msg/AppliedMarginArray.msg"
    text = path.read_text(encoding="utf-8")
    assert text == (
        "std_msgs/Header header\n"
        "uint32[] obstacle_ids\n"
        "float64[] beta_applied\n"
        "string[] accepted_sources\n"
    )


def test_id_topic_and_final_margin_publication_contract():
    obs = (REPO_ROOT / "planner/vomp_planner/traj_planner/include/obs_manager/obs_manager.hpp").read_text(encoding="utf-8")
    guard = (REPO_ROOT / "planner/semantic_guard/src/beta_ground_truth_node.cpp").read_text(encoding="utf-8")
    mpc = (REPO_ROOT / "planner/mpc_secbf/src/mpc_secbf_node.cpp").read_text(encoding="utf-8")
    launch = (REPO_ROOT / "planner/mpc_secbf/launch/mpc_secbf.launch").read_text(encoding="utf-8")
    cmake = (REPO_ROOT / "planner/mpc_secbf/CMakeLists.txt").read_text(encoding="utf-8")
    package = (REPO_ROOT / "planner/mpc_secbf/package.xml").read_text(encoding="utf-8")

    assert "obs_predict_ids" in obs
    assert "UInt32MultiArray" in obs
    assert "obs_predict_ids" in guard
    assert "semantic_guard::AppliedMarginArray" in mpc
    assert "beta_applied_final" in mpc
    assert "publishAcceptedMargins" in mpc
    assert "obs_predict_ids_topic" in launch
    assert "beta_applied_final_topic" in launch
    assert "semantic_guard" in cmake
    assert "<build_depend>semantic_guard</build_depend>" in package
    assert "<exec_depend>semantic_guard</exec_depend>" in package


def test_global_seesm_predicate_contract():
    obs = (TRAJ_PLANNER_DIR / "include/obs_manager/obs_manager.hpp").read_text(encoding="utf-8")
    astar_h = (TRAJ_PLANNER_DIR / "include/path_search/theta_astar.h").read_text(encoding="utf-8")
    astar_cpp = (TRAJ_PLANNER_DIR / "src/theta_astar.cpp").read_text(encoding="utf-8")

    assert "semantic_guard/AppliedMarginArray.h" in obs
    assert "beta_applied_final" in obs
    assert "/safety_margin/beta\"" not in obs
    assert "AppliedMarginArray" in obs
    assert "is_SEESM_unsafe" in obs
    assert "tau_result.tau = 0.0" in obs
    assert "tau_result.computed = true" in obs
    assert "tau_result.valid = false" in obs
    assert 'tau_result.reason = "disabled"' in obs
    assert "(p_rel + tau_result.tau * v_rel).norm() - radius - robot_R" in obs
    assert "const double h_phys = p_rel.norm() - radius - robot_R" in obs
    assert "const double h_seesm = h_eesm - beta_applied" in obs
    assert "global_seesm_enable" in astar_h
    assert "is_used_global_seesm_" in astar_h
    assert "is_SEESM_unsafe" in astar_cpp


def test_global_seesm_log_columns_exist():
    obs = (TRAJ_PLANNER_DIR / "include/obs_manager/obs_manager.hpp").read_text(encoding="utf-8")
    for value in (
        "beta_applied",
        "accepted_source",
        "margin_age_ms",
        "primitive_rejected",
        "shot_rejected",
        "reason",
        "global_replan_ms",
    ):
        assert value in obs


def test_global_seesm_final_beta_dynamic_tau_and_log_contract():
    obs = (TRAJ_PLANNER_DIR / "include/obs_manager/obs_manager.hpp").read_text(encoding="utf-8")
    global_launch = (TRAJ_PLANNER_DIR / "launch/plan_global_fsm.launch").read_text(encoding="utf-8")
    planner_launch = (REPO_ROOT / "swarm_test/launch/secbf_planner.launch").read_text(encoding="utf-8")

    assert 'nh.subscribe("/safety_margin/beta_applied_final"' in obs
    assert "beta_applied = margin_entry.beta_applied" in obs
    assert "beta_requested" not in obs
    assert "beta_candidate" not in obs

    dynamic_params = {
        "search/dynamic_tau_enabled": "bool",
        "search/dynamic_tau/Ke": "double",
        "search/dynamic_tau/Tmax": "double",
        "search/dynamic_tau/min_speed": "double",
        "search/dynamic_tau/min_distance": "double",
        "search/dynamic_tau/max_tau": "double",
    }
    root = ET.fromstring(global_launch)
    global_nodes = root.findall("node")
    assert len(global_nodes) == 2
    for node in global_nodes:
        params = {param.attrib["name"]: param for param in node.findall("param")}
        for name, expected_type in dynamic_params.items():
            assert params[name].attrib.get("type") == expected_type
    assert global_launch.count('search/dynamic_tau_enabled') == 2
    assert global_launch.count('search/dynamic_tau/Ke') == 2
    assert global_launch.count('search/dynamic_tau/Tmax') == 2
    assert global_launch.count('search/dynamic_tau/min_speed') == 2
    assert global_launch.count('search/dynamic_tau/min_distance') == 2
    assert global_launch.count('search/dynamic_tau/max_tau') == 2

    for name in (
        "dynamic_tau_enabled",
        "dynamic_tau_ke",
        "dynamic_tau_tmax",
        "dynamic_tau_min_speed",
        "dynamic_tau_min_distance",
        "dynamic_tau_max_tau",
    ):
        assert f'<arg name="{name}"' in planner_launch
        assert f'<arg name="{name}" value="$(arg {name})"/>' in planner_launch

    header_chunks = (
        '"t,replan_id,global_seesm_enable,obs_id,beta_applied,accepted_source,"',
        '"margin_age_ms,h_phys,h_eesm,h_seesm,primitive_rejected,shot_rejected,reason,global_replan_ms,"',
        '"T_i,f_r,f_v,f_T,tau_computed,tau_active,tau_valid,tau_reason\\n"',
    )
    assert obs.index(header_chunks[0]) < obs.index(header_chunks[1]) < obs.index(header_chunks[2])
    for field in (
        "tau_result.tau",
        "tau_result.T_i",
        "tau_result.f_r",
        "tau_result.f_v",
        "tau_result.f_T",
        "tau_result.valid",
        "sanitizeCsvField(tau_result.reason)",
    ):
        assert field in obs


def test_global_seesm_margin_age_uses_message_stamp_when_available():
    obs = (TRAJ_PLANNER_DIR / "include/obs_manager/obs_manager.hpp").read_text(encoding="utf-8")

    assert "entry.message_stamp = message_stamp;" in obs
    assert "entry.receipt_time = receipt_time;" in obs
    assert (
        "const ros::Time margin_time =\n"
        "            margin_entry.message_stamp.isZero() ? margin_entry.receipt_time : margin_entry.message_stamp;"
        in obs
    )
    assert "margin_age_ms = std::max(0.0, (receipt_time - margin_time).toSec() * 1000.0);" in obs
    assert "const bool margin_is_stale =\n            !margin_time.isZero() && (receipt_time - margin_time).toSec() > global_seesm_margin_timeout_;" in obs
    assert "(receipt_time - margin_entry.receipt_time).toSec() > global_seesm_margin_timeout_" not in obs


def test_global_seesm_launch_and_dependency_contract():
    planner_launch = (REPO_ROOT / "swarm_test/launch/secbf_planner.launch").read_text(encoding="utf-8")
    global_launch = (TRAJ_PLANNER_DIR / "launch/plan_global_fsm.launch").read_text(encoding="utf-8")
    cmake = (TRAJ_PLANNER_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    package = (TRAJ_PLANNER_DIR / "package.xml").read_text(encoding="utf-8")

    global_args = include_args(planner_launch, "plan_global_fsm.launch")
    assert global_args["global_seesm_enable"] == "$(arg global_seesm_enable)"
    assert global_args["global_seesm_margin_topic"] == "$(arg global_seesm_margin_topic)"
    assert global_args["global_seesm_log_path"] == "$(arg output_dir)/global_seesm_log.csv"

    assert '<arg name="global_seesm_enable" default="false"/>' in planner_launch
    assert '<arg name="global_seesm_margin_topic" default="/safety_margin/beta_applied_final"/>' in planner_launch

    assert '<arg name="global_seesm_enable" default="false"/>' in global_launch
    assert '<arg name="global_seesm_margin_topic" default="/safety_margin/beta_applied_final"/>' in global_launch
    assert '<arg name="global_seesm_log_path" default=""/>' in global_launch
    assert global_launch.count('search/global_seesm_enable') == 2
    assert global_launch.count('search/global_seesm_tau') == 2
    assert global_launch.count('search/global_seesm_margin_timeout') == 2
    assert global_launch.count('search/global_seesm_log_path') == 2

    assert "semantic_guard" in cmake
    assert "<build_depend>semantic_guard</build_depend>" in package
    assert "<build_export_depend>semantic_guard</build_export_depend>" in package
    assert "<exec_depend>semantic_guard</exec_depend>" in package


def load_scenarios():
    with CONFIG_PATH.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)["scenarios"]


def test_paper_aliases_have_exact_baseline_ids():
    assert runner.resolve_baseline_alias("Standard_MPC_CBF") == "Standard_MPC_CBF"
    assert runner.resolve_baseline_alias("EESM_MPC_ECBF") == "No_semantic"
    assert runner.resolve_baseline_alias("SEESM_Without_FPU") == "Unguarded_SEESM"
    assert runner.resolve_baseline_alias("Proposed_MPC_SECBF") == "SEESM_Ours"


def test_seed_manifest_materializes_the_same_paired_trial(tmp_path):
    seed_manifest = tmp_path / "seed.csv"
    seed_manifest.write_text(
        "\n".join(
            [
                "trial_id,seed,scenario_id,obstacle_id,start_x_offset_m,start_y_offset_m,speed_scale,start_delay_offset_s",
                "pilot_001,20260710,head_on_context_int,obs_001,0.3,0.0,1.1,0.5",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    scene = load_scenarios()["head_on_context_int"]
    trial = runner.load_seed_manifest(seed_manifest)[0]

    assert runner.materialize_trial(scene, trial) == runner.materialize_trial(scene, trial)


def test_seed_manifest_rejects_any_header_drift(tmp_path):
    seed_manifest = tmp_path / "seed.csv"
    seed_manifest.write_text(
        "\n".join(
            [
                "trial_id,seed,scenario_id,obstacle_id,start_x_offset_m,start_y_offset_m,speed_scale",
                "pilot_001,20260710,head_on_context_int,obs_001,0.3,0.0,1.1",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="header"):
        runner.load_seed_manifest(seed_manifest)


def test_materialize_trial_persists_effective_manifest_metadata_and_aliases(tmp_path):
    scene = load_scenarios()["head_on_context_int"]
    trial = {
        "trial_id": "pilot_001",
        "seed": 20260710,
        "scenario_id": "head_on_context_int",
        "rows": [
            {
                "trial_id": "pilot_001",
                "seed": 20260710,
                "scenario_id": "head_on_context_int",
                "obstacle_id": "obs_001",
                "start_x_offset_m": 0.3,
                "start_y_offset_m": 0.0,
                "speed_scale": 1.1,
                "start_delay_offset_s": 0.5,
            }
        ],
    }
    run_dir = tmp_path / "run"
    run_dir.mkdir()

    effective_scenario, obstacles = runner.materialize_trial(scene, trial)
    classes_arg = runner.obstacle_classes(obstacles)
    meta_path = runner.write_run_meta(
        run_dir,
        "head_on_context_int",
        "SEESM_Ours",
        effective_scenario,
        classes_arg,
        len(obstacles),
        30,
        trial=trial,
        resolved_baseline_id="SEESM_Ours",
        requested_baseline_label="Proposed_MPC_SECBF",
    )
    runner.write_trial_artifacts(run_dir, effective_scenario, trial, resolved_baseline_id="SEESM_Ours")

    meta = yaml.safe_load(meta_path.read_text(encoding="utf-8"))
    effective_yaml = yaml.safe_load((run_dir / "effective_obstacles.yaml").read_text(encoding="utf-8"))
    manifest_rows = list(runner.csv.DictReader((run_dir / "trial_manifest_row.csv").open("r", encoding="utf-8")))

    assert meta["baseline_id"] == "SEESM_Ours"
    assert meta["requested_baseline_label"] == "Proposed_MPC_SECBF"
    assert meta["paper_baseline_label"] == "Proposed_MPC_SECBF"
    assert meta["trial_manifest"]["trial_id"] == "pilot_001"
    assert meta["trial_manifest"]["seed"] == 20260710
    assert meta["perturbations"][0]["obstacle_id"] == "obs_001"
    assert effective_yaml["obstacles"][0]["obstacle_id"] == "obs_001"
    assert manifest_rows[0]["trial_id"] == "pilot_001"


@pytest.mark.parametrize(
    ("baseline_id", "expected_value"),
    [
        ("B1_ACBF_fixed", "false"),
        ("No_semantic", "false"),
        ("Unguarded_SEESM", "true"),
        ("SEESM_Ours", "true"),
    ],
)
def test_global_seesm_enable_values_match_baseline_contract(tmp_path, baseline_id, expected_value):
    scene = load_scenarios()["head_on_context_int"]
    classes_arg = runner.obstacle_classes(scene["obstacles"])
    planner_cmd, _ = runner.build_commands(
        "head_on_context_int",
        baseline_id,
        tmp_path,
        tmp_path / "obstacles_param.yaml",
        classes_arg,
        len(scene["obstacles"]),
        scene,
    )

    if baseline_id == "B1_ACBF_fixed":
        assert all(not item.startswith("global_seesm_enable:=") for item in planner_cmd)
    else:
        assert f"global_seesm_enable:={expected_value}" in planner_cmd


def test_generate_seed_manifest_is_deterministic_and_uses_prefix(tmp_path):
    output_a = tmp_path / "a.csv"
    output_b = tmp_path / "b.csv"
    cmd = [
        sys.executable,
        str(GENERATOR_PATH),
        "--scenario",
        "head_on_context_int",
        "--count",
        "2",
        "--seed",
        "20260710",
        "--prefix",
        "pilot",
    ]

    subprocess.run(cmd + ["--output", str(output_a)], check=True, cwd=REPO_ROOT, text=True)
    subprocess.run(cmd + ["--output", str(output_b)], check=True, cwd=REPO_ROOT, text=True)

    assert output_a.read_text(encoding="utf-8") == output_b.read_text(encoding="utf-8")
    rows = list(runner.csv.DictReader(output_a.open("r", encoding="utf-8")))
    assert rows[0]["trial_id"] == "pilot_001"
    assert rows[1]["trial_id"] == "pilot_002"
    assert [int(row["seed"]) for row in rows] == [20260710, 20260711]


def test_visual_closure_manifest_has_three_fixed_trial_groups():
    manifest = REPO_ROOT / "swarm_test/config/seed_manifests/20260710_visual_closure.csv"
    rows = list(runner.csv.DictReader(manifest.open("r", encoding="utf-8")))
    groups = {(row["scenario_id"], row["trial_id"], int(row["seed"])) for row in rows}

    assert groups == {
        ("head_on_context_int", "pilot_001", 20260710),
        ("crossing_context_int", "pilot_001", 20260710),
        ("local_crowding_context_int", "pilot_001", 20260710),
    }


def test_trial_materialization_uses_forward_map_x_bounds():
    scene = load_scenarios()["local_crowding_context_int"]
    trial = {
        "scenario_id": "local_crowding_context_int",
        "trial_id": "pilot_001",
        "seed": 20260710,
        "rows": [{
            "obstacle_id": "obs_003",
            "start_x_offset_m": 0.002648,
            "start_y_offset_m": 0.140720,
            "speed_scale": 1.017007,
            "start_delay_offset_s": 0.060865,
        }],
    }

    effective, _ = runner.materialize_trial(scene, trial)

    assert effective["obstacles"][2]["x"] == pytest.approx(7.202648)
