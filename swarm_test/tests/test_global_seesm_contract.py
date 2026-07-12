import pathlib
import xml.etree.ElementTree as ET


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
TRAJ_PLANNER_DIR = REPO_ROOT / "planner/vomp_planner/traj_planner"


def include_args(launch_text, include_suffix):
    root = ET.fromstring(launch_text)
    for include in root.findall("include"):
        if include.attrib.get("file", "").endswith(include_suffix):
            return {arg.attrib["name"]: arg.attrib.get("value") for arg in include.findall("arg")}
    raise AssertionError(f"include not found: {include_suffix}")


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
    assert "p_rel + tau_global_ * v_rel" in obs
    assert "norm() - radius - robot_R - beta_applied" in obs
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
