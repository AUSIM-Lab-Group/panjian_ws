import pathlib


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]


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
