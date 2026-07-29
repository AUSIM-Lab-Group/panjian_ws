import pathlib
import xml.etree.ElementTree as ET

import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
REAL_LAUNCH = REPO_ROOT / "swarm_test/launch_exp/scout_secbf_real.launch"


def launch_args(path):
    root = ET.fromstring(path.read_text(encoding="utf-8"))
    return {
        arg.attrib["name"]: arg.attrib.get("default")
        for arg in root.findall("arg")
    }


def test_real_launch_is_separate_and_dry_run_by_default():
    text = REAL_LAUNCH.read_text(encoding="utf-8")
    args = launch_args(REAL_LAUNCH)

    assert args["cmd_vel_topic"] == "/cmd_vel_secbf_dryrun"
    assert args["v_max"] == "0.15"
    assert args["reverse_v_max"] == "0.15"
    assert args["o_max"] == "0.30"
    assert args["track_id_start"] == "4000"
    assert args["obstacle_ids"] == "[4000]"
    assert args["obstacle_classes"] == "[pedestrian]"
    assert "beta_ground_truth.launch" in text
    assert "mpc_secbf.launch" in text
    assert "plan_global_fsm.launch" in text
    assert "semantic_fusion" not in text
    assert "beta_guard.launch" not in text
    assert "map_generator" not in text
    assert "scout_simulator" not in text


def test_existing_simulation_entrypoint_and_runner_are_unchanged():
    runner = (
        REPO_ROOT / "swarm_test/scripts/run_secbf_sim_experiments.py"
    ).read_text(encoding="utf-8")
    planner = REPO_ROOT / "swarm_test/launch/secbf_planner.launch"

    assert planner.exists()
    assert '"planner": "secbf_planner.launch"' in runner
    assert "scout_secbf_real.launch" not in runner


def test_fast_lio_real_and_sim_configs_are_separate():
    real_config = yaml.safe_load(
        (REPO_ROOT / "state_estimation/FAST_LIO/config/velodyne.yaml").read_text(
            encoding="utf-8"
        )
    )
    sim_config = yaml.safe_load(
        (
            REPO_ROOT / "state_estimation/FAST_LIO/config/velodyne_sim.yaml"
        ).read_text(encoding="utf-8")
    )
    sim_launch = (
        REPO_ROOT / "state_estimation/FAST_LIO/launch/mapping_velodyne_use.launch"
    ).read_text(encoding="utf-8")

    assert real_config["preprocess"]["blind"] == 0.2
    assert real_config["pcd_save"]["pcd_save_en"] is False
    assert sim_config["preprocess"]["blind"] == 2
    assert sim_config["pcd_save"]["pcd_save_en"] is True
    assert "velodyne_sim.yaml" in sim_launch


def test_tracker_uses_parameterized_monotonic_ids_and_source_time():
    header = (
        REPO_ROOT
        / "perception/dynamic_perception/include/dynamic_perception/cluster_track.hpp"
    ).read_text(encoding="utf-8")
    predictor_header = (
        REPO_ROOT
        / "perception/dynamic_perception/include/dynamic_perception/obstacle_prediction.h"
    ).read_text(encoding="utf-8")
    source = (
        REPO_ROOT / "perception/dynamic_perception/src/obstacle_prediction.cpp"
    ).read_text(encoding="utf-8")
    launch = (
        REPO_ROOT / "swarm_test/launch_exp/start_perception.launch"
    ).read_text(encoding="utf-8")

    assert "static std::atomic" not in header
    assert "id_int = track_id;" in header
    assert "track_id_start_" in predictor_header
    assert "track_ids_.next()" in source
    assert "isValidTrackAssignment" in source
    assert "num_frames_skipped = 0" in source
    assert "ros::Time().fromSec(time_pcloud)" in source
    assert 'name="track_id_start" default="4000"' in launch


def test_obs_manager_is_the_only_typed_snapshot_builder():
    obs = (
        REPO_ROOT
        / "planner/vomp_planner/traj_planner/include/obs_manager/obs_manager.hpp"
    ).read_text(encoding="utf-8")
    global_launch = (
        REPO_ROOT
        / "planner/vomp_planner/traj_planner/launch/plan_global_fsm.launch"
    ).read_text(encoding="utf-8")

    assert obs.count('advertise<semantic_guard::PredictedObstacleArray>') == 1
    assert "predicted_obstacle_timeout_" in obs
    assert "source_age <= predicted_obstacle_timeout_" in obs
    assert "snapshot.header.stamp = active_obstacles.front()->start_time" in obs
    assert "refusing duplicate/invalid obstacle" in obs
    assert global_launch.count("obs_manager/predicted_obstacle_timeout") == 2


def test_minimum_detection_tool_covers_required_topics():
    script = (
        REPO_ROOT / "swarm_test/scripts/check_scout_real_topics.py"
    ).read_text(encoding="utf-8")
    for topic in (
        "/rslidar_points",
        "/velodyne_points",
        "/fastLIO/non_ground_points",
        "/clustering/cluster_array",
        "/obstacle_prediction_node/obstacle_prediction/trajs_predicted",
        "/globalFsm_by_adsm/teacher_obstacle_snapshot",
    ):
        assert topic in script
    assert "--rosbag-path" in script
    assert "Detection (%)" in script
