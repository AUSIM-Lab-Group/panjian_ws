import copy
import math
import pathlib
import sys
from types import SimpleNamespace

import pytest
import yaml


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / "swarm_test/scripts"
CONFIG = REPO_ROOT / "swarm_test/config/secbf_scenarios.yaml"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import reference_path_waypoints  # noqa: E402
import run_secbf_sim_experiments as runner  # noqa: E402


FIG4 = (
    "drmpc_fig4_scene_1_arc",
    "drmpc_fig4_scene_2_vertical",
    "drmpc_fig4_scene_3_reverse_arc",
    "drmpc_fig4_scene_4_reverse_vertical",
)


def load_scenarios():
    with CONFIG.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)["scenarios"]


def xy_from_mapping(point):
    return (float(point["x"]), float(point["y"]))


def line_center_crossing_time(obs):
    start_x = float(obs["start"]["x"])
    goal_x = float(obs["goal"]["x"])
    if start_x == goal_x or start_x * goal_x >= 0.0:
        raise ValueError("obstacle does not cross x=0")
    fraction = abs(start_x) / abs(goal_x - start_x)
    return float(obs["start_delay"]) + float(obs["travel_time"]) * fraction


def assert_waypoints_close(actual, expected, tol=1e-9):
    assert len(actual) == len(expected)
    for actual_point, expected_point in zip(actual, expected):
        assert float(actual_point[0]) == pytest.approx(float(expected_point[0]), abs=tol)
        assert float(actual_point[1]) == pytest.approx(float(expected_point[1]), abs=tol)


def test_fig4_scenarios_are_registered_scripted_and_non_orca():
    scenarios = load_scenarios()
    assert [runner.SCENARIO_INDEX[name] for name in FIG4] == [216, 217, 218, 219]
    expected_corridor_widths = {
        "drmpc_fig4_scene_1_arc": 1.5,
        "drmpc_fig4_scene_2_vertical": 6.0,
        "drmpc_fig4_scene_3_reverse_arc": 1.5,
        "drmpc_fig4_scene_4_reverse_vertical": 6.0,
    }
    expected_spacing = {
        "drmpc_fig4_scene_1_arc": 0.4,
        "drmpc_fig4_scene_2_vertical": 0.6,
        "drmpc_fig4_scene_3_reverse_arc": 0.4,
        "drmpc_fig4_scene_4_reverse_vertical": 0.6,
    }

    for name in FIG4:
        scene = scenarios[name]
        assert len(scene["obstacles"]) == 6
        if name == "drmpc_fig4_scene_2_vertical":
            assert min(float(item["start_delay"]) for item in scene["obstacles"]) < 1.0
        elif name == "drmpc_fig4_scene_4_reverse_vertical":
            assert {float(item["start_delay"]) for item in scene["obstacles"]} == {3.0, 4.8}
        else:
            assert {float(item["start_delay"]) for item in scene["obstacles"]} == {3.0}
        assert all(item["motion_type"] == "line" for item in scene["obstacles"])
        assert "orca" not in yaml.safe_dump(scene).lower()
        assert float(scene["corridor_width"]) == pytest.approx(expected_corridor_widths[name])
        assert scene["reference_path"]["spacing"] == pytest.approx(expected_spacing[name])


def test_fig4_waypoints_are_reversed_pairs_with_final_goals():
    scenarios = load_scenarios()
    waypoints = {
        name: reference_path_waypoints.generate_waypoints(scenarios[name]["reference_path"])
        for name in FIG4
    }

    assert_waypoints_close(
        waypoints["drmpc_fig4_scene_3_reverse_arc"],
        list(reversed(waypoints["drmpc_fig4_scene_1_arc"])),
    )
    assert_waypoints_close(
        waypoints["drmpc_fig4_scene_4_reverse_vertical"],
        list(reversed(waypoints["drmpc_fig4_scene_2_vertical"])),
    )

    expected_spacing = {
        "drmpc_fig4_scene_1_arc": 0.4,
        "drmpc_fig4_scene_2_vertical": 0.6,
        "drmpc_fig4_scene_3_reverse_arc": 0.4,
        "drmpc_fig4_scene_4_reverse_vertical": 0.6,
    }

    for name in FIG4:
        scene = scenarios[name]
        goal = xy_from_mapping(scene["goal"])
        assert_waypoints_close([waypoints[name][-1]], [goal])
        assert max(
            math.dist(start, end) for start, end in zip(waypoints[name], waypoints[name][1:])
        ) <= expected_spacing[name] + 1e-9


def test_fig4_humans_start_away_from_robot_start_and_goal():
    scenarios = load_scenarios()
    min_separation = 1.3
    for name in FIG4:
        scene = scenarios[name]
        robot_start = xy_from_mapping(scene["start"])
        robot_goal = xy_from_mapping(scene["goal"])
        for obs in scene["obstacles"]:
            human_start = xy_from_mapping(obs["start"])
            assert math.dist(human_start, robot_start) > min_separation
            assert math.dist(human_start, robot_goal) > min_separation


def test_vertical_fig4_scenes_include_true_crossing_obstacles():
    scenarios = load_scenarios()
    for name in ("drmpc_fig4_scene_2_vertical", "drmpc_fig4_scene_4_reverse_vertical"):
        crossings = 0
        for obs in scenarios[name]["obstacles"]:
            start_x = float(obs["start"]["x"])
            goal_x = float(obs["goal"]["x"])
            if start_x * goal_x < 0.0:
                crossings += 1
        assert crossings >= 3


def test_scene2_crossing_obstacles_reach_centerline_while_robot_is_still_in_corridor():
    scenarios = load_scenarios()
    scene = scenarios["drmpc_fig4_scene_2_vertical"]
    crossings = [
        obs for obs in scene["obstacles"] if float(obs["start"]["x"]) * float(obs["goal"]["x"]) < 0.0
    ]
    crossings.sort(key=lambda obs: float(obs["start"]["y"]))
    crossing_times = [line_center_crossing_time(obs) for obs in crossings[:3]]

    assert crossing_times[0] < 6.0
    assert crossing_times[1] < 8.5
    assert crossing_times[2] < 11.0


def test_scene2_crossing_obstacles_start_closer_and_move_more_gently():
    scenarios = load_scenarios()
    scene = scenarios["drmpc_fig4_scene_2_vertical"]
    crossings = [
        obs for obs in scene["obstacles"] if float(obs["start"]["x"]) * float(obs["goal"]["x"]) < 0.0
    ]
    crossings.sort(key=lambda obs: float(obs["start"]["y"]))

    assert len(crossings) >= 3
    assert max(abs(float(obs["start"]["x"])) for obs in crossings[:3]) <= 4.5
    assert min(float(obs["travel_time"]) for obs in crossings[:3]) >= 7.0


def test_scene2_first_crossing_is_outward_shifted_without_timing_changes():
    scenarios = load_scenarios()
    scene = scenarios["drmpc_fig4_scene_2_vertical"]
    crossings = [
        obs for obs in scene["obstacles"] if float(obs["start"]["x"]) * float(obs["goal"]["x"]) < 0.0
    ]
    crossings.sort(key=lambda obs: float(obs["start"]["y"]))
    first = crossings[0]

    assert float(first["start"]["y"]) == pytest.approx(-3.3)
    assert float(first["goal"]["y"]) == pytest.approx(-3.3)
    assert float(first["travel_time"]) == pytest.approx(7.2)
    assert float(first["start_delay"]) == pytest.approx(0.8)
    assert float(first["start"]["x"]) == pytest.approx(-4.5)
    assert float(first["goal"]["x"]) == pytest.approx(3.0)


def test_vertical_fig4_scenes_include_static_path_occupiers():
    scenarios = load_scenarios()
    expected_points = {
        "drmpc_fig4_scene_2_vertical": {(0.0, -1.5), (0.0, 3.0)},
        "drmpc_fig4_scene_4_reverse_vertical": {(0.0, -1.5), (0.0, 3.0)},
    }

    for name in ("drmpc_fig4_scene_2_vertical", "drmpc_fig4_scene_4_reverse_vertical"):
        static_entries = []
        for obs in scenarios[name]["obstacles"]:
            start = (float(obs["start"]["x"]), float(obs["start"]["y"]))
            goal = (float(obs["goal"]["x"]), float(obs["goal"]["y"]))
            if start == goal:
                static_entries.append({"start": start, "goal": goal})

        assert len(static_entries) == 2
        assert all(entry["start"] == entry["goal"] for entry in static_entries)
        assert {entry["start"] for entry in static_entries} == expected_points[name]


def test_vertical_fig4_scaled_geometry_matches_1p5x_probe():
    scenarios = load_scenarios()
    expected = {
        "drmpc_fig4_scene_2_vertical": {
            "map": (15.0, 18.0),
            "start": (0.0, -6.0),
            "goal": (0.0, 6.0),
            "corridor_width": 6.0,
            "spacing": 0.6,
            "planner_goal_min_distance": 3.0,
        },
        "drmpc_fig4_scene_4_reverse_vertical": {
            "map": (15.0, 18.0),
            "start": (0.0, 6.0),
            "goal": (0.0, -6.0),
            "corridor_width": 6.0,
            "spacing": 0.6,
            "planner_goal_min_distance": 3.0,
        },
    }

    for name, cfg in expected.items():
        scene = scenarios[name]
        assert float(scene["map"]["x"]) == pytest.approx(cfg["map"][0])
        assert float(scene["map"]["y"]) == pytest.approx(cfg["map"][1])
        assert xy_from_mapping(scene["start"]) == pytest.approx(cfg["start"])
        assert xy_from_mapping(scene["goal"]) == pytest.approx(cfg["goal"])
        assert float(scene["corridor_width"]) == pytest.approx(cfg["corridor_width"])
        assert float(scene["reference_path"]["spacing"]) == pytest.approx(cfg["spacing"])
        assert float(scene["reference_path"]["planner_goal_min_distance"]) == pytest.approx(
            cfg["planner_goal_min_distance"]
        )


def test_scene4_safe_priority_adjustments_match_spec():
    scenarios = load_scenarios()
    scene2 = scenarios["drmpc_fig4_scene_2_vertical"]
    scene4 = scenarios["drmpc_fig4_scene_4_reverse_vertical"]

    assert (
        float(scene2["obstacles"][3]["start"]["x"]),
        float(scene2["obstacles"][3]["start"]["y"]),
    ) == pytest.approx((0.0, 3.0))

    upper_crossing = scene4["obstacles"][0]
    upper_static = scene4["obstacles"][1]
    diagonal_child = scene4["obstacles"][5]

    assert float(upper_crossing["start"]["y"]) == pytest.approx(2.7)
    assert float(upper_crossing["goal"]["y"]) == pytest.approx(2.7)
    assert float(upper_crossing["travel_time"]) == pytest.approx(20.0)
    assert float(upper_static["start"]["x"]) == pytest.approx(0.0)
    assert float(upper_static["goal"]["x"]) == pytest.approx(0.0)
    assert float(diagonal_child["start_delay"]) == pytest.approx(4.8)


def test_reference_path_config_and_meta_are_written(tmp_path):
    scenarios = load_scenarios()
    scene = scenarios["drmpc_fig4_scene_1_arc"]
    run_dir = tmp_path / "run"
    run_dir.mkdir()

    path_file, waypoints = runner.write_reference_path_config(run_dir, scene)
    config = yaml.safe_load(path_file.read_text(encoding="utf-8"))
    assert config["threshold"] == pytest.approx(0.35)
    assert config["start_delay"] == pytest.approx(3.0)
    assert config["planner_goal_min_distance"] == pytest.approx(2.0)
    assert_waypoints_close(config["waypoints"], waypoints)
    assert_waypoints_close([config["final_goal"]], [waypoints[-1]])

    classes_arg = runner.obstacle_classes(scene["obstacles"])
    meta_path = runner.write_run_meta(
        run_dir,
        "drmpc_fig4_scene_1_arc",
        "SEESM_Ours",
        scene,
        classes_arg,
        len(scene["obstacles"]),
        30,
        waypoints,
    )
    meta = yaml.safe_load(meta_path.read_text(encoding="utf-8"))
    assert meta["start"] == pytest.approx([0.0, 4.0, 0.0])
    assert meta["goal"] == pytest.approx([0.0, -4.0, 0.0])
    assert_waypoints_close(meta["reference_waypoints"], waypoints)
    assert meta["reference_path"] == scene["reference_path"]
    assert meta["corridor_width"] == pytest.approx(1.5)
    assert meta["planner_goal_min_distance"] == pytest.approx(2.0)


def test_scene_specific_planner_v_max_is_written_and_launched(tmp_path):
    scenarios = load_scenarios()
    scene = copy.deepcopy(scenarios["drmpc_fig4_scene_2_vertical"])
    scene["planner_v_max"] = 0.9
    run_dir = tmp_path / "run"
    run_dir.mkdir()

    _, waypoints = runner.write_reference_path_config(run_dir, scene)
    classes_arg = runner.obstacle_classes(scene["obstacles"])
    meta_path = runner.write_run_meta(
        run_dir,
        "drmpc_fig4_scene_2_vertical",
        "SEESM_Ours",
        scene,
        classes_arg,
        len(scene["obstacles"]),
        30,
        waypoints,
    )
    meta = yaml.safe_load(meta_path.read_text(encoding="utf-8"))
    planner_cmd, _ = runner.build_commands(
        "drmpc_fig4_scene_2_vertical",
        "SEESM_Ours",
        run_dir,
        run_dir / "obstacles_param.yaml",
        classes_arg,
        len(scene["obstacles"]),
        scene,
    )

    assert meta["planner_v_max"] == pytest.approx(0.9)
    assert meta["corridor_width"] == pytest.approx(6.0)
    assert meta["planner_goal_min_distance"] == pytest.approx(3.0)
    assert "v_max:=0.9" in planner_cmd


def test_fig4_reference_goal_modes_and_generated_paths(tmp_path):
    scenarios = load_scenarios()
    expected_modes = {
        "drmpc_fig4_scene_1_arc": "waypoints",
        "drmpc_fig4_scene_2_vertical": "final_only",
        "drmpc_fig4_scene_3_reverse_arc": "waypoints",
        "drmpc_fig4_scene_4_reverse_vertical": "final_only",
    }

    for name, expected_mode in expected_modes.items():
        scene = scenarios[name]
        run_dir = tmp_path / name
        run_dir.mkdir()

        assert runner.reference_goal_mode(scene) == expected_mode
        path_file, waypoints = runner.write_reference_path_config(run_dir, scene)
        assert path_file == run_dir / "reference_path.yaml"
        assert path_file.exists()
        assert_waypoints_close([waypoints[-1]], [xy_from_mapping(scene["goal"])])

        classes_arg = runner.obstacle_classes(scene["obstacles"])
        meta_path = runner.write_run_meta(
            run_dir,
            name,
            "SEESM_Ours",
            scene,
            classes_arg,
            len(scene["obstacles"]),
            30,
            waypoints,
        )
        meta = yaml.safe_load(meta_path.read_text(encoding="utf-8"))
        assert_waypoints_close(meta["reference_waypoints"], waypoints)
        assert meta["reference_path"] == scene["reference_path"]


@pytest.mark.parametrize(
    ("scenario_name", "uses_waypoints"),
    [
        ("drmpc_fig4_scene_1_arc", True),
        ("drmpc_fig4_scene_2_vertical", False),
        ("drmpc_fig4_scene_3_reverse_arc", True),
        ("drmpc_fig4_scene_4_reverse_vertical", False),
    ],
)
def test_fig4_build_commands_select_reference_waypoints_by_goal_mode(
    tmp_path, scenario_name, uses_waypoints
):
    scene = load_scenarios()[scenario_name]
    classes_arg = runner.obstacle_classes(scene["obstacles"])
    _, start_cmd = runner.build_commands(
        scenario_name,
        "SEESM_Ours",
        tmp_path,
        tmp_path / "obstacles_param.yaml",
        classes_arg,
        len(scene["obstacles"]),
        scene,
    )

    goal_x, goal_y = xy_from_mapping(scene["goal"])
    assert f"goal_x:={goal_x}" in start_cmd
    assert f"goal_y:={goal_y}" in start_cmd
    assert ("use_reference_path:=true" in start_cmd) is uses_waypoints
    assert any(arg.startswith("reference_path_file:=") for arg in start_cmd) is uses_waypoints
    assert any(arg.startswith("final_goal_x:=") for arg in start_cmd) is uses_waypoints
    assert any(arg.startswith("final_goal_y:=") for arg in start_cmd) is uses_waypoints


def test_reference_goal_mode_rejects_unknown_value():
    scenario = {"reference_path": {"type": "line", "goal_mode": "unexpected"}}

    with pytest.raises(ValueError, match="goal_mode"):
        runner.reference_goal_mode(scenario)


def test_reference_goal_mode_rejects_non_string_value():
    scenario = {"reference_path": {"type": "line", "goal_mode": []}}

    with pytest.raises(ValueError, match="goal_mode"):
        runner.reference_goal_mode(scenario)


def test_reference_goal_mode_rejects_final_only_arc():
    scenario = {"reference_path": {"type": "arc", "goal_mode": "final_only"}}

    with pytest.raises(ValueError, match="line"):
        runner.reference_goal_mode(scenario)


def test_reference_goal_mode_defaults_to_waypoints():
    scenario = {"reference_path": {"type": "line"}}

    assert runner.reference_goal_mode(scenario) == "waypoints"
    assert runner.uses_reference_waypoints(scenario)


def test_build_commands_uses_reference_waypoint_resolver(tmp_path, monkeypatch):
    scene = load_scenarios()["drmpc_fig4_scene_1_arc"]
    classes_arg = runner.obstacle_classes(scene["obstacles"])
    calls = []

    def resolve(scenario):
        calls.append(scenario)
        return False

    monkeypatch.setattr(runner, "uses_reference_waypoints", resolve)
    _, start_cmd = runner.build_commands(
        "drmpc_fig4_scene_1_arc",
        "SEESM_Ours",
        tmp_path,
        tmp_path / "obstacles_param.yaml",
        classes_arg,
        len(scene["obstacles"]),
        scene,
    )

    assert calls == [scene]
    assert "use_reference_path:=true" not in start_cmd


def test_existing_non_reference_scene_keeps_direct_goal_commands(tmp_path):
    scene = load_scenarios()["head_on_context_bl"]
    classes_arg = runner.obstacle_classes(scene["obstacles"])

    assert runner.reference_goal_mode(scene) is None
    _, start_cmd = runner.build_commands(
        "head_on_context_bl",
        "SEESM_Ours",
        tmp_path,
        tmp_path / "obstacles_param.yaml",
        classes_arg,
        len(scene["obstacles"]),
        scene,
    )

    goal_x, goal_y = xy_from_mapping(scene["goal"])
    assert f"goal_x:={goal_x}" in start_cmd
    assert f"goal_y:={goal_y}" in start_cmd
    assert "use_reference_path:=true" not in start_cmd
    assert not any(arg.startswith("reference_path_file:=") for arg in start_cmd)


def test_run_one_rejects_invalid_goal_mode_before_side_effects(tmp_path, monkeypatch):
    scenario_id = "drmpc_fig4_scene_2_vertical"
    baseline_id = "SEESM_Ours"
    timestamp = "invalid_goal_mode"
    scenario = copy.deepcopy(load_scenarios()[scenario_id])
    scenario["reference_path"]["goal_mode"] = "unexpected"
    output_root = tmp_path / "runs"
    args = SimpleNamespace(output_root=output_root, dry_run=False)

    def fail_if_reached(*args, **kwargs):
        pytest.fail("filesystem or process side effect was reached")

    for name in (
        "write_obstacle_params",
        "write_reference_path_config",
        "write_run_meta",
        "start_process",
    ):
        monkeypatch.setattr(runner, name, fail_if_reached)

    with pytest.raises(ValueError, match="goal_mode"):
        runner.run_one(scenario_id, baseline_id, scenario, args, timestamp)

    expected_run_dir = output_root / f"{timestamp}_{scenario_id}_{baseline_id}"
    assert not output_root.exists()
    assert not expected_run_dir.exists()
