import importlib.util
import pathlib
import sys
import types

import pytest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
START_LAUNCH = REPO_ROOT / "swarm_test/launch/start_test.launch"
PLANNER_LAUNCH = REPO_ROOT / "swarm_test/launch/secbf_planner.launch"
DATA_PROCESSOR = REPO_ROOT / "swarm_test/src/data_processor.cpp"
SCOUT_XML = REPO_ROOT / "simulation_tools/robot_simulator/launch/scout_simulator.xml"
SCOUT_SOURCE = (
    REPO_ROOT / "simulation_tools/robot_simulator/src/scout_simulator.cpp"
)
SCRIPT_DIR = REPO_ROOT / "swarm_test/scripts"


def load_goal_publisher():
    spec = importlib.util.spec_from_file_location(
        "reference_path_goal_publisher",
        SCRIPT_DIR / "reference_path_goal_publisher.py",
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    sys.path.insert(0, str(SCRIPT_DIR))
    spec.loader.exec_module(module)
    return module


def stub_ros_modules():
    sys.modules["rospy"] = types.SimpleNamespace(
        ROSException=RuntimeError,
        ROSInterruptException=RuntimeError,
    )
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.PoseStamped = object
    geometry_msgs.msg = geometry_msgs_msg
    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs_msg = types.ModuleType("nav_msgs.msg")
    nav_msgs_msg.Odometry = object
    nav_msgs.msg = nav_msgs_msg
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs_msg
    sys.modules["nav_msgs"] = nav_msgs
    sys.modules["nav_msgs.msg"] = nav_msgs_msg


def clear_stub_modules():
    sys.modules.pop("reference_path_goal_publisher", None)
    sys.modules.pop("reference_path_waypoints", None)
    sys.modules.pop("rospy", None)
    sys.modules.pop("geometry_msgs", None)
    sys.modules.pop("geometry_msgs.msg", None)
    sys.modules.pop("nav_msgs", None)
    sys.modules.pop("nav_msgs.msg", None)
    if str(SCRIPT_DIR) in sys.path:
        sys.path.remove(str(SCRIPT_DIR))


def test_launch_and_source_contracts_exist():
    start_launch = START_LAUNCH.read_text(encoding="utf-8")
    planner_launch = PLANNER_LAUNCH.read_text(encoding="utf-8")
    data_processor_source = DATA_PROCESSOR.read_text(encoding="utf-8")
    scout_xml = SCOUT_XML.read_text(encoding="utf-8")
    scout_source = SCOUT_SOURCE.read_text(encoding="utf-8")

    assert "reference_path_file" in start_launch
    assert "use_reference_path" in start_launch
    assert "reference_path_goal_publisher.py" in start_launch
    assert "final_goal_x" in start_launch and "final_goal_y" in start_launch
    assert (
        "init_x" in planner_launch
        and "init_y" in planner_launch
        and "init_yaw" in planner_launch
    )
    assert "p_init_yaw" in scout_xml
    assert 'nh.param("p_init_yaw"' in scout_source
    assert "use_fixed_final_goal" in data_processor_source


def test_goal_publisher_rejects_yaml_without_waypoints(tmp_path):
    stub_ros_modules()
    try:
        module = load_goal_publisher()
        config_path = tmp_path / "reference_path.yaml"
        config_path.write_text(
            "threshold: 0.3\nstart_delay: 0.0\nfinal_goal: [1.0, 2.0]\n",
            encoding="utf-8",
        )
        with pytest.raises(ValueError, match="waypoints"):
            module.load_reference_path_config(config_path)
    finally:
        clear_stub_modules()
