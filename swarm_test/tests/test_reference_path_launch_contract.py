import importlib.util
import pathlib
import sys
import types
import xml.etree.ElementTree as ET

import pytest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
START_LAUNCH = REPO_ROOT / "swarm_test/launch/start_test.launch"
PLANNER_LAUNCH = REPO_ROOT / "swarm_test/launch/secbf_planner.launch"
DATA_PROCESSOR = REPO_ROOT / "swarm_test/src/data_processor.cpp"
PACKAGE_XML = REPO_ROOT / "swarm_test/package.xml"
CMAKELISTS = REPO_ROOT / "swarm_test/CMakeLists.txt"
SCOUT_XML = REPO_ROOT / "simulation_tools/robot_simulator/launch/scout_simulator.xml"
SCOUT_SOURCE = (
    REPO_ROOT / "simulation_tools/robot_simulator/src/scout_simulator.cpp"
)
SCRIPT_DIR = REPO_ROOT / "swarm_test/scripts"


def include_args(launch_text, include_suffix):
    root = ET.fromstring(launch_text)
    for include in root.findall("include"):
        if include.attrib.get("file", "").endswith(include_suffix):
            return {arg.attrib["name"]: arg.attrib.get("value") for arg in include.findall("arg")}
    raise AssertionError(f"include not found: {include_suffix}")


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


def load_goal_publisher_without_preloaded_script_path():
    spec = importlib.util.spec_from_file_location(
        "reference_path_goal_publisher",
        SCRIPT_DIR / "reference_path_goal_publisher.py",
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
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
    package_xml = PACKAGE_XML.read_text(encoding="utf-8")
    cmakelists = CMAKELISTS.read_text(encoding="utf-8")
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
    global_fsm_args = include_args(planner_launch, "plan_global_fsm.launch")
    assert global_fsm_args["map_size_x_"] == "$(arg map_size_x)"
    assert global_fsm_args["map_size_y_"] == "$(arg map_size_y)"
    assert global_fsm_args["map_size_z_"] == "$(arg map_size_z)"
    assert "p_init_yaw" in scout_xml
    assert 'nh.param("p_init_yaw"' in scout_source
    assert "use_fixed_final_goal" in data_processor_source
    assert "geometry_msgs" in cmakelists
    assert "<build_depend>geometry_msgs</build_depend>" in package_xml
    assert "<build_export_depend>geometry_msgs</build_export_depend>" in package_xml
    assert "<exec_depend>geometry_msgs</exec_depend>" in package_xml


def test_planner_launch_allows_v_max_override():
    root = ET.fromstring(PLANNER_LAUNCH.read_text(encoding="utf-8"))
    v_max_arg = next(arg for arg in root.findall("arg") if arg.attrib.get("name") == "v_max")

    assert "default" in v_max_arg.attrib
    assert v_max_arg.attrib["default"] == "1.5"
    assert "value" not in v_max_arg.attrib


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


def test_goal_publisher_rejects_mismatched_final_goal(tmp_path):
    stub_ros_modules()
    try:
        module = load_goal_publisher()
        config_path = tmp_path / "reference_path.yaml"
        config_path.write_text(
            "\n".join(
                [
                    "waypoints:",
                    "  - [0.0, 0.0]",
                    "  - [1.0, 0.0]",
                    "threshold: 0.3",
                    "start_delay: 0.0",
                    "final_goal: [1.2, 0.0]",
                ]
            )
            + "\n",
            encoding="utf-8",
        )
        with pytest.raises(ValueError, match="final_goal"):
            module.load_reference_path_config(config_path)
    finally:
        clear_stub_modules()


def test_goal_publisher_prefers_source_helper_over_catkin_relay(tmp_path):
    stub_ros_modules()
    fake_devel = tmp_path / "devel_lib"
    fake_devel.mkdir()
    (fake_devel / "reference_path_waypoints.py").write_text(
        "# catkin relay imported as module without exported symbols\n",
        encoding="utf-8",
    )
    sys.path.insert(0, str(fake_devel))
    try:
        module = load_goal_publisher_without_preloaded_script_path()
        assert module.WaypointProgress.__name__ == "WaypointProgress"
    finally:
        if str(fake_devel) in sys.path:
            sys.path.remove(str(fake_devel))
        clear_stub_modules()


def test_goal_publisher_uses_latched_repeated_goal_delivery():
    source = (SCRIPT_DIR / "reference_path_goal_publisher.py").read_text(encoding="utf-8")

    assert "latch=True" in source
    assert "get_num_connections" in source
    assert "Published reference waypoint" in source
