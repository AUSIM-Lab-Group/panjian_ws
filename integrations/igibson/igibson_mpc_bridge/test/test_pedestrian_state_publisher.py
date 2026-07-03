#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import types
import unittest


SCRIPT_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"


def load_module(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPT_DIR / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class Header:
    def __init__(self):
        self.frame_id = ""
        self.stamp = None


class Pose:
    def __init__(self):
        self.position = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.orientation = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)


class Scale:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Color:
    def __init__(self):
        self.r = 0.0
        self.g = 0.0
        self.b = 0.0
        self.a = 0.0


class Marker:
    ADD = 0
    CYLINDER = 3
    TEXT_VIEW_FACING = 9

    def __init__(self):
        self.header = Header()
        self.ns = ""
        self.id = 0
        self.type = 0
        self.action = 0
        self.pose = Pose()
        self.scale = Scale()
        self.color = Color()
        self.text = ""


class MarkerArray:
    def __init__(self):
        self.markers = []


class Float32MultiArray:
    def __init__(self):
        self.data = []


class PedestrianStatePublisherTests(unittest.TestCase):
    def setUp(self):
        sys.modules["rospy"] = types.SimpleNamespace()
        std_msgs = types.ModuleType("std_msgs")
        std_msgs_msg = types.ModuleType("std_msgs.msg")
        std_msgs_msg.Float32MultiArray = Float32MultiArray
        std_msgs.msg = std_msgs_msg
        sys.modules["std_msgs"] = std_msgs
        sys.modules["std_msgs.msg"] = std_msgs_msg

        visualization_msgs = types.ModuleType("visualization_msgs")
        visualization_msgs_msg = types.ModuleType("visualization_msgs.msg")
        visualization_msgs_msg.Marker = Marker
        visualization_msgs_msg.MarkerArray = MarkerArray
        visualization_msgs.msg = visualization_msgs_msg
        sys.modules["visualization_msgs"] = visualization_msgs
        sys.modules["visualization_msgs.msg"] = visualization_msgs_msg
        self.module = load_module("pedestrian_state_publisher")

    def tearDown(self):
        for name in [
            "pedestrian_state_publisher",
            "std_msgs",
            "std_msgs.msg",
            "visualization_msgs",
            "visualization_msgs.msg",
            "rospy",
        ]:
            sys.modules.pop(name, None)

    def test_parse_flat_states_converts_rows_to_named_fields(self):
        states = self.module.parse_flat_states([1, 0.5, -0.2, 1.0, 0.0, 0.39, 1, 2])

        self.assertEqual(states[0]["id"], 1)
        self.assertEqual(states[0]["behavior_id"], 2)
        self.assertAlmostEqual(states[0]["radius"], 0.39)

    def test_parse_flat_states_rejects_partial_rows(self):
        with self.assertRaises(ValueError):
            self.module.parse_flat_states([1, 0.0, 0.0])

    def test_build_marker_array_creates_cylinder_and_label_for_each_pedestrian(self):
        states = self.module.parse_flat_states([2, 1.0, 0.5, -0.2, 0.0, 0.4, 1, 3])

        marker_array = self.module.build_marker_array(states, frame_id="odom", stamp="now")

        self.assertEqual(len(marker_array.markers), 2)
        body = marker_array.markers[0]
        label = marker_array.markers[1]
        self.assertEqual(body.header.frame_id, "odom")
        self.assertEqual(body.type, Marker.CYLINDER)
        self.assertAlmostEqual(body.pose.position.x, 1.0)
        self.assertAlmostEqual(body.pose.position.y, 0.5)
        self.assertAlmostEqual(body.scale.x, 0.8)
        self.assertEqual(label.type, Marker.TEXT_VIEW_FACING)
        self.assertIn("stop_go", label.text)


if __name__ == "__main__":
    unittest.main()
