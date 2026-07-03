#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import tempfile
import textwrap
import types
import unittest


SCRIPT_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"


def load_module(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPT_DIR / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class Float32MultiArray:
    def __init__(self):
        self.data = []


class ScriptedPedestrianManagerTests(unittest.TestCase):
    def setUp(self):
        sys.modules["rospy"] = types.SimpleNamespace()
        std_msgs = types.ModuleType("std_msgs")
        std_msgs_msg = types.ModuleType("std_msgs.msg")
        std_msgs_msg.Float32MultiArray = Float32MultiArray
        std_msgs.msg = std_msgs_msg
        sys.modules["std_msgs"] = std_msgs
        sys.modules["std_msgs.msg"] = std_msgs_msg
        self.module = load_module("scripted_pedestrian_manager")

    def tearDown(self):
        for name in ["scripted_pedestrian_manager", "std_msgs", "std_msgs.msg", "rospy"]:
            sys.modules.pop(name, None)

    def test_load_config_and_build_pedestrians_preserves_behaviors(self):
        config_text = textwrap.dedent(
            """
            corridor:
              frame_id: odom
              length: 8.0
              width: 2.0
            pedestrians:
              - id: 1
                behavior: normal
                start: [-3.0, -0.6]
                goal: [3.0, -0.6]
                speed: 1.0
                radius: 0.39
              - id: 2
                behavior: hurried
                start: [3.0, 0.0]
                goal: [-3.0, 0.0]
                speed: 1.3
                radius: 0.39
              - id: 3
                behavior: stop_go
                start: [-2.0, 0.6]
                goal: [2.0, 0.6]
                speed: 0.8
                radius: 0.39
                stop_after: 2.0
                stop_duration: 1.0
            """
        )
        with tempfile.NamedTemporaryFile("w", suffix=".yaml") as handle:
            handle.write(config_text)
            handle.flush()

            config = self.module.load_config(handle.name)
            pedestrians = self.module.build_pedestrians(config)

        self.assertEqual([p["behavior"] for p in pedestrians], ["normal", "hurried", "stop_go"])
        self.assertEqual([p["behavior_id"] for p in pedestrians], [1, 2, 3])
        self.assertEqual(pedestrians[0]["class_id"], 1)

    def test_compute_pedestrian_state_moves_between_start_and_goal(self):
        pedestrian = {
            "id": 1,
            "behavior": "normal",
            "behavior_id": 1,
            "class_id": 1,
            "start": [-3.0, 0.0],
            "goal": [3.0, 0.0],
            "speed": 1.0,
            "radius": 0.39,
        }

        state = self.module.compute_pedestrian_state(pedestrian, elapsed=2.0)

        self.assertEqual(state[0], 1.0)
        self.assertAlmostEqual(state[1], -1.0)
        self.assertAlmostEqual(state[2], 0.0)
        self.assertAlmostEqual(state[3], 1.0)
        self.assertAlmostEqual(state[4], 0.0)
        self.assertAlmostEqual(state[5], 0.39)
        self.assertEqual(state[6], 1.0)
        self.assertEqual(state[7], 1.0)

    def test_stop_go_pedestrian_stops_without_advancing_motion_time(self):
        pedestrian = {
            "id": 3,
            "behavior": "stop_go",
            "behavior_id": 3,
            "class_id": 1,
            "start": [0.0, 0.0],
            "goal": [4.0, 0.0],
            "speed": 1.0,
            "radius": 0.39,
            "stop_after": 2.0,
            "stop_duration": 2.0,
        }

        stopped = self.module.compute_pedestrian_state(pedestrian, elapsed=3.0)
        resumed = self.module.compute_pedestrian_state(pedestrian, elapsed=5.0)

        self.assertAlmostEqual(stopped[1], 2.0)
        self.assertAlmostEqual(stopped[3], 0.0)
        self.assertAlmostEqual(resumed[1], 3.0)
        self.assertAlmostEqual(resumed[3], 1.0)

    def test_pack_states_returns_float32_multi_array_layout(self):
        message = self.module.pack_states(
            [
                [1, 0.0, 0.0, 1.0, 0.0, 0.39, 1, 1],
                [2, 1.0, 0.0, -1.0, 0.0, 0.39, 1, 2],
            ]
        )

        self.assertEqual(len(message.data), 16)
        self.assertEqual(message.data[:8], [1.0, 0.0, 0.0, 1.0, 0.0, 0.39, 1.0, 1.0])


if __name__ == "__main__":
    unittest.main()
