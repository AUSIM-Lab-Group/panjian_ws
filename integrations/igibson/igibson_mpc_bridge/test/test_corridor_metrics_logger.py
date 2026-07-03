#!/usr/bin/env python3
import csv
import importlib.util
import pathlib
import sys
import tempfile
import types
import unittest


SCRIPT_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"


def load_module(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPT_DIR / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class CorridorMetricsLoggerTests(unittest.TestCase):
    def setUp(self):
        sys.modules["rospy"] = types.SimpleNamespace()
        nav_msgs = types.ModuleType("nav_msgs")
        nav_msgs_msg = types.ModuleType("nav_msgs.msg")
        nav_msgs_msg.Odometry = object
        nav_msgs.msg = nav_msgs_msg
        sys.modules["nav_msgs"] = nav_msgs
        sys.modules["nav_msgs.msg"] = nav_msgs_msg
        std_msgs = types.ModuleType("std_msgs")
        std_msgs_msg = types.ModuleType("std_msgs.msg")
        std_msgs_msg.Float32MultiArray = object
        std_msgs.msg = std_msgs_msg
        sys.modules["std_msgs"] = std_msgs
        sys.modules["std_msgs.msg"] = std_msgs_msg
        self.module = load_module("corridor_metrics_logger")

    def tearDown(self):
        for name in [
            "corridor_metrics_logger",
            "pedestrian_state_publisher",
            "nav_msgs",
            "nav_msgs.msg",
            "std_msgs",
            "std_msgs.msg",
            "rospy",
        ]:
            sys.modules.pop(name, None)

    def test_compute_metrics_detects_personal_space_and_collision_flags(self):
        states = [
            {"id": 1, "x": 1.0, "y": 0.0, "vx": 0.0, "vy": 0.0, "radius": 0.39, "class_id": 1, "behavior_id": 1},
            {"id": 2, "x": 4.0, "y": 0.0, "vx": 0.0, "vy": 0.0, "radius": 0.39, "class_id": 1, "behavior_id": 2},
        ]

        metrics = self.module.compute_metrics(
            robot_x=0.0,
            robot_y=0.0,
            robot_radius=0.35,
            pedestrians=states,
            personal_space_threshold=1.2,
        )

        self.assertAlmostEqual(metrics["min_distance_to_pedestrian"], 1.0)
        self.assertTrue(metrics["personal_space_violation"])
        self.assertFalse(metrics["collision_flag"])

        collision = self.module.compute_metrics(0.0, 0.0, 0.35, [states[0]], 0.8)
        self.assertFalse(collision["personal_space_violation"])
        self.assertFalse(collision["collision_flag"])

        near_collision = dict(states[0])
        near_collision["x"] = 0.6
        collision = self.module.compute_metrics(0.0, 0.0, 0.35, [near_collision], 1.2)
        self.assertTrue(collision["collision_flag"])

    def test_write_csv_row_includes_robot_pose_pedestrians_and_flags(self):
        row = self.module.build_log_row(
            stamp=12.5,
            robot_pose={"x": 0.1, "y": -0.2, "yaw": 0.3},
            pedestrians=[
                {"id": 1, "x": 1.0, "y": 0.0, "vx": 0.2, "vy": 0.0, "radius": 0.39, "class_id": 1, "behavior_id": 1}
            ],
            metrics={
                "min_distance_to_pedestrian": 0.9,
                "personal_space_violation": True,
                "collision_flag": False,
            },
        )

        with tempfile.NamedTemporaryFile("w+", suffix=".csv") as handle:
            writer = csv.DictWriter(handle, fieldnames=self.module.CSV_FIELDS)
            writer.writeheader()
            writer.writerow(row)
            handle.seek(0)
            rows = list(csv.DictReader(handle))

        self.assertEqual(rows[0]["time"], "12.500")
        self.assertEqual(rows[0]["robot_x"], "0.100")
        self.assertIn('"id": 1', rows[0]["pedestrian_states"])
        self.assertEqual(rows[0]["personal_space_violation"], "1")
        self.assertEqual(rows[0]["collision_flag"], "0")


if __name__ == "__main__":
    unittest.main()
