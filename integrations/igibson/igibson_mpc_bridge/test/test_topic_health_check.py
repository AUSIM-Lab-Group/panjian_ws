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


class TopicHealthCheckTests(unittest.TestCase):
    def setUp(self):
        sys.modules["rospy"] = types.SimpleNamespace()
        self.module = load_module("topic_health_check")

    def tearDown(self):
        for name in ["topic_health_check", "rospy"]:
            sys.modules.pop(name, None)

    def test_find_missing_topics_reports_topics_not_in_master_list(self):
        required = ["/Odometry", "/camera/color/image_raw", "/mobile_base/commands/velocity"]
        available = [("/Odometry", ["nav_msgs/Odometry"]), ("/mobile_base/commands/velocity", ["geometry_msgs/Twist"])]

        missing = self.module.find_missing_topics(required, available)

        self.assertEqual(missing, ["/camera/color/image_raw"])

    def test_exit_code_is_zero_only_when_no_topics_are_missing(self):
        self.assertEqual(self.module.exit_code_for_missing([]), 0)
        self.assertEqual(self.module.exit_code_for_missing(["/Odometry"]), 2)


if __name__ == "__main__":
    unittest.main()
