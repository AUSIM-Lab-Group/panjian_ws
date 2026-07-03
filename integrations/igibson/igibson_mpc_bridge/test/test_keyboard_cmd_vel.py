#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import unittest


SCRIPT_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"


def load_module(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPT_DIR / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class KeyboardCmdVelTests(unittest.TestCase):
    def setUp(self):
        self.module = load_module("keyboard_cmd_vel")

    def tearDown(self):
        sys.modules.pop("keyboard_cmd_vel", None)

    def test_wasd_updates_velocity_and_clamps_to_limits(self):
        linear, angular = 0.0, 0.0

        linear, angular, should_quit = self.module.apply_key(
            "w", linear, angular, linear_step=0.1, angular_step=0.2, max_linear=0.3, max_angular=0.5
        )
        linear, angular, should_quit = self.module.apply_key(
            "w", linear, angular, linear_step=0.1, angular_step=0.2, max_linear=0.3, max_angular=0.5
        )
        linear, angular, should_quit = self.module.apply_key(
            "w", linear, angular, linear_step=0.1, angular_step=0.2, max_linear=0.3, max_angular=0.5
        )
        linear, angular, should_quit = self.module.apply_key(
            "w", linear, angular, linear_step=0.1, angular_step=0.2, max_linear=0.3, max_angular=0.5
        )
        linear, angular, should_quit = self.module.apply_key(
            "a", linear, angular, linear_step=0.1, angular_step=0.2, max_linear=0.3, max_angular=0.5
        )

        self.assertAlmostEqual(linear, 0.3)
        self.assertAlmostEqual(angular, 0.2)
        self.assertFalse(should_quit)

    def test_stop_and_quit_keys_zero_velocity(self):
        linear, angular, should_quit = self.module.apply_key(
            " ", 0.2, -0.3, linear_step=0.1, angular_step=0.2, max_linear=0.3, max_angular=0.5
        )
        self.assertEqual((linear, angular, should_quit), (0.0, 0.0, False))

        linear, angular, should_quit = self.module.apply_key(
            "q", 0.2, -0.3, linear_step=0.1, angular_step=0.2, max_linear=0.3, max_angular=0.5
        )
        self.assertEqual((linear, angular, should_quit), (0.0, 0.0, True))


if __name__ == "__main__":
    unittest.main()
