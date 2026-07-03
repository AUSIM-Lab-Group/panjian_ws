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


class Vector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class Twist:
    def __init__(self):
        self.linear = Vector()
        self.angular = Vector()


class CmdVelRelayTests(unittest.TestCase):
    def setUp(self):
        sys.modules["rospy"] = types.SimpleNamespace()
        geometry_msgs = types.ModuleType("geometry_msgs")
        geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
        geometry_msgs_msg.Twist = Twist
        geometry_msgs.msg = geometry_msgs_msg
        sys.modules["geometry_msgs"] = geometry_msgs
        sys.modules["geometry_msgs.msg"] = geometry_msgs_msg
        self.module = load_module("cmd_vel_relay")

    def tearDown(self):
        for name in ["cmd_vel_relay", "geometry_msgs", "geometry_msgs.msg", "rospy"]:
            sys.modules.pop(name, None)

    def test_build_relay_twist_clamps_linear_and_angular_velocity(self):
        incoming = Twist()
        incoming.linear.x = 1.2
        incoming.linear.y = 0.5
        incoming.angular.z = -2.0
        incoming.angular.x = 0.7

        relayed = self.module.build_relay_twist(incoming, max_linear=0.3, max_angular=0.8)

        self.assertEqual(relayed.linear.x, 0.3)
        self.assertEqual(relayed.linear.y, 0.0)
        self.assertEqual(relayed.linear.z, 0.0)
        self.assertEqual(relayed.angular.x, 0.0)
        self.assertEqual(relayed.angular.y, 0.0)
        self.assertEqual(relayed.angular.z, -0.8)

    def test_build_relay_twist_preserves_safe_forward_and_turn_commands(self):
        incoming = Twist()
        incoming.linear.x = -0.2
        incoming.angular.z = 0.4

        relayed = self.module.build_relay_twist(incoming, max_linear=0.3, max_angular=0.8)

        self.assertEqual(relayed.linear.x, -0.2)
        self.assertEqual(relayed.angular.z, 0.4)


if __name__ == "__main__":
    unittest.main()
