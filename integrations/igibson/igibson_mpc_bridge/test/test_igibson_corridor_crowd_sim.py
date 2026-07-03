#!/usr/bin/env python3
import importlib.util
import math
import numpy as np
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


class FakeBackend:
    def __init__(self):
        self.spawned = []
        self.updated = []

    def spawn(self, pedestrian_id, state, mesh_style):
        handle = "ped_{}".format(pedestrian_id)
        self.spawned.append((pedestrian_id, mesh_style, state["behavior_id"]))
        return handle

    def update(self, handle, position, orientation, linear_velocity):
        self.updated.append((handle, position, orientation, linear_velocity))


class FakePybullet:
    def __init__(self):
        self.num_joints = {}
        self.calls = []

    def getNumJoints(self, body_id):
        return self.num_joints.get(body_id, 0)

    def setCollisionFilterPair(self, robot_body_id, object_body_id, robot_link_id, object_link_id, enableCollision):
        self.calls.append((robot_body_id, object_body_id, robot_link_id, object_link_id, enableCollision))


class FakeImage:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id="")
        self.height = 0
        self.width = 0
        self.encoding = ""
        self.is_bigendian = 0
        self.step = 0
        self.data = b""


class IGibsonCorridorCrowdSimTests(unittest.TestCase):
    def setUp(self):
        self.module = load_module("igibson_corridor_crowd_sim")

    def tearDown(self):
        sys.modules.pop("igibson_corridor_crowd_sim", None)

    def test_parse_flat_states_uses_expected_layout(self):
        states = self.module.parse_flat_states([3, 1.0, 2.0, 0.0, -0.5, 0.39, 1, 3])

        self.assertEqual(states[0]["id"], 3)
        self.assertEqual(states[0]["behavior_id"], 3)
        self.assertAlmostEqual(states[0]["vy"], -0.5)

    def test_initial_states_from_config_places_people_at_start_positions(self):
        config = {
            "pedestrians": [
                {"id": 1, "behavior": "normal", "start": [-3.0, -0.6], "radius": 0.39, "class_id": 1},
                {"id": 2, "behavior": "hurried", "start": [3.0, 0.0], "radius": 0.4, "class_id": 1},
            ]
        }

        states = self.module.initial_states_from_config(config)

        self.assertEqual(states[0]["id"], 1)
        self.assertAlmostEqual(states[0]["x"], -3.0)
        self.assertEqual(states[0]["behavior_id"], 1)
        self.assertEqual(states[1]["behavior_id"], 2)

    def test_yaw_from_velocity_follows_motion_and_preserves_previous_when_stopped(self):
        self.assertAlmostEqual(self.module.yaw_from_velocity(1.0, 0.0, previous_yaw=0.7), 0.0)
        self.assertAlmostEqual(self.module.yaw_from_velocity(0.0, 1.0, previous_yaw=0.7), math.pi / 2.0)
        self.assertAlmostEqual(self.module.yaw_from_velocity(0.0, 0.0, previous_yaw=0.7), 0.7)

    def test_controller_spawns_mesh_styles_and_updates_pose_from_states(self):
        backend = FakeBackend()
        controller = self.module.PedestrianObjectController(backend=backend, z_offset=0.05)
        states = [
            {"id": 1, "x": 0.0, "y": -0.5, "vx": 1.0, "vy": 0.0, "radius": 0.39, "class_id": 1, "behavior_id": 1},
            {"id": 2, "x": 1.0, "y": 0.0, "vx": -1.0, "vy": 0.0, "radius": 0.39, "class_id": 1, "behavior_id": 2},
            {"id": 3, "x": 2.0, "y": 0.5, "vx": 0.0, "vy": 0.0, "radius": 0.39, "class_id": 1, "behavior_id": 3},
        ]

        controller.sync(states)

        self.assertEqual(backend.spawned, [(1, 0, 1), (2, 1, 2), (3, 2, 3)])
        self.assertEqual(len(backend.updated), 3)
        self.assertEqual(backend.updated[0][1], [0.0, -0.5, 0.05])
        self.assertAlmostEqual(controller.last_yaw[1], 0.0)
        self.assertAlmostEqual(controller.last_yaw[2], math.pi)
        self.assertAlmostEqual(controller.last_yaw[3], 0.0)

    def test_controller_offsets_odom_frame_states_into_igibson_world_frame(self):
        backend = FakeBackend()
        controller = self.module.PedestrianObjectController(backend=backend, z_offset=0.05, world_offset_xy=[10.0, 20.0])
        states = [
            {"id": 1, "x": 1.0, "y": -0.5, "vx": 1.0, "vy": 0.0, "radius": 0.39, "class_id": 1, "behavior_id": 1}
        ]

        controller.sync(states)

        self.assertEqual(backend.updated[0][1], [11.0, 19.5, 0.05])

    def test_controller_accepts_numpy_world_offset(self):
        backend = FakeBackend()
        controller = self.module.PedestrianObjectController(
            backend=backend,
            z_offset=0.05,
            world_offset_xy=np.array([10.0, 20.0]),
        )
        states = [
            {"id": 1, "x": 1.0, "y": -0.5, "vx": 1.0, "vy": 0.0, "radius": 0.39, "class_id": 1, "behavior_id": 1}
        ]

        controller.sync(states)

        self.assertEqual(backend.updated[0][1], [11.0, 19.5, 0.05])

    def test_stop_go_update_keeps_last_heading_when_velocity_zero(self):
        backend = FakeBackend()
        controller = self.module.PedestrianObjectController(backend=backend, z_offset=0.05)
        moving = [{"id": 3, "x": 0.0, "y": 0.0, "vx": 0.0, "vy": 1.0, "radius": 0.39, "class_id": 1, "behavior_id": 3}]
        stopped = [{"id": 3, "x": 0.0, "y": 1.0, "vx": 0.0, "vy": 0.0, "radius": 0.39, "class_id": 1, "behavior_id": 3}]

        controller.sync(moving)
        controller.sync(stopped)

        self.assertAlmostEqual(controller.last_yaw[3], math.pi / 2.0)
        self.assertEqual(backend.updated[-1][1], [0.0, 1.0, 0.05])

    def test_numpy_to_image_msg_builds_rgb8_message_without_cv_bridge(self):
        rgb = np.arange(12, dtype=np.uint8).reshape((2, 2, 3))

        message = self.module.numpy_to_image_msg(rgb, "rgb8", "camera", "stamp", FakeImage)

        self.assertEqual(message.header.stamp, "stamp")
        self.assertEqual(message.header.frame_id, "camera")
        self.assertEqual(message.height, 2)
        self.assertEqual(message.width, 2)
        self.assertEqual(message.encoding, "rgb8")
        self.assertEqual(message.step, 6)
        self.assertEqual(message.data, rgb.tobytes())

    def test_numpy_to_image_msg_builds_float_depth_message_without_cv_bridge(self):
        depth = np.array([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32)

        message = self.module.numpy_to_image_msg(depth, "32FC1", "camera", "stamp", FakeImage)

        self.assertEqual(message.height, 2)
        self.assertEqual(message.width, 2)
        self.assertEqual(message.encoding, "32FC1")
        self.assertEqual(message.step, 8)
        self.assertEqual(len(message.data), 16)

    def test_fallback_humanoid_specs_include_visible_body_parts(self):
        specs = self.module.build_simple_humanoid_specs(radius=0.39, height=1.7)

        names = {spec["name"] for spec in specs}
        self.assertIn("torso", names)
        self.assertIn("head", names)
        self.assertIn("left_leg", names)
        self.assertIn("right_leg", names)
        self.assertIn("left_arm", names)
        self.assertIn("right_arm", names)
        self.assertGreaterEqual(len(specs), 6)

        head = next(spec for spec in specs if spec["name"] == "head")
        self.assertEqual(head["shape"], "sphere")
        self.assertGreater(head["position"][2], 1.3)

        torso = next(spec for spec in specs if spec["name"] == "torso")
        self.assertEqual(torso["shape"], "capsule")
        self.assertGreater(torso["height"], 0.6)

    def test_prepare_dynamic_object_rendering_disables_optimized_renderer(self):
        config = {"scene": "gibson", "optimized_renderer": True}

        prepared = self.module.prepare_dynamic_object_rendering(config)

        self.assertIs(prepared, config)
        self.assertFalse(prepared["optimized_renderer"])

    def test_disable_collision_between_robot_and_pedestrian_body_links(self):
        fake_p = FakePybullet()
        fake_p.num_joints = {10: 2, 20: 1}

        self.module.disable_body_collisions(fake_p, [10], [20])

        self.assertEqual(
            fake_p.calls,
            [
                (10, 20, -1, -1, 0),
                (10, 20, -1, 0, 0),
                (10, 20, 0, -1, 0),
                (10, 20, 0, 0, 0),
                (10, 20, 1, -1, 0),
                (10, 20, 1, 0, 0),
            ],
        )


if __name__ == "__main__":
    unittest.main()
