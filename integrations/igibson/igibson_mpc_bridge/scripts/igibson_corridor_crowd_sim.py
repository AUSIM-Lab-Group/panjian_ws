#!/usr/bin/env python3
import logging
import math
import os
import pathlib
import time

import yaml


STATE_STRIDE = 8
BEHAVIOR_IDS = {
    "normal": 1,
    "hurried": 2,
    "stop_go": 3,
}
MESH_STYLE_BY_BEHAVIOR_ID = {
    1: 0,
    2: 1,
    3: 2,
}
DEFAULT_CONFIG = pathlib.Path(__file__).resolve().parents[1] / "config" / "corridor_crowd.yaml"


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def prepare_dynamic_object_rendering(igibson_config):
    igibson_config["optimized_renderer"] = False
    return igibson_config


def parse_flat_states(flat_values):
    values = [float(value) for value in flat_values]
    if len(values) % STATE_STRIDE != 0:
        raise ValueError("Pedestrian state array length must be a multiple of {}".format(STATE_STRIDE))

    states = []
    for offset in range(0, len(values), STATE_STRIDE):
        row = values[offset : offset + STATE_STRIDE]
        states.append(
            {
                "id": int(row[0]),
                "x": row[1],
                "y": row[2],
                "vx": row[3],
                "vy": row[4],
                "radius": row[5],
                "class_id": int(row[6]),
                "behavior_id": int(row[7]),
            }
        )
    return states


def initial_states_from_config(config):
    states = []
    for raw in config.get("pedestrians", []):
        behavior = str(raw.get("behavior", "normal"))
        start = raw.get("start", [0.0, 0.0])
        states.append(
            {
                "id": int(raw["id"]),
                "x": float(start[0]),
                "y": float(start[1]),
                "vx": 0.0,
                "vy": 0.0,
                "radius": float(raw.get("radius", 0.39)),
                "class_id": int(raw.get("class_id", 1)),
                "behavior_id": BEHAVIOR_IDS.get(behavior, 1),
            }
        )
    return states


def yaw_from_velocity(vx, vy, previous_yaw=0.0, min_speed=1e-3):
    speed = math.hypot(float(vx), float(vy))
    if speed < min_speed:
        return float(previous_yaw)
    return math.atan2(float(vy), float(vx))


def quaternion_from_yaw(yaw):
    half = float(yaw) * 0.5
    return [0.0, 0.0, math.sin(half), math.cos(half)]


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def numpy_to_image_msg(array, encoding, frame_id, stamp, image_cls):
    contiguous = array.copy(order="C") if not array.flags["C_CONTIGUOUS"] else array
    message = image_cls()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = int(contiguous.shape[0])
    message.width = int(contiguous.shape[1])
    message.encoding = encoding
    message.is_bigendian = 0
    if contiguous.ndim == 2:
        channels = 1
    else:
        channels = int(contiguous.shape[2])
    message.step = int(message.width * channels * contiguous.dtype.itemsize)
    message.data = contiguous.tobytes()
    return message


def build_simple_humanoid_specs(radius=0.39, height=1.7, color=None):
    body_color = list(color or [0.2, 0.45, 1.0, 1.0])
    skin_color = [0.93, 0.72, 0.55, 1.0]
    dark_color = [0.08, 0.10, 0.12, 1.0]
    face_color = [1.0, 0.15, 0.08, 1.0]

    radius = float(radius)
    height = float(height)
    head_radius = max(0.12, radius * 0.36)
    torso_radius = max(0.13, radius * 0.48)
    torso_height = max(0.58, height * 0.42)
    leg_radius = max(0.055, radius * 0.20)
    arm_radius = max(0.045, radius * 0.16)
    leg_height = max(0.55, height * 0.44)
    arm_height = max(0.45, height * 0.36)

    torso_z = leg_height + torso_height * 0.48
    head_z = min(height - head_radius * 0.75, torso_z + torso_height * 0.62 + head_radius)
    arm_z = torso_z
    leg_z = leg_height * 0.5

    return [
        {
            "name": "torso",
            "shape": "capsule",
            "radius": torso_radius,
            "height": torso_height,
            "position": [0.0, 0.0, torso_z],
            "color": body_color,
        },
        {
            "name": "head",
            "shape": "sphere",
            "radius": head_radius,
            "position": [0.0, 0.0, head_z],
            "color": skin_color,
        },
        {
            "name": "left_leg",
            "shape": "capsule",
            "radius": leg_radius,
            "height": leg_height,
            "position": [0.0, radius * 0.20, leg_z],
            "color": dark_color,
        },
        {
            "name": "right_leg",
            "shape": "capsule",
            "radius": leg_radius,
            "height": leg_height,
            "position": [0.0, -radius * 0.20, leg_z],
            "color": dark_color,
        },
        {
            "name": "left_arm",
            "shape": "capsule",
            "radius": arm_radius,
            "height": arm_height,
            "position": [0.0, radius * 0.62, arm_z],
            "color": skin_color,
        },
        {
            "name": "right_arm",
            "shape": "capsule",
            "radius": arm_radius,
            "height": arm_height,
            "position": [0.0, -radius * 0.62, arm_z],
            "color": skin_color,
        },
        {
            "name": "face",
            "shape": "sphere",
            "radius": max(0.035, radius * 0.10),
            "position": [torso_radius + max(0.035, radius * 0.08), 0.0, head_z],
            "color": face_color,
        },
    ]


def body_ids_from(entity):
    if entity is None or not hasattr(entity, "get_body_ids"):
        return []
    body_ids = entity.get_body_ids()
    if body_ids is None:
        return []
    return list(body_ids)


def _link_indices(pybullet_module, body_id):
    return [-1] + list(range(pybullet_module.getNumJoints(body_id)))


def disable_body_collisions(pybullet_module, robot_body_ids, object_body_ids):
    for robot_body_id in robot_body_ids:
        for object_body_id in object_body_ids:
            for robot_link_id in _link_indices(pybullet_module, robot_body_id):
                for object_link_id in _link_indices(pybullet_module, object_body_id):
                    pybullet_module.setCollisionFilterPair(
                        robot_body_id,
                        object_body_id,
                        robot_link_id,
                        object_link_id,
                        enableCollision=0,
                    )


class PedestrianObjectController:
    def __init__(
        self,
        backend,
        z_offset=0.05,
        mesh_style_by_behavior_id=None,
        mesh_yaw_offset=0.0,
        world_offset_xy=None,
    ):
        self.backend = backend
        self.z_offset = float(z_offset)
        self.mesh_style_by_behavior_id = dict(mesh_style_by_behavior_id or MESH_STYLE_BY_BEHAVIOR_ID)
        self.mesh_yaw_offset = float(mesh_yaw_offset)
        if world_offset_xy is None:
            world_offset_xy = [0.0, 0.0]
        self.world_offset_xy = [float(world_offset_xy[0]), float(world_offset_xy[1])]
        self.handles = {}
        self.last_yaw = {}

    def sync(self, states):
        for state in states:
            pedestrian_id = int(state["id"])
            previous_yaw = self.last_yaw.get(pedestrian_id, 0.0)
            yaw = yaw_from_velocity(state["vx"], state["vy"], previous_yaw=previous_yaw)
            self.last_yaw[pedestrian_id] = yaw

            if pedestrian_id not in self.handles:
                mesh_style = self.mesh_style_by_behavior_id.get(int(state["behavior_id"]), 0)
                self.handles[pedestrian_id] = self.backend.spawn(pedestrian_id, state, mesh_style)

            position = [
                self.world_offset_xy[0] + float(state["x"]),
                self.world_offset_xy[1] + float(state["y"]),
                self.z_offset,
            ]
            orientation = quaternion_from_yaw(normalize_angle(yaw + self.mesh_yaw_offset))
            velocity = [float(state["vx"]), float(state["vy"]), 0.0]
            self.backend.update(self.handles[pedestrian_id], position, orientation, velocity)


def _create_simple_human_class():
    import pybullet as p
    from igibson.objects.object_base import BaseObject

    class SimpleHumanObject(BaseObject):
        def __init__(self, pos, radius=0.3, height=1.7, color=None, **kwargs):
            super(SimpleHumanObject, self).__init__(**kwargs)
            self.pos = pos
            self.radius = float(radius)
            self.height = float(height)
            self.color = color or [0.2, 0.45, 1.0, 1.0]
            self.part_specs = build_simple_humanoid_specs(self.radius, self.height, self.color)

        def _world_position(self, base_position, relative_position, orientation=None):
            yaw = 0.0
            if orientation:
                yaw = math.atan2(
                    2.0 * (orientation[3] * orientation[2] + orientation[0] * orientation[1]),
                    1.0 - 2.0 * (orientation[1] * orientation[1] + orientation[2] * orientation[2]),
                )
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            rel_x, rel_y, rel_z = relative_position
            return [
                base_position[0] + cos_yaw * rel_x - sin_yaw * rel_y,
                base_position[1] + sin_yaw * rel_x + cos_yaw * rel_y,
                base_position[2] + rel_z,
            ]

        def _shape_ids(self, spec):
            shape = spec["shape"]
            if shape == "sphere":
                collision_id = p.createCollisionShape(p.GEOM_SPHERE, radius=spec["radius"])
                visual_id = p.createVisualShape(
                    p.GEOM_SPHERE,
                    radius=spec["radius"],
                    rgbaColor=spec["color"],
                )
                return collision_id, visual_id
            if shape == "capsule":
                collision_id = p.createCollisionShape(
                    p.GEOM_CAPSULE,
                    radius=spec["radius"],
                    height=spec["height"],
                )
                visual_id = p.createVisualShape(
                    p.GEOM_CAPSULE,
                    radius=spec["radius"],
                    length=spec["height"],
                    rgbaColor=spec["color"],
                )
                return collision_id, visual_id
            raise ValueError("Unsupported simple humanoid shape: {}".format(shape))

        def _load(self, simulator):
            body_ids = []
            for spec in self.part_specs:
                collision_id, visual_id = self._shape_ids(spec)
                body_id = p.createMultiBody(
                    baseMass=0,
                    baseCollisionShapeIndex=collision_id,
                    baseVisualShapeIndex=visual_id,
                    basePosition=self._world_position(self.pos, spec["position"], [0, 0, 0, 1]),
                    baseOrientation=[0, 0, 0, 1],
                )
                simulator.load_object_in_renderer(self, body_id, self.class_id, **self._rendering_params)
                body_ids.append(body_id)
            return body_ids

        def reset_position_orientation(self, position, orientation):
            for body_id, spec in zip(self.get_body_ids(), self.part_specs):
                p.resetBasePositionAndOrientation(
                    body_id,
                    self._world_position(position, spec["position"], orientation),
                    orientation,
                )

        def set_position_orientation(self, position, orientation):
            self.reset_position_orientation(position, orientation)

    return SimpleHumanObject


class IGibsonPedestrianBackend:
    def __init__(self, simulator, fallback_height=1.7, fallback_colors=None, robots=None, disable_robot_collision=True):
        self.simulator = simulator
        self.fallback_height = float(fallback_height)
        self.robots = list(robots or [])
        self.disable_robot_collision = bool(disable_robot_collision)
        self.fallback_colors = fallback_colors or {
            1: [0.1, 0.8, 0.35, 1.0],
            2: [1.0, 0.35, 0.05, 1.0],
            3: [0.15, 0.45, 1.0, 1.0],
        }

    def _disable_robot_collisions(self, obj, pedestrian_id):
        if not self.disable_robot_collision:
            return
        try:
            import pybullet as p

            robot_body_ids = []
            for robot in self.robots:
                robot_body_ids.extend(body_ids_from(robot))
            object_body_ids = body_ids_from(obj)
            disable_body_collisions(p, robot_body_ids, object_body_ids)
            logging.info(
                "Disabled robot-pedestrian physical collision for pedestrian %s (%s robot bodies, %s pedestrian bodies)",
                pedestrian_id,
                len(robot_body_ids),
                len(object_body_ids),
            )
        except Exception as exc:
            logging.warning("Could not disable robot-pedestrian collision for pedestrian %s: %s", pedestrian_id, exc)

    def spawn(self, pedestrian_id, state, mesh_style):
        pos = [float(state["x"]), float(state["y"]), 0.05]
        name = "scripted_pedestrian_{}".format(pedestrian_id)
        try:
            from igibson.objects.pedestrian import Pedestrian

            obj = Pedestrian(
                style=mesh_style,
                pos=pos,
                name=name,
                category="agent",
                rendering_params={"use_pbr": False, "use_pbr_mapping": False},
            )
            self.simulator.import_object(obj)
            self._disable_robot_collisions(obj, pedestrian_id)
            logging.info("Loaded iGibson Pedestrian mesh %s for pedestrian %s", mesh_style, pedestrian_id)
            return obj
        except Exception as exc:
            logging.warning(
                "Failed to load Pedestrian mesh for pedestrian %s (%s); using simplified humanoid fallback",
                pedestrian_id,
                exc,
            )
            SimpleHumanObject = _create_simple_human_class()
            obj = SimpleHumanObject(
                pos=pos,
                radius=float(state.get("radius", 0.39)),
                height=self.fallback_height,
                color=self.fallback_colors.get(int(state["behavior_id"])),
                name=name,
                category="agent",
                rendering_params={"use_pbr": False, "use_pbr_mapping": False},
            )
            self.simulator.import_object(obj)
            self._disable_robot_collisions(obj, pedestrian_id)
            return obj

    def update(self, handle, position, orientation, linear_velocity):
        import pybullet as p

        target_position = list(position)
        if hasattr(handle, "_igibson_mpc_center_z_offset"):
            target_position[2] += handle._igibson_mpc_center_z_offset
        if hasattr(handle, "reset_position_orientation"):
            handle.reset_position_orientation(target_position, orientation)
        if hasattr(handle, "set_position_orientation"):
            handle.set_position_orientation(target_position, orientation)
        if hasattr(handle, "set_velocities"):
            handle.set_velocities([(linear_velocity, [0.0, 0.0, 0.0])])
        for body_id in handle.get_body_ids():
            p.changeDynamics(body_id, -1, activationState=p.ACTIVATION_STATE_WAKE_UP)


def _resolve_igibson_ros_config(config_path=None):
    if config_path and os.path.exists(config_path):
        return config_path
    try:
        import rospkg

        path = rospkg.RosPack().get_path("igibson-ros")
        candidate = os.path.join(path, "turtlebot_rgbd.yaml")
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass
    for candidate in [
        "/opt/catkin_ws/src/igibson-ros/turtlebot_rgbd.yaml",
        "/opt/iGibson/igibson/examples/ros/igibson-ros/turtlebot_rgbd.yaml",
    ]:
        if os.path.exists(candidate):
            return candidate
    raise FileNotFoundError("Could not locate turtlebot_rgbd.yaml for igibson-ros")


class CorridorCrowdSimNode:
    def __init__(self):
        import numpy as np
        import rospy
        import tf
        from geometry_msgs.msg import PoseStamped, Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs import point_cloud2 as pc2
        from sensor_msgs.msg import CameraInfo
        from sensor_msgs.msg import Image as ImageMsg
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import Float32MultiArray, Header

        from igibson.envs.igibson_env import iGibsonEnv

        self.np = np
        self.rospy = rospy
        self.tf = tf
        self.pc2 = pc2
        self.CameraInfo = CameraInfo
        self.Header = Header
        self.ImageMsg = ImageMsg
        self.Odometry = Odometry

        config_path = rospy.get_param("~config", str(DEFAULT_CONFIG))
        self.config = load_yaml(config_path)
        igibson_config_path = _resolve_igibson_ros_config(rospy.get_param("~igibson_config", ""))
        igibson_config = prepare_dynamic_object_rendering(load_yaml(igibson_config_path))
        mode = rospy.get_param("~mode", self.config.get("igibson", {}).get("mode", "gui_non_interactive"))
        rospy.loginfo("iGibson optimized_renderer disabled so dynamic pedestrian meshes can be rendered")

        self.cmdx = 0.0
        self.cmdy = 0.0
        self.latest_states = initial_states_from_config(self.config)

        self.image_pub = rospy.Publisher("/gibson_ros/camera/rgb/image", ImageMsg, queue_size=10)
        self.depth_pub = rospy.Publisher("/gibson_ros/camera/depth/image", ImageMsg, queue_size=10)
        self.depth_alias_pub = rospy.Publisher("/gibson_ros/camera/rgb/depth", ImageMsg, queue_size=10)
        self.lidar_pub = rospy.Publisher("/gibson_ros/lidar/points", PointCloud2, queue_size=10)
        self.depth_raw_pub = rospy.Publisher("/gibson_ros/camera/depth/image_raw", ImageMsg, queue_size=10)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.gt_pose_pub = rospy.Publisher("/ground_truth_odom", Odometry, queue_size=10)
        self.camera_info_pub = rospy.Publisher("/gibson_ros/camera/depth/camera_info", CameraInfo, queue_size=10)

        rospy.Subscriber("/mobile_base/commands/velocity", Twist, self.cmd_callback, queue_size=10)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback, queue_size=10)
        rospy.Subscriber("/reset_pose", PoseStamped, self.tp_robot_callback, queue_size=10)
        rospy.Subscriber(
            rospy.get_param("~pedestrian_state_topic", "/pedestrian_states"),
            Float32MultiArray,
            self.pedestrian_states_callback,
            queue_size=10,
        )

        self.br = tf.TransformBroadcaster()
        self.env = iGibsonEnv(config_file=igibson_config, mode=mode, action_timestep=1 / 30.0)
        self.env.reset()

        human_cfg = self.config.get("igibson_humanoids", {})
        style_by_behavior = {
            BEHAVIOR_IDS.get(key, int(key) if str(key).isdigit() else 1): int(value)
            for key, value in human_cfg.get("mesh_styles", {}).items()
        }
        if not style_by_behavior:
            style_by_behavior = MESH_STYLE_BY_BEHAVIOR_ID
        self.pedestrian_controller = PedestrianObjectController(
            backend=IGibsonPedestrianBackend(
                self.env.simulator,
                fallback_height=human_cfg.get("fallback_height", 1.7),
                robots=self.env.robots,
                disable_robot_collision=human_cfg.get("disable_robot_collision", True),
            ),
            z_offset=human_cfg.get("z_offset", 0.05),
            mesh_style_by_behavior_id=style_by_behavior,
            mesh_yaw_offset=human_cfg.get("mesh_yaw_offset", 0.0),
            world_offset_xy=self.env.task.initial_pos[:2],
        )
        rospy.loginfo(
            "Corridor pedestrians are placed in iGibson world frame with odom offset [%.3f, %.3f]",
            float(self.env.task.initial_pos[0]),
            float(self.env.task.initial_pos[1]),
        )
        self.tp_time = None

    def pedestrian_states_callback(self, message):
        try:
            self.latest_states = parse_flat_states(message.data)
        except ValueError as exc:
            self.rospy.logwarn("Ignoring malformed pedestrian states in iGibson sim: %s", exc)

    def _sync_pedestrians(self):
        if self.latest_states:
            self.pedestrian_controller.sync(self.latest_states)
            self.env.simulator.sync(force_sync=True)

    def _publish_camera(self, obs, now):
        rgb = (obs["rgb"] * 255).astype(self.np.uint8)
        normalized_depth = obs["depth"].astype(self.np.float32)
        depth = normalized_depth * self.env.sensors["vision"].depth_high
        depth_raw_image = (obs["depth"] * 1000).astype(self.np.uint16)

        image_message = numpy_to_image_msg(rgb, "rgb8", "camera_depth_optical_frame", now, self.ImageMsg)
        depth_message = numpy_to_image_msg(depth, "32FC1", "camera_depth_optical_frame", now, self.ImageMsg)
        depth_raw_message = numpy_to_image_msg(
            depth_raw_image,
            "16UC1",
            "camera_depth_optical_frame",
            now,
            self.ImageMsg,
        )

        self.image_pub.publish(image_message)
        self.depth_pub.publish(depth_message)
        self.depth_alias_pub.publish(depth_message)
        self.depth_raw_pub.publish(depth_raw_message)

        camera_info = self.CameraInfo(
            height=256,
            width=256,
            distortion_model="plumb_bob",
            D=[0.0, 0.0, 0.0, 0.0, 0.0],
            K=[128, 0.0, 128, 0.0, 128, 128, 0.0, 0.0, 1.0],
            R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            P=[128, 0.0, 128, 0.0, 0.0, 128, 128, 0.0, 0.0, 0.0, 1.0, 0.0],
        )
        camera_info.header.stamp = now
        camera_info.header.frame_id = "camera_depth_optical_frame"
        self.camera_info_pub.publish(camera_info)

    def _publish_lidar(self, obs, now):
        scan = obs["scan"]
        lidar_header = self.Header()
        lidar_header.stamp = now
        lidar_header.frame_id = "scan_link"

        laser_linear_range = self.env.sensors["scan_occ"].laser_linear_range
        laser_angular_range = self.env.sensors["scan_occ"].laser_angular_range
        min_laser_dist = self.env.sensors["scan_occ"].min_laser_dist
        n_horizontal_rays = self.env.sensors["scan_occ"].n_horizontal_rays

        laser_angular_half_range = laser_angular_range / 2.0
        angle = self.np.arange(
            -self.np.radians(laser_angular_half_range),
            self.np.radians(laser_angular_half_range),
            self.np.radians(laser_angular_range) / n_horizontal_rays,
        )
        unit_vector_laser = self.np.array([[self.np.cos(ang), self.np.sin(ang), 0.0] for ang in angle])
        lidar_points = unit_vector_laser * (scan * (laser_linear_range - min_laser_dist) + min_laser_dist)
        self.lidar_pub.publish(self.pc2.create_cloud_xyz32(lidar_header, lidar_points.tolist()))

    def _publish_odom(self, now):
        odom = [
            self.np.array(self.env.robots[0].get_position()) - self.np.array(self.env.task.initial_pos),
            self.np.array(self.env.robots[0].get_rpy()) - self.np.array(self.env.task.initial_orn),
        ]
        yaw = odom[-1][-1]
        quat = self.tf.transformations.quaternion_from_euler(0, 0, yaw)
        self.br.sendTransform((odom[0][0], odom[0][1], 0), quat, now, "base_footprint", "odom")

        odom_msg = self.Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose.position.x = odom[0][0]
        odom_msg.pose.pose.position.y = odom[0][1]
        (
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w,
        ) = quat
        odom_msg.twist.twist.linear.x = (self.cmdx + self.cmdy) * 5
        odom_msg.twist.twist.angular.z = (self.cmdy - self.cmdx) * 5 * 8.695652173913043
        self.odom_pub.publish(odom_msg)

        gt_pose_msg = self.Odometry()
        gt_pose_msg.header.stamp = now
        gt_pose_msg.header.frame_id = "ground_truth_odom"
        gt_pose_msg.child_frame_id = "base_footprint"
        xyz = self.env.robots[0].get_position()
        rpy = self.env.robots[0].get_rpy()
        gt_pose_msg.pose.pose.position.x = xyz[0]
        gt_pose_msg.pose.pose.position.y = xyz[1]
        gt_pose_msg.pose.pose.position.z = xyz[2]
        (
            gt_pose_msg.pose.pose.orientation.x,
            gt_pose_msg.pose.pose.orientation.y,
            gt_pose_msg.pose.pose.orientation.z,
            gt_pose_msg.pose.pose.orientation.w,
        ) = self.tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        gt_pose_msg.twist.twist.linear.x = self.cmdx
        gt_pose_msg.twist.twist.angular.z = -self.cmdy
        self.gt_pose_pub.publish(gt_pose_msg)

    def run(self):
        while not self.rospy.is_shutdown():
            self._sync_pedestrians()
            obs, _, _, _ = self.env.step([self.cmdx, self.cmdy])
            now = self.rospy.Time.now()
            self._publish_camera(obs, now)
            if (self.tp_time is None) or ((self.rospy.Time.now() - self.tp_time).to_sec() > 1.0):
                self._publish_lidar(obs, now)
            self._publish_odom(now)

    def cmd_callback(self, data):
        self.cmdx = data.linear.x
        self.cmdy = -data.angular.z

    def tp_robot_callback(self, data):
        self.rospy.loginfo("Teleporting robot")
        position = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        orientation = [
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w,
        ]
        self.env.robots[0].reset_new_pose(position, orientation)
        self.tp_time = self.rospy.Time.now()


def main():
    import rospy

    logging.basicConfig(level=logging.INFO)
    rospy.init_node("igibson_corridor_crowd_sim")
    node = CorridorCrowdSimNode()
    rospy.loginfo("Started iGibson corridor crowd sim with visible pedestrian objects")
    node.run()


if __name__ == "__main__":
    main()
