#!/usr/bin/env python3
import math
import pathlib
import time

import rospy
import yaml
from std_msgs.msg import Float32MultiArray


BEHAVIOR_IDS = {
    "normal": 1,
    "hurried": 2,
    "stop_go": 3,
}

DEFAULT_CONFIG = pathlib.Path(__file__).resolve().parents[1] / "config" / "corridor_crowd.yaml"


def load_config(path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _as_xy(value, field_name):
    if len(value) < 2:
        raise ValueError("{} must contain at least x and y".format(field_name))
    return [float(value[0]), float(value[1])]


def build_pedestrians(config):
    pedestrians = []
    for raw in config.get("pedestrians", []):
        behavior = str(raw.get("behavior", "normal"))
        if behavior not in BEHAVIOR_IDS:
            raise ValueError("Unsupported pedestrian behavior: {}".format(behavior))
        pedestrians.append(
            {
                "id": int(raw["id"]),
                "behavior": behavior,
                "behavior_id": BEHAVIOR_IDS[behavior],
                "class_id": int(raw.get("class_id", 1)),
                "radius": float(raw.get("radius", 0.39)),
                "speed": float(raw.get("speed", 1.0)),
                "start": _as_xy(raw["start"], "start"),
                "goal": _as_xy(raw["goal"], "goal"),
                "stop_after": float(raw.get("stop_after", 0.0)),
                "stop_duration": float(raw.get("stop_duration", 0.0)),
            }
        )
    return pedestrians


def _stop_go_motion_time(pedestrian, elapsed):
    stop_after = float(pedestrian.get("stop_after", 0.0))
    stop_duration = float(pedestrian.get("stop_duration", 0.0))
    if pedestrian.get("behavior") != "stop_go" or stop_after <= 0.0 or stop_duration <= 0.0:
        return float(elapsed), False

    cycle = stop_after + stop_duration
    full_cycles = int(float(elapsed) // cycle)
    remainder = float(elapsed) - full_cycles * cycle
    motion_time = full_cycles * stop_after + min(remainder, stop_after)
    return motion_time, remainder >= stop_after


def _round_trip_pose(start, goal, speed, motion_time):
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    distance = math.hypot(dx, dy)
    speed = float(speed)
    if distance <= 1e-9 or speed <= 0.0:
        return start[0], start[1], 0.0, 0.0

    one_way_time = distance / speed
    cycle_time = 2.0 * one_way_time
    cycle_position = float(motion_time) % cycle_time
    ux = dx / distance
    uy = dy / distance

    if cycle_position <= one_way_time:
        progress = cycle_position / one_way_time
        direction = 1.0
    else:
        progress = (cycle_time - cycle_position) / one_way_time
        direction = -1.0

    x = start[0] + dx * progress
    y = start[1] + dy * progress
    vx = direction * ux * speed
    vy = direction * uy * speed
    return x, y, vx, vy


def compute_pedestrian_state(pedestrian, elapsed):
    motion_time, stopped = _stop_go_motion_time(pedestrian, elapsed)
    x, y, vx, vy = _round_trip_pose(
        pedestrian["start"],
        pedestrian["goal"],
        pedestrian["speed"],
        motion_time,
    )
    if stopped:
        vx = 0.0
        vy = 0.0

    return [
        float(pedestrian["id"]),
        float(x),
        float(y),
        float(vx),
        float(vy),
        float(pedestrian["radius"]),
        float(pedestrian["class_id"]),
        float(pedestrian["behavior_id"]),
    ]


def pack_states(states):
    message = Float32MultiArray()
    message.data = [float(value) for state in states for value in state]
    return message


def now_sec():
    try:
        return rospy.Time.now().to_sec()
    except Exception:
        return time.time()


class ScriptedPedestrianManager:
    def __init__(self):
        config_path = rospy.get_param("~config", str(DEFAULT_CONFIG))
        self.config = load_config(config_path)
        self.pedestrians = build_pedestrians(self.config)

        publisher_cfg = self.config.get("publisher", {})
        self.state_topic = rospy.get_param(
            "~state_topic", publisher_cfg.get("scripted_state_topic", "/scripted_pedestrian_states")
        )
        self.update_rate_hz = float(rospy.get_param("~update_rate_hz", publisher_cfg.get("update_rate_hz", 20.0)))
        self.use_igibson_objects = bool(rospy.get_param("~use_igibson_objects", False))
        self.start_time = now_sec()
        self.pub = rospy.Publisher(self.state_topic, Float32MultiArray, queue_size=10)

        if self.use_igibson_objects:
            rospy.logwarn(
                "use_igibson_objects is requested, but this first version uses scripted state plus RViz markers."
            )

    def build_message(self, elapsed):
        states = [compute_pedestrian_state(pedestrian, elapsed) for pedestrian in self.pedestrians]
        return pack_states(states)

    def run(self):
        rate = rospy.Rate(self.update_rate_hz)
        while not rospy.is_shutdown():
            self.pub.publish(self.build_message(now_sec() - self.start_time))
            rate.sleep()


def main():
    rospy.init_node("scripted_pedestrian_manager")
    node = ScriptedPedestrianManager()
    rospy.loginfo("Publishing scripted corridor pedestrians on %s", node.state_topic)
    node.run()


if __name__ == "__main__":
    main()
