#!/usr/bin/env python3
import csv
import json
import math
import pathlib
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


STATE_STRIDE = 8
CSV_FIELDS = [
    "time",
    "robot_x",
    "robot_y",
    "robot_yaw",
    "pedestrian_states",
    "min_distance_to_pedestrian",
    "personal_space_violation",
    "collision_flag",
]


def parse_flat_states(flat_values):
    values = [float(value) for value in flat_values]
    if len(values) % STATE_STRIDE != 0:
        raise ValueError("Pedestrian state array length must be a multiple of {}".format(STATE_STRIDE))
    pedestrians = []
    for offset in range(0, len(values), STATE_STRIDE):
        row = values[offset : offset + STATE_STRIDE]
        pedestrians.append(
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
    return pedestrians


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def extract_robot_pose(odom):
    pose = odom.pose.pose
    return {
        "x": float(pose.position.x),
        "y": float(pose.position.y),
        "yaw": quaternion_to_yaw(pose.orientation),
    }


def compute_metrics(robot_x, robot_y, robot_radius, pedestrians, personal_space_threshold):
    min_distance = float("inf")
    collision_flag = False
    for pedestrian in pedestrians:
        distance = math.hypot(float(pedestrian["x"]) - robot_x, float(pedestrian["y"]) - robot_y)
        min_distance = min(min_distance, distance)
        if distance <= robot_radius + float(pedestrian["radius"]):
            collision_flag = True

    personal_space_violation = min_distance < float(personal_space_threshold)
    return {
        "min_distance_to_pedestrian": min_distance,
        "personal_space_violation": bool(personal_space_violation),
        "collision_flag": bool(collision_flag),
    }


def build_log_row(stamp, robot_pose, pedestrians, metrics):
    min_distance = metrics["min_distance_to_pedestrian"]
    if math.isinf(min_distance):
        min_distance_text = "inf"
    else:
        min_distance_text = "{:.3f}".format(min_distance)

    return {
        "time": "{:.3f}".format(float(stamp)),
        "robot_x": "{:.3f}".format(float(robot_pose["x"])),
        "robot_y": "{:.3f}".format(float(robot_pose["y"])),
        "robot_yaw": "{:.3f}".format(float(robot_pose["yaw"])),
        "pedestrian_states": json.dumps(pedestrians, sort_keys=True),
        "min_distance_to_pedestrian": min_distance_text,
        "personal_space_violation": "1" if metrics["personal_space_violation"] else "0",
        "collision_flag": "1" if metrics["collision_flag"] else "0",
    }


def default_log_dir():
    return pathlib.Path(__file__).resolve().parents[2] / "logs"


def now_sec():
    try:
        return rospy.Time.now().to_sec()
    except Exception:
        return time.time()


class CorridorMetricsLogger:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.states_topic = rospy.get_param("~states_topic", "/pedestrian_states")
        self.robot_radius = float(rospy.get_param("~robot_radius", 0.35))
        self.personal_space_threshold = float(rospy.get_param("~personal_space_threshold", 1.2))
        self.log_rate_hz = float(rospy.get_param("~log_rate_hz", 10.0))
        self.log_dir = pathlib.Path(rospy.get_param("~log_dir", str(default_log_dir())))
        self.log_dir.mkdir(parents=True, exist_ok=True)

        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.log_path = self.log_dir / "corridor_metrics_{}.csv".format(stamp)
        self.handle = open(str(self.log_path), "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.handle, fieldnames=CSV_FIELDS)
        self.writer.writeheader()

        self.robot_pose = None
        self.pedestrians = []

        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)
        self.states_sub = rospy.Subscriber(self.states_topic, Float32MultiArray, self.states_callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.log_rate_hz), self.timer_callback)

    def odom_callback(self, message):
        self.robot_pose = extract_robot_pose(message)

    def states_callback(self, message):
        try:
            self.pedestrians = parse_flat_states(message.data)
        except ValueError as exc:
            rospy.logwarn("Ignoring malformed pedestrian states for metrics: %s", exc)

    def timer_callback(self, _event):
        if self.robot_pose is None:
            return
        metrics = compute_metrics(
            self.robot_pose["x"],
            self.robot_pose["y"],
            self.robot_radius,
            self.pedestrians,
            self.personal_space_threshold,
        )
        self.writer.writerow(build_log_row(now_sec(), self.robot_pose, self.pedestrians, metrics))
        self.handle.flush()

    def close(self):
        self.handle.close()


def main():
    rospy.init_node("corridor_metrics_logger")
    logger = CorridorMetricsLogger()
    rospy.loginfo("Logging corridor crowd metrics to %s", logger.log_path)
    rospy.on_shutdown(logger.close)
    rospy.spin()


if __name__ == "__main__":
    main()
