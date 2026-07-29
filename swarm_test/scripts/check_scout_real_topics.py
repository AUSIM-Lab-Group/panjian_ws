#!/usr/bin/env python3
"""Record Scout real-chain topic activity for one clearance checkpoint."""

import argparse
import datetime as dt
import time
from pathlib import Path

import rospy
from dynamic_perception.msg import CloudClusterArray
from dynamic_simulator.msg import DynTraj
from semantic_guard.msg import PredictedObstacleArray
from sensor_msgs.msg import PointCloud2


TOPICS = (
    ("/rslidar_points", PointCloud2, lambda msg: msg.width * msg.height > 0),
    ("/velodyne_points", PointCloud2, lambda msg: msg.width * msg.height > 0),
    ("/fastLIO/non_ground_points", PointCloud2, lambda msg: msg.width * msg.height > 0),
    ("/clustering/cluster_array", CloudClusterArray, lambda msg: bool(msg.clusters)),
    (
        "/obstacle_prediction_node/obstacle_prediction/trajs_predicted",
        DynTraj,
        lambda msg: msg.id >= 0,
    ),
    (
        "/globalFsm_by_adsm/teacher_obstacle_snapshot",
        PredictedObstacleArray,
        lambda msg: bool(msg.obstacle_ids),
    ),
)


class TopicStats:
    def __init__(self, predicate):
        self.predicate = predicate
        self.messages = 0
        self.positive = 0

    def callback(self, msg):
        self.messages += 1
        if self.predicate(msg):
            self.positive += 1


def append_report(path, clearance_m, duration_sec, rosbag_path, stats):
    path.parent.mkdir(parents=True, exist_ok=True)
    new_file = not path.exists()
    with path.open("a", encoding="utf-8") as handle:
        if new_file:
            handle.write("# Scout Minimum Detection Distance Report\n\n")
            handle.write(
                "| Time | Clearance from robot envelope (m) | Topic | Messages | Positive | Rate (Hz) | Detection (%) | Rosbag | Result |\n"
            )
            handle.write("|---|---:|---|---:|---:|---:|---:|---|---|\n")
        timestamp = dt.datetime.now().astimezone().isoformat(timespec="seconds")
        for topic, _, _ in TOPICS:
            item = stats[topic]
            rate = item.messages / duration_sec
            detection = 100.0 * item.positive / item.messages if item.messages else 0.0
            result = "PASS" if item.messages > 0 and item.positive > 0 else "FAIL"
            handle.write(
                f"| {timestamp} | {clearance_m:.2f} | `{topic}` | "
                f"{item.messages} | {item.positive} | {rate:.2f} | "
                f"{detection:.1f} | `{rosbag_path}` | {result} |\n"
            )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--clearance-m", type=float, required=True)
    parser.add_argument("--duration-sec", type=float, default=10.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("minimum_detection_distance_report.md"),
    )
    parser.add_argument("--rosbag-path", default="pending")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.clearance_m < 0.0 or args.duration_sec <= 0.0:
        raise SystemExit("clearance and duration must be positive")

    rospy.init_node("scout_real_topic_check", anonymous=True)
    stats = {topic: TopicStats(predicate) for topic, _, predicate in TOPICS}
    subscribers = [
        rospy.Subscriber(topic, msg_type, stats[topic].callback, queue_size=20)
        for topic, msg_type, _ in TOPICS
    ]

    deadline = time.monotonic() + args.duration_sec
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        time.sleep(0.05)

    append_report(
        args.output, args.clearance_m, args.duration_sec, args.rosbag_path, stats
    )
    for subscriber in subscribers:
        subscriber.unregister()
    failed = [
        topic
        for topic, _, _ in TOPICS
        if stats[topic].messages == 0 or stats[topic].positive == 0
    ]
    if failed:
        rospy.logerr("Scout topic check failed: %s", ", ".join(failed))
        raise SystemExit(2)
    rospy.loginfo("Scout topic check passed for clearance %.2f m", args.clearance_m)


if __name__ == "__main__":
    main()
