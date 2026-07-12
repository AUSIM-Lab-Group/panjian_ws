#!/usr/bin/env python3

"""Publish reference-path waypoints as /move_base_simple/goal poses."""

import math
from pathlib import Path
import sys

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
else:
    sys.path.remove(str(SCRIPT_DIR))
    sys.path.insert(0, str(SCRIPT_DIR))

import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from reference_path_waypoints import WaypointProgress


ODOM_TIMEOUT_SECONDS = 5.0
FINAL_GOAL_TOLERANCE = 1e-6
DEFAULT_PLANNER_GOAL_MIN_DISTANCE = 1.2
GOAL_CONNECTION_WAIT_SECONDS = 2.0
GOAL_DELIVERY_REPEATS = 5
GOAL_DELIVERY_PERIOD_SECONDS = 0.1


def load_reference_path_config(path_file):
    config = yaml.safe_load(Path(path_file).read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError("reference path config must be a mapping")
    if "waypoints" not in config:
        raise ValueError("reference path config missing waypoints")

    waypoints = [tuple(map(float, point)) for point in config["waypoints"]]
    threshold = float(config["threshold"])
    start_delay = float(config["start_delay"])
    final_goal = tuple(map(float, config["final_goal"]))
    if len(final_goal) != 2:
        raise ValueError("reference path final_goal must contain exactly two values")
    if math.dist(final_goal, waypoints[-1]) > FINAL_GOAL_TOLERANCE:
        raise ValueError("reference path final_goal must match the final waypoint")

    return {
        "waypoints": waypoints,
        "threshold": threshold,
        "start_delay": start_delay,
        "final_goal": final_goal,
        "planner_goal_min_distance": float(
            config.get("planner_goal_min_distance", DEFAULT_PLANNER_GOAL_MIN_DISTANCE)
        ),
    }


class ReferencePathGoalPublisher:
    def __init__(self):
        path_file = Path(rospy.get_param("~path_file"))
        config = load_reference_path_config(path_file)
        self._waypoints = config["waypoints"]
        self._threshold = config["threshold"]
        self._start_delay = config["start_delay"]
        self._planner_goal_min_distance = config["planner_goal_min_distance"]
        self._progress = WaypointProgress(self._waypoints, self._threshold)
        self._publisher = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1, latch=True
        )
        self._odom_subscriber = rospy.Subscriber(
            "/robot1/odom", Odometry, self._odom_callback, queue_size=1
        )
        self._last_position = None
        self._activation_time = None
        self._goal_published = False
        self._completed = False
        self._node_start_time = rospy.Time.now()

    def _segment_yaw(self, index):
        if index < len(self._waypoints) - 1:
            start = self._waypoints[index]
            end = self._waypoints[index + 1]
        else:
            start = self._waypoints[index - 1]
            end = self._waypoints[index]
        return math.atan2(end[1] - start[1], end[0] - start[0])

    def _build_goal(self):
        current_target = self._progress.current
        current_index = self._progress.index
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "world"
        goal.pose.position.x = current_target[0]
        goal.pose.position.y = current_target[1]
        goal.pose.position.z = 0.0
        yaw = self._segment_yaw(current_index)
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def _advance_to_planner_lookahead(self):
        if self._last_position is None:
            return
        advanced = self._progress.advance_to_min_distance(
            self._last_position, self._planner_goal_min_distance
        )
        if advanced:
            rospy.loginfo(
                "Advanced reference target to waypoint %d/%d for %.2fm planner lookahead",
                self._progress.index,
                len(self._waypoints) - 1,
                self._planner_goal_min_distance,
            )

    def _publish_current_goal(self):
        self._advance_to_planner_lookahead()
        start_wait = rospy.Time.now()
        while (
            not rospy.is_shutdown()
            and self._publisher.get_num_connections() == 0
            and (rospy.Time.now() - start_wait).to_sec() < GOAL_CONNECTION_WAIT_SECONDS
        ):
            rospy.sleep(0.05)
        goal = self._build_goal()
        try:
            for _ in range(GOAL_DELIVERY_REPEATS):
                self._publisher.publish(goal)
                rospy.loginfo(
                    "Published reference waypoint %d/%d at (%.3f, %.3f), connections=%d",
                    self._progress.index,
                    len(self._waypoints) - 1,
                    goal.pose.position.x,
                    goal.pose.position.y,
                    self._publisher.get_num_connections(),
                )
                rospy.sleep(GOAL_DELIVERY_PERIOD_SECONDS)
        except Exception as exc:
            raise RuntimeError(f"failed to publish reference waypoint: {exc}") from exc
        self._goal_published = True

    def _finish_if_final_reached(self):
        if self._last_position is None:
            return False
        final_waypoint = self._waypoints[-1]
        if math.dist(self._last_position, final_waypoint) > self._threshold:
            return False
        self._completed = True
        rospy.loginfo("Reference path final waypoint reached")
        rospy.signal_shutdown("reference path complete")
        return True

    def _odom_callback(self, msg):
        self._last_position = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        if not self._goal_published or self._completed:
            return
        if self._progress.index >= len(self._waypoints) - 1:
            self._finish_if_final_reached()
            return
        if not self._progress.update(self._last_position):
            return
        self._publish_current_goal()

    def run(self):
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if self._activation_time is None:
                if (now - self._node_start_time).to_sec() < self._start_delay:
                    rate.sleep()
                    continue
                self._activation_time = now
            if not self._goal_published:
                if self._last_position is None:
                    if (now - self._activation_time).to_sec() > ODOM_TIMEOUT_SECONDS:
                        raise RuntimeError(
                            "no odometry received within 5 seconds after activation"
                        )
                    rate.sleep()
                    continue
                self._publish_current_goal()
                if self._finish_if_final_reached():
                    return
            rate.sleep()


def main():
    rospy.init_node("reference_path_goal_publisher")
    try:
        ReferencePathGoalPublisher().run()
    except ValueError as exc:
        rospy.logerr("Invalid reference path config: %s", exc)
        return 1
    except RuntimeError as exc:
        rospy.logerr("Reference path goal publisher failed: %s", exc)
        return 1
    except Exception as exc:  # pragma: no cover - ROS runtime safety net
        rospy.logerr("Unexpected reference path goal publisher failure: %s", exc)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
