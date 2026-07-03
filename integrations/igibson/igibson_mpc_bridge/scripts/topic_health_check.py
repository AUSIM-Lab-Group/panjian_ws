#!/usr/bin/env python3
import time

import rospy


DEFAULT_REQUIRED_TOPICS = [
    "/Odometry",
    "/camera/color/image_raw",
    "/camera/depth/image_raw",
    "/fastLIO/non_ground_points",
    "/mobile_base/commands/velocity",
]


def find_missing_topics(required_topics, available_topics):
    available_names = {topic for topic, _types in available_topics}
    return [topic for topic in required_topics if topic not in available_names]


def exit_code_for_missing(missing_topics):
    return 0 if not missing_topics else 2


def wait_for_topics(required_topics, timeout_sec, poll_sec=0.5):
    deadline = time.time() + timeout_sec
    last_missing = list(required_topics)
    while time.time() <= deadline and not rospy.is_shutdown():
        available = rospy.get_published_topics(namespace="/")
        last_missing = find_missing_topics(required_topics, available)
        if not last_missing:
            return []
        rospy.sleep(poll_sec)
    return last_missing


def main():
    rospy.init_node("igibson_topic_health_check")
    required_topics = rospy.get_param("~required_topics", DEFAULT_REQUIRED_TOPICS)
    timeout_sec = float(rospy.get_param("~timeout_sec", 10.0))
    missing = wait_for_topics(required_topics, timeout_sec)
    if missing:
        rospy.logerr("Missing required iGibson bridge topics: %s", ", ".join(missing))
    else:
        rospy.loginfo("All required iGibson bridge topics are published")
    raise SystemExit(exit_code_for_missing(missing))


if __name__ == "__main__":
    main()
