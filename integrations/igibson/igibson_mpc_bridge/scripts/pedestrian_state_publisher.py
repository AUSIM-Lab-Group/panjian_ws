#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


STATE_STRIDE = 8
BEHAVIOR_NAMES = {
    1: "normal",
    2: "hurried",
    3: "stop_go",
}


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


def build_state_message(states):
    message = Float32MultiArray()
    message.data = []
    for state in states:
        message.data.extend(
            [
                float(state["id"]),
                float(state["x"]),
                float(state["y"]),
                float(state["vx"]),
                float(state["vy"]),
                float(state["radius"]),
                float(state["class_id"]),
                float(state["behavior_id"]),
            ]
        )
    return message


def _color_for_behavior(behavior_id):
    if int(behavior_id) == 2:
        return 1.0, 0.35, 0.05, 0.85
    if int(behavior_id) == 3:
        return 0.15, 0.45, 1.0, 0.85
    return 0.1, 0.8, 0.35, 0.85


def _fill_marker_header(marker, frame_id, stamp):
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp


def build_marker_array(states, frame_id, stamp=None, marker_height=1.7):
    marker_array = MarkerArray()
    for state in states:
        body = Marker()
        _fill_marker_header(body, frame_id, stamp)
        body.ns = "pedestrians"
        body.id = int(state["id"])
        body.type = Marker.CYLINDER
        body.action = Marker.ADD
        body.pose.position.x = float(state["x"])
        body.pose.position.y = float(state["y"])
        body.pose.position.z = float(marker_height) / 2.0
        body.pose.orientation.w = 1.0
        body.scale.x = float(state["radius"]) * 2.0
        body.scale.y = float(state["radius"]) * 2.0
        body.scale.z = float(marker_height)
        body.color.r, body.color.g, body.color.b, body.color.a = _color_for_behavior(state["behavior_id"])
        marker_array.markers.append(body)

        label = Marker()
        _fill_marker_header(label, frame_id, stamp)
        label.ns = "pedestrian_labels"
        label.id = int(state["id"]) + 1000
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = float(state["x"])
        label.pose.position.y = float(state["y"])
        label.pose.position.z = float(marker_height) + 0.25
        label.pose.orientation.w = 1.0
        label.scale.z = 0.24
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 0.95
        label.text = "ped{} {}".format(state["id"], BEHAVIOR_NAMES.get(int(state["behavior_id"]), "unknown"))
        marker_array.markers.append(label)
    return marker_array


class PedestrianStatePublisher:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/scripted_pedestrian_states")
        self.output_topic = rospy.get_param("~output_topic", "/pedestrian_states")
        self.marker_topic = rospy.get_param("~marker_topic", "/pedestrian_markers")
        self.frame_id = rospy.get_param("~frame_id", "odom")
        self.marker_height = float(rospy.get_param("~marker_height", 1.7))

        self.state_pub = rospy.Publisher(self.output_topic, Float32MultiArray, queue_size=10)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, Float32MultiArray, self.callback, queue_size=10)

    def callback(self, message):
        try:
            states = parse_flat_states(message.data)
        except ValueError as exc:
            rospy.logwarn("Ignoring malformed pedestrian states: %s", exc)
            return

        stamp = rospy.Time.now()
        self.state_pub.publish(build_state_message(states))
        self.marker_pub.publish(build_marker_array(states, self.frame_id, stamp, self.marker_height))


def main():
    rospy.init_node("pedestrian_state_publisher")
    PedestrianStatePublisher()
    rospy.loginfo("Publishing pedestrian states and RViz markers")
    rospy.spin()


if __name__ == "__main__":
    main()
