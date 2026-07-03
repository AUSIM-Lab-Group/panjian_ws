#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist


def clamp(value, limit):
    limit = abs(float(limit))
    return max(-limit, min(limit, float(value)))


def build_relay_twist(incoming, max_linear, max_angular):
    outgoing = Twist()
    outgoing.linear.x = clamp(incoming.linear.x, max_linear)
    outgoing.linear.y = 0.0
    outgoing.linear.z = 0.0
    outgoing.angular.x = 0.0
    outgoing.angular.y = 0.0
    outgoing.angular.z = clamp(incoming.angular.z, max_angular)
    return outgoing


class CmdVelRelay:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/cmd_vel_secbf_dryrun")
        self.output_topic = rospy.get_param("~output_topic", "/mobile_base/commands/velocity")
        self.max_linear = rospy.get_param("~max_linear", 0.3)
        self.max_angular = rospy.get_param("~max_angular", 0.8)
        self.pub = rospy.Publisher(self.output_topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, Twist, self.callback, queue_size=10)

    def callback(self, msg):
        self.pub.publish(build_relay_twist(msg, self.max_linear, self.max_angular))


def main():
    rospy.init_node("igibson_cmd_vel_relay")
    CmdVelRelay()
    rospy.loginfo("Relaying MPC-SECBF cmd_vel to iGibson velocity command")
    rospy.spin()


if __name__ == "__main__":
    main()
