#!/usr/bin/env python3
import select
import sys
import termios
import tty


HELP_TEXT = """
iGibson keyboard cmd_vel

  w/s : increase/decrease linear x
  a/d : increase/decrease angular z
  x or space : stop
  q : stop and quit

Default output: /cmd_vel_secbf_dryrun
"""


def clamp(value, lower, upper):
    return max(float(lower), min(float(upper), float(value)))


def apply_key(key, linear, angular, linear_step, angular_step, max_linear, max_angular):
    key = key.lower()
    should_quit = False

    if key == "w":
        linear += linear_step
    elif key == "s":
        linear -= linear_step
    elif key == "a":
        angular += angular_step
    elif key == "d":
        angular -= angular_step
    elif key in (" ", "x"):
        linear = 0.0
        angular = 0.0
    elif key in ("q", "\x03"):
        linear = 0.0
        angular = 0.0
        should_quit = True

    linear = clamp(linear, -max_linear, max_linear)
    angular = clamp(angular, -max_angular, max_angular)
    return linear, angular, should_quit


def read_key(timeout):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if ready:
        return sys.stdin.read(1)
    return ""


def make_twist(twist_cls, linear, angular):
    message = twist_cls()
    message.linear.x = float(linear)
    message.angular.z = float(angular)
    return message


class KeyboardCmdVel:
    def __init__(self, rospy, twist_cls):
        self.rospy = rospy
        self.twist_cls = twist_cls
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel_secbf_dryrun")
        self.linear_step = float(rospy.get_param("~linear_step", 0.05))
        self.angular_step = float(rospy.get_param("~angular_step", 0.15))
        self.max_linear = float(rospy.get_param("~max_linear", 0.3))
        self.max_angular = float(rospy.get_param("~max_angular", 0.8))
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.linear = 0.0
        self.angular = 0.0
        self.pub = rospy.Publisher(self.output_topic, twist_cls, queue_size=1)

    def publish(self):
        self.pub.publish(make_twist(self.twist_cls, self.linear, self.angular))

    def run(self):
        if not sys.stdin.isatty():
            self.rospy.logerr("keyboard_cmd_vel.py needs an interactive terminal. Run it with rosrun in a focused shell.")
            return

        settings = termios.tcgetattr(sys.stdin)
        rate = self.rospy.Rate(self.publish_rate)
        print(HELP_TEXT)
        print("Publishing to {}".format(self.output_topic))
        try:
            tty.setcbreak(sys.stdin.fileno())
            while not self.rospy.is_shutdown():
                key = read_key(0.02)
                if key:
                    self.linear, self.angular, should_quit = apply_key(
                        key,
                        self.linear,
                        self.angular,
                        self.linear_step,
                        self.angular_step,
                        self.max_linear,
                        self.max_angular,
                    )
                    print("linear.x={:.2f}, angular.z={:.2f}".format(self.linear, self.angular))
                    if should_quit:
                        break
                self.publish()
                rate.sleep()
        finally:
            self.linear = 0.0
            self.angular = 0.0
            self.publish()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    import rospy
    from geometry_msgs.msg import Twist

    rospy.init_node("keyboard_cmd_vel")
    KeyboardCmdVel(rospy, Twist).run()


if __name__ == "__main__":
    main()
