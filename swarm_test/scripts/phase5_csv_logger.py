#!/usr/bin/env python3
"""Write Phase 5 experiment CSV logs from simulation topics."""

import math
from pathlib import Path

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


def parse_classes(text):
    text = (text or "").strip()
    if text.startswith("[") and text.endswith("]"):
        text = text[1:-1]
    return [item.strip() for item in text.split(",") if item.strip()]


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class Phase5CsvLogger:
    def __init__(self):
        self.output_dir = Path(rospy.get_param("~output_dir", "swarm_test/output/secbf_runs/manual"))
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.pre_step = int(rospy.get_param("~pre_step", 20))
        self.robot_radius = float(rospy.get_param("~robot_radius", 0.4))
        self.guard_tau = float(rospy.get_param("~guard_tau", 0.2))
        self.classes = parse_classes(rospy.get_param("~obstacle_classes", "[unknown]"))

        self.robot = None
        self.cmd_v = 0.0
        self.cmd_w = 0.0

        self.robot_file = (self.output_dir / "robot_log.csv").open("w", encoding="utf-8")
        self.obstacle_file = (self.output_dir / "obstacle_log.csv").open("w", encoding="utf-8")
        self.event_file = (self.output_dir / "event_log.csv").open("w", encoding="utf-8")

        self.robot_file.write("t,x,y,yaw,v,w,cmd_v,cmd_w\n")
        self.obstacle_file.write("t,id,class,x,y,radius,vx,vy,d_i,rel_v,TTC,h_EE\n")
        self.event_file.write("t,event,detail\n")
        self.write_event("start", "phase5_csv_logger")

        odom_topic = rospy.get_param("~odom_topic", "/robot1/odom")
        cmd_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel1")
        obs_topic = rospy.get_param("~obs_predict_topic", "/globalFsm_by_adsm/obs_predict_pub")

        self.sub_odom = rospy.Subscriber(odom_topic, Odometry, self.odom_cb, queue_size=20)
        self.sub_cmd = rospy.Subscriber(cmd_topic, Twist, self.cmd_cb, queue_size=20)
        self.sub_obs = rospy.Subscriber(obs_topic, Float32MultiArray, self.obs_cb, queue_size=20)
        rospy.on_shutdown(self.close)

    def write_event(self, event, detail):
        self.event_file.write(f"{rospy.Time.now().to_sec():.9f},{event},{detail}\n")
        self.event_file.flush()

    def cmd_cb(self, msg):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z

    def odom_cb(self, msg):
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.robot = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "vx": v * math.cos(yaw),
            "vy": v * math.sin(yaw),
            "yaw": yaw,
            "v": v,
            "w": w,
        }
        self.robot_file.write(
            f"{rospy.Time.now().to_sec():.9f},{self.robot['x']:.9f},{self.robot['y']:.9f},"
            f"{yaw:.9f},{v:.9f},{w:.9f},{self.cmd_v:.9f},{self.cmd_w:.9f}\n"
        )
        self.robot_file.flush()

    def obs_cb(self, msg):
        if self.pre_step <= 0 or not msg.data:
            return
        total_points = len(msg.data) // 7
        obs_count = total_points // self.pre_step if self.pre_step > 0 else 0
        if self.classes:
            obs_count = min(obs_count, len(self.classes))
        if obs_count <= 0:
            return

        t = rospy.Time.now().to_sec()
        for idx in range(obs_count):
            base = 7 * idx * self.pre_step
            if base + 6 >= len(msg.data):
                continue
            x = float(msg.data[base + 0])
            y = float(msg.data[base + 1])
            radius = float(msg.data[base + 2])
            vx = float(msg.data[base + 5])
            vy = float(msg.data[base + 6])
            cls = self.classes[idx] if idx < len(self.classes) else "unknown"

            d_i = rel_v = ttc = h_ee = 0.0
            if self.robot is not None:
                px = x - self.robot["x"]
                py = y - self.robot["y"]
                rvx = vx - self.robot["vx"]
                rvy = vy - self.robot["vy"]
                center_dist = math.hypot(px, py)
                rel_v = math.hypot(rvx, rvy)
                d_i = center_dist
                closing_speed = 0.0
                if center_dist > 1e-6:
                    closing_speed = max(-((px / center_dist) * rvx + (py / center_dist) * rvy), 0.0)
                ttc = center_dist / closing_speed if closing_speed > 1e-6 else -1.0
                hx = px + self.guard_tau * rvx
                hy = py + self.guard_tau * rvy
                h_ee = math.hypot(hx, hy) - radius - self.robot_radius

            self.obstacle_file.write(
                f"{t:.9f},{idx},{cls},{x:.9f},{y:.9f},{radius:.9f},{vx:.9f},{vy:.9f},"
                f"{d_i:.9f},{rel_v:.9f},{ttc:.9f},{h_ee:.9f}\n"
            )
        self.obstacle_file.flush()

    def close(self):
        try:
            self.write_event("stop", "phase5_csv_logger")
        except Exception:
            pass
        for file_obj in (self.robot_file, self.obstacle_file, self.event_file):
            try:
                file_obj.close()
            except Exception:
                pass


if __name__ == "__main__":
    rospy.init_node("phase5_csv_logger")
    Phase5CsvLogger()
    rospy.spin()
