#!/usr/bin/env python3
"""Write Phase 5 experiment CSV logs from simulation topics."""

import math
from pathlib import Path

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


DYNAMIC_TAU_MODES = ("legacy_gate", "teacher_tca", "teacher_ke_tca")


def parse_classes(text):
    text = (text or "").strip()
    if text.startswith("[") and text.endswith("]"):
        text = text[1:-1]
    return [item.strip() for item in text.split(",") if item.strip()]


def yaw_from_quat(q):
    values = (float(q.x), float(q.y), float(q.z), float(q.w))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("odometry quaternion is non-finite")
    norm_squared = sum(value * value for value in values)
    if not math.isfinite(norm_squared) or norm_squared <= 1.0e-24:
        raise ValueError("odometry quaternion is degenerate")
    inv_norm = 1.0 / math.sqrt(norm_squared)
    x, y, z, w = (value * inv_norm for value in values)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _positive_sign(value):
    return 1.0 if value > 0.0 else 0.0


def _tau_result(mode, reason, tau=0.0, computed=False):
    return {
        "mode": mode,
        "reason": reason,
        "tau": tau,
        "computed": computed,
        "active": math.isfinite(tau) and tau > 0.0,
    }


def compute_dynamic_tau(lx, ly, vx, vy, inflated_radius, config):
    """Mirror semantic_guard/dynamic_tau.hpp for Phase-5 audit logging."""
    mode = config["mode"]
    common = (lx, ly, vx, vy, inflated_radius)
    if not all(math.isfinite(value) for value in common):
        return _tau_result(mode, "non_finite_input")

    if mode == "legacy_gate":
        params = (
            config["ke"],
            config["t_max"],
            config["min_speed"],
            config["min_distance"],
            config["max_tau"],
        )
        if not all(math.isfinite(value) for value in params):
            return _tau_result(mode, "non_finite_input")
        if (
            inflated_radius < 0.0
            or config["ke"] < 0.0
            or config["t_max"] < 0.0
            or config["min_speed"] < 0.0
            or config["min_distance"] < 0.0
            or config["max_tau"] <= 0.0
        ):
            return _tau_result(mode, "invalid_config")

        distance = math.hypot(lx, ly)
        speed = math.hypot(vx, vy)
        if distance <= config["min_distance"]:
            return _tau_result(mode, "distance_degenerate")
        if speed <= config["min_speed"]:
            return _tau_result(mode, "speed_degenerate")

        nx, ny = lx / distance, ly / distance
        nvx, nvy = vx / speed, vy / speed
        cos_delta = nx * nvx + ny * nvy
        dot = lx * vx + ly * vy
        cone_value = (
            dot * dot
            + (inflated_radius * inflated_radius - distance * distance)
            * speed
            * speed
        )
        if math.isfinite(cone_value):
            velocity_gate = _positive_sign(cone_value)
        else:
            lateral_ratio = min(1.0, abs(nx * nvy - ny * nvx))
            velocity_gate = _positive_sign(
                inflated_radius - distance * lateral_ratio
            )

        approach_cos = max(0.0, -cos_delta)
        clearance = max(0.0, distance - inflated_radius)
        t_i = clearance * approach_cos / speed
        if not math.isfinite(t_i):
            return _tau_result(mode, "tau_invalid")
        if t_i <= 0.0 or cos_delta >= 0.0:
            return _tau_result(mode, "receding_or_nonclosing", computed=True)
        if velocity_gate == 0.0:
            return _tau_result(mode, "velocity_gate", computed=True)
        if _positive_sign(config["t_max"] - t_i) == 0.0:
            return _tau_result(mode, "time_gate", computed=True)

        raw_tau = config["ke"] * t_i
        if not math.isfinite(raw_tau) or raw_tau <= 0.0:
            return _tau_result(mode, "tau_invalid")
        tau = min(raw_tau, config["max_tau"])
        return _tau_result(mode, "active", tau=tau, computed=True)

    teacher_params = (config["max_tau"], config["delta_tau"])
    if mode == "teacher_ke_tca":
        teacher_params += (config["ke"],)
    if not all(math.isfinite(value) for value in teacher_params):
        return _tau_result(mode, "non_finite_input")
    if (
        inflated_radius < 0.0
        or config["max_tau"] <= 0.0
        or config["delta_tau"] <= 0.0
        or (mode == "teacher_ke_tca" and config["ke"] <= 0.0)
    ):
        return _tau_result(mode, "invalid_config")

    speed = math.hypot(vx, vy)
    speed_squared = vx * vx + vy * vy
    relative_dot = lx * vx + ly * vy
    denominator = speed_squared + config["delta_tau"]
    if not all(
        math.isfinite(value)
        for value in (speed, speed_squared, relative_dot, denominator)
    ) or denominator <= 0.0:
        return _tau_result(mode, "arithmetic_invalid")
    t_ca_raw = -relative_dot / denominator
    if not math.isfinite(t_ca_raw):
        return _tau_result(mode, "arithmetic_invalid")
    t_ca = min(max(t_ca_raw, 0.0), config["max_tau"])
    if t_ca <= 0.0:
        reason = "teacher_receding" if relative_dot > 0.0 else "teacher_tangent"
        return _tau_result(mode, reason, computed=True)

    tau_unclipped = config["ke"] * t_ca if mode == "teacher_ke_tca" else t_ca
    if not math.isfinite(tau_unclipped) or tau_unclipped <= 0.0:
        return _tau_result(mode, "arithmetic_invalid")
    tau = min(tau_unclipped, config["max_tau"])
    clipped = (
        tau_unclipped > config["max_tau"]
        if mode == "teacher_ke_tca"
        else t_ca_raw > config["max_tau"]
    )
    reason = mode + ("_clipped" if clipped else "_active")
    return _tau_result(mode, reason, tau=tau, computed=True)


class Phase5CsvLogger:
    def __init__(self):
        self.output_dir = Path(
            rospy.get_param("~output_dir", "swarm_test/output/secbf_runs/manual")
        )
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.pre_step = int(rospy.get_param("~pre_step", 20))
        self.robot_radius = float(rospy.get_param("~robot_radius", 0.4))
        self.dynamic_tau_enabled = bool(
            rospy.get_param("~dynamic_tau_enabled", True)
        )
        self.dynamic_tau = {
            "mode": str(rospy.get_param("~dynamic_tau/mode", "teacher_tca")),
            "delta_tau": float(
                rospy.get_param("~dynamic_tau/delta_tau", 1e-6)
            ),
            "ke": float(rospy.get_param("~dynamic_tau/Ke", 0.30)),
            "t_max": float(rospy.get_param("~dynamic_tau/Tmax", 2.0)),
            "min_speed": float(
                rospy.get_param("~dynamic_tau/min_speed", 1e-6)
            ),
            "min_distance": float(
                rospy.get_param("~dynamic_tau/min_distance", 1e-6)
            ),
            "max_tau": float(rospy.get_param("~dynamic_tau/max_tau", 2.0)),
        }
        self._validate_dynamic_tau_config()
        self.classes = parse_classes(
            rospy.get_param("~obstacle_classes", "[unknown]")
        )

        self.robot = None
        self.cmd_v = 0.0
        self.cmd_w = 0.0

        self.robot_file = (self.output_dir / "robot_log.csv").open(
            "w", encoding="utf-8"
        )
        self.obstacle_file = (self.output_dir / "obstacle_log.csv").open(
            "w", encoding="utf-8"
        )
        self.event_file = (self.output_dir / "event_log.csv").open(
            "w", encoding="utf-8"
        )

        self.robot_file.write("t,x,y,yaw,v,w,cmd_v,cmd_w\n")
        # Keep the original prefix and h_EE alias stable for old consumers.
        self.obstacle_file.write(
            "t,id,class,x,y,radius,vx,vy,d_i,rel_v,TTC,h_EE,"
            "tau,tau_mode,h_phys,h_eesm,tau_computed,tau_active,tau_reason\n"
        )
        self.event_file.write("t,event,detail\n")
        self.write_event("start", "phase5_csv_logger")

        odom_topic = rospy.get_param("~odom_topic", "/robot1/odom")
        cmd_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel1")
        obs_topic = rospy.get_param(
            "~obs_predict_topic", "/globalFsm_by_adsm/obs_predict_pub"
        )

        self.sub_odom = rospy.Subscriber(
            odom_topic, Odometry, self.odom_cb, queue_size=20
        )
        self.sub_cmd = rospy.Subscriber(
            cmd_topic, Twist, self.cmd_cb, queue_size=20
        )
        self.sub_obs = rospy.Subscriber(
            obs_topic, Float32MultiArray, self.obs_cb, queue_size=20
        )
        rospy.on_shutdown(self.close)

    def _validate_dynamic_tau_config(self):
        if not self.dynamic_tau_enabled:
            return
        mode = self.dynamic_tau["mode"]
        if mode not in DYNAMIC_TAU_MODES:
            raise ValueError(
                "unsupported dynamic_tau/mode={!r}; expected {}".format(
                    mode, ", ".join(DYNAMIC_TAU_MODES)
                )
            )
        if mode == "legacy_gate":
            values = (
                self.dynamic_tau["ke"],
                self.dynamic_tau["t_max"],
                self.dynamic_tau["min_speed"],
                self.dynamic_tau["min_distance"],
                self.dynamic_tau["max_tau"],
            )
            valid = all(math.isfinite(value) for value in values) and (
                self.dynamic_tau["ke"] >= 0.0
                and self.dynamic_tau["t_max"] >= 0.0
                and self.dynamic_tau["min_speed"] >= 0.0
                and self.dynamic_tau["min_distance"] >= 0.0
                and self.dynamic_tau["max_tau"] > 0.0
            )
        else:
            values = (
                self.dynamic_tau["delta_tau"],
                self.dynamic_tau["max_tau"],
            )
            if mode == "teacher_ke_tca":
                values += (self.dynamic_tau["ke"],)
            valid = all(math.isfinite(value) for value in values) and (
                self.dynamic_tau["delta_tau"] > 0.0
                and self.dynamic_tau["max_tau"] > 0.0
                and (
                    mode != "teacher_ke_tca"
                    or self.dynamic_tau["ke"] > 0.0
                )
            )
        if not valid:
            raise ValueError("invalid dynamic tau configuration for mode=" + mode)

    def write_event(self, event, detail):
        self.event_file.write(
            f"{rospy.Time.now().to_sec():.9f},{event},{detail}\n"
        )
        self.event_file.flush()

    def cmd_cb(self, msg):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z

    def odom_cb(self, msg):
        try:
            yaw = yaw_from_quat(msg.pose.pose.orientation)
        except ValueError as exc:
            rospy.logerr_throttle(1.0, "phase5_csv_logger: %s", exc)
            return
        body_vx = msg.twist.twist.linear.x
        body_vy = msg.twist.twist.linear.y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        world_vx = cos_yaw * body_vx - sin_yaw * body_vy
        world_vy = sin_yaw * body_vx + cos_yaw * body_vy
        w = msg.twist.twist.angular.z
        self.robot = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "vx": world_vx,
            "vy": world_vy,
            "yaw": yaw,
            "v": body_vx,
            "w": w,
        }
        self.robot_file.write(
            f"{rospy.Time.now().to_sec():.9f},{self.robot['x']:.9f},"
            f"{self.robot['y']:.9f},{yaw:.9f},{body_vx:.9f},{w:.9f},"
            f"{self.cmd_v:.9f},{self.cmd_w:.9f}\n"
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

            d_i = rel_v = ttc = h_phys = h_eesm = 0.0
            tau_result = _tau_result(
                self.dynamic_tau["mode"],
                "no_odometry" if self.dynamic_tau_enabled else "disabled",
                computed=not self.dynamic_tau_enabled,
            )
            if self.robot is not None:
                # Use the same simultaneous relative convention as Guard:
                # l=p_robot-p_obstacle, v=v_robot-v_obstacle.
                lx = self.robot["x"] - x
                ly = self.robot["y"] - y
                rvx = self.robot["vx"] - vx
                rvy = self.robot["vy"] - vy
                center_dist = math.hypot(lx, ly)
                rel_v = math.hypot(rvx, rvy)
                d_i = center_dist
                closing_speed = 0.0
                if center_dist > 1e-6:
                    closing_speed = max(
                        -((lx / center_dist) * rvx + (ly / center_dist) * rvy),
                        0.0,
                    )
                ttc = center_dist / closing_speed if closing_speed > 1e-6 else -1.0
                inflated_radius = radius + self.robot_radius
                if self.dynamic_tau_enabled:
                    tau_result = compute_dynamic_tau(
                        lx, ly, rvx, rvy, inflated_radius, self.dynamic_tau
                    )
                else:
                    tau_result = _tau_result(
                        self.dynamic_tau["mode"], "disabled", computed=True
                    )
                hx = lx + tau_result["tau"] * rvx
                hy = ly + tau_result["tau"] * rvy
                h_phys = center_dist - inflated_radius
                h_eesm = math.hypot(hx, hy) - inflated_radius

            self.obstacle_file.write(
                f"{t:.9f},{idx},{cls},{x:.9f},{y:.9f},{radius:.9f},"
                f"{vx:.9f},{vy:.9f},{d_i:.9f},{rel_v:.9f},{ttc:.9f},"
                f"{h_eesm:.9f},{tau_result['tau']:.9f},"
                f"{tau_result['mode']},{h_phys:.9f},{h_eesm:.9f},"
                f"{int(tau_result['computed'])},{int(tau_result['active'])},"
                f"{tau_result['reason']}\n"
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
