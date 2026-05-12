#!/usr/bin/env python3
"""
Latency Measurement Script for MPC-SECBF Pipeline
Measures end-to-end and per-module latency.

Usage:
    rosrun swarm_test measure_latency.py
"""
import rospy
import time
import numpy as np
from std_msgs.msg import Float32MultiArray, Header
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist

from semantic_fusion.msg import SemanticObstacleArray


class LatencyMeasurer:
    def __init__(self):
        rospy.init_node("latency_measurer")

        self.yolo_times = []
        self.fusion_times = []
        self.beta_times = []
        self.mpc_times = []
        self.e2e_times = []

        # Track timestamps
        self.last_image_time = None
        self.last_yolo_time = None
        self.last_fusion_time = None
        self.last_beta_time = None

        # Subscribers
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb)
        rospy.Subscriber("/yolo/detections", Detection2DArray, self.yolo_cb)
        rospy.Subscriber("/semantic_obstacles", SemanticObstacleArray, self.fusion_cb)
        rospy.Subscriber("/safety_margin/beta", Float32MultiArray, self.beta_cb)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_cb)

        self.duration = rospy.get_param("~duration", 30.0)
        rospy.loginfo(f"Latency measurer started. Recording for {self.duration}s...")

    def image_cb(self, msg):
        self.last_image_time = time.time()

    def yolo_cb(self, msg):
        now = time.time()
        if self.last_image_time:
            dt = (now - self.last_image_time) * 1000
            self.yolo_times.append(dt)
        self.last_yolo_time = now

    def fusion_cb(self, msg):
        now = time.time()
        if self.last_yolo_time:
            dt = (now - self.last_yolo_time) * 1000
            self.fusion_times.append(dt)
        self.last_fusion_time = now

    def beta_cb(self, msg):
        now = time.time()
        if self.last_fusion_time:
            dt = (now - self.last_fusion_time) * 1000
            self.beta_times.append(dt)
        self.last_beta_time = now

    def cmd_cb(self, msg):
        now = time.time()
        if self.last_image_time:
            dt = (now - self.last_image_time) * 1000
            self.e2e_times.append(dt)

    def report(self):
        print("\n" + "=" * 60)
        print("LATENCY REPORT (MPC-SECBF Pipeline)")
        print("=" * 60)

        modules = [
            ("YOLO (image→detections)", self.yolo_times),
            ("Fusion (det+cluster→semantic)", self.fusion_times),
            ("Beta+Guard (semantic→β)", self.beta_times),
            ("End-to-End (image→cmd_vel)", self.e2e_times),
        ]

        for name, times in modules:
            if times:
                arr = np.array(times)
                print(f"\n{name}:")
                print(f"  mean:  {arr.mean():.1f} ms")
                print(f"  p50:   {np.percentile(arr, 50):.1f} ms")
                print(f"  p95:   {np.percentile(arr, 95):.1f} ms")
                print(f"  p99:   {np.percentile(arr, 99):.1f} ms")
                print(f"  max:   {arr.max():.1f} ms")
                print(f"  count: {len(arr)}")
            else:
                print(f"\n{name}: NO DATA")

        print("\n" + "=" * 60)
        if self.e2e_times:
            p95 = np.percentile(self.e2e_times, 95)
            target = 70.0
            status = "✅ PASS" if p95 < target else "❌ FAIL"
            print(f"NFR-5 Check: p95 E2E = {p95:.1f}ms (target < {target}ms) → {status}")
        print("=" * 60)

    def run(self):
        rospy.sleep(self.duration)
        self.report()


if __name__ == "__main__":
    node = LatencyMeasurer()
    node.run()
