#!/usr/bin/env python3

import os
import numpy as np
from math import cos, sin

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry 

from gazebo_msgs.msg import ModelStates, LinkStates

import tf
import rospy


class vehicle_pose_and_velocity_updater:
	def __init__(self):
		rospy.init_node('scout_odom_use', log_level=rospy.DEBUG)
		self.odom_pub = rospy.Publisher('/state_ukf/odom', Odometry, queue_size = 1)
		rospy.Subscriber('/Odometry', Odometry, self.odom_cb, queue_size=1)
		self.timer = rospy.Timer(rospy.Duration(0.01), self.pubModlestate)
		self.odom_ = Odometry()
		rospy.spin()
	
		
	def odom_cb(self, msg:Odometry):
		self.odom_ = msg
		self.odom_.header.frame_id = 'world'
		self.odom_.child_frame_id = 'base_link'
		self.odom_.header.stamp = rospy.Time.now()
		# self.pubModelstate()
	def pubModlestate(self, timer):
		self.odom_pub.publish(self.odom_)
		br = tf.TransformBroadcaster()
		br.sendTransform((self.odom_.pose.pose.position.x, self.odom_.pose.pose.position.y, 0.2),
				   (self.odom_.pose.pose.orientation.x, self.odom_.pose.pose.orientation.y, self.odom_.pose.pose.orientation.z, self.odom_.pose.pose.orientation.w),
				   rospy.Time.now(),
				   'base_link',
				   'world')



if __name__ == "__main__":
	try:
		vehicle_pose_and_velocity_updater()
	except:
		rospy.logwarn("cannot start vehicle odom and tf updater")