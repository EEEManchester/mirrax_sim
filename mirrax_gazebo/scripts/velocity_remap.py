#!/usr/bin/env python

""" Remaps cmd_vel so that motion is more realistic
"""

import rospy, sys
import string
import warnings
from math import pi
from termcolor import colored
import numpy as np
import argparse

from gazebo_msgs.srv import *
from gazebo_ros import gazebo_interface
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import tf

class VelocityRemap:
	def __init__(self):
		rospy.init_node('GazeboVelocityRemap') 		# Initialises node

		self.model_name = 'mirrax'
		self.reference_frame = 'world'

		self.cmd_sub_ = rospy.Subscriber('/mirrax/cmd_vel', Twist, self.vel_callback, queue_size=1)
		self.cmd_pub_ = rospy.Publisher('/mirrax/sim_cmd_vel', Twist, queue_size=1)

	def vel_callback(self,msg):
		print msg
		data = Twist()
		# data.linear = msg.linear
		# data.angular = msg.angular
		if (msg.angular.z) < 0.1:
			data.angular.z = (13.0*msg.angular.z-0.1)**2
			print data.angular.z

		self.cmd_pub_.publish(data)

if __name__ == "__main__":
	
	vremap = VelocityRemap()
	
	rospy.sleep(0.1)
	rospy.spin()
	