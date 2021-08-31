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
from sensor_msgs.msg import JointState
import tf

class GazeboSimMotion:
	def __init__(self):
		rospy.init_node('GazeboSimMotion') 		# Initialises node

		self.setpoint_sub_ = rospy.Subscriber('/mirrax/setpoints', JointState, self.setpoint_callback, queue_size=1)

		self.cmd_pub_ = rospy.Publisher('/mirrax/sim_cmd_vel', Twist, queue_size=1)

		rospy.set_param('/joint5_offset', 0)
		rospy.set_param('/joint6_offset', 0)

	def setpoint_callback(self,msg):
		
		data = Twist()
		data.linear.x = msg.velocity[0]
		data.linear.y = msg.velocity[1]
		data.angular.z = msg.velocity[2]

		self.cmd_pub_.publish(data)

if __name__ == "__main__":
	
	gzsim = GazeboSimMotion()
	
	rospy.sleep(0.1)
	rospy.loginfo("Sim gazebo motion ready!")
	rospy.spin()
	