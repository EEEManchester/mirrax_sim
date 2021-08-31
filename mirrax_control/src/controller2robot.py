#!/usr/bin/env python

## ROS messages & libraries
import rospy
from sensor_msgs.msg import Joy 		# sub msg for Joy
from std_msgs.msg import Float64 		# pub msg for Gazebo joints
from geometry_msgs.msg import Twist	# pub msg for base

## Python libraries
import numpy as np

#####################################################################

class ControllerRemap:
	def __init__(self, initialise=False):
		rospy.init_node('ControllerRemap') 		# Initialises node
		self.rate = rospy.Rate(20)	# Controller rate

		## Joint position increments per control rate
		self.LEG_INC = 0.01
		self.ARM_INC = 0.02
		self.joint_inc = [self.LEG_INC,self.LEG_INC,self.ARM_INC,self.ARM_INC,self.ARM_INC]

		## Scaling factor for base twist cmd
		self.LIN_VEL = 0.2
		self.ANG_VEL = 0.2

		self.setpoint_pos = [0.0]*5		# Setpoint for joints

		## Joint limits
		self.joint_min = [-0.02, -np.pi, -np.pi/2, -np.pi/2, 0.0]
		self.joint_max = [np.pi, 0.02, np.pi, np.pi, np.pi]

		self.__initialise__()

	def joint_state_callback(self, msg):
		""" robot joint state callback """
		self.current_joint_state_ = msg

	def joy_callback(self, msg):
		""" joy callback """
		self.joy_input_ = msg

	def __initialise__(self):
		""" Initialises robot and empty messages """

		## set up publisher and subscriber topics
		self.__initialise_topics__()

		## initialises timer
		self.timer = rospy.Timer(rospy.Duration(1./20.), self.timer_callback)

		## Initialise empty messages
		self.current_joint_state_ = None
		self.joy_input_ = None
		
	def __initialise_topics__(self):
		""" Initialises publishers and subscribers """

		##***************** PUBLISHERS ***************##
		self.joints_pub_ = {}
		self.joints_pub_[0] = rospy.Publisher('/mirrax/joint_5/command', Float64, queue_size=1)
		self.joints_pub_[1] = rospy.Publisher('/mirrax/joint_6/command', Float64, queue_size=1)
		self.joints_pub_[2] = rospy.Publisher('/mirrax/joint_7/command', Float64, queue_size=1)
		self.joints_pub_[3] = rospy.Publisher('/mirrax/joint_8/command', Float64, queue_size=1)
		self.joints_pub_[4] = rospy.Publisher('/mirrax/joint_9/command', Float64, queue_size=1)
		self.twist_pub_ = rospy.Publisher('/mirrax/sim_cmd_vel', Twist, queue_size=1)

		##***************** SUBSCRIBERS ***************##
		self.joy_sub_ = rospy.Subscriber('/joy', Joy, self.joy_callback, queue_size=1)

		rospy.sleep(0.5) # sleep for short while for topics to be initiated properly

	def timer_callback(self,timer):
		""" Timer for fixed control rate to move joints and base """

		if self.joy_input_ == None:
			return

		## Remap joy for base velocity
		twist_cmd = Twist()
		twist_cmd.linear.x = self.joy_input_.axes[1]*self.LIN_VEL
		twist_cmd.linear.y = self.joy_input_.axes[0]*self.LIN_VEL
		twist_cmd.angular.z = self.joy_input_.axes[3]*self.ANG_VEL

		## Remap joy for joint positions
		if (self.joy_input_.buttons[3] == 1):
			## Move to default position if activated
			print 'Default position'
			self.setpoint_pos = [0.0]*5
		else:
			## Cast button from int into float
			self.joy_input_.buttons = map(float,self.joy_input_.buttons)

			## Remap buttons for joint positions
			joy_map = [0.0]*5
			joy_map[0] = -1.0*self.joy_input_.buttons[4] + 1.0*self.joy_input_.buttons[6]
			joy_map[1] = -1.0*self.joy_input_.buttons[5] + 1.0*self.joy_input_.buttons[7]
			joy_map[2] = self.joy_input_.axes[6]
			joy_map[3] = self.joy_input_.axes[7]
			joy_map[4] = -1.0*self.joy_input_.buttons[8] + 1.0*self.joy_input_.buttons[9]
			
			self.setpoint_pos = map(self.update_joint_pos, 
															joy_map, self.setpoint_pos,
															self.joint_inc,
															self.joint_min,
															self.joint_max)

		## Publish topics
		self.twist_pub_.publish(twist_cmd)
		for i in range(0,5):
			self.joints_pub_[i].publish(Float64(self.setpoint_pos[i]))
		
	def update_joint_pos(self, joy_map, setpoint_pos, 
												pos_inc, pos_min, pos_max):
		
		## Update the joint position
		new_pos = setpoint_pos + pos_inc*joy_map

		## Limit the min and max joint position
		if (new_pos < pos_min):
			new_pos = pos_min
		elif (new_pos > pos_max):
			new_pos = pos_max

		return new_pos

if __name__ == "__main__":

	manager = ControllerRemap(True)

	rospy.loginfo('Robot Ready!')
	rospy.spin()