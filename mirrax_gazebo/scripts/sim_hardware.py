#!/usr/bin/env python

"""
Simulates Dynamixel motor output
"""

import rospy
import tf2_ros
import geometry_msgs.msg
import math
import numpy as np
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16, Float64
from geometry_msgs.msg import Twist
from robotis_controller_msgs.msg import SyncWriteMultiFloat

JOINT_NAMES = ["wheel_1", "wheel_2", "wheel_3", "wheel_4",
               "joint_5", "joint_6", "joint_7", "joint_8"]

class SimHardware():
    def __init__(self):
        rospy.init_node('sim_hardware', anonymous=True)

        self.rate = rospy.Rate(20)
        
        self.ACTUATOR_TICKS = 10240

        self.position = [0]*len(JOINT_NAMES)
        self.velocity = [0]*len(JOINT_NAMES)

        # Subscribers
        self.js_sub_ = rospy.Subscriber('/robotis/indirect_sync_write_multi_float', SyncWriteMultiFloat, self.writeToDxlCallback)
        self.lin_sub_ = rospy.Subscriber('/linear_actuator', Int16, self.linearActCallback)
        self.setpoint_sub_ = rospy.Subscriber('/mirrax/setpoints', JointState, self.setpoint_callback, queue_size=1)

        # Publishers
        self.js_pub_ = rospy.Publisher('/robotis/present_joint_states', JointState, queue_size=1)
        self.lin_pub_ = rospy.Publisher('/linear_actuator_feedback', Int16, queue_size=1)
        self.cmd_pub_ = rospy.Publisher('/mirrax/sim_cmd_vel', Twist, queue_size=1)
        self.j5_pub_ = rospy.Publisher('/mirrax/joint_5/command', Float64, queue_size=1)
        self.j6_pub_ = rospy.Publisher('/mirrax/joint_6/command', Float64, queue_size=1)

        rospy.sleep(0.5)
        self.bootstrapHardwareOutput()

    def bootstrapHardwareOutput(self):

        joint_msg = JointState()
        joint_msg.header.frame_id = ""
        joint_msg.name = JOINT_NAMES
        joint_msg.position = [0]*len(JOINT_NAMES)
        joint_msg.velocity = [0]*len(JOINT_NAMES)

        for i in range(0,len(JOINT_NAMES)):
            joint_msg.position[i] = 0.
            joint_msg.velocity[i] = 0.
        
        joint_msg.position[4] = 0.#1.89
        joint_msg.position[5] = 0.#-0.2

        for i in range(0,10):
            joint_msg.header.stamp = rospy.get_rostime()
            self.js_pub_.publish(joint_msg)
            self.lin_pub_.publish(Int16(0))
            rospy.sleep(0.05)

        self.position = joint_msg.position
        self.velocity = joint_msg.velocity
        
    def setpoint_callback(self,msg):
        data = Twist()
        for i in range(0,len(msg.name)):
            if (msg.name[i] == "x"):
                data.linear.x = msg.velocity[i]
            elif (msg.name[i] == "y"):
                data.linear.y = msg.velocity[i]
            elif (msg.name[i] == "roll"):
                data.angular.x = msg.velocity[i]
            elif (msg.name[i] == "yaw"):
                data.angular.z = msg.velocity[i]
        
        self.cmd_pub_.publish(data)
    
    def linearActCallback(self, msg):
        
        self.lin_pub_.publish(Int16(msg.data))

    def writeToDxlCallback(self, msg):
        
        joint_msg = JointState()
        joint_msg.header.frame_id = ""
        # joint_msg.name = JOINT_NAMES
        # joint_msg.position = [0]*len(JOINT_NAMES)
        # joint_msg.velocity = [0]*len(JOINT_NAMES)
        joint_msg.name = msg.joint_name
        joint_msg.position = [0]*len(msg.joint_name)
        joint_msg.velocity = [0]*len(msg.joint_name)
        
        for i in range(0,len(msg.joint_name)):
            idx = JOINT_NAMES.index(msg.joint_name[i])
            
            if (msg.joint_name[i] == "joint_5" or
                msg.joint_name[i] == "joint_6" or
                msg.joint_name[i] == "joint_7" or
                msg.joint_name[i] == "joint_8"):
                
                self.position[idx] = msg.value[i]
                self.velocity[idx] = 0.

                # Publish to Gazebo
                if (msg.joint_name[i] == "joint_5"):
                    self.j5_pub_.publish(msg.value[i])
                elif (msg.joint_name[i] == "joint_6"):
                    self.j6_pub_.publish(msg.value[i])
                
            elif (msg.joint_name[i] == "wheel_1" or
                msg.joint_name[i] == "wheel_2" or 
                msg.joint_name[i] == "wheel_3" or 
                msg.joint_name[i] == "wheel_4"):

                self.position[idx] = 0.
                self.velocity[idx] = msg.value[i]
        
        joint_msg.header.stamp = rospy.get_rostime()
        joint_msg.position = self.position
        joint_msg.velocity = self.velocity

        self.js_pub_.publish(joint_msg)

if __name__ == '__main__':
  print ("Sim hardware")
  try:
      sim_hw = SimHardware()
      rospy.spin()
  except rospy.ROSInterruptException:
      pass