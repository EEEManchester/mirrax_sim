#!/usr/bin/env python  
import rospy
from rospy import Time 
# Because of transformations
import tf_conversions

import tf2_ros
import tf
import geometry_msgs.msg
import turtlesim.msg
from nav_msgs.msg import Odometry

def handle_odom(msg):
	
	tran = (msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z)
	rots = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)

	br = tf.TransformBroadcaster()
	br.sendTransform(tran, rots, Time.now(), 'floating_base', '/world')

if __name__ == '__main__':
	
	rospy.init_node('tf_robot_broadcaster')
	rospy.Subscriber('/mirrax/ground_truth', Odometry, handle_odom, queue_size=1)
	
	rospy.spin()