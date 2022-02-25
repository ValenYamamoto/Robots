#!/usr/bin/python3

import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

def publish_twist_msg( linear, angular ):
	twist = Twist()
	twist.linear.x, twist.linear.y, twist.linear.z = linear
	twist.angular.x, twist.angular.y, twist.angular.z = angular
	pub.publish( twist )

rospy.init_node('send_msgs')
reset_simulation = rospy.ServiceProxy( '/gazebo/reset_world', Empty )

reset_simulation()
pub = rospy.Publisher( "base_controller/command", Twist, queue_size=10 )

