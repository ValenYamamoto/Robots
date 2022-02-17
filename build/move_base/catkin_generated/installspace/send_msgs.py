#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist

joint_names = ["torso_lift_joint", 
	       "shoulder_pan_joint",
	       "shoulder_lift_joint",
	       "upperarm_roll_joint",
	       "elbow_flex_joint",
	       "forearm_roll_joint",
	       "wrist_flex_joint",
	       "wrist_roll_joint"]
def publish_twist_msg( linear, angular ):
	twist = Twist()
	twist.linear.x, twist.linear.y, twist.linear.z = linear
	twist.angular.x, twist.angular.y, twist.angular.z = angular
	pub.publish( twist )

linear_values = [(1, 0, 0), (0, 0, 0)]
angular_values = [(0, 0, 0), (0, 0, 1)]

if __name__ == "__main__":
	try:
		rospy.init_node('send_msgs')
		rospy.loginfo( "Node Initialized" )
		pub = rospy.Publisher( "base_controller/command", Twist, queue_size=10 )
		rospy.loginfo( "publisher Initialized" )
		rate = rospy.Rate( 10 )
		rospy.loginfo("Initialized")
		while not rospy.is_shutdown():
			for i in range( 2 ):
				publish_twist_msg( linear_values[i], angular_values[i] )
				rate.sleep()

	except rospy.ROSInterruptException:
		rospy.loginfo( "Ran into Exception" )	
