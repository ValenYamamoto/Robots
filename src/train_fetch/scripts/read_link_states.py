#!/usr/bin/python3
import rospy
import numpy as np
from gazebo_msgs.msg import LinkStates

NUM_LINKS = 10

link_names = {"fetch::torso_lift_link":0, 
	       "fetch::shoulder_pan_link":1,
	       "fetch::shoulder_lift_link":2,
	       "fetch::upperarm_roll_link":3,
	       "fetch::elbow_flex_link":4,
	       "fetch::forearm_roll_link":5,
	       "fetch::wrist_flex_link":6,
	       "fetch::wrist_roll_link":7,
	       "fetch::l_gripper_finger_link":8,
	       "fetch::r_gripper_finger_link":9
}

def printInfo( data ):
	states = np.empty( NUM_LINKS, dtype="object" )
	print( "type pose", type( data.pose ) )
	for (name, pose, twist) in zip( data.name, data.pose, data.twist ):
		if name in link_names:
			print( "name:", name )
			print( "pose:", pose.orientation )
			states[link_names[name]] = pose.orientation
			print()
	print( states )
	print( "\n\n" )

def listener():
	rospy.init_node( 'link_state_listener', anonymous=True )
	rospy.Subscriber( '/gazebo/link_states', LinkStates, printInfo, queue_size=1 )

	rospy.spin()

if __name__ == "__main__":
	listener()
