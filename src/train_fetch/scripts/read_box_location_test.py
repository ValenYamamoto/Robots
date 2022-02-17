#!/usr/bin/python3

import rospy
from gazebo_msgs.msg import LinkStates
import utils
import time


link_data = 0
def link_position_interrupt( data ):
    global link_data
    link_data = data

if __name__ == "__main__":
    rospy.init_node( 'read_box_location', anonymous=False )
    rospy.Subscriber( "/gazebo/link_states", LinkStates, link_position_interrupt )
    time.sleep( 1 )
    print( utils.get_box_position( link_data ) )
    
