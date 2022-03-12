#!/usr/bin/python3
import utils
import argparse
import rospy
import sys
from gazebo_msgs.srv import SetModelState


rospy.init_node( "namespace_test" )
print( sys.argv )
namespace = sys.argv[-2]
spot = float( sys.argv[-1] )
rospy.wait_for_service( namespace + '/gazebo/set_model_state' )
move_model = rospy.ServiceProxy( namespace + '/gazebo/set_model_state', SetModelState )
utils.move_model( move_model, ( spot, 0, 0 ) )

