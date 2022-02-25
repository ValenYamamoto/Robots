#!/usr/bin/python3

import rospy
from arm import ArmController
import utils
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import subprocess


def getOpenFiles():
    open_file = subprocess.Popen(["lsof"], stdout=subprocess.PIPE )
    open_file = subprocess.check_output(["wc", "-l"], stdin=open_file.stdout )
    output = open_file.decode( 'utf-8' )
    return int( output.rstrip() )
#    print( "open files ", output )

def move_model_msg():
    state_msg = ModelState()
    state_msg.model_name = 'contact_box'
    state_msg.pose.position.x = 10;
    state_msg.pose.position.y = 10;
    state_msg.pose.position.z = 10;
    state_msg.pose.orientation.x = 0;
    state_msg.pose.orientation.y = 0;
    state_msg.pose.orientation.z = 0;
    state_msg.pose.orientation.w = 0;
    return state_msg

if __name__ == "__main__":
    rospy.init_node( "arm_test" )
    rospy.wait_for_service( '/gazebo/set_model_state' )
    move_box = rospy.ServiceProxy( '/gazebo/set_model_state', SetModelState )
    
    for i in range( 10 ):
        print( i )
        before = getOpenFiles()
        utils.spawn_model()
        after = getOpenFiles()
        print( after, after - before )

        before = getOpenFiles()
        utils.move_model((10, 10, 10))
        #move_box( move_model_msg() )
        after = getOpenFiles()
        print( after, after - before )

        before = getOpenFiles()
        utils.delete_model()
        after = getOpenFiles()
        print( after, after - before )

