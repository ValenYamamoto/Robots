#!/usr/bin/python3

import rospy
from arm import ArmController
import utils
from sensor_msgs.msg import JointState
import subprocess

joint_data = 0
def joint_position_interrupt( data ):
    global joint_data
    joint_data = data

if __name__ == "__main__":
    rospy.init_node( "arm_test" )
    rospy.Subscriber( "/joint_states", JointState, joint_position_interrupt )
    #open_file = subprocess.Popen( "lsof | wc -l", shell=True, stdout=subprocess.PIPE )
    #print( "open files ", open_file.stdout.read() )
    #d = rospy.Duration( 1.0 )
    #rospy.sleep( d )
    current_state = utils.get_joint_data( joint_data ) 
    arm = ArmController()
    move_result = arm.send_arm_goal( [0, 0, 0, 0, 0, 0, 0], current_state )
    #print( move_result )

    #d = rospy.Duration( 5 )
    #rospy.sleep( d )
    #open_file = subprocess.Popen( "lsof | wc -l", shell=True, stdout=subprocess.PIPE )
    #print( "open files ", open_file.stdout.read() )

    #current_state = utils.get_joint_data( joint_data ) 
    #print( current_state )
    #move_result = arm.send_arm_goal( [14, 2, 3, 1, 1, 1, 1], current_state )
    #print( move_result )
    #open_file = subprocess.Popen( "lsof | wc -l", shell=True, stdout=subprocess.PIPE )
    #print( "open files ", open_file.stdout.read() )
    #d = rospy.Duration( 5.0 )
    #rospy.sleep( d )


