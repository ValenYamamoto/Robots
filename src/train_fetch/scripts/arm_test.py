#!/usr/bin/python3

import rospy
from arm import ArmController
import utils
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import SetModelState
import subprocess

joint_data = 0
def joint_position_interrupt( data ):
    global joint_data
    joint_data = data

if __name__ == "__main__":
    rospy.init_node( "arm_test" )
    rospy.Subscriber( "/joint_states", JointState, joint_position_interrupt )
    move_model = rospy.ServiceProxy( '/gazebo/set_model_state', SetModelState )
    #open_file = subprocess.Popen( "lsof | wc -l", shell=True, stdout=subprocess.PIPE )
    #print( "open files ", open_file.stdout.read() )
    d = rospy.Duration( 1.0 )
    rospy.sleep( d )
    utils.spawn_model( model_name='contact_box', pose=(0.75, 0, 0) )

    """
    a = utils.move_model( move_model, (0, 0, 3) )
    rospy.sleep( d )
    b = utils.move_model( move_model )
    print( a, b )
    """
    current_state = utils.get_joint_data( joint_data ) 
    arm = ArmController()
    move_result = arm.send_arm_goal( [0, 0, 0, 0, 0, 0, 0], current_state )
    a = utils.move_model( move_model, (0, 0, 3) )

    INITIAL_ARM_POSITION = [-1.57, 0.9, -1.57, -1, 0, 0, 0]
    utils.reset_world( move_model )
    move_result = arm.send_arm_goal( INITIAL_ARM_POSITION, current_state )
    print( move_result, "MOVE ORIGIN" )
    a = utils.move_model( move_model )
    move_result = arm.send_arm_goal( [0, 0.9, 0, 0, 0, 0, 0], current_state )
    print( move_result, "MOVE RESULT" )

    utils.move_model( move_model, (0, 0, 3) )
    arm.send_arm_goal( [0, 0, 0, 0, 0, 0, 0] )
    utils.reset_world( move_model )
    arm.send_arm_goal( INITIAL_ARM_POSITION )
    #utils.spawn_model()
    utils.move_model( move_model )


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


