#!/usr/bin/python3

import rospy
from arm import ArmController
import utils
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import SetModelState
import subprocess

joint_data = 0
link_data = 0
def joint_position_interrupt( data ):
    global joint_data
    joint_data = data

def link_position_interrupt( data ):
    global link_data
    link_data = data

if __name__ == "__main__":
    rospy.init_node( "arm_test" )
    rospy.Subscriber( "/ns1/joint_states", JointState, joint_position_interrupt )
    rospy.Subscriber( "/ns1/gazebo/link_states", LinkStates, link_position_interrupt )
    move_model = rospy.ServiceProxy( '/ns1/gazebo/set_model_state', SetModelState )
    #open_file = subprocess.Popen( "lsof | wc -l", shell=True, stdout=subprocess.PIPE )
    #print( "open files ", open_file.stdout.read() )
    d = rospy.Duration( 1.0 )
    rospy.sleep( d )
    utils.spawn_model( model_name='contact_box', pose=(0.75, 0, 0), namespace='/ns1/' )

    """
    a = utils.move_model( move_model, (0, 0, 3) )
    rospy.sleep( d )
    b = utils.move_model( move_model )
    print( a, b )
    """
    current_state = utils.get_joint_data( joint_data ) 
    arm = ArmController()
    move_result = arm.send_arm_goal( [0, 0, 0, 0, 0, 0, 0], current_state, True )
    #a = utils.move_model( move_model, (0, 0, 3) )

    INITIAL_ARM_POSITION = [-1.57, 0.9, -1.57, -1, 0, 0, 0]
    GOAL_ARM_POSITION = [1.57, 0.8, 1.57, -1, 0, 0, 0]
    utils.reset_world( move_model )
    for i in range( 30 ):
        move_result = arm.send_arm_goal( INITIAL_ARM_POSITION, current_state )
        #print( move_result, "MOVE ORIGIN" )
        new_state_grip_pos = utils.get_gripper_pos( link_data )
        print( new_state_grip_pos )
        #a = utils.move_model( move_model )
        move_result = arm.send_arm_goal( GOAL_ARM_POSITION, current_state )
        #print( move_result, "MOVE RESULT" )
        new_state_grip_pos = utils.get_gripper_pos( link_data )
        print( new_state_grip_pos )

    """
    #utils.move_model( move_model, (0, 0, 3) )
    arm.send_arm_goal( [0, 0, 0, 0, 0, 0, 0] )
    utils.reset_world( move_model )
    move_result = arm.send_arm_goal( [0, 1.57, 0, 0, 0, 0, 0 ] )
    #utils.spawn_model()
    print( move_result, "MOVE RESULT" )
    arm.send_arm_goal( [0, 0, 0, 0, 0, 0, 0 ], ignore_contact=True )
    #utils.move_model( move_model )
    """


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


