#!/usr/bin/python3
import sys
import rospy
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates

from arm import ArmController

import utils

link_data = 0
joint_data = 0
def link_position_interrupt( data ):
                global link_data
                link_data = data

def joint_position_interrupt( data ):
                global joint_data
                joint_data = data

INITIAL_ARM_POSITION = [-1.57, 0.9, -1.57, -1, 0, 0, 0]
GOAL_ARM_POSITION = [1.57, 0.8, 1.57, -1, 0, 0, 0]

rospy.init_node( 'arm_position', anonymous=False )
rospy.Subscriber( "/joint_states", JointState, joint_position_interrupt )
rospy.Subscriber( "/gazebo/link_states", LinkStates, link_position_interrupt )
arm_controller = ArmController()
#utils.delete_model()
arm_controller.send_arm_goal( [0, 0, 0, 0, 0, 0, 0] )
arm_controller.send_arm_goal( INITIAL_ARM_POSITION )
"""
new_state_grip_pos = utils.get_gripper_pos( link_data )
print(  new_state_grip_pos )
"""
#move_result = arm_controller.send_arm_goal( GOAL_ARM_POSITION )
move_result = arm_controller.send_arm_goal( [0.8, -0.7, -0.0023, 0.6417, 0.115, -0.5438, 0.6674] )
print( 'move result:', move_result )
new_state_grip_pos = utils.get_gripper_pos( link_data )
print(  new_state_grip_pos )
