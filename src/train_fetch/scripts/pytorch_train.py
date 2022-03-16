#!/usr/bin/python3

import sys

import rospy
from arm import ArmController
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

import torch
import torch.nn.functional as F

from models import ActorModelNoSigma, CriticModel
from PPO import get_advantages, ppo_loss, reward_function, ppo_train_loop

import numpy as np

import utils

# arm position constants
INITIAL_ARM_POSITION = [-1.57, 0.9, -1.57, -1, 0, 0, 0]
GRIPPER_GOAL = (0.5717, 0.605, 0.714526)
INITIAL_DISTANCE = 1.144344254

# for the ROS sensor data
link_data = [0]
joint_data = [0]

def link_position_interrupt( data ):
                global link_data
                link_data[0] = data

def joint_position_interrupt( data ):
                global joint_data
                joint_data[0] = data


def distance_from_goal( current, goal, tol=1e-1 ):
                d = distance(current, goal )
                return d, abs( d ) < tol

def distance( x, y ):
                return sum( (a-b)**2 for a, b in zip( x, y ) ) ** 0.5

def ppo_loop( params ):
                # set up ROS stuff
                rospy.Subscriber( "/ns1/joint_states", JointState, joint_position_interrupt )
                rospy.Subscriber( "/ns1/gazebo/link_states", LinkStates, link_position_interrupt )
                rospy.wait_for_service( '/ns1/gazebo/set_model_state' )
                move_model = rospy.ServiceProxy( '/ns1/gazebo/set_model_state', SetModelState )

                # create arm interface
                arm_controller = ArmController()

                # create log file name
                logfilename = utils.get_logfilename()

                # set up arm and box
                utils.delete_model()
                arm_controller.send_arm_goal( INITIAL_ARM_POSITION )
                utils.spawn_model()


                # set up model, load weights if needed
                model_actor = ActorModelNoSigma( params['params']['num_joints'], params['params']['output_dims'] ).double()
                model_critic = CriticModel( params['params']['num_joints'] ).double()
                if params['params']['load_file']:
                            print( "Loading files" )
                            model_actor.load_state_dict( torch.load( params['params']['actor_file'] ) )
                            model_critic.load_state_dict( torch.load( params['params']['critic_file'] ) )
                            model_actor.setStdDev( params['params']['initial_std_dev'] )

                ppo_train_loop( arm_controller, model_actor, model_critic, params, joint_data, link_data, move_model, logfilename )

                                                

if __name__ == "__main__":
        try:
                        rospy.init_node( 'arm_train', anonymous=False )
                        params = utils.read_yaml( '/home/simulator/Robots/src/train_fetch/scripts/params.yaml' )
                        ppo_loop( params )
        except rospy.ROSInterruptException:
                        print( "SIGKILL" )


