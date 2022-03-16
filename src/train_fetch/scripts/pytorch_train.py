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
from PPO import get_advantages, ppo_loss, reward_function

import numpy as np

import utils

# saved model files to load, if no load uncomment last line
ACTOR_FILE = '/home/simulator/Robots/src/train_fetch/saved_models/0312-1925/actor35' 
CRITIC_FILE = '/home/simulator/Robots/src/train_fetch/saved_models/0312-1925/critic35' 
INITIAL_STD_DEV = 1.57*(0.9**4)
#ACTOR_FILE = None

# learning rates
ACTOR_LR = 1e-4
CRITIC_LR = 1e-3

# reward for bad move
CONTACT_REWARD = -50

# I/O shape constants
NUM_JOINTS = 7
OUTPUT_DIMS = 7

# PPO loop constants
NUM_EPOCHS = 100
PPO_STEPS = 128
STEPS_PER_ITER = 5
ITER_PER_EP=32

# Std Dev decay constants
DECAY_STD_EPOCHS=5
MIN_STD_DEV = 0.25

# arm position constants
INITIAL_ARM_POSITION = [-1.57, 0.9, -1.57, -1, 0, 0, 0]
GRIPPER_GOAL = (0.5717, 0.605, 0.714526)
INITIAL_DISTANCE = 1.144344254

# for the ROS sensor data
link_data = 0
joint_data = 0

def link_position_interrupt( data ):
                global link_data
                link_data = data

def joint_position_interrupt( data ):
                global joint_data
                joint_data = data


def distance_from_goal( current, goal, tol=1e-1 ):
                d = distance(current, goal )
                return d, abs( d ) < tol

def distance( x, y ):
                return sum( (a-b)**2 for a, b in zip( x, y ) ) ** 0.5

def ppo_loop():
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
                model_actor = ActorModelNoSigma( NUM_JOINTS, OUTPUT_DIMS ).double()
                model_critic = CriticModel( NUM_JOINTS ).double()
                if ACTOR_FILE:
                            model_actor.load_state_dict( torch.load( ACTOR_FILE ) )
                            model_critic.load_state_dict( torch.load( CRITIC_FILE ) )
                            model_actor.setStdDev( INITIAL_STD_DEV )

                # set up loss function and optimizers
                critic_mse = torch.nn.MSELoss()
                actor_optimizer = torch.optim.Adam( model_actor.parameters(), lr=ACTOR_LR )
                critic_optimizer = torch.optim.Adam( model_critic.parameters(), lr=CRITIC_LR )

                # get current joint state
                current_state = utils.get_joint_data( joint_data ) 

                # start training
                for epoch in range( NUM_EPOCHS ):
                                if epoch != 0 and epoch % 5 == 0: # save models
                                                torch.save( model_actor.state_dict(), f"/home/simulator/Robots/src/train_fetch/saved_models/actor{epoch}" )
                                                torch.save( model_critic.state_dict(), f"/home/simulator/Robots/src/train_fetch/saved_models/critic{epoch}" )

                                states, actions, entropies, action_probs, values, masks, rewards, old_probs = [], [], [], [], [], [], [], []
                                for it in range( PPO_STEPS ):
                                                log_string = f"epoch: {epoch}, step: {it}\n"
                                                print( "epoch:", epoch, " step:", it )


                                                # get current position + get action
                                                current_state_tensor = torch.tensor( current_state.reshape( (1, NUM_JOINTS) ) )
                                                mu = model_actor( current_state_tensor )

                                                # get action by sampling Distribution
                                                dist = model_actor.getDistribution( current_state_tensor )
                                                action = dist.sample()
                                                action = torch.clamp( action, min=-6.28, max=6.18 )
                                                action_prob = dist.log_prob( action )
                                                log_string += f"\taction: {action}\n"
                                                old_prob = dist.log_prob( mu )
                                                entropy = dist.entropy()


                                                # get Q value
                                                q_value = model_critic( current_state_tensor )
                                                log_string += f"\tq_value: {q_value}\n"

                                                # move arm
                                                move_result = arm_controller.send_arm_goal( action[0], current_state )

                                                # get updated position
                                                new_state_grip_pos = utils.get_gripper_pos( link_data )
                                                new_state = utils.get_joint_data( joint_data )
                                                
                                                # Was it a safe move?
                                                distance = 0
                                                if move_result == 0:
                                                                distance, done = distance_from_goal( new_state_grip_pos, GRIPPER_GOAL )
                                                                pos_tol = [ max( 0.05, 0.2*abs(x-y).item() ) for x, y in zip( action[0], current_state ) ]
                                                                diff = [ abs(x-y).item() for x, y in zip( action[0], new_state ) ]
                                                                bad_move =  any( x > y for x, y in zip( diff, pos_tol ) )
                                                                if bad_move:
                                                                            reward, done = CONTACT_REWARD, True
                                                                            log_string += f"\tCOLLISION\n"
                                                                elif not done:
                                                                            reward = reward_function( distance )
                                                                else:
                                                                            reward = 100
                                                else:
                                                                reward,done = CONTACT_REWARD, True
                                                                pos_tol = [ max( 0.05, 0.2*abs(x-y).item() ) for x, y in zip( action[0], current_state ) ]
                                                                diff = [ abs(x-y).item() for x, y in zip( action[0], new_state ) ]
                                                                if( move_result == 1 or any( x > y for x, y in zip( diff, pos_tol ) ) ):
                                                                            reward,done = CONTACT_REWARD, True
                                                                            log_string += f"\tCOLLISION\n"
                                                                else:
                                                                            distance, done = distance_from_goal( new_state_grip_pos, GRIPPER_GOAL )
                                                                            if not done:
                                                                                        reward = reward_function( distance )
                                                                            else:
                                                                                        reward = 100
                                                                        
                                                """
                                                # restart episodes every 32 moves
                                                if not done and  (it+1)%32==0:
                                                                done = True
                                                """

                                                mask = not done
                                                log_string += f"\treward: {reward}, distance: {distance} entropy {torch.mean(entropy).item()}\n"

                                                states.append( current_state_tensor )
                                                actions.append( action )
                                                entropies.append( entropy )
                                                old_probs.append( old_prob )
                                                action_probs.append( action_prob )
                                                values.append( q_value )
                                                masks.append( mask )
                                                rewards.append( reward )
                                                

                                                current_state = new_state

                                                # if hit something or goal, reset
                                                if done:
                                                                #pass
                                                                rospy.loginfo( "Resetting" )
                                                                utils.move_model( move_model, (2, 0, 0.25 ) )
                                                                arm_controller.send_arm_goal( [0, 0, 0, 0, 0, 0, 0], ignore_contact=True )
                                                                arm_controller.send_arm_goal( INITIAL_ARM_POSITION, ignore_contact=True )
                                                                utils.reset_world( move_model )
                                                                utils.move_model( move_model )
                                                                current_state = np.array( INITIAL_ARM_POSITION )

                                                # write out to file
                                                open_file = utils.get_logfile( logfilename )
                                                open_file.write( log_string )
                                                open_file.close()


                                # get last q_value
                                q_value = model_critic( current_state_tensor )
                                values.append( q_value )

                                # calculate advantages
                                returns, advantages = get_advantages( values, masks, rewards )


                                # reshape everything to same size
                                entropies = torch.mean( torch.reshape( torch.stack( entropies, dim=1 ), (-1, NUM_JOINTS) ), dim=1, keepdim=True )
                                actions = torch.reshape( torch.stack( actions, dim=1 ), (-1, NUM_JOINTS ) )
                                action_probs = torch.reshape( torch.stack( action_probs, dim=1 ), (-1, NUM_JOINTS ) )
                                old_probs = torch.reshape( torch.stack( old_probs, dim=1 ), (-1, NUM_JOINTS ) )
                                values = torch.reshape( torch.stack( values[:-1], dim=1 ), (-1, 1 ) )
                                masks = torch.reshape( torch.tensor(masks), (-1, 1) )
                                rewards = torch.reshape( torch.tensor(rewards), (-1, 1 ))

                                # zero gradients
                                actor_optimizer.zero_grad()
                                critic_optimizer.zero_grad()

                                # calculate loss for actor
                                loss = ppo_loss( action_probs, old_probs, entropies, advantages, rewards, values )
                                loss.backward(retain_graph=True)
                                actor_optimizer.step()

                                # calculate loss for critic
                                returns = torch.reshape( returns, (-1, 1 ) )
                                critic_loss = critic_mse( values, returns )

                                # write out loss info
                                open_file = utils.get_logfile( logfilename )
                                log_string = f'LOSS epoch {epoch} actor {loss.item()} critic {critic_loss.item()} \n'
                                open_file.write( log_string )
                                open_file.close()

                                # do update
                                critic_loss.backward( retain_graph=True )
                                for _ in range( STEPS_PER_ITER ):
                                                actor_optimizer.step()
                                                critic_optimizer.step()

                                if (epoch+1) % DECAY_STD_EPOCHS == 0:
                                                model_actor.decayStdDev( MIN_STD_DEV )

                                                

if __name__ == "__main__":
        try:
                        rospy.init_node( 'arm_train', anonymous=False )
                        ppo_loop()
        except rospy.ROSInterruptException:
                        print( "SIGKILL" )


