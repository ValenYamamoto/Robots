#!/usr/bin/python3

import sys
import rospy
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

import torch
import torch.nn.functional as F

import numpy as np
from arm import ArmController

import utils

ACTOR_FILE = '/home/simulator/Robots/src/train_fetch/saved_models/0223-1932/actor15' 
CRITIC_FILE = '/home/simulator/Robots/src/train_fetch/saved_models/0223-1932/critic15' 

ACTOR_LR = 5e-5
CRITIC_LR = 1e-3

CONTACT_REWARD = -50

NUM_JOINTS = 7
OUTPUT_DIMS = 7

NUM_EPOCHS = 100
PPO_STEPS = 128
STEPS_PER_ITER = 6
GAMMA = 0.99
LAMBDA = 0.95
CRITIC_DISCOUNT = 0.5
ENTROPY_BETA = 5
CLIPPING_VALUE = 0.2

INITIAL_ARM_POSITION = [-1.57, 0.9, -1.57, -1, 0, 0, 0]
GRIPPER_GOAL = (0.56033, 0.6330974, 0.295395)
INITIAL_DISTANCE = 1.144344254

link_data = 0
joint_data = 0

def link_position_interrupt( data ):
                global link_data
                link_data = data

def joint_position_interrupt( data ):
                global joint_data
                joint_data = data


class ActorModel( torch.nn.Module ):
                def __init__( self, input_dims, output_dims, hidden_dims=1024 ):
                                super( ActorModel, self ).__init__()
                                self.linear1 = torch.nn.Linear( input_dims, hidden_dims )
                                self.linear2 = torch.nn.Linear( hidden_dims, hidden_dims )
                                self.linear3 = torch.nn.Linear( hidden_dims, hidden_dims )
                                self.linear4 = torch.nn.Linear( hidden_dims, hidden_dims )
                                self.mu = torch.nn.Linear( hidden_dims, output_dims )
                                self.sigma = torch.nn.Linear( hidden_dims, output_dims )

                def forward( self, x ):
                                mid = F.relu( self.linear1( x ) )
                                mid = F.relu( self.linear2( mid ) )
                                mid = F.relu( self.linear3( mid ) )
                                mid = F.relu( self.linear4( mid ) )
                                mean = self.mu( mid )
                                sigma = torch.clamp( self.sigma( mid ), min=1e-10 )
                                return mean, sigma

                def getDistribution( self, x ):
                                mu, sigma = self.forward( x )
                                dist = torch.distributions.Normal( mu, sigma )
                                return dist

class CriticModel( torch.nn.Module ):
                def __init__( self, input_dims, hidden_dims=100 ):
                                super( CriticModel, self ).__init__()
                                self.linear1 = torch.nn.Linear( input_dims, hidden_dims )
                                self.linear2 = torch.nn.Linear( hidden_dims, hidden_dims )
                                self.linear3 = torch.nn.Linear( hidden_dims, hidden_dims )
                                self.outputLinear = torch.nn.Linear( hidden_dims, 1 )

                def forward( self, x ):
                                return  self.outputLinear( F.relu( self.linear3( F.relu( self.linear2( F.relu( self.linear1( x ) ) ) ) )  ) )

def distance_from_goal( current, goal, tol=1e-1 ):
                d = distance(current, goal )
                return d, abs( d ) < tol

def distance( x, y ):
                return sum( (a-b)**2 for a, b in zip( x, y ) ) ** 0.5

def get_advantages( values, masks, rewards ):
                returns = []
                gae = 0
                for i in reversed( range( len( rewards )) ):
                                delta = rewards[i] + GAMMA*values[i+1]*masks[i] - values[i]
                                gae = delta + GAMMA*LAMBDA*masks[i]*gae
                                #print( "gae", gae + values[i] )
                                returns.insert(0, gae+values[i] )
                #print( "returns", returns, "values", values[:-1] )
                adv = torch.tensor( returns ) - torch.tensor( values[:-1] )
                return torch.tensor( returns), (adv - torch.mean( adv))/(torch.std(adv)+1e-10)


def ppo_loss( action_probs, probs, entropies, advantages, rewards, values ):
                ratio = torch.exp( action_probs - probs )
                advantages = advantages.reshape( (-1, 1) )
                #print( ratio.shape, action_probs.shape, advantages.shape )
                #ratio = torch.exp( action_probs.sum( 1, keepdim=True ) - probs.sum( 1, keepdim=True ) )
                p1 = ratio * advantages
                p2 = torch.clamp( ratio, 1-CLIPPING_VALUE, 1+CLIPPING_VALUE ) * advantages
                actor_loss = -torch.mean( torch.min( p1, p2 ) )
                critic_loss = torch.mean( torch.square( rewards - values ) )
                entropy = torch.mean( entropies )
                total_loss = CRITIC_DISCOUNT*critic_loss + actor_loss - ENTROPY_BETA * entropy
                return total_loss

def reward_function( distance ):
                return 1- ((distance / INITIAL_DISTANCE) **2 )

def ppo_loop():
                rospy.Subscriber( "/joint_states", JointState, joint_position_interrupt )
                rospy.Subscriber( "/gazebo/link_states", LinkStates, link_position_interrupt )
                rospy.wait_for_service( '/gazebo/set_model_state' )
                move_model = rospy.ServiceProxy( '/gazebo/set_model_state', SetModelState )

                arm_controller = ArmController()
                logfilename = utils.get_logfilename()

                utils.delete_model()
                arm_controller.send_arm_goal( INITIAL_ARM_POSITION )
                utils.spawn_model()


                model_actor = ActorModel( NUM_JOINTS, OUTPUT_DIMS ).double()
                model_critic = CriticModel( NUM_JOINTS ).double()
                if ACTOR_FILE:
                            model_actor.load_state_dict( torch.load( ACTOR_FILE ) )
                            model_critic.load_state_dict( torch.load( CRITIC_FILE ) )
                critic_mse = torch.nn.MSELoss()
                actor_optimizer = torch.optim.Adam( model_actor.parameters(), lr=ACTOR_LR )
                critic_optimizer = torch.optim.Adam( model_critic.parameters(), lr=CRITIC_LR )

                #current_state = np.ones( 7 )
                current_state = utils.get_joint_data( joint_data ) 
                for epoch in range( NUM_EPOCHS ):
                                if epoch != 0 and epoch % 5 == 0:
                                                torch.save( model_actor.state_dict(), f"/home/simulator/Robots/src/train_fetch/saved_models/actor{epoch}" )
                                                torch.save( model_critic.state_dict(), f"/home/simulator/Robots/src/train_fetch/saved_models/critic{epoch}" )
                                states, actions, entropies, action_probs, values, masks, rewards, probs = [], [], [], [], [], [], [], []
                                for it in range( PPO_STEPS ):
                                                log_string = f"epoch: {epoch}, step: {it}\n"
                                                print( "epoch:", epoch, " step:", it )
                                                current_state_tensor = torch.tensor( current_state.reshape( (1, NUM_JOINTS) ) )
                                                mu, sigma = model_actor( current_state_tensor )
                                                #print( "mu:", mu )
                                                dist = model_actor.getDistribution( current_state_tensor )
                                                action = dist.sample()
                                                action_prob = dist.log_prob( action )
                                                print( "action:", action )
                                                log_string += f"\taction: {action}\n"
                                                prob = dist.log_prob( mu )
                                                entropy = dist.entropy()

                                                """
                                                print( "mu", mu.shape )
                                                print( "sigma", sigma.shape )
                                                print( "action", action.shape )
                                                print( "action_prob", action_prob.shape )
                                                print( "prob", prob.shape )
                                                print( "entropy", entropy.shape )
                                                """

                                                q_value = model_critic( current_state_tensor )
                                                log_string += f"\tq_value: {q_value}\n"

                                                #move_result = 0
                                                move_result = arm_controller.send_arm_goal( action[0], current_state )

                                                #new_state_grip_pos = ( 1, 1, 1 )
                                                #new_state = np.ones( 7 )
                                                new_state_grip_pos = utils.get_gripper_pos( link_data )
                                                #print( "GRIPPER:", new_state_grip_pos )
                                                new_state = utils.get_joint_data( joint_data )
                                                
                                                distance = 0
                                                if move_result == 0:
                                                                distance, done = distance_from_goal( new_state_grip_pos, GRIPPER_GOAL )
                                                                if not done:
                                                                            reward = reward_function( distance )
                                                                else:
                                                                            reward = 100
                                                else:
                                                                reward,done = CONTACT_REWARD, True
                                                                print( "MOVE RESULT:", move_result )
                                                                log_string += f"\tCOLLISION\n"

                                                mask = not done
                                                print( "reward:", reward, " distance:", distance )
                                                log_string += f"\treward: {reward}, distance: {distance} entropy {torch.mean(entropy).item()}\n"

                                                states.append( current_state_tensor )
                                                actions.append( action )
                                                entropies.append( entropy )
                                                probs.append( prob )
                                                action_probs.append( action_prob )
                                                values.append( q_value )
                                                masks.append( mask )
                                                rewards.append( reward )
                                                

                                                current_state = new_state

                                                if done:
                                                                #pass
                                                                rospy.loginfo( "Resetting" )
                                                                #utils.delete_model()
                                                                utils.move_model( move_model, (0, 0, 3) )
                                                                arm_controller.send_arm_goal( [0, 0, 0, 0, 0, 0, 0] )
                                                                utils.reset_world( move_model )
                                                                arm_controller.send_arm_goal( INITIAL_ARM_POSITION )
                                                                #utils.spawn_model()
                                                                utils.move_model( move_model )
                                                                current_state = np.array( INITIAL_ARM_POSITION )
                                                open_file = utils.get_logfile( logfilename )
                                                open_file.write( log_string )
                                                open_file.close()


                                #print( "entropy stack", np.vstack( entropies ), entropies )
                                #print( torch.stack( entropies, dim=1 )[0].shape )
                                q_value = model_critic( current_state_tensor )
                                values.append( q_value )

                                returns, advantages = get_advantages( values, masks, rewards )
                                """
                                print( returns, advantages )
                                print( "advantages", advantages.shape, advantages )
                                print( "returns", returns.shape, returns )
                                """


                                entropies = torch.mean( torch.reshape( torch.stack( entropies, dim=1 ), (-1, NUM_JOINTS) ), dim=1, keepdim=True )
                                actions = torch.reshape( torch.stack( actions, dim=1 ), (-1, NUM_JOINTS ) )
                                action_probs = torch.reshape( torch.stack( action_probs, dim=1 ), (-1, NUM_JOINTS ) )
                                probs = torch.reshape( torch.stack( probs, dim=1 ), (-1, NUM_JOINTS ) )
                                values = torch.reshape( torch.stack( values[:-1], dim=1 ), (-1, 1 ) )
                                masks = torch.reshape( torch.tensor(masks), (-1, 1) )
                                rewards = torch.reshape( torch.tensor(rewards), (-1, 1 ))
                                """
                                print( "action", actions.shape, actions )
                                print( "action_prob", action_probs.shape, action_probs )
                                print( "prob", probs.shape, probs )
                                print( "entropy", entropies.shape, entropies )
                                print( "values", values.shape, values )
                                print( "masks", masks.shape, masks )
                                print( "values", values.shape, values )
                                """

                                actor_optimizer.zero_grad()
                                critic_optimizer.zero_grad()
                                loss = ppo_loss( action_probs, probs, entropies, advantages, rewards, values )
                                print( "actor loss", loss )
                                loss.backward(retain_graph=True)
                                actor_optimizer.step()

                                returns = torch.reshape( returns, (-1, 1 ) )
                                critic_loss = critic_mse( values, returns )
                                """
                                print( "returns", returns )
                                print( "values", values )
                                """
                                print( "critic loss", critic_loss )

                                open_file = utils.get_logfile( logfilename )
                                log_string = f'LOSS epoch {epoch} actor {loss.item()} critic {critic_loss.item()}\n'
                                open_file.write( log_string )
                                open_file.close()

                                critic_loss.backward( retain_graph=True )
                                for _ in range( STEPS_PER_ITER ):
                                                actor_optimizer.step()
                                                critic_optimizer.step()
                                print( "****************************************")

                                                

if __name__ == "__main__":
        try:
                        rospy.init_node( 'arm_train', anonymous=False )
                        ppo_loop()
        except rospy.ROSInterruptException:
                        print( "SIGKILL" )


