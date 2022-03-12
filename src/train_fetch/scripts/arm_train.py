#!/usr/bin/python3

import rospy
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates

import sys
import numpy as np
import keras
import tensorflow as tf
import keras.backend as K
from keras import layers
from tensorflow.keras.optimizers import Adam
from arm import ArmController

from tensorflow.python.framework.ops import disable_eager_execution
disable_eager_execution()

import utils

CONTACT_REWARD = -500

NUM_JOINTS = 7
INPUT_DIMS = (NUM_JOINTS,)
OUTPUT_DIMS = 7

#-----------------PPO ARGS------------
NUM_EPOCHS = 60
PPO_STEPS = 1 #128
GAMMA = 0.99
LAMBDA = 0.95
CRITIC_DISCOUNT = 0.5
ENTROPY_BETA = 0.001
CLIPPING_VALUE = 0.2


#-------------ARM POSITIONS---------------------
INITIAL_ARM_POSITION = [-1.57, 0.9, -1.57, -1, 0, 0, 0]
GRIPPER_GOAL = (0.56033, 0.5976645, 0.295395)
INITIAL_DISTANCE = 1.144344254

#----------SENSOR DATA----------------
link_data = 0
joint_data = 0

def link_position_interrupt( data ):
        global link_data
        link_data = data

def joint_position_interrupt( data ):
        global joint_data
        joint_data = data

def get_model_actor( input_dims, output_dims ):
        inputs = layers.Input( shape=input_dims )
        oldpolicy_pred = layers.Input( shape=(1, output_dims, ) )
        advantages = layers.Input( shape=(1, 1, ) )
        rewards = layers.Input( shape=(1, 1, ) )
        values = layers.Input( shape=(1, 1, ) )

        x = layers.Dense( 512, activation='relu' )( inputs )
        x = layers.Dense( 512, activation='relu' )( x )
        out = layers.Dense( output_dims )( x )

        model = keras.Model( inputs=[inputs, oldpolicy_pred, advantages, rewards, values], outputs=[out], name='actor_model' )
        adam_opt = Adam( learning_rate=1e-2 )
        model.compile( optimizer=adam_opt, loss=[ppo_loss(
                oldpolicy_preds=oldpolicy_pred,
                advantages=advantages,
                rewards=rewards,
                values=values)]
        )
        model.summary()

        return model


def get_model_critic( input_dims ):
        inputs = layers.Input( shape=input_dims )
        
        x = layers.Dense( 100, activation='relu' )( inputs )
        x = layers.Dense( 100, activation='relu' )( x )
        out = layers.Dense( 1, activation='tanh' )( x )

        model = keras.Model( inputs=inputs, outputs=out, name='critic_model' )
        model.compile( optimizer=Adam( learning_rate=1e-2 ), loss='mse' )

        return model


def distance_from_goal( current, goal, tol=1e-2 ):
        d = distance( current, goal )
        return d, abs( d ) < tol


def distance( x, y ):
        return sum( (a-b)**2 for a, b in zip( x, y ) ) ** 0.5


def get_advantages( values, masks, rewards ):
        returns = []
        gae = 0
        for i in reversed( range( len(rewards) ) ):
                delta = rewards[i] + GAMMA*values[i+1]*masks[i] - values[i]
                gae = delta + GAMMA*LAMBDA*masks[i]*gae
                returns.insert( 0, gae+values[i] )
        adv = np.array( returns ) - values[:-1]
        return returns, (adv - np.mean(adv))/(np.std(adv) + 1e-10)

def ppo_loss( oldpolicy_preds, advantages, rewards, values ):
        def loss( y_true, y_pred ):
                ratio = K.exp( K.log(y_pred+1e-10) - K.log(oldpolicy_preds+1e-10) )
                ratio = tf.math.divide( y_pred, oldpolicy_preds )
                ratio = tf.compat.v1.Print( ratio, [y_true], "y_true" )
                ratio = tf.compat.v1.Print( ratio, [y_pred], "y_pred" )
                p1 = ratio * advantages
                #p1 = tf.compat.v1.Print( p1, [p1], "p1" )
                p2 = K.clip( ratio, min_value=1-CLIPPING_VALUE, max_value=1+CLIPPING_VALUE) * advantages
                #p2 = tf.compat.v1.Print( p2, [p2], "p2" )
                actor_loss = -K.mean( K.minimum(p1, p2) )
                #actor_loss = tf.compat.v1.Print( actor_loss, [actor_loss], "actor_loss" )
                critic_loss = K.mean( K.square( rewards-values ) )
                #critic_loss = tf.compat.v1.Print( critic_loss, [critic_loss], "critic_loss" )
                total_loss = CRITIC_DISCOUNT*critic_loss + actor_loss# - ENTROPY_BETA * K.mean( -(y_pred * K.log(y_pred+1e-10)) )
                total_loss = tf.compat.v1.Print( total_loss, [tf.size(total_loss)], "total_loss" )
                return total_loss
        return loss


def reward_function( distance ):
        return 1 - ( distance / INITIAL_DISTANCE ) ** 0.5

def ppo_loop():
        rospy.Subscriber( "/joint_states", JointState, joint_position_interrupt )
        rospy.Subscriber( "/gazebo/link_states", LinkStates, link_position_interrupt )
        #states, actions, values, masks, rewards = [], [], [], [], []
        arm_controller = ArmController()
        utils.delete_model()
        arm_controller.send_arm_goal( INITIAL_ARM_POSITION )
        utils.spawn_model()

        DUMMY_N = np.zeros((1, 1, OUTPUT_DIMS ))
        DUMMY_1 = np.zeros((1, 1, 1))

        model_actor = get_model_actor( INPUT_DIMS, OUTPUT_DIMS )
        #print( model_actor.summary() )
        model_critic = get_model_critic( INPUT_DIMS )

        #current_state = utils.get_joint_data( joint_data )
        #ppo_random_init( model_actor, model_critic, arm_controller )
        ppo_use_models( model_actor, model_critic, arm_controller )

def ppo_use_models( model_actor, model_critic, arm_controller ):
        DUMMY_N = np.zeros((1, 1, OUTPUT_DIMS ))
        DUMMY_1 = np.zeros((1, 1, 1))
        current_state = utils.get_joint_data( joint_data )
        for epoch in range( NUM_EPOCHS ):
                states, actions, values, masks, rewards = [], [], [], [], []
                for it in range( PPO_STEPS ):
                        current_state_tensor = current_state.reshape((1, NUM_JOINTS) )
                        #print( current_state.shape, current_state_tensor.shape )
                        action = model_actor.predict( [current_state_tensor, DUMMY_N, DUMMY_1, DUMMY_1, DUMMY_1], batch_size=1 )
                        q_value = model_critic.predict( current_state_tensor, batch_size=1 )
                        print( "epoch:", epoch, "Step:", it )
                        print( "action:", action )
                        #print( "q_value:", q_value )


                        # MOVE the ARM
                        move_result = arm_controller.send_arm_goal( action[0] )

                        # UPDATES
                        new_state_grip_pos = utils.get_gripper_pos( link_data )
                        new_state = utils.get_joint_data( joint_data )
                        if move_result == 0:
                                distance, done = distance_from_goal( new_state_grip_pos, GRIPPER_GOAL )
                                reward = reward_function( distance )
                        else:
                                reward, done = CONTACT_REWARD, True
                        mask = not done
                        print( "reward:", reward, " done:", done ) 

                        states.append( current_state_tensor )
                        actions.append( action )
                        values.append( q_value )
                        masks.append( mask )
                        rewards.append( reward )

                        current_state = new_state
                        
                        if done:
                                rospy.loginfo( "Resetting" )
                                #rospy.loginfo( "Deleting Box ..." )
                                utils.delete_model()
                                #rospy.loginfo( "Resettin Arm ..." )
                                arm_controller.send_arm_goal( [0, 0, 0, 0, 0, 0, 0] )
                                utils.reset_world()
                                arm_controller.send_arm_goal( INITIAL_ARM_POSITION )
                                utils.spawn_model()
                                current_state = np.array( INITIAL_ARM_POSITION )

                                

                q_value = model_critic.predict( current_state_tensor, steps=1 )
                values.append( q_value )
                returns, advantages = get_advantages( values, masks, rewards )
                print( "TRAINING ACTOR" )
                actor_loss = model_actor.fit(
                        [np.reshape(np.zeros(7), (-1, NUM_JOINTS)), np.reshape(actions, (-1,1,OUTPUT_DIMS)), np.reshape(advantages, (-1,1,1)),
                         np.reshape(rewards, (-1,1,1 )), np.reshape(values[:-1], (-1,1,1))],
                        [(np.reshape( np.ones(7), newshape=(-1, 7) ))], verbose=True, shuffle=True, epochs=10 #, callbacks=[tensor_board]
                )
                print( "TRAINING CRITIC" )
                critic_loss = model_critic.fit([np.reshape(states,(-1,NUM_JOINTS))], [np.reshape(returns, newshape=(-1, 1))], shuffle=True, epochs=10, verbose=True )


def ppo_random_init( model_actor, model_critic, arm_controller ):
        DUMMY_N = np.zeros((1, 1, OUTPUT_DIMS ))
        DUMMY_1 = np.zeros((1, 1, 1))
        current_state = utils.get_joint_data( joint_data )
        for epoch in range( 1 ):
                states, actions, values, masks, rewards = [], [], [], [], []
                for it in range( 10 ):
                        current_state_tensor = current_state.reshape((1, NUM_JOINTS) )
                        #print( current_state.shape, current_state_tensor.shape )
                        if it == 0:
                            action = np.array( [-1,0,0,0,0,0,0] ).reshape((1,7))
                        elif it == 1:
                            action = np.array( [1.57, 0.5, 1.57, -1,0,0,0] ).reshape((1,7))
                        else:
                            action = (3.14--3.14) * np.random.random((1, 7)) + -3.14 
                        q_value = model_critic.predict( current_state_tensor, batch_size=1 )
                        print( "epoch:", epoch, "Step:", it )
                        print( "action:", action )
                        #print( "q_value:", q_value )


                        # MOVE the ARM
                        move_result = arm_controller.send_arm_goal( action[0] )

                        # UPDATES
                        new_state_grip_pos = utils.get_gripper_pos( link_data )
                        new_state = utils.get_joint_data( joint_data )
                        if move_result == 0:
                                distance, done = distance_from_goal( new_state_grip_pos, GRIPPER_GOAL )
                                reward = reward_function( distance )
                        else:
                                reward, done = CONTACT_REWARD, True
                        mask = not done
                        print( "reward:", reward, " done:", done ) 

                        states.append( current_state_tensor )
                        actions.append( action )
                        values.append( q_value )
                        masks.append( mask )
                        rewards.append( reward )

                        current_state = new_state
                        
                        if done:
                                rospy.loginfo( "Resetting" )
                                #rospy.loginfo( "Deleting Box ..." )
                                utils.delete_model()
                                #rospy.loginfo( "Resettin Arm ..." )
                                arm_controller.send_arm_goal( [0, 0, 0, 0, 0, 0, 0] )
                                utils.reset_world()
                                arm_controller.send_arm_goal( INITIAL_ARM_POSITION )
                                utils.spawn_model()
                                current_state = np.array( INITIAL_ARM_POSITION )

                                

                q_value = model_critic.predict( current_state_tensor, steps=1 )
                values.append( q_value )
                returns, advantages = get_advantages( values, masks, rewards )
                print( "TRAINING ACTOR" )
                actor_loss = model_actor.fit(
                        [np.reshape(np.zeros(7), (-1, NUM_JOINTS)), np.reshape(actions, (-1,1,OUTPUT_DIMS)), np.reshape(advantages, (-1,1,1)),
                         np.reshape(rewards, (-1,1,1 )), np.reshape(values[:-1], (-1,1,1))],
                        [(np.reshape( np.array([1,1,1,1,1,1,1]), newshape=(-1, 7) ))], verbose=True, shuffle=True, epochs=10 #, callbacks=[tensor_board]
                )
                print( "TRAINING CRITIC" )
                critic_loss = model_critic.fit([np.reshape(states,(-1,NUM_JOINTS))], [np.reshape(returns, newshape=(-1, 1))], shuffle=True, epochs=10, verbose=True )

if __name__ == "__main__":
        try:
                rospy.init_node( 'arm_train', anonymous=False )
                ppo_loop()
        except ROSInterruptException:
                print( "SIGKILL" )
