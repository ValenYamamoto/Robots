#!/usr/bin/python3

import sys
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
import tensorflow_probability as tfp
tfd = tfp.distributions

import utils

CONTACT_REWARD = -500

NUM_JOINTS = 7
INPUT_DIMS = (NUM_JOINTS,)
OUTPUT_DIMS = 7

#-----------------PPO ARGS------------
NUM_EPOCHS = 60
PPO_STEPS = 2 #128
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

class ActorModel( tf.keras.Model ):
        def __init__( self, output_dims, **kwargs ):
                super().__init__( **kwargs )
                self.dense1 = layers.Dense( 512, activation='relu', name='input_layer' )
                self.dense2 = layers.Dense( 512, activation='relu', name='hidden' )
                self.prediction = layers.Dense( 2*output_dims, name='output' )

        def call( self, inputs, train=False, probs=None, action_probs=None, entropies=None, advantages=None, rewards=None, values=None ):
                if train:
                        loss = custom_loss( action_probs, probs, entropies, advantages, rewards, values )
                        self.add_loss( loss )
                else:
                        self.add_loss( tf.zeros( 1 ) )
                return self.prediction( self.dense2( self.dense1( inputs ) ) )


def custom_loss( action_probs, probs, entropies, advantages, rewards, values ):
        ratio = tf.math.exp( action_probs - probs )
        ratio = tf.compat.v1.Print( ratio, [ratio], "ratio" )
        p1 = ratio * advantages
        p1 = tf.compat.v1.Print( p1, [p1], "p1" )
        p2 = K.clip( ratio, min_value=1-CLIPPING_VALUE, max_value=1+CLIPPING_VALUE) * advantages
        p2 = tf.compat.v1.Print( p2, [p2], "p2" )
        actor_loss = -K.mean( K.minimum(p1, p2) )
        actor_loss = tf.compat.v1.Print( actor_loss, [actor_loss], "actor_loss" )
        critic_loss = K.mean( K.square( rewards-values ) )
        critic_loss = tf.compat.v1.Print( critic_loss, [critic_loss], "critic_loss" )
        entropy = tf.math.reduce_mean( entropies )
        total_loss = CRITIC_DISCOUNT*critic_loss + actor_loss - ENTROPY_BETA * entropy 
        total_loss = tf.compat.v1.Print( total_loss, [total_loss], "total_loss" )
        return total_loss


def get_model_actor( input_dims, output_dims ):
        inputs = layers.Input( shape=input_dims )
        probs = layers.Input( shape=( output_dims, ) )
        action_probs = layers.Input( shape=( output_dims, ) )
        entropies = layers.Input( shape=( 1, 1, ) )
        advantages = layers.Input( shape=(1, 1, ) )
        rewards = layers.Input( shape=(1, 1, ) )
        values = layers.Input( shape=(1, 1, ) )

        x = layers.Dense( 512, activation='relu', name='input_layer' )( inputs )
        hidden = layers.Dense( 512, activation='relu', name='hidden' )( x )
        mu = layers.Dense( 2*output_dims, name='output' )( hidden )
        #sigma = layers.Dense( output_dims, activation='relu', name='sigma' )( x )

        model = keras.Model( inputs=[inputs, action_probs, probs, entropies, advantages, rewards, values], outputs=[mu], name='actor_model' )
        adam_opt = Adam( learning_rate=1e-2 )
        model.compile( optimizer=adam_opt, loss=[
            ppo_loss(
                action_probs=action_probs,
                probs=probs,
                entropies=entropies,
                advantages=advantages,
                rewards=rewards,
                values=values)
            ],
        )
        #model.summary()

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
        return np.array( returns ), (adv - np.mean(adv))/(np.std(adv) + 1e-10)

def ppo_loss( action_probs, probs, entropies, advantages, rewards, values ):
        def loss( y_true, y_pred ):
                #newpolicy = tf.compat.v1.Print( newpolicy, [newpolicy.shape], "newpolicy" )
                #newpolicy = tf.compat.v1.Print( newpolicy, [newpolicy], "newpolicy" )
                #ratio = layers.Subtract()( [newpolicy, probs] )
                #ratio = tf.compat.v1.Print( ratio, [newpolicy], "ratio" )

                ratio = tf.math.exp( action_probs - probs )
                ratio = tf.compat.v1.Print( ratio, [ratio], "ratio" )
                p1 = ratio * advantages
                p1 = tf.compat.v1.Print( p1, [p1], "p1" )
                p2 = K.clip( ratio, min_value=1-CLIPPING_VALUE, max_value=1+CLIPPING_VALUE) * advantages
                p2 = tf.compat.v1.Print( p2, [p2], "p2" )
                actor_loss = -K.mean( K.minimum(p1, p2) )
                actor_loss = tf.compat.v1.Print( actor_loss, [actor_loss], "actor_loss" )
                critic_loss = K.mean( K.square( rewards-values ) )
                critic_loss = tf.compat.v1.Print( critic_loss, [critic_loss], "critic_loss" )
                entropy = tf.math.reduce_mean( entropies )
                total_loss = CRITIC_DISCOUNT*critic_loss + actor_loss - ENTROPY_BETA * entropy 
                total_loss = tf.compat.v1.Print( total_loss, [total_loss], "total_loss" )
                return total_loss
        return loss


def reward_function( distance ):
        return 1 - ( distance / INITIAL_DISTANCE ) ** 0.5

def ppo_loop():
        """
        rospy.Subscriber( "/joint_states", JointState, joint_position_interrupt )
        rospy.Subscriber( "/gazebo/link_states", LinkStates, link_position_interrupt )
        #states, actions, values, masks, rewards = [], [], [], [], []
        arm_controller = ArmController()
        utils.delete_model()
        arm_controller.send_arm_goal( INITIAL_ARM_POSITION )
        utils.spawn_model()
        """

        #model_actor = get_model_actor( INPUT_DIMS, OUTPUT_DIMS )
        model_actor = ActorModel( OUTPUT_DIMS )
        adam_opt = Adam( learning_rate=1e-2 )
        model_actor.compile( optimizer=adam_opt )
        #print( model_actor.summary() )
        model_critic = get_model_critic( INPUT_DIMS )

        #ppo_use_models( model_actor, model_critic, arm_controller )
        ppo_use_models( model_actor, model_critic )

def ppo_use_models( model_actor, model_critic, arm_controller=None ):
        DUMMY_N = np.zeros(( 1, OUTPUT_DIMS ))
        DUMMY_1 = np.zeros((1, 1, 1))
        #current_state = utils.get_joint_data( joint_data )
        current_state = np.ones( 7 ) 
        for epoch in range( NUM_EPOCHS ):
                states, actions, entropies, action_probs, values, masks, rewards, probs = [], [], [], [], [], [], [], []
                for it in range( PPO_STEPS ):
                        current_state_tensor = current_state.reshape((1, NUM_JOINTS) )
                        #print( current_state.shape, current_state_tensor.shape )
                        #prediction = model_actor.predict( [current_state_tensor, DUMMY_N, DUMMY_N, DUMMY_1, DUMMY_1, DUMMY_1, DUMMY_1], batch_size=1 )
                        prediction = model_actor.predict( current_state_tensor )
                        print( 'PREDICTION', prediction.shape )
                        mu, sigma = prediction[:,:OUTPUT_DIMS], tf.math.abs( prediction[:,OUTPUT_DIMS:] )
                        sigma = sigma + 1e-4
                        dist = tfd.Normal( loc=mu, scale=sigma )
                        entropy = dist.entropy()[0]
                        action = dist.sample()
                        action_prob = dist.log_prob( action )[0]
                        prob = dist.log_prob( mu )[0]
                        with tf.compat.v1.Session() as sess:
                                print( "action prob", action_prob.eval(), action_prob.shape )
                                print( "prob", prob.eval(), prob.shape )
                        #print( "mu", mu )
                        #print( "sigma", sigma )
                        q_value = model_critic.predict( current_state_tensor, batch_size=1 )
                        print( "epoch:", epoch, "Step:", it )
                        print( "action:", action )
                        #print( "q_value:", q_value )


                        # MOVE the ARM
                        #move_result = arm_controller.send_arm_goal( action[0] )
                        move_result = 0

                        # UPDATES
                        #new_state_grip_pos = utils.get_gripper_pos( link_data )
                        new_state_grip_pos = (1,1,1)
                        #new_state = utils.get_joint_data( joint_data )
                        new_state = np.ones( 7 )
                        if move_result == 0:
                                distance, done = distance_from_goal( new_state_grip_pos, GRIPPER_GOAL )
                                reward = reward_function( distance )
                        else:
                                reward, done = CONTACT_REWARD, True
                        mask = not done
                        print( "reward:", reward, " done:", done ) 

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
                                pass
                        """
                                rospy.loginfo( "Resetting" )
                                #rospy.loginfo( "Deleting Box ..." )
                                utils.delete_model()
                                #rospy.loginfo( "Resettin Arm ..." )
                                arm_controller.send_arm_goal( [0, 0, 0, 0, 0, 0, 0] )
                                utils.reset_world()
                                arm_controller.send_arm_goal( INITIAL_ARM_POSITION )
                                utils.spawn_model()
                                current_state = np.array( INITIAL_ARM_POSITION )
                        """

                                

                q_value = model_critic.predict( current_state_tensor, batch_size=1 )
                values.append( q_value )
                returns, advantages = get_advantages( values, masks, rewards )
                print( returns.shape, advantages.dtype )
                action_probs = tf.stack( action_probs )
                probs = tf.stack( probs )
                entropies = tf.math.reduce_sum( tf.stack( entropies ), axis=1, keepdims=True )
                entropies = tf.reshape( entropies, [PPO_STEPS, 1, 1] )
                advantages = advantages.reshape( (-1, 1, 1 ) ).astype( 'float32' )
                values = np.array(values[:-1] ).reshape( (-1, 1, 1 ) ).astype( 'float32' )
                rewards = np.array( rewards ).reshape( (-1, 1, 1 ) ).astype( 'float32' )

                model_actor( tf.zeros( [PPO_STEPS, NUM_JOINTS] ), train=True, probs=probs, action_probs=action_probs, entropies=entropies, advantages=advantages, rewards=rewards, values=values )
                """
                print( entropies.shape )
                ratio = tf.math.exp( action_probs - probs )
                print( "RATIO" )
                p1 = ratio * advantages
                print( "P1" )
                p2 = K.clip( ratio, min_value=1-CLIPPING_VALUE, max_value=1+CLIPPING_VALUE) * advantages
                print( "P2" )
                actor_loss = -K.mean( K.minimum(p1, p2) )
                print( "ACTOR LOSS" )
                critic_loss = K.mean( K.square( rewards-values ) )
                print( "CRITIC LOSS" )
                entropy = tf.math.reduce_mean( entropies )
                print( "ENTROPY" )
                temp = ENTROPY_BETA * entropy
                print( temp.dtype )
                temp = CRITIC_DISCOUNT * critic_loss
                print( critic_loss.dtype )
                print( actor_loss.dtype )
                total_loss = CRITIC_DISCOUNT*critic_loss + actor_loss - ENTROPY_BETA * entropy 
                with tf.compat.v1.Session() as sess:
                        print( "TOTAL LOSS", total_loss.eval() )
                """
                print( "TRAINING ACTOR" )
                actor_loss = model_actor.fit( tf.zeros( [PPO_STEPS, NUM_JOINTS] ), tf.zeros( [PPO_STEPS, OUTPUT_DIMS] ), verbose=True, shuffle=False, epochs=10, steps_per_epoch=1 )
                """
                actor_loss = model_actor.fit(
                        [np.reshape(states, (-1, NUM_JOINTS)), action_probs, probs, entropies,
                            advantages, rewards, values],
                        [tf.zeros([PPO_STEPS, 14])], verbose=True, shuffle=True, epochs=10, steps_per_epoch=1#, callbacks=[tensor_board]
                )
                """
                print( "TRAINING CRITIC" )
                critic_loss = model_critic.fit([np.reshape(states,(-1,NUM_JOINTS))], [np.reshape(returns, newshape=(-1, 1))], shuffle=True, epochs=10, verbose=True )


if __name__ == "__main__":
        try:
                #rospy.init_node( 'arm_train', anonymous=False )
                ppo_loop()
        except rospy.ROSInterruptException:
                print( "SIGKILL" )
