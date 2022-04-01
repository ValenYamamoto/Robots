import torch
import numpy as np
import utils
import rospy


GAMMA = 0.99
LAMBDA = 0.95
CRITIC_DISCOUNT = 0.5
ENTROPY_BETA = 5
CLIPPING_VALUE = 0.2

INITIAL_DISTANCE = 1.144344254

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
                ratio = torch.exp( action_probs - probs.detach() )
                advantages = advantages.reshape( (-1, 1) )
                p1 = ratio * advantages
                p2 = torch.clamp( ratio, 1-CLIPPING_VALUE, 1+CLIPPING_VALUE ) * advantages
                actor_loss = -torch.min( p1, p2 ) 
                critic_loss = torch.mean( torch.square( rewards - values ) )
                #entropy = torch.mean( entropies )
                print( "ACTOR LOSS:", actor_loss )
                print( "CRITIC LOSS:", critic_loss )
                global ENTROPY_BETA
                total_loss = CRITIC_DISCOUNT*critic_loss + actor_loss
                ENTROPY_BETA = choose_beta( total_loss )
                total_loss = total_loss - ENTROPY_BETA * entropies
                return total_loss.mean()

def choose_beta( raw_loss ):
                raw_loss = torch.mean( raw_loss.detach() )
                if raw_loss > 500:
                                return 100
                elif raw_loss > 200:
                                return 50
                elif raw_loss > 100:
                                return 5
                elif raw_loss > 10:
                                return 0.1
                return 00.01

def check_valid_move( current_state, new_state, action ):
                pos_tol = [ max( 0.05, 0.2*abs(x-y).item() ) for x, y in zip( action[0], current_state ) ]
                diff = [ abs(x-y).item() for x, y in zip( action[0], new_state ) ]
                bad_move = any( x > y for x, y in zip( diff, pos_tol ) )
                if bad_move:
                            return 1
                return 0

def reward_function( distance, point, last_distance, is_collision, is_done, params, box_position=(0.75, 0, 0), box_size=0.5 ):
                distance_to_goal = distance_to_goal_reward_function( distance, params )
                distance_to_cube = distance_to_cube_reward_function( point, box_position, box_size, params['reward_constants']['box_threshold'] )
                distance_moved = distance_moved_reward_function( distance, last_distance )
                return params['reward_constants']['goal_distance_constant'] * distance_to_goal \
                        + params['reward_constants']['cube_distance_constant'] * min( distance_to_cube, 0 ) \
                        + params['reward_constants']['distance_moved_constant'] * distance_moved \
                        + params['reward_constants']['collision_constant'] * - max( 0, is_collision ) \
                        + params['reward_constants']['done_constant'] * max( 0, is_done )


def distance_to_goal_reward_function( distance, params ):
                return 1- ((distance / params['arm_constants']['initial_distance']) **2 )

def distance_to_cube_reward_function( point, box_position, box_size, threshold_distance ):
                x, y, z = [ box_position[i] + point[i] for i in range( len(point ) ) ]
                d = torch.sqrt( max( 0, x - box_size ) ** 2
                                + max( 0, y- box_size ) ** 2
                                + max( 0, z - box_size ) ** 2
                              )
                return (d / threshold_distance) ** 2 - 1

def distance_moved_reward_function( current_distance, last_distance ):
                return abs( current_distance - last_position )
                

def ppo_train_loop( arm_controller, model_actor, model_critic, params, joint_data, link_data, move_model, logfilename ):
                # set up loss function and optimizers
                critic_mse = torch.nn.MSELoss()
                actor_optimizer = torch.optim.Adam( model_actor.parameters(), lr=params['params']['actor_lr'] )
                critic_optimizer = torch.optim.Adam( model_critic.parameters(), lr=params['params']['critic_lr'] )

                # get current joint state
                current_state = utils.get_joint_data( joint_data[0] ) 


                # set up last distance from goal
                last_distance = params['arm_constants']['initial_distance']

                # begin training
                for epoch in range( params['params']['num_epochs'] ):
                                if epoch != 0 and epoch % 5 == 0: # save models
                                                torch.save( model_actor.state_dict(), f"/home/simulator/Robots/src/train_fetch/saved_models/actor{epoch}" )
                                                torch.save( model_critic.state_dict(), f"/home/simulator/Robots/src/train_fetch/saved_models/critic{epoch}" )

                                states, actions, entropies, action_probs, values, masks, rewards, old_probs = [], [], [], [], [], [], [], []
                                for it in range( params['params']['ppo_steps'] ):
                                                log_string = f"epoch: {epoch}, step: {it}\n"
                                                print( "epoch:", epoch, " step:", it )


                                                # get current position + get action
                                                current_state_tensor = torch.tensor( current_state.reshape( (1, params['params']['num_joints']) ) )
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
                                                new_state_grip_pos = utils.get_gripper_pos( link_data[0] )
                                                new_state = utils.get_joint_data( joint_data[0] )

                                                # Was it a safe move?
                                                bad_move = check_valid_move( current_state, new_state, action )
                                                move_result = move_result | bad_move

                                                # get updated distance + reward
                                                distance, done = utils.distance_from_goal( new_state_grip_pos, params['arm_constants']['gripper_goal'] )
                                                reward = reward_function( distance, point, last_distance, move_result, done, params, box_position=(0.75, 0, 0), box_size=0.5 ):
                                                if move_result != 0:
                                                                done = True
                                                                log_string += f"\tCOLLISION\n"
                                                                        

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
                                                last_distance = distance

                                                # if hit something or goal, reset
                                                if done:
                                                                rospy.loginfo( "Resetting" )
                                                                utils.reset_arm( arm_controller, params['arm_constants']['initial_arm_position'], move_model )
                                                                last_distance = params['arm_constants']['initial_distance']

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
                                entropies = torch.mean( torch.reshape( torch.stack( entropies, dim=1 ), (-1, params['params']['num_joints']) ), dim=1, keepdim=True )
                                actions = torch.reshape( torch.stack( actions, dim=1 ), (-1, params['params']['num_joints'] ) )
                                action_probs = torch.reshape( torch.stack( action_probs, dim=1 ), (-1, params['params']['num_joints'] ) )
                                old_probs = torch.reshape( torch.stack( old_probs, dim=1 ), (-1, params['params']['num_joints'] ) )
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
                                for _ in range( params['params']['steps_per_iter'] ):
                                                actor_optimizer.step()
                                                critic_optimizer.step()

                                if (epoch+1) % params['params']['test_freq'] == 0:
                                                print( "TESTING" )
                                                ppo_test( arm_controller, model_actor, model_critic, params, joint_data, link_data, move_model, logfilename )
                                if (epoch+1) % params['params']['decay_std_epochs'] == 0:
                                                model_actor.decayStdDev( params['params']['min_std_dev'] )


def ppo_test( arm_controller, model_actor, model_critic, params, joint_data, link_data, move_model, logfilename ):
                critic_mse = torch.nn.MSELoss()
                # reset arm
                utils.reset_arm( arm_controller, params['arm_constants']['initial_arm_position'], move_model )
                # get current joint state
                current_state = utils.get_joint_data( joint_data[0] ) 

                # begin test
                states, actions, entropies, action_probs, values, masks, rewards, old_probs = [], [], [], [], [], [], [], []
                for it in range( params['params']['ppo_steps'] ):
                                # get current position + get action
                                current_state_tensor = torch.tensor( current_state.reshape( (1, params['params']['num_joints']) ) )
                                action = model_actor( current_state_tensor )

                                # get Q value
                                q_value = model_critic( current_state_tensor )

                                # move arm
                                move_result = arm_controller.send_arm_goal( action[0], current_state )

                                # get updated position
                                new_state_grip_pos = utils.get_gripper_pos( link_data[0] )
                                new_state = utils.get_joint_data( joint_data[0] )
                                
                                # Was it a safe move?
                                distance = 0
                                bad_move = check_valid_move( current_state, new_state, action )
                                distance, done = utils.distance_from_goal( new_state_grip_pos, params['arm_constants']['gripper_goal'] )
                                move_result = move_result | bad_move
                                if move_result == 0:
                                                if not done:
                                                            reward = reward_function( distance, params )
                                                else:
                                                            reward = 100
                                else:
                                                reward,done = params['params']['contact_reward'], True
                                                        
                                mask = not done

                                states.append( current_state_tensor )
                                actions.append( action )
                                values.append( q_value )
                                masks.append( mask )
                                rewards.append( reward )
                                

                                current_state = new_state

                                # if hit something or goal, reset
                                if done:
                                                utils.reset_arm( arm_controller, params['arm_constants']['initial_arm_position'], move_model )


                # get last q_value
                q_value = model_critic( current_state_tensor )
                values.append( q_value )

                # calculate advantages
                returns, advantages = get_advantages( values, masks, rewards )


                # reshape everything to same size
                actions = torch.reshape( torch.stack( actions, dim=1 ), (-1, params['params']['num_joints'] ) )
                values = torch.reshape( torch.stack( values[:-1], dim=1 ), (-1, 1 ) )
                masks = torch.reshape( torch.tensor(masks), (-1, 1) )
                rewards = torch.reshape( torch.tensor(rewards), (-1, 1 ))

                # calculate loss for critic
                returns = torch.reshape( returns, (-1, 1 ) )
                critic_loss = critic_mse( values, returns )

                # write out loss info
                open_file = utils.get_logfile( logfilename )
                total_rewards = torch.sum( rewards )
                log_string = f'TEST rewards {total_rewards.item()} critic {critic_loss.item()} \n'
                open_file.write( log_string )
                open_file.close()
