
import torch
import numpy as np

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

def reward_function( distance ):
                return 1- ((distance / INITIAL_DISTANCE) **2 )
