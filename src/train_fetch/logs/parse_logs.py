import sys
import pandas
import argparse


def parse_rewards( f ):
    all_total_rewards = []
    all_mean_rewards = []
    current_epoch = 0
    current_rewards = []
    current_max = -1000
    last_iter = 0
    for line in f:
        if line.startswith( "epoch" ):
            s = line.split()
            epoch, iteration = int( s[1][:-1] ), int( s[3] )
            last_iter = iteration
            if epoch == current_epoch:
                while not line.lstrip().startswith( "reward" ):
                    line = next(f)
                reward = float( line.lstrip().split()[1][:-1] )
                current_max = max( current_max, reward )
                current_rewards.append( reward )
            else:
                #print( f"HERE {epoch} {current_epoch}" )
                total = sum( current_rewards )
                mean = sum( current_rewards ) / len( current_rewards )
                all_total_rewards.append( total )
                all_mean_rewards.append( mean )
                while not line.lstrip().startswith( "reward" ):
                    line = next(f)
                reward = float( line.lstrip().split()[1][:-1] )
                current_max = max( current_max, reward )
                current_rewards = [ reward ]
            current_epoch = epoch
    if current_rewards:
        total = sum( current_rewards )
        mean = sum( current_rewards ) / len( current_rewards )
        all_total_rewards.append( total )
        all_mean_rewards.append( mean )

    return all_total_rewards, all_mean_rewards, current_max, last_iter

"""
epoch: 0, step: 0
	action: tensor([[ 0.0356, -0.0631,  0.0705,  0.0578,  0.0161,  0.0037, -0.0290]],
       dtype=torch.float64)
	q_value: tensor([[-0.0906]], dtype=torch.float64, grad_fn=<AddmmBackward0>)
	reward: 0.338445123740281, distance: 0.9307641144358516 entropy -4.612273175343211
LOSS epoch 0 actor 0.14302722137766102 critic 11.563734072513316
"""
def create_df( f ):
    current_epoch = -1
    avg_q_values = []
    avg_rewards = []
    total_rewards = []
    avg_entropy = []
    epoch_max = []
    hit_goals = []
    loss_actor = []
    loss_critic = []

    q_values = []
    rewards = []
    entropies = []
    for line in f:
        if line.startswith( "epoch" ):
            epoch = int( line.split()[1][:-1] )
            if epoch != current_epoch:
                if current_epoch != -1:
                    avg_q_values.append( sum( q_values ) / len( q_values ) )
                    avg_rewards.append( sum( rewards ) / len( rewards ) )
                    avg_entropy.append( sum( entropies ) / len( entropies ) )
                    total_rewards.append( sum( rewards ) )
                current_epoch = epoch
                q_values = []
                rewards = []
                entropies = []
                epoch_max.append(-1000)
                hit_goals.append(0)
            next( f ) # action
            next( f ) # action
            q_string = next(f)
            first, last = 0, 0
            for i in range(len(q_string)):
                if q_string[i] == '[':
                   first = i
                elif q_string[i] == ']':
                    last = i
                    break
            q = float( q_string[first+2: last] )
            reward = next(f).lstrip()
            if reward.startswith( "COLL" ):
                reward = next(f).lstrip()
            reward = reward.split()
            r_value, entropy = float( reward[1][:-1] ), float( reward[-1] )
            q_values.append( q )
            rewards.append( r_value )
            entropies.append( entropy )

            if r_value == 100:
                hit_goals[epoch] += 1
            epoch_max[epoch] = max( epoch_max[epoch], r_value )

        elif line.startswith( "LOSS" ):
            loss = line.split()
            actor, critic = float( loss[4] ), float( loss[6] )
            loss_actor.append( actor )
            loss_critic.append( critic )
    if len( epoch_max ) > len( total_rewards ):
        epoch_max = epoch_max[:-1]
        hit_goals = hit_goals[:-1]
    dic = {
            "epoch": list( range( 0, len(avg_q_values) ) ),
            "total rewards" : total_rewards,
            "avg rewards" : avg_rewards,
            "avg q value" : avg_q_values,
            "avg entropy" : avg_entropy,
            "max reward" : epoch_max,
            "hit goal" : hit_goals,
            "actor loss" : loss_actor,
            "critic loss" : loss_critic
    }
    return pandas.DataFrame.from_dict( dic )


                

    

if __name__ == "__main__":
    parser = argparse.ArgumentParser( )
    parser.add_argument( 'filename', type=str )
    parser.add_argument( '-s', '--stats', action='store_true' )
    parser.add_argument( '-df', '--dataframe', action='store_true' )
    parser.add_argument( '-p', '--print', action='store_true' )
    args = parser.parse_args()
    filename = args.filename 
    if args.stats:
        with open( filename ) as f:
            totals, means, reward, last_iter = parse_rewards( f )
        print( "TOTALS" )
        print( totals )
        print( "MEANS" )
        print( means )
        print( "epochs:", len( totals ), "max reward:", reward, "step:", last_iter )
    if args.print:
        with open( filename ) as f:
            df = create_df( f )
        print( df )
    if args.dataframe:
        with open( filename ) as f:
            df = create_df( f )
        df.to_pickle( filename[:-4]+".p" )

            
