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

import numpy as np

import utils

ACTOR_FILE = "/home/simulator/Robots/src/train_fetch/saved_models/0307-1843/actor5"
CRITIC_FILE = "/home/simulator/Robots/src/train_fetch/saved_models/0307-1843/critic5"
"""
ACTOR_FILE = None
"""

ACTOR_LR = 5e-3
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


def link_position_interrupt(data):
    global link_data
    link_data = data


def joint_position_interrupt(data):
    global joint_data
    joint_data = data


class ActorModel(torch.nn.Module):
    def __init__(self, input_dims, output_dims, hidden_dims=1024):
        super(ActorModel, self).__init__()
        self.linear1 = torch.nn.Linear(input_dims, hidden_dims)
        self.linear2 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.linear3 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.linear4 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.linear5 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.linear6 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.mu = torch.nn.Linear(hidden_dims, output_dims)
        self.sigma = torch.nn.Linear(hidden_dims, output_dims)

    def forward(self, x):
        mid = F.relu(self.linear1(x))
        mid = F.relu(self.linear2(mid))
        mid = F.relu(self.linear3(mid))
        mid = F.relu(self.linear4(mid))
        mid = F.relu(self.linear5(mid))
        mid = F.relu(self.linear6(mid))
        mean = torch.clamp(self.mu(mid), min=-6.28, max=6.28)
        sigma = torch.clamp(self.sigma(mid), min=1e-10)
        return mean, sigma

    def getDistribution(self, x):
        mu, sigma = self.forward(x)
        dist = torch.distributions.Normal(mu, sigma)
        return dist


class CriticModel(torch.nn.Module):
    def __init__(self, input_dims, hidden_dims=100):
        super(CriticModel, self).__init__()
        self.linear1 = torch.nn.Linear(input_dims, hidden_dims)
        self.linear2 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.linear3 = torch.nn.Linear(hidden_dims, hidden_dims)
        self.outputLinear = torch.nn.Linear(hidden_dims, 1)

    def forward(self, x):
        return self.outputLinear(
            F.relu(self.linear3(F.relu(self.linear2(F.relu(self.linear1(x))))))
        )


def distance_from_goal(current, goal, tol=1e-1):
    d = distance(current, goal)
    return d, abs(d) < tol


def distance(x, y):
    return sum((a - b) ** 2 for a, b in zip(x, y)) ** 0.5


def get_advantages(values, masks, rewards):
    returns = []
    gae = 0
    for i in reversed(range(len(rewards))):
        delta = rewards[i] + GAMMA * values[i + 1] * masks[i] - values[i]
        gae = delta + GAMMA * LAMBDA * masks[i] * gae
        # print( "gae", gae + values[i] )
        returns.insert(0, gae + values[i])
    # print( "returns", returns, "values", values[:-1] )
    adv = torch.tensor(returns) - torch.tensor(values[:-1])
    return torch.tensor(returns), (adv - torch.mean(adv)) / (torch.std(adv) + 1e-10)


def ppo_loss(action_probs, probs, entropies, advantages, rewards, values):
    ratio = torch.exp(action_probs - probs)
    advantages = advantages.reshape((-1, 1))
    # print( ratio.shape, action_probs.shape, advantages.shape )
    # ratio = torch.exp( action_probs.sum( 1, keepdim=True ) - probs.sum( 1, keepdim=True ) )
    p1 = ratio * advantages
    p2 = torch.clamp(ratio, 1 - CLIPPING_VALUE, 1 + CLIPPING_VALUE) * advantages
    actor_loss = -torch.mean(torch.min(p1, p2))
    critic_loss = torch.mean(torch.square(rewards - values))
    entropy = torch.mean(entropies)
    global ENTROPY_BETA
    total_loss = CRITIC_DISCOUNT * critic_loss + actor_loss
    ENTROPY_BETA = choose_beta(total_loss)
    total_loss = total_loss - ENTROPY_BETA * entropy
    return total_loss


def choose_beta(raw_loss):
    if raw_loss > 500:
        return 100
    elif raw_loss > 200:
        return 50
    elif raw_loss > 100:
        return 5
    elif raw_loss > 10:
        return 0.1
    return 00.01


def reward_function(distance):
    return 1 - ((distance / INITIAL_DISTANCE) ** 2)


def ppo_loop():
    namespace = "/ns1"
    rospy.Subscriber(namespace + "/joint_states", JointState, joint_position_interrupt)
    rospy.Subscriber(
        namespace + "/gazebo/link_states", LinkStates, link_position_interrupt
    )
    rospy.wait_for_service(namespace + "/gazebo/set_model_state")
    move_model = rospy.ServiceProxy(
        namespace + "/gazebo/set_model_state", SetModelState
    )

    arm_controller = ArmController()

    utils.delete_model()
    arm_controller.send_arm_goal(INITIAL_ARM_POSITION)

    model_actor = ActorModel(NUM_JOINTS, OUTPUT_DIMS).double()
    model_critic = CriticModel(NUM_JOINTS).double()
    if ACTOR_FILE:
        model_actor.load_state_dict(torch.load(ACTOR_FILE))
        model_critic.load_state_dict(torch.load(CRITIC_FILE))

    current_state = utils.get_joint_data(joint_data)
    current_state_tensor = torch.tensor(current_state.reshape((1, NUM_JOINTS)))
    mu, sigma = model_actor(current_state_tensor)
    print("MU:", mu)
    print("SIGMA:", sigma)
    dist = model_actor.getDistribution(current_state_tensor)
    action = dist.sample()
    print("ACTION:", action[0])
    action_prob = dist.log_prob(action)
    prob = dist.log_prob(mu)
    entropy = dist.entropy()

    ACTION = [-6.28, -6.28, 5.7620, 6.18, 5.7698, -6.28, 6.18]
    print("ACTION", ACTION)
    # move_result = arm_controller.send_arm_goal( action[0], current_state )
    move_result = arm_controller.send_arm_goal(
        [-6.28, -6.28, 5.7620, 6.18, 5.7698, -6.28, 6.18], current_state
    )
    print("MOVE RESULT:", move_result)

    new_state_grip_pos = utils.get_gripper_pos(link_data)
    new_state = utils.get_joint_data(joint_data)

    distance = 0
    if move_result == 0:
        distance, done = distance_from_goal(new_state_grip_pos, GRIPPER_GOAL)
        pos_tol = [
            max(0.05, 0.2 * abs(x - y).item()) for x, y in zip(ACTION, current_state)
        ]
        print("TOLERANCES:", pos_tol)
        diff = [abs(x - y).item() for x, y in zip(ACTION, new_state)]
        print("DIFF:", diff)
        print("ANY:", any(x > y for x, y in zip(diff, pos_tol)))
        bad = any(x > y for x, y in zip(diff, pos_tol))
        if bad:
            reward = CONTACT_REWARD
            print("BAD")
        elif not done:
            reward = reward_function(distance)
            print("OK")
        else:
            reward = 100
    else:
        reward, done = CONTACT_REWARD, True
        pos_tol = [
            max(0.05, 0.2 * abs(x - y).item()) for x, y in zip(ACTION, current_state)
        ]
        print("TOLERANCES:", pos_tol)
        diff = [abs(x - y).item() for x, y in zip(ACTION, new_state)]
        print("DIFF:", diff)
        print("ANY:", any(x > y for x, y in zip(diff, pos_tol)))
        if move_result == 1 or any(x > y for x, y in zip(diff, pos_tol)):
            reward, done = CONTACT_REWARD, True
            print("NOT WITHIN TOL")
        else:
            distance, done = distance_from_goal(new_state_grip_pos, GRIPPER_GOAL)
            if not done:
                reward = reward_function(distance)
            else:
                reward = 100


if __name__ == "__main__":
    try:
        rospy.init_node("arm_train", anonymous=False)
        ppo_loop()
    except rospy.ROSInterruptException:
        print("SIGKILL")
