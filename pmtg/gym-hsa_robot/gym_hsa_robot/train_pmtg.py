"""Augmented Random Search with PMTG
    Credit for ARS Algorithm: https://github.com/vinits5/augmented-random-search and https://github.com/modestyachts/ARS
"""

import gym
import numpy as np
import argparse
import gym_hsa_robot
from gym import wrappers
import os
from tensorboardX import SummaryWriter

#################### Environment and Agent ####################
def create_env(env_name):
    env = gym.make(env_name)
    return env

def policy(state, weights):
    return np.matmul(weights, state.reshape(-1,1))

def test_env(env, policy, weights, normalizer=None, eval_policy=False):
    # Argument:
        # env:			Object of the gym environment.
        # policy:		A function that will take weights, state and returns actions
    state = env.reset()
    done = False
    total_reward = 0.0
    total_states = []
    steps = 0

    # Here, inject PMTG
    while not done and steps<5000:
        if normalizer:
            if not eval_policy: normalizer.observe(state)
            state = normalizer.normalize(state)
        action = policy(state, weights)
        next_state, reward, done, _ = env.step(action)

        # Trick to avoid local optima.
        if abs(next_state[2]) < 0.001:
            reward = -100
            done = True

        total_states.append(state)
        total_reward += reward
        steps += 1
        state = next_state
    if eval_policy: return float(total_reward), steps
    else: return float(total_reward)

#################### ARS algorithm ####################
def sort_directions(data, b):
    reward_p, reward_n = data
    reward_max = []
    for rp, rn in zip(reward_p, reward_n):
        reward_max.append(max(rp, rn))

    # ipdb.set_trace()
    idx = np.argsort(reward_max)	# Sort rewards and get indices.
    idx = np.flip(idx)				# Flip to get descending order.

    return idx

def update_weights(data, lr, b, weights):
    reward_p, reward_n, delta = data
    idx = sort_directions([reward_p, reward_n], b)

    step = np.zeros(weights.shape)
    for i in range(b):
        step += [reward_p[idx[i]] - reward_n[idx[i]]]*delta[idx[i]]

    sigmaR = np.std(np.array(reward_p)[idx][:b] + np.array(reward_n)[idx][:b])
    weights += (lr*1.0)/(b*sigmaR*1.0)*step

    return weights

def sample_delta_normal(size):
    return np.random.normal(size=size)

def sample_delta(size):
    return np.random.randn(*size)

#################### Normalizing the states #################### 
class Normalizer():
    def __init__(self, nb_inputs):
        self.n = np.zeros(nb_inputs)
        self.mean = np.zeros(nb_inputs)
        self.mean_diff = np.zeros(nb_inputs)
        self.var = np.zeros(nb_inputs)

    def observe(self, x):
        self.n += 1.
        last_mean = self.mean.copy()
        self.mean += (x - self.mean) / self.n
        self.mean_diff += (x - last_mean) * (x - self.mean)
        self.var = (self.mean_diff / self.n).clip(min=1e-2)

    def normalize(self, inputs):
        obs_mean = self.mean
        obs_std = np.sqrt(self.var)
        return (inputs - obs_mean) / obs_std

    def store(self, path):
        np.savetxt(os.path.join(path, 'mean.txt'), self.mean)
        np.savetxt(os.path.join(path, 'var.txt'), self.var)

#################### Training ARS Class #################### 
class ARS:
    def __init__(self, args):
        self.v = args.v
        self.N = args.N
        self.b = args.b
        self.lr = args.lr
        self.args = args

        if not os.path.exists(args.log): 
            os.mkdir(args.log)
            os.mkdir(os.path.join(args.log, 'models'))
            os.mkdir(os.path.join(args.log, 'videos'))

        self.env = create_env(args.env)

        self.size = [self.env.action_space.shape[0], self.env.observation_space.shape[0]]
        self.weights = np.zeros(self.size)
        
        # Have to figure out this normalizer stuff
        if args.normalizer: self.normalizer = Normalizer([1,self.size[1]])
        else: self.normalizer=None

    def save_policy(self, counter):
        path = os.path.join(self.args.log, 'models', 'policy'+str(counter))
        if not os.path.exists(path): os.mkdir(path)
        np.savetxt(os.path.join(path, 'weights.txt'), self.weights)
        self.normalizer.store(path)

    def train_one_epoch(self):
        delta = [sample_delta(self.size) for _ in range(self.N)]

        reward_p = [test_env(self.env, policy, self.weights + self.v*x, normalizer=self.normalizer) for x in delta]
        reward_n = [test_env(self.env, policy, self.weights - self.v*x, normalizer=self.normalizer) for x in delta]
        
        return update_weights([reward_p, reward_n, delta], self.lr, self.b, self.weights)

    def train(self):
        writer = SummaryWriter(self.args.log)
        print('Training Begins!')
        counter = 0
        self.env.reset()
    

        while counter < 100:
            print('Counter: {}'.format(counter))
            self.weights = self.train_one_epoch()

            test_reward, num_plays = test_env(self.env, policy, self.weights, normalizer=self.normalizer, eval_policy=True)
            self.save_policy(counter)
            writer.add_scalar('test_reward', test_reward, counter)
            writer.add_scalar('episodic_steps', num_plays, counter)
            print('Iteration: {} and Reward: {}'.format(counter, test_reward))
            counter += 1

        counter = 0
        while True:
            print(test_env(self.env, policy, self.weights, normalizer=self.normalizer))
            self.env.render()
            counter += 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='ARS Parameters')
    parser.add_argument('--v', type=float, default=0.03, help='noise in delta')
    parser.add_argument('--N', type=int, default=16, help='No of perturbations')
    parser.add_argument('--b', type=int, default=16, help='No of top performing directions')
    parser.add_argument('--lr', type=float, default=0.02, help='Learning Rate')
    parser.add_argument('--normalizer', type=bool, default=True, help='use normalizer')
    parser.add_argument('--env', type=str, default='hsa_robot-v0', help='name of environment')
    parser.add_argument('--log', type=str, default='exp_hsa_test', help='Log folder to store videos')

    args = parser.parse_args()
    ars = ARS(args)
    ars.train()