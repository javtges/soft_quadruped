'''
Quote from:
https://towardsdatascience.com/introduction-to-augmented-random-search-d8d7b55309bd

Let 𝛎 a positive constant < 1
Let 𝝰 be the learning rate
Let N the number of perturbations
Let 𝜃 a (p x n) matrix representing the parameters of the policy 𝜋
Let 𝜹i a (p x n) matrix representing the ith perturbation
1. While end condition not satisfied do:
2. Generate N perturbations 𝜹 from a normal distribution
3. Normalize 𝜋i+ = (𝜃+𝛎𝜹i)ᵀx and 𝜋i- = (𝜃-𝛎𝜹i)ᵀx for i = 1 to N
4. Generate 2N episodes and their 2N rewards using 𝜋i+ and 𝜋i- and collect the rewards ri+ and ri-
5. Sort all 𝜹 by max(ri+, ri-)
6. Update 𝜃 = 𝜃 + (𝝰/(b*𝞼ᵣ)) Σ(ri+ - ri-)𝜹i (where i = 1 to b)
7. End While


Policy must have:
Robot state (x,y,velocity)
TG Params (width, height)
Phase of each foot (perhaps?)
Return: TG Params, 8 parameters for leg modifications

    Dimensions: 6 x 10(?)

TG must have:
Input: width, height, timestep (opt)
Find ellipse with width, height, at certain timestep
Return: theta, eps (leg position)
'''


import argparse
from multiprocessing import Process, Pipe
import multiprocessing as mp
import time
import pybullet_envs
from gym import wrappers
import gym
import numpy as np
import os
import inspect
import gym_hsa_robot
currentdir = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

# Importing the libraries


'''
Need 4 classes here:

Hyperparameters class (opt)
Worker Class
Normalizer
ARS Class
'''

# Setting the Hyper Parameters


class Ellipse_TG():

    def __init__(self):
        self.cycle_length = 240
        self.phase = 0
        self.center_x = 0.0
        self.center_y = -0.07
        self.width = 0.02
        self.height = 0.02
        self.n_params = 2

    def rotate(l, n):
        return l[n:] + l[:n]

    def make_circle(self, x_center, y_center, r_x, r_y, n):

        x_cir = []
        y_cir = []
        for i in range(n):
            xx = r_x * np.sin(2*np.pi * i / n) + x_center
            x_cir.append(xx)

            yy = r_y * np.cos(2*np.pi * i / n) + y_center
            y_cir.append(yy)

        return x_cir, y_cir

    def make_traj(self, offset):
        offset = offset*-1
        test = np.linspace(0, 1, 240)
        eps_list = []
        theta_list = []

        for t in test:

            eps = 0.02 * np.cos(2*np.pi*t)
            theta = 0.02 * np.sin(2*np.pi*t)
            eps_list.append(eps)
            theta_list.append(theta)

        eps_list = rotate(eps_list, offset)
        theta_list = rotate(theta_list, offset)

        # print(eps_list, theta_list)

        return eps_list, theta_list

    def joints_to_xy_legframe(self, theta, eps):
        x = (0.05+eps)*np.sin(theta)
        y = -(0.05+eps)*np.cos(theta)
        return x, y

    def xy_legframe_to_joints(self, x, y):
        l = np.linalg.norm([x, y])
        # print("l", l)
        theta = np.arctan2(y, x) + 1.5707
        # print("theta", theta)

        eps = l - 0.07

        return theta, eps

    def legframe_to_footframe(self, x, y):
        x_foot = x
        y_foot = y+0.05
        return x_foot, y_foot

    def step_traj(self, width, height, n):
        '''
        Given: a width, height, find the (theta, eps) that makes sense at the given timestep
        '''
        x, y = make_circle(self, 0.0, -0.07, width, height, self.cycle_length)
        for idx, val in enumerate(x):
            eps, theta = xy_legframe_to_joints(x[idx], y[idx])

        ep_out = eps[self.phase]
        theta_out = theta[self.phase]
        self.phase += 1
        if self.phase == self.cycle_length:
            self.phase = 0

        return ep_out, theta_out


class Hp():

    def __init__(self):
        self.nb_steps = 10000
        self.episode_length = 1000
        self.learning_rate = 0.02
        self.nb_directions = 16
        self.nb_best_directions = 8
        assert self.nb_best_directions <= self.nb_directions
        self.noise = 0.03
        self.seed = 42
        self.env_name = 'hsa_robot-v0'


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


class Policy():

    def __init__(self, input_size, output_size, env_name, traj_generator, args):
        
        # self.tg_AC = Ellipse_TG()
        # self.tg_BC = Ellipse_TG()
        # self.tg_BC.phase = 120 # Set the TG's phase to 120, 180 degrees out of phase of the other two legs
        
        try:
            self.theta = np.load(args.policy)
        except:
            self.theta = np.zeros((output_size, input_size))
        self.env_name = env_name
        print("Starting policy theta=", self.theta)
        print("Policy size=", self.theta.shape)

    def evaluate(self, input, delta, direction, hp):
        if direction is None:
            return np.clip(self.theta.dot(input), -1.0, 1.0)
        elif direction == "positive":
            return np.clip((self.theta + hp.noise * delta).dot(input), -1.0, 1.0)
        else:
            return np.clip((self.theta - hp.noise * delta).dot(input), -1.0, 1.0)

    def sample_deltas(self):
        return [np.random.randn(*self.theta.shape) for _ in range(hp.nb_directions)]

    def update(self, rollouts, sigma_r, args):
        step = np.zeros(self.theta.shape)
        for r_pos, r_neg, d in rollouts:
            step += (r_pos - r_neg) * d
        self.theta += hp.learning_rate / \
            (hp.nb_best_directions * sigma_r) * step
        timestr = time.strftime("%Y%m%d-%H%M%S")

        # np.save(args.logdir + "/policy_" + self.env_name +
        # "_" + timestr + ".npy", self.theta)
        # print(self.theta, self.theta.shape)


def explore(env, normalizer, policy, direction, delta, hp, traj_generators):
    state = env.reset()
    done = False
    num_plays = 0.
    sum_rewards = 0
    while not done and num_plays < hp.episode_length:
        normalizer.observe(state) # Augment this to include the variables we need from the TG / residuals
        state = normalizer.normalize(state)
        action = policy.evaluate(state, delta, direction, hp)
        
        # Due to PMTG, our action now becomes... 9 + (x_val, y_val, width, height) * 4  = 25 dimensional
        
        # Change the variable "action" so that it's 9-dimensional (the shape of the environment's input), using the TG
        # Sample from all 4 trajectory generators, make an action from all of them

        # Here, generate the trajectory from the trajectory generator. Use the actions
        state, reward, done, _ = env.step(action)
        reward = max(min(reward, 1), -1)
        sum_rewards += reward
        num_plays += 1
    return sum_rewards


def train(env, policy, normalizer, hp, traj_generators, args):

    for step in range(hp.nb_steps):

        deltas = policy.sample_deltas()
        pos_rewards = [0] * hp.nb_directions
        neg_rewards = [0] * hp.nb_directions

        # Getting the positive rewards in the positive directions
        for k in range(hp.nb_directions):
            pos_rewards[k] = explore(
                env, normalizer, policy, "positive", deltas[k], hp, traj_generators)

            # Getting the negative rewards in the negative/opposite directions
        for k in range(hp.nb_directions):
            neg_rewards[k] = explore(
                env, normalizer, policy, "negative", deltas[k], hp, traj_generators)

        all_rewards = np.array(pos_rewards + neg_rewards)
        sigma_r = all_rewards.std()

        # Sorting the rollouts by the max(r_pos, r_neg) and selecting the best directions
        scores = {
            k: max(r_pos, r_neg)
            for k, (r_pos, r_neg) in enumerate(zip(pos_rewards, neg_rewards))
        }
        order = sorted(scores.keys(), key=lambda x: -
                       scores[x])[:hp.nb_best_directions]
        rollouts = [(pos_rewards[k], neg_rewards[k], deltas[k])
                    for k in order]

        # Updating our policy
        policy.update(rollouts, sigma_r, args)

        # Printing the final reward of the policy after the update
        reward_evaluation = explore(env, normalizer, policy, None, None, hp, traj_generator)
        print('Step:', step, 'Reward:', reward_evaluation)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--logdir', help='Directory root to log policy files (npy)', type=str, default='.')

    args = parser.parse_args()

    hp = Hp()
    TG_fl = Ellipse_TG()
    TG_fr = Ellipse_TG()
    TG_rl = Ellipse_TG()
    TG_rr = Ellipse_TG()
    
    tg_arr = [TG_fl,TG_fr,TG_rl,TG_rr]

    print("seed = ", hp.seed)
    np.random.seed(hp.seed)

    # make the environment
    env = gym.make("hsa_robot-v0")

    # number of inputs: number of columns
    # number of outputs: number of rows
    n_inputs = env.observation_space.shape[0] + TG_fl.n_params*4
    n_outputs = env.action_space.shape[0] + 8 + TG_fl.n_params*4
    
    print("Observation space =", n_inputs)
    print("Action space =", n_outputs)

    policy = Policy(input_size=n_inputs, output_size=n_outputs,
                    env_name=hp.env_name, traj_generator=tg_arr, args=args)

    normalizer = Normalizer(n_inputs)

    # Now, we start training

    train(env, policy, normalizer, hp, tg_arr, args)
