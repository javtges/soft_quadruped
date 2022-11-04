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


import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

# Importing the libraries
import os
import numpy as np
import gym
from gym import wrappers
import pybullet_envs
import time
import multiprocessing as mp
from multiprocessing import Process, Pipe
import argparse


'''
Need 4 classes here:

Hyperparameters class (opt)
Worker Class
Normalizer
ARS Class
'''

# Setting the Hyper Parameters
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

  def __init__(self, input_size, output_size, env_name, args):
    try:
      self.theta = np.load(args.policy)
    except:
      self.theta = np.zeros((output_size, input_size))
    self.env_name = env_name
    print("Starting policy theta=", self.theta)

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
    self.theta += hp.learning_rate / (hp.nb_best_directions * sigma_r) * step
    timestr = time.strftime("%Y%m%d-%H%M%S")
    np.save(args.logdir + "/policy_" + self.env_name + "_" + timestr + ".npy", self.theta)
    
    
    
    
    
    
    
if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
      '--logdir', help='Directory root to log policy files (npy)', type=str, default='.')
    
    args = parser.parse_args()
    
    hp = Hp()
    
    print("seed = ", hp.seed)
    np.random.seed(hp.seed)
    
    # Here, we now gotta:
    
    # make the environment
    
    
    n_inputs = env.observation_space.shape[0] # + TG.n_params()
    n_outputs = env.action_space.shape[0] # + TG.n_params()
    
    Policy(input_size=n_inputs, output_size=n_outputs, env_name=hp.env_name, *args)
    
    