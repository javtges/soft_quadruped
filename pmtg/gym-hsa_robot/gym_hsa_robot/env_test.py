import pybullet as p
from gym import wrappers
import gym
import numpy as np
import gym_hsa_robot


env = gym.make('hsa_robot-v0')
env.reset()
for _ in range(1000):
    env.render()
    env.step(env.action_space.sample()) # take a random action
    print(_)
env.close()