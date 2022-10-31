""" This verifies that Gym sees our environment """

import gym
import numpy as np
import gym_hsa_robot
import time
env = gym.make('hsa_robot-v0')
env.reset()

# A, B?, D?, C

while True:
    for i in range(24):

        # Zero
        
        action = [0,0,0,0,0,0,0,0,0]
        observation, reward, done, info = env.step(action)
        time.sleep(1/240)
    
    for i in range(24):

        # Expand ABCD
        
        action = [0,0,-0.02,0,-0.02,0,-0.02,0,-0.02]
        observation, reward, done, info = env.step(action)
        time.sleep(1/240)
        
    for i in range(24):
        
        # Bend AC
        
        action = [0,-0.3,-0.02,0,-0.02,0,-0.02,-0.3,-0.02]
        observation, reward, done, info = env.step(action)
        time.sleep(1/240)
        
    for i in range(24):
        
        # AC Still bent, contract BD
        
        action = [0,-0.3,-0.02,0,0.02,0,0.02,-0.3,-0.02]
        observation, reward, done, info = env.step(action)
        time.sleep(1/240)
        
    for i in range(24):

        # Expand ABCD
        
        action = [0,0,-0.02,0,-0.02,0,-0.02,0,-0.02]
        observation, reward, done, info = env.step(action)
        time.sleep(1/240)
        
    for i in range(24):

        # AC still expanded, contract BD
        
        action = [0,0,-0.02,0,0.02,0,0.02,0,-0.02]
        observation, reward, done, info = env.step(action)
        time.sleep(1/240)

    for i in range(24):
        
        # Bend BD
        
        action = [0,0,-0.02,-0.3,-0.02,-0.3,-0.02,0,-0.02]
        observation, reward, done, info = env.step(action)
        time.sleep(1/240)
