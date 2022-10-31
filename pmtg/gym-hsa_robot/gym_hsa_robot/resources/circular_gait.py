""" This verifies that Gym sees our environment """

import gym
import numpy as np
import gym_hsa_robot
import time
from gym_hsa_robot.resources.trajectory_generator import make_traj
env = gym.make('hsa_robot-v0')
env.reset()

# A, B?, D?, C
ac_eps, ac_theta = make_traj(0)
bd_eps, bd_theta = make_traj(120)

print(ac_eps[0], ac_eps[120])
print(bd_eps[0], bd_eps[120])

print(ac_theta)

while True:
    for i in range(240):

        # Zero
        
        action = [0,ac_eps[i],ac_theta[i],bd_eps[i],bd_theta[i],bd_eps[i],bd_theta[i],ac_eps[i],ac_theta[i]]
        observation, reward, done, info = env.step(action)
        time.sleep(1/240)
    
    