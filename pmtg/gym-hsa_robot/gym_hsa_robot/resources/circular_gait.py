""" This verifies that Gym sees our environment """

import gym
import numpy as np
import gym_hsa_robot
import time
import matplotlib.pyplot as plt
from gym_hsa_robot.resources.trajectory_generator import make_traj, make_circle, xy_legframe_to_joints, rotate
env = gym.make('hsa_robot-v0')
env.reset()

# # FL, FR, RL, RR
# ac_eps, ac_theta = make_traj(0)
# bd_eps, bd_theta = make_traj(120)

# print(ac_eps[0], ac_eps[120])
# print(bd_eps[0], bd_eps[120])

# print(ac_theta)

x,y = make_circle(0, -0.074, 0.02, 0.004, 240)
ac_eps = []
ac_theta = []
plt.scatter(x,y)
plt.show()

for idx, val in enumerate(x):
    # eps, theta = xy_legframe_to_joints(x[idx], y[idx])

    theta, eps = xy_legframe_to_joints(x[idx], y[idx])
    
    print(theta, eps)
    ac_eps.append(eps)
    ac_theta.append(theta)

# ac_theta, ac_eps = xy_legframe_to_joints(x, y)
# print(ac_theta, ac_eps)
    
bd_eps = rotate(ac_eps, 120)
bd_theta = rotate(ac_theta, 120)

# print(ac_eps, bd_eps)
tot_reward = 0

while True:
    for i in range(240):

        # fl_theta, fl_eps, fr_theta, fr_eps, rl_theta, rl_eps
        
        action = [0,ac_theta[i],ac_eps[i],bd_theta[i],bd_eps[i],bd_theta[i],bd_eps[i],ac_theta[i],ac_eps[i]]
        observation, reward, done, info = env.step(action)
        env.render()
        tot_reward += reward
        
        
        print("observation: ",observation)
        print("reward: ", reward)
        print("total reward: ", tot_reward)