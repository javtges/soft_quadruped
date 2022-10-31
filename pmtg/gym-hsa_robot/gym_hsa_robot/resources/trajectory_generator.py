import gym
import numpy as np
import pybullet as p
import time
from gym_hsa_robot.resources.hsa_robot import HSARobot

def rotate(l, n):
    return l[n:] + l[:n]


def make_traj(offset):
    offset = offset*-1
    test = np.linspace(0, 1, 240)
    # print(test)
    eps_list = []
    theta_list = []

    for t in test:

        # eps = -0.05 + 0.02/ np.sin(2.0 * np.arctan( 1/(np.tan(np.pi * t) - 1) - (np.sin(np.pi * t) / (np.sin(np.pi * t) - np.cos( np.pi * t )))    ))
        # theta = 2 * np.arctan( (np.tan(np.pi*t) - 1) / (np.tan(np.pi*t) + 1) )
        
        
        eps = 0.02 * np.cos(2*np.pi*t)
        theta = 0.02 * np.sin(2*np.pi*t)
        eps_list.append(eps)
        theta_list.append(theta)
        
    eps_list = rotate(eps_list, offset)
    theta_list = rotate(theta_list, offset)
    
    # print(eps_list, theta_list)
        
    return eps_list, theta_list


make_traj(0)