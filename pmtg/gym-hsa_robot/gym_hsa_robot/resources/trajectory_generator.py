import gym
import numpy as np
import pybullet as p
import time
from gym_hsa_robot.resources.hsa_robot import HSARobot

def rotate(l, n):
    return l[n:] + l[:n]

def make_circle(x_center, y_center,r_x, r_y, n):

    x_cir = []
    y_cir = []
    for i in range(n):
        xx = r_x * np.sin(2*np.pi * i / n) + x_center
        x_cir.append(xx)
        
        yy = r_y * np.cos(2*np.pi * i / n) + y_center
        y_cir.append(yy)
        
    return x_cir, y_cir


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

def joints_to_xy_legframe(theta, eps):
    x = (0.05+eps)*np.sin(theta)
    y = -(0.05+eps)*np.cos(theta)
    return x, y

def xy_legframe_to_joints(x, y):
    l = np.linalg.norm([x,y])
    # print("l", l)
    theta = np.arctan2(y,x) + 1.5707
    # print("theta", theta)
        
    eps = l - 0.05
    
    return theta, eps
    
def legframe_to_footframe(x, y):
    x_foot = x
    y_foot = y+0.05
    return x_foot, y_foot

# make_traj(0)
# x,y = joints_to_xy_legframe(0.02, 0.02)
# print("we want 0.02, 0.02", xy_legframe_to_joints(x, y))


# x,y = joints_to_xy_legframe(0.0, 0.0)
# print("we want 0 0",xy_legframe_to_joints(x, y))

# x,y = joints_to_xy_legframe(0.02, 0.0)
# print("we want 0.02 0",xy_legframe_to_joints(x, y))

# x,y = joints_to_xy_legframe(0.0, 0.02)
# print("we want 0 0.2",xy_legframe_to_joints(x, y))

# x,y = joints_to_xy_legframe(0.0, -0.02)
# print("we want 0 -0.2",xy_legframe_to_joints(x, y))