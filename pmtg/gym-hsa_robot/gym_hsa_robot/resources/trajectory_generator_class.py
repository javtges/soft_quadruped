import gym
import numpy as np
import pybullet as p
import time
import gym_hsa_robot
from gym_hsa_robot.resources.hsa_robot import HSARobot
from gym_hsa_robot.resources.trajectory_generator import make_traj, make_circle, xy_legframe_to_joints, rotate

''' Contains the TG class.

'''


class Ellipse_TG():

    def __init__(self):
        '''
        Initializes the trajectory generator object.
        '''
        self.cycle_length = 240
        self.phase = 0
        self.center_x = 0.0
        self.center_y = -0.07
        self.width = 0.02
        self.height = 0.02
        self.n_params = 2

    def rotate(l, n):
        '''
        Rotates an array by the defined amount.
        '''
        return l[n:] + l[:n]

    def make_circle(self, x_center, y_center, r_x, r_y, n):
        '''
        Generates an ellipse.
        
        Inputs: center_x, center_y, width, height, and number of points
        Outputs: list of x values, list of y values
        '''
        x_cir = []
        y_cir = []
        for i in range(n):
            xx = r_x * np.sin(2*np.pi * i / n) + x_center
            x_cir.append(xx)

            yy = r_y * np.cos(2*np.pi * i / n) + y_center
            y_cir.append(yy)

        return x_cir, y_cir

    def make_traj(self, offset):
        '''
        Generates an ellipse.
        
        Inputs: center_x, center_y, width, height, and number of points
        Outputs: list of x values, list of y values
        '''
        
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
        '''
        Converts from joints to legframe.
        
        Inputs: theta, eps (rotation, extension)
        Outputs: x and y value in the legframe
        '''
        x = (0.07+eps)*np.sin(theta)
        y = -(0.07+eps)*np.cos(theta)
        return x, y

    def xy_legframe_to_joints(self, x, y):
        '''
        Converts from legframe xy to joints.
        
        Inputs: x and y value in the legframe
        Outputs: theta, eps (rotation, extension)
        '''
        l = np.linalg.norm([x, y], axis=0)
        # print("l", l.shape)
        theta = np.arctan2(y, x) + 1.5707
        # print("theta", theta.shape)

        eps = l - 0.07

        return theta, eps

    def legframe_to_footframe(self, x, y):
        '''
        Converts from legframe xy to footframe.
        
        Inputs: x and y value in the legframe
        Outputs: x and y value in the footframe
        '''
        
        x_foot = x
        y_foot = y+0.07
        return x_foot, y_foot

    def step_traj(self, width, height, res_x=0, res_y=0, step_theta=True, step_time=None):
        '''
        Given: a width, height, and set of residuals, find the (theta, eps) that makes sense at the given timestep.
        
        Provides for timestepping variable amounts, depending on what the policy dictates.
        '''
        x, y = self.make_circle(0.0, -0.07, width, height, self.cycle_length)
        x = np.asarray(x) #+ res_x
        y = np.asarray(y) #+ res_y
        
        
        # for idx, val in enumerate(x):
        #     eps, theta = self.xy_legframe_to_joints(x[idx], y[idx])
            
        theta, eps = self.xy_legframe_to_joints(x, y)

        
        # print("phase:", self.phase)
        
        if step_time:
            self.phase = int(step_time * self.cycle_length)
            
        if self.phase > self.cycle_length:
            self.phase = self.phase - self.cycle_length
        
        ep_out = eps[int(self.phase)] # here, we add residual
        theta_out = theta[int(self.phase)] # here we add residual
        # print(self.phase)
        
        if step_theta:
            self.phase += 1
            if self.phase == self.cycle_length:
                self.phase = 0

        return ep_out, theta_out
    
    
if __name__ == '__main__':
    '''
    Tests the traj_generators.
    
    '''
    env = gym.make('hsa_robot-v0')
    env.reset()
    
    fl = Ellipse_TG()
    
    fr = Ellipse_TG()
    fr.phase = fr.cycle_length/2
    
    rl = Ellipse_TG()
    rl.phase = rl.cycle_length/2
    
    rr = Ellipse_TG()
    traj_generators = [fl, fr, rl, rr]
    
    
    while True:
        for i in range(240):

            # print("leg1")
            eps_fl, theta_fl = traj_generators[0].step_traj(width=0.02, height=0.02)
            # print("leg2")
            eps_fr, theta_fr = traj_generators[1].step_traj(width=0.02, height=0.02)
            # print("leg3")
            eps_rl, theta_rl = traj_generators[2].step_traj(width=0.02, height=0.02)
            # print("leg4")
            eps_rr, theta_rr = traj_generators[3].step_traj(width=0.02, height=0.02)
            print(eps_fl, theta_fl)
            
            action = [0, theta_fl, eps_fl, theta_fr, eps_fr, theta_rl, eps_rl, theta_rr, eps_rr]
            observation, reward, done, info = env.step(action)
            # time.sleep(1/240)
            env.render()
            # plt.show(a)
            
            # print(observation)
            # print(traj_generators[0].phase)