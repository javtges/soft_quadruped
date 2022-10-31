import gym
import numpy as np
import pybullet as p
import math
from gym_hsa_robot.resources.plane import Plane
from gym_hsa_robot.resources.hsa_robot import HSARobot


class HSARobot_Env(gym.Env):
    
    metadata = {'render.modes': ['human']}

    
    def __init__(self):
        
        self.client = p.connect(p.GUI)
        p.setTimeStep(1/30, self.client)
        
        # Here, define my action space and my observation space
        
        self.np_random, _ = gym.utils.seeding.np_random()
        self.robot = None
        self.done = False
        self.rendered_img = None
        self.render_rot_matrix = None
        
        
        self.reset()
    
    def step(self, action):
        # Feed action to the robot and get observation of its state
        self.robot.apply_action(action)
        p.stepSimulation()
        robot_ob = self.robot.get_observation()
        
        reward = np.linalg.norm(robot_ob[-3:])

        # Done by running off boundaries

        # Done by reaching goal
        if reward > 0.1:
            self.done = True
            reward = 50

        ob = np.array(robot_ob, dtype=np.float32)
        return ob, reward, self.done, dict()
    
    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0, 0, -10)
        
        # Reload the plane and robot
        Plane(self.client)
        self.robot = HSARobot(self.client)


        # Get observation to return
        robot_ob = self.robot.get_observation()

        return np.array(robot_ob, dtype=np.float32)
    
    def render(self):
        pass
    
    def close(self):
        p.disconnect((self.client))
    
    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]
    
    
    