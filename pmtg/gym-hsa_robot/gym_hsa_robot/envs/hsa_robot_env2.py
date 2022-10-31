import gym
import numpy as np
import pybullet as p
import math
from gym_hsa_robot.resources.plane import Plane
from gym_hsa_robot.resources.hsa_robot import HSARobot


class HSARobot_Env(gym.Env):
    
    def __init__(self):
        
        
        self.robot = None
        
        self.reset()
    
    def step(self, action):
        # Feed action to the car and get observation of car's state
        self.robot.apply_action(action)
        p.stepSimulation()
        robot_ob = self.robot.get_observation()

        # Compute reward as L2 change in distance to goal
        dist_to_goal = math.sqrt(((car_ob[0] - self.goal[0]) ** 2 +
                                  (car_ob[1] - self.goal[1]) ** 2))
        reward = max(self.prev_dist_to_goal - dist_to_goal, 0)
        self.prev_dist_to_goal = dist_to_goal

        # Done by running off boundaries
        if (car_ob[0] >= 10 or car_ob[0] <= -10 or
                car_ob[1] >= 10 or car_ob[1] <= -10):
            self.done = True
        # Done by reaching goal
        elif dist_to_goal < 1:
            self.done = True
            reward = 50

        ob = np.array(car_ob + self.goal, dtype=np.float32)
        return ob, reward, self.done, dict()
    
    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0, 0, -10)
        
        # Reload the plane and robot
        Plane(self.client)
        self.robot = Robot(self.client)


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
    
    
    