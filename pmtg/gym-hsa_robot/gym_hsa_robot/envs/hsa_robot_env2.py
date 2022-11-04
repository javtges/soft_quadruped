import gym
import numpy as np
import pybullet as p
import math
from gym_hsa_robot.resources.plane import Plane
from gym_hsa_robot.resources.hsa_robot import HSARobot
import matplotlib.pyplot as plt


class HSARobot_Env(gym.Env):
    
    metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 20 }

    
    def __init__(self):
        
        self.client = p.connect(p.GUI)
        # self.client = p.connect(p.DIRECT)
        p.setTimeStep(1/240, self.client)
        
        # Here, define my action space and my observation space
        
        self.action_space = gym.spaces.Box(np.array([-0.01, -0.02, -0.3, -0.02, -0.3, -0.02, -0.3, -0.02, -0.3]), np.array([0.01, 0.02, 0.3, 0.02, 0.3, 0.02, 0.3, 0.02, 0.3]))
        self.observation_space = gym.spaces.Box(np.array([-1000, -1000, -1, -1, -1000, -1000]), np.array([1000, 1000, 1, 1, 1000, 1000]))

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
        
        # reward = np.linalg.norm(robot_ob[-3:])
        reward = robot_ob[4]

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
    
    def render(self, mode='human'):
        if self.rendered_img is None:
            self.rendered_img = plt.imshow(np.zeros((100, 100, 4)))

        # Base information
        robot_id, client_id = self.robot.get_ids()
        proj_matrix = p.computeProjectionMatrixFOV(fov=80, aspect=1,
                                                   nearVal=0.01, farVal=100)
        pos, ori = [list(l) for l in
                    p.getBasePositionAndOrientation(robot_id, client_id)]
        pos[2] = 0.2

        # Rotate camera direction
        rot_mat = np.array(p.getMatrixFromQuaternion(ori)).reshape(3, 3)
        camera_vec = np.matmul(rot_mat, [1, 0, 0])
        up_vec = np.matmul(rot_mat, np.array([0, 0, 1]))
        # view_matrix = p.computeViewMatrix(pos, pos + camera_vec, up_vec)

        view_matrix = p.computeViewMatrix([0,1,0.5], [0,0,0], up_vec)

        # Display image
        frame = p.getCameraImage(100, 100, view_matrix, proj_matrix)[2]
        # frame = p.getCameraImage(100, 100)[2]
        frame = np.reshape(frame, (100, 100, 4))
        self.rendered_img.set_data(frame)
        plt.draw()
        plt.pause(.00001)
    
    def close(self):
        p.disconnect((self.client))
    
    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]
    
    
    