import gym
import numpy as np
import pybullet as p
import math
import time
from gym_hsa_robot.resources.plane import Plane
from gym_hsa_robot.resources.hsa_robot import HSARobot
import matplotlib.pyplot as plt

NUM_LEGS = 4
NUM_JOINTS = 8
RENDER_WIDTH = 960
RENDER_HEIGHT = 720


class HSARobot_Env(gym.Env):
    
    metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 20 }

    
    def __init__(self):
        
        # if render:
        #     self.client = p.connect(p.GUI)
        # else:
        #     self.client = p.connect(p.DIRECT)
        self.client = p.connect(p.GUI)
        p.setTimeStep(1/240, self.client)
        
        # Here, define my action space and my observation space
        
        self.action_space = gym.spaces.Box(np.array([-0.01, -0.3, -0.02, -0.3, -0.02, -0.3, -0.02, -0.3, -0.02]), np.array([0.01, 0.3, 0.02, 0.3, 0.03, 0.3, 0.02, 0.3, 0.02]))
        self.observation_space = gym.spaces.Box(np.array([-1000, -1000, -3.15, -3.15, -3.15, -1000, -1000]), np.array([1000, 1000, 3.15, 3.15, 3.15, 1000, 1000]))

        self.np_random, _ = gym.utils.seeding.np_random()
        self.robot = None
        self.done = False
        self.rendered_img = None
        self.render_rot_matrix = None
        self._last_frame_time = time.time()
        self._cam_dist = 1.0
        self._cam_yaw = 0
        self._cam_pitch = -30       
        self.prev_x = 0
        
        self.reset()
    
    def step(self, action):
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        # Feed action to the robot and get observation of its state
        self.robot.apply_action(action)
        p.stepSimulation()
        robot_ob = self.robot.get_observation()
        
        # reward = np.linalg.norm(robot_ob[-3:])
        
        # What this should do is measure the distance between the last step and this one
        reward = robot_ob[0] - self.prev_x
        self.prev_x = robot_ob[0]
        
        if abs(robot_ob[2]) > 1.57:
            self.done = True
            print("fell over!")
            reward = -50
        
        
        ob = np.array(robot_ob, dtype=np.float32)
        # print(ob)
        return ob, reward, self.done, dict()
    
    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0, 0, -10)
        
        # Reload the plane and robot
        Plane(self.client)
        self.robot = HSARobot(self.client)
        self.done = False # oops

        # Get observation to return
        robot_ob = self.robot.get_observation()
        self.prev_x = robot_ob[0]

        return np.array(robot_ob, dtype=np.float32)
    
    def render(self, mode='rgb_array'):
        
        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.7,0,0.05],
                                                            distance=.7,
                                                            yaw=90,
                                                            pitch=-70,
                                                            roll=0,
                                                            upAxisIndex=2)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                     aspect=float(960) /720,
                                                     nearVal=0.1,
                                                     farVal=100.0)
        (_, _, px, _, _) = p.getCameraImage(width=960,
                                              height=720,
                                              viewMatrix=view_matrix,
                                              projectionMatrix=proj_matrix,
                                              renderer=p.ER_BULLET_HARDWARE_OPENGL)

        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (720,960, 4))

        rgb_array = rgb_array[:, :, :3]
        return rgb_array
        
    
    def close(self):
        p.disconnect((self.client))
    
    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]
    
    
    