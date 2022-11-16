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
        
        # self.client = p.connect(p.GUI)
        self.client = p.connect(p.DIRECT)
        p.setTimeStep(1/240, self.client)
        
        # Here, define my action space and my observation space
        
        self.action_space = gym.spaces.Box(np.array([-0.01, -0.02, -0.3, -0.02, -0.3, -0.02, -0.3, -0.02, -0.3]), np.array([0.01, 0.02, 0.3, 0.02, 0.3, 0.02, 0.3, 0.02, 0.3]))
        self.observation_space = gym.spaces.Box(np.array([-1000, -1000, -1, -1, -1000, -1000]), np.array([1000, 1000, 1, 1, 1000, 1000]))

        self.np_random, _ = gym.utils.seeding.np_random()
        self.robot = None
        self.done = False
        self.rendered_img = None
        self.render_rot_matrix = None
        self._last_frame_time = time.time()
        self._cam_dist = 1.0
        self._cam_yaw = 0
        self._cam_pitch = -30       
        
        self.reset()
    
    def step(self, action):
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
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
        self.done = False # oops

        # Get observation to return
        robot_ob = self.robot.get_observation()

        return np.array(robot_ob, dtype=np.float32)
    
    def render(self, mode='rgb_array'):
        
        # """This renders the system. Adapted from:
        # https://github.com/OpenQuadruped/spot_mini_mini
        # """        
        
        # if self.rendered_img is None:
        #     self.rendered_img = plt.imshow(np.zeros((100, 100, 4)))

        # # # Base information
        # robot_id, client_id = self.robot.get_ids()
        
        # time_spent = time.time() - self._last_frame_time
        # self._last_frame_time = time.time()
        # time_to_sleep = (1/240) - time_spent
        # # if time_to_sleep > 0:
        # #     time.sleep(time_to_sleep)
        # base_pos, ori = p.getBasePositionAndOrientation(robot_id, client_id)
        # # Keep the previous orientation of the camera set by the user.
        # [yaw, pitch,
        #     dist] = p.getDebugVisualizerCamera()[8:11]
        # p.resetDebugVisualizerCamera(
        #     dist, yaw, pitch, base_pos)
        
        # # if mode != "rgb_array":
        # #     return np.array([])
        # base_pos = self.robot.GetBasePosition()
        # view_matrix = p.computeViewMatrixFromYawPitchRoll(
        #     cameraTargetPosition=base_pos,
        #     distance=self._cam_dist,
        #     yaw=self._cam_yaw,
        #     pitch=self._cam_pitch,
        #     roll=0,
        #     upAxisIndex=2)
        # proj_matrix = p.computeProjectionMatrixFOV(
        #     fov=60,
        #     aspect=float(RENDER_WIDTH) / RENDER_HEIGHT,
        #     nearVal=0.1,
        #     farVal=100.0)
        # (_, _, px, _, _) = p.getCameraImage(
        #     width=RENDER_WIDTH,
        #     height=RENDER_HEIGHT,
        #     viewMatrix=view_matrix,
        #     projectionMatrix=proj_matrix,
        #     renderer=p.ER_BULLET_HARDWARE_OPENGL)
        # rgb_array = np.array(px)
        # rgb_array = rgb_array[:, :, :3]
        
        # self.rendered_img.set_data(rgb_array)
        # plt.draw()
        # plt.pause(.00001)
        # return rgb_array
        
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
    
    
    