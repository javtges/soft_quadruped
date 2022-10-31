import pybullet as p
import os
import math
import numpy as np

class HSARobot:
    def __init__(self, client):
        self.client = client
        
        f_name = os.path.join(os.path.dirname(__file__), 'hsa_turtle_test.urdf')
        
        self.robot = p.loadURDF(fileName=f_name)
        
        self.leg_joints = [1, 3, 5, 7]
        self.foot_joints = [2, 4, 6, 8]
        
        self.friction = 0.8
        
        
    def get_ids(self):
        return self.robot, self.client
    
    def apply_action(self, action):
        
        """We expect our action to be eight-dimensional: 4 coordinates of leg joints,
        and 4 coordinates of foot joints.
        """
        
        # May want to change this to get rid of the 0th index
        
        leg_positions = [action[1], action[3], action[5], action[7]]
        foot_positions = [action[2], action[4], action[6], action[8]]
        
        p.setJointMotorControlArray(self.robot, self.leg_joints,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=leg_positions,
                                    physicsClientId=self.client)
        
        p.setJointMotorControlArray(self.robot, self.foot_joints,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=foot_positions,
                                    physicsClientId=self.client)
        
    def get_observation(self):
        """Get the position and orientation of the HSA robot.
        """
        
        pos, ang = p.getBasePositionAndOrientation(self.robot, self.client)
        ang = p.getEulerFromQuaternion(ang)
        ori = (math.cos(ang[2]), math.sin(ang[2]))
        pos = pos[:2]
        # Get the velocity of the robot
        vel = p.getBaseVelocity(self.car, self.client)[0][0:2]

        # Concatenate position, orientation, velocity
        observation = (pos + ori + vel)

        return observation