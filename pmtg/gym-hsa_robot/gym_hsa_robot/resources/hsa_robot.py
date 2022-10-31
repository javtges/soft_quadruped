import pybullet as p
import os
import math
import numpy as np

class HSARobot:
    def __init__(self, client):
        self.client = client
        
        f_name = os.path.join(os.path.dirname(__file__), 'hsa_turtle_test.urdf')
        
        self.robot = p.loadURDF(fileName=f_name)