import csv
import time
from datetime import datetime
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp



def interpolate_with_motor_values(b1, b2):
    if ((b1 > 90 and b2 > 90) or (b1 < 90 and b2 < 90)):
        # This means we're bending in one direction or the other
        upper = [b1+10, b2+10] # this translates to bending further
        lower = [b1-10, b2-10] # this translates to bending less
    else:
        if (b1>90):
            upper = [b1+10, b2-10] # this translates to contracting further
            lower = [b1-10, b2+10] # this translates to contracting less
        else:
            upper = [b1-10, b2+10] # this translates to contracting further
            lower = [b1+10, b2-10] # this translates to contracting less
            
    # Find the matrices associated with upper and lower motor values
    
    # Make 2 scipy rotation objects
    
    # slerp = Slerp(key_times, key_rots)
    
    # interp_rots = slerp.(time) # the middle value here between the two
    
    # interpolate the translation values normally (linearly)
    # translate to quaternion, rotation matrix, return


def interpolate_with_matrix(xyz, rot):
    '''
    Given an SE3 matrix, find the suitable rotation values
    '''
    
    
    pass