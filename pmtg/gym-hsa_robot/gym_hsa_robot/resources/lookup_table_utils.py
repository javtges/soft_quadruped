import csv
import time
from datetime import datetime
import numpy as np
from numpy import genfromtxt
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from scipy.interpolate import NearestNDInterpolator



class LookupTable:
    
    def __init__(self, lookup_table_filename):
        
        # with open(lookup_table_filename, 'r') as lookup:
        #     # lookup.
        #     pass
        
        self.data = genfromtxt(lookup_table_filename, delimiter=',')
        print(self.data.shape)
        self.num1 = self.data[:,0]
        self.num2 = self.data[:,2]
        self.x = self.data[:,4]
        self.y = self.data[:,6]
        
        self.interp_x = NearestNDInterpolator(list(zip(self.num1, self.num2)), self.x)
        self.interp_y = NearestNDInterpolator(list(zip(self.num1, self.num2)), self.y)
        self.interp_num1 = NearestNDInterpolator(list(zip(self.x, self.y)), self.num1)
        self.interp_num2 = NearestNDInterpolator(list(zip(self.x, self.y)), self.num2)
        # print(self.interp_x(111, 129))
        
        
        
    def interpolate_with_motor_values(self, b1, b2):
        x = self.interp_x(b1, b2)
        y = self.interp_y(b1, b2)
        return x, y
    
    
    def interpolate_with_xy(self, x, y):
        n1 = self.interp_num1(x, y)
        n2 = self.interp_num2(x, y)
        return n1, n2
    
    
if __name__ == '__main__':
    
    a = LookupTable(lookup_table_filename='/home/james/final_project/src/table_221107_212842__no_reset_0')