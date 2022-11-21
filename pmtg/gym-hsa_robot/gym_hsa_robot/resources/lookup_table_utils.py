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
        self.num1 = self.data[:,0] # this should be X 
        self.num2 = self.data[:,2] # this should be Z
        self.x = self.data[:,4]
        self.y = self.data[:,6]
        
        self.interp_x = NearestNDInterpolator(list(zip(self.num1, self.num2)), self.x)
        self.interp_y = NearestNDInterpolator(list(zip(self.num1, self.num2)), self.y)
        self.interp_num1 = NearestNDInterpolator(list(zip(self.x, self.y)), self.num1)
        self.interp_num2 = NearestNDInterpolator(list(zip(self.x, self.y)), self.num2)
        
        self.width = np.max(self.x) - np.min(self.x)
        self.height = np.max(self.y) - np.min(self.y)
        print(self.width)
        # print(self.interp_x(111, 129))
        
        
        
    def interpolate_with_motor_values(self, b1, b2):
        x = self.interp_x(b1, b2)
        y = self.interp_y(b1, b2)
        return x, y
    
    
    def interpolate_with_xy(self, x, y):
        n1 = self.interp_num1(x, y)
        n2 = self.interp_num2(x, y)
        return n1, n2
    
    # Function from https://stackoverflow.com/questions/47177493/python-point-on-a-line-closest-to-third-point
    def p(self, p1, p2, p3):
        (x1, y1), (x2, y2), (x3, y3) = p1, p2, p3
        dx, dy = x2-x1, y2-y1
        det = dx*dx + dy*dy
        a = (dy*(y3-y1)+dx*(x3-x1))/det
        return x1+a*dx, y1+a*dy
    
    # Function adapted from https://stackoverflow.com/questions/72031447/how-to-calculate-the-proportional-percentage-of-a-point-of-an-x-y-coordinate-wit
    def percent_along_line(self, p1, p2, p3):
        (X1, Y1), (X2, Y2), (XF, YF) = p1, p2, p3
        dx = X2-X1
        dy = Y2-Y1
        if dx>dy:
            t = (XF - X1) / dx
        else:
            t = (YF - Y1) / dy
        return t
    
    def interpolate_bilinear(self, x, y):
        """Given a coordinate requested by a trajectory, find the required motor commands.
        Searches the lookup table and finds the two closest points.
        Then, draws a line between them and picks the midpoint
        Uses the midpoint to interpolate the motor values of the two points

        Args:
            x (_type_): X
            y (_type_): Y

        Returns:
            n1: Motor number 1
            n2: Motor number 2
        """
        n1 = []
        n2 = []
        # Get the list of distances from the point
        # Find the lowest and second lowest elements in the list
        
        for idx, val in enumerate(x):
        
        
            distances = np.sqrt( np.square(self.x - x[idx]), np.square(self.y - y[idx]))
            n = np.argpartition(distances, 2)[:2]
            
            
            print("point 3", x[idx], y[idx])
            x1 = self.x[n[0]]
            y1 = self.y[n[0]]
            
            print("point 1", x1, y1)
            x2 = self.x[n[1]]
            y2 = self.y[n[1]]
            
            print("point 2", x2, y2)
            
            n1_1 = self.num1[n[0]]
            n2_1 = self.num2[n[0]]
            print("numbers_p1", n1_1, n2_1)
            
            n1_2 = self.num1[n[1]]
            n2_2 = self.num2[n[1]]
            print("numbers_p2", n1_2, n2_2)
            
            diff_n1 = n1_2 - n1_1
            diff_n2 = n2_2 - n2_1
            
            # One suggestion here is to average the two numbers
            # n1a = (n1_1 + n2_1)/2
            # n2a = (n1_2 + n2_2)/2
            
            # Other idea, from Matt: Find the distance between the two points, find the percentage of
            # the distance along the line
            
            x4, y4 = self.p((x1,y1), (x2,y2), (x[idx],y[idx]))
            
            print("point 4", x4, y4)
            t = self.percent_along_line((x1,y1), (x2,y2), (x4,y4))
            
            print("T?", t)
            
            n1a = np.clip(int(n1_1 + t * diff_n1),0,180)
            n2a = np.clip(int(n2_1 + t * diff_n2),0,180)
            
            n1.append(n1a)
            n2.append(n2a)
            print("n1, n2", n1a, n2a)
            
        n1 = np.asarray(n1)
        n2 = np.asarray(n2)
        
        return n1, n2
    
    
if __name__ == '__main__':
    
    a = LookupTable(lookup_table_filename='/home/james/final_project/src/table_221107_212842__no_reset_0')
    n1, n2 = a.interpolate_bilinear([1], [1])
    print(n1, n2)