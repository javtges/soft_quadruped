import csv
import time
from datetime import datetime
import numpy as np
from numpy import genfromtxt
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from scipy.interpolate import NearestNDInterpolator
from gym_hsa_robot.train_ars import Ellipse_TG


class LookupTable:
    
    def __init__(self, lookup_table_filename):
        
        '''
        Initializes the lookup table from a CSV file. Sets up various methods and interpolators.
        
        
        '''
        
        # with open(lookup_table_filename, 'r') as lookup:
        #     # lookup.
        #     pass
        
        self.data = genfromtxt(lookup_table_filename, delimiter=',')
        print(self.data.shape)
        self.num1 = self.data[:,0]  
        self.num2 = self.data[:,2] 
        self.x = self.data[:,4] # this should be X
        self.y = self.data[:,6] # this should be Z in the camera frame (Y in the planar frame)
        
        self.x_avg = np.average(self.x)
        self.y_avg = np.average(self.y)
        
        self.interp_x = NearestNDInterpolator(list(zip(self.num1, self.num2)), self.x)
        self.interp_y = NearestNDInterpolator(list(zip(self.num1, self.num2)), self.y)
        self.interp_num1 = NearestNDInterpolator(list(zip(self.x, self.y)), self.num1)
        self.interp_num2 = NearestNDInterpolator(list(zip(self.x, self.y)), self.num2)
        
        self.width = np.max(self.x) - np.min(self.x)
        minh = 0
        maxh = 0
        for idx , val in enumerate(self.x):
            if abs(val) > 0.002:
                pass
            else:
                if self.y[idx] > maxh:
                    maxh = self.y[idx]
                if self.y[idx] < minh:
                    minh = self.y[idx]

        self.eps = maxh-minh
        
        self.theta = np.arctan(0.5*self.width / 0.07)
                
        
    def interpolate_with_motor_values(self, b1, b2):
        
        
        '''
        Uses NearestNDInterpolator to go from motor values to XY coordinates. Not used at any point.
        '''
        x = self.interp_x(b1, b2)
        y = self.interp_y(b1, b2)
        return x, y
    
    
    def interpolate_with_xy(self, x, y):
        
        '''
        Uses NearestNDInterpolator to go from XY commands to motor commands. This is the old method of interpolation and is only used for testing.
        '''
        
        n1 = self.interp_num1(x, y)
        n2 = self.interp_num2(x, y)
        return n1, n2
    
    # Function from https://stackoverflow.com/questions/47177493/python-point-on-a-line-closest-to-third-point
    def p(self, p1, p2, p3):
        '''
        Finds a point p3 along the line that connects points p1 and p2
        '''
        (x1, y1), (x2, y2), (x3, y3) = p1, p2, p3
        dx, dy = x2-x1, y2-y1
        # print("dx, dy: ", dx, dy)
        if dx != 0.0 and dy != 0.0:
            det = dx*dx + dy*dy
            # print("det: ", det)
            a = (dy*(y3-y1)+dx*(x3-x1))/det
            return x1+a*dx, y1+a*dy
        else:
            # print("points are the same!", p1)
            return x1, y1
        
    
    # Function adapted from https://stackoverflow.com/questions/72031447/how-to-calculate-the-proportional-percentage-of-a-point-of-an-x-y-coordinate-wit
    def percent_along_line(self, p1, p2, p3):
        
        '''
        Given p1, p2, and p3, where p3 is between p1 and p2, find the percentage of the way that p3 is from p1 to p2.
        
        ''' 
        (X1, Y1), (X2, Y2), (XF, YF) = p1, p2, p3
        dx = X2-X1
        dy = Y2-Y1
        if dx != 0 and dy != 0:
            if dx>dy:
                t = (XF - X1) / dx
            else:
                t = (YF - Y1) / dy
            return t
        else:
            return 0
        
    def search_lut_by_motor_values(self, n1, n2):
        '''
        Given n1, n2, find the XY values in the lookup table
        '''
        
        if n1 < min(self.num1):
            n1 = min(self.num1)
            
        if n2 < min(self.num2):
            n2 = min(self.num2)
            
        for line in self.data:
            if line[0] == n1 and line[2] == n2:
                
                return line[4],line[6]
    
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
            
            
            # print("point 3", x[idx], y[idx])
            x1 = self.x[n[0]]
            y1 = self.y[n[0]]
            
            # print("point 1", x1, y1)
            x2 = self.x[n[1]]
            y2 = self.y[n[1]]
            
            # print("point 2", x2, y2)
            
            n1_1 = self.num1[n[0]]
            n2_1 = self.num2[n[0]]
            # print("numbers_p1", n1_1, n2_1)
            
            n1_2 = self.num1[n[1]]
            n2_2 = self.num2[n[1]]
            # print("numbers_p2", n1_2, n2_2)
            
            diff_n1 = n1_2 - n1_1
            diff_n2 = n2_2 - n2_1
            
            # One suggestion here is to average the two numbers
            # n1a = (n1_1 + n2_1)/2
            # n2a = (n1_2 + n2_2)/2
            
            # Other idea, from Matt: Find the distance between the two points, find the percentage of
            # the distance along the line
            
            x4, y4 = self.p((x1,y1), (x2,y2), (x[idx],y[idx]))
            
            # print("point 4", x4, y4)
            t = self.percent_along_line((x1,y1), (x2,y2), (x4,y4))
            
            # print("T?", t)
            
            n1a = np.clip(int(n1_1 + t * diff_n1),0,180)
            n2a = np.clip(int(n2_1 + t * diff_n2),0,180)
            
            n1.append(n1a)
            n2.append(n2a)
            # print("n1, n2", n1a, n2a)
            
        n1 = np.asarray(n1)
        n2 = np.asarray(n2)
        
        return n1, n2
    
    def interpolate_bilinear_xy(self, x, y):
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
        x4_ar = []
        y4_ar = []
        # Get the list of distances from the point
        # Find the lowest and second lowest elements in the list
        
        for idx, val in enumerate(x):
        
            distances = np.sqrt( np.square(self.x - x[idx]) + np.square(self.y - y[idx]))
            # print(distances.shape)
            n = np.argpartition(distances, 2)[:2]
            # print(n)
            
            # print("point 3 on the circle", x[idx], y[idx])
            
            x1 = self.x[n[0]]
            y1 = self.y[n[0]]
            # print("point 1, closest to the point on the circle", x1, y1, distances[n[0]])
            
            x2 = self.x[n[1]]
            y2 = self.y[n[1]]
            # print("point 2, second closest to the circle", x2, y2, distances[n[0]])
            
            n1_1 = self.num1[n[0]]
            n2_1 = self.num2[n[0]]
            # print("numbers_p1", n1_1, n2_1)
            
            n1_2 = self.num1[n[1]]
            n2_2 = self.num2[n[1]]
            # print("numbers_p2", n1_2, n2_2)
            
            diff_n1 = n1_2 - n1_1
            diff_n2 = n2_2 - n2_1
            
            # One suggestion here is to average the two numbers
            # n1a = (n1_1 + n2_1)/2
            # n2a = (n1_2 + n2_2)/2
            
            # Other idea, from Matt: Find the distance between the two points, find the percentage of
            # the distance along the line
            
            x4, y4 = self.p((x1,y1), (x2,y2), (x[idx],y[idx]))
            
            # print("point 4, on the line between points 1 and 2", x4, y4)
            
            t = self.percent_along_line((x1,y1), (x2,y2), (x4,y4))
                        
            n1a = np.clip(int(n1_1 + t * diff_n1),0,180)
            n2a = np.clip(int(n2_1 + t * diff_n2),0,180)
            
            x4_ar.append(x4)
            y4_ar.append(y4)
            
        x4 = np.asarray(x4_ar)
        y4 = np.asarray(y4_ar)
        
        return x4, y4
    
    def interpolate_with_motors(self, n1, n2):
        
        '''
        Given two motor commands n1 and n2, find the corresponding XY values in the legframe.
        
        So, we need to interpolate somehow. How to do this?
        
        N1 and N2 map to X and Y
        
        180, 180 -> Full extended
        40, 40 -> Full contracted
        180, 40 -> Left(?)
        40, 180 -> Right(?)
        
        Find high and low for n1,n2 pairs. Ex: 94,101 becomes (90,100) and (100,110)
        Find the two (x,y) pairs for the higher and lower pairs
        
        '''
        n1_low = max(10 * int(n1/10), 0)
        n1_high = min(n1_low + 10, 180)
        n1_r = n1 % 10
        
        n2_low = max(10 * int(n2/10), 0)
        n2_high = min(n2_low + 10, 180)
        n2_r = n2 % 10
        
        # print(n1_low, n2_low, n1_high, n2_high)
        
        x_low, y_low = self.search_lut_by_motor_values(n1_low, n2_low)
        x_high, y_high = self.search_lut_by_motor_values(n1_high, n2_high)
        
        a, b = self.p((n1_low,n2_low), (n1_high,n2_high), (n1,n2))
        t = self.percent_along_line((n1_low,n2_low), (n1_high,n2_high), (a,b))

        x = x_low + t * (x_high - x_low)
        y = y_low + t * (y_high - y_low)
        
        return x, y
    
    
if __name__ == '__main__':
    
    a = LookupTable(lookup_table_filename='/home/james/final_project/src/table_221107_212842__no_reset_0')
    etg = Ellipse_TG()
    
    x, y = etg.make_circle(0.0, -0.074, 0.015, 0.003, n=10)
    n1, n2 = a.interpolate_bilinear(x, y)
    # print("reg", n1, n2)
    # n1, n2 = a.interpolate_bilinear_scalar([0], [1])
    print("new", n1, n2)