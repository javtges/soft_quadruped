import serial
import cv2
import pyrealsense2 as rs
import numpy as np
import yaml
import time
import readchar
import struct

print(time.time())
eps = 0.01
z = np.zeros((2,16))

a = np.random.choice([-eps,0,eps], 16)
z[0][:] = a
print(z)
print(a.shape)

def make_policies(params, eps):
    t = 10
    p_count = 16
    R_list = np.zeros((t,p_count))
    eps_list = np.zeros((t,p_count))
    for row in range(t):
        e = np.random.choice([-eps,0,eps], 16)
        eps_list[row][:] = e
        R_list[row][:] = e + params

    return R_list, eps_list

# r , e = make_policies(np.ones((16,))*90, eps)
# print(r)


policy = [0, 10, 20, 180]
print(np.sum(policy))

try:
    ser = serial.Serial('/dev/ttyACM1',115200,rtscts=1)
except:                      
    ser = serial.Serial('/dev/ttyACM0',115200,rtscts=1)

# print('Opening port: ')
# print(ser.name)

def send_policy(policy):

    print("Sending Policy", policy)
    for i in range(len(policy)):
        if policy[i] < 0:
            policy[i] = 0
    policy = np.multiply(policy, 1000) # Move decimal place 3 to the left
    print(policy)
    policy = [int(x)%1000000 for x in policy] # always only 4 digits long
    print(policy)
    policy = [str(x).zfill(5) for x in policy]
    print(policy)
    ser.write((str(policy)+'\n').encode())

while True:
    send_policy(policy)
    time.sleep(2)
