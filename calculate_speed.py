import matplotlib.pyplot as plt
import numpy as np
import csv


'''
Finds the speed from a CSV file. Plots the trajectory as well.
'''



# lookup_table_filename = "bilinear_good_v1_221121_012641.csv"

# lookup_table_filename = "nn_interp_v1_221121_012844.csv"

# bilinear_good_v2_221121_020558.csv

# lookup_table_filename = "/home/james/final_project/src/pranav_gait_fixed_221213_040415.csv"
lookup_table_filename = "/home/james/final_project/src/policytest_221218_161619_epoch15_broken"
data = np.genfromtxt(lookup_table_filename, delimiter=',')
print(data.shape)
x = data[:,4] # this should be X
y = data[:,5] # this should be Y
z = data[:,6] # this should be Z in the camera frame (Y in the planar frame)
time = data[:,-1]
elapsed = time[-1] - time[1]

# print(x[-1], y[-1], x[1], y[1])
dX = x[-1] - x[1]
dY = y[-1] - y[1]
dZ = z[-1] - z[1]

print("X and Y distance traveled:", dX, dY, dZ, np.sqrt(dX*dX + dY*dY + dZ*dZ))
print("Time elapsed:", elapsed)

speed = np.sqrt(dX*dX + dY*dY + dZ*dZ) / elapsed
print("Speed in m/s:", speed)

plt.plot(x,y)
plt.show()