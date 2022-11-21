import matplotlib.pyplot as plt
import numpy as np
import csv



# lookup_table_filename = "bilinear_good_v1_221121_012641.csv"

# lookup_table_filename = "nn_interp_v1_221121_012844.csv"

lookup_table_filename = "bilinear_good_v2_221121_020558.csv"
data = np.genfromtxt(lookup_table_filename, delimiter=',')
print(data.shape)
x = data[:,4] # this should be X
y = data[:,5] # this should be Y
z = data[:,6] # this should be Z in the camera frame (Y in the planar frame)
time = data[:,-1]
elapsed = time[-1] - time[0]

# print(x[-1], y[-1], x[1], y[1])
dX = x[-1] - x[1]
dY = y[-1] - y[1]

print("X and Y distance traveled:", dX, dY)

speed = np.sqrt(dX*dX + dY*dY) / elapsed
print("Speed in m/s:", speed)

plt.plot(x,y)
plt.show()