import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt



# lookup_table_filename = "/home/james/final_project/src/policytest_221218_161619_epoch15_broken"
# lookup_table_filename = "/home/james/final_project/src/policytest_221218_074039_zero_30s"
lookup_table_filename = "/home/james/final_project/src/policytest_221218_074039_zero_30s"

data = np.genfromtxt(lookup_table_filename, delimiter=',')

roll = []
pitch = []
yaw = []
x = []
y = []
z = []

# Loop through rows
for f in data:
    
    
    r = R.from_matrix([[f[7], f[8], f[9]],
                    [f[10], f[11], f[12]],
                    [f[13], f[14], f[15]]])

    r_mat = r.as_matrix()

    r_rpy = r.as_euler("XYZ", degrees=False)
    
    roll.append(r_rpy[0])
    pitch.append(r_rpy[1])
    yaw.append(r_rpy[2])
    
    x.append(f[4])
    y.append(f[5])
    z.append(f[6])
    
    
    
ax1 = plt.subplot(211)
ax1.plot(roll)
ax1.plot(pitch)
ax1.plot(yaw)

ax2 = plt.subplot(212)
ax2.plot(x)
ax2.plot(y)
ax2.plot(z)

plt.show()