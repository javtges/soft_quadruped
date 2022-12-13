import numpy as np
import csv
from matplotlib import pyplot as plt
from gym_hsa_robot.resources.trajectory_generator import make_traj, make_circle, xy_legframe_to_joints, rotate
from gym_hsa_robot.resources.lookup_table_utils import LookupTable

'''
Testing file only.

Plots interpolated values from lookup table to verify functionality.

'''


# lut = LookupTable(lookup_table_filename='/home/james/final_project/src/table_221107_212842__no_reset_0')
lut = LookupTable(lookup_table_filename='/home/james/final_project/src/lookup_table_unique2.csv')

print("x radius ", lut.width/2, "y radius ", lut.eps/2)
x_cir, y_cir = make_circle(0, -0.004, lut.width/2, lut.eps/2, 10)
print("x circle, y circle", x_cir, y_cir)

num1, num2 = lut.interpolate_bilinear(x_cir, y_cir)
# num1, num2 = lut.interpolate_with_xy(x_cir, y_cir)

# num1, num2 = lut.interpolate_bilinear(x, y)

x, y = lut.interpolate_bilinear_xy(x_cir, y_cir)

lut_y = [-1 * a for a in lut.y]
y_cir = [-1 * a for a in y_cir]
y = [-1 * a for a in y]

print(num1, num2)

plt.scatter(lut.x, lut_y, s=2, label="Lookup Table Values")
plt.scatter(x_cir,y_cir, label="Desired Circle Values")
plt.scatter(x,y, label="Interpolated Circle Values")
plt.title("Lookup Table with Desired Circle and Interpolation")
plt.xlabel("X Coordinate (m)")
plt.ylabel("Y Coordinate (m)")
plt.legend()
# plt.scatter(x,y)
plt.show()