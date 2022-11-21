import numpy as np
import csv
from matplotlib import pyplot as plt
from gym_hsa_robot.resources.trajectory_generator import make_traj, make_circle, xy_legframe_to_joints, rotate
from gym_hsa_robot.resources.lookup_table_utils import LookupTable


lut = LookupTable(lookup_table_filename='/home/james/final_project/src/table_221107_212842__no_reset_0')

x_cir, y_cir = make_circle(lut.x_avg, lut.y_avg, lut.width/2, lut.eps/2, 10)

num1, num2 = lut.interpolate_bilinear(x_cir, y_cir)

x, y = lut.interpolate_with_motor_values(num1, num2)


plt.scatter(lut.x, lut.y)
plt.scatter(x_cir,y_cir)
# plt.scatter(x,y)
plt.show()

# plt.show()

# plt.show()

print(num1, num2)