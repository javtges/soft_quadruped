import numpy as np
from gym_hsa_robot.resources.lookup_table_utils import LookupTable
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.offsetbox import AnchoredText
import csv
import pandas as pd



lut = LookupTable(lookup_table_filename='/home/james/final_project/src/lookup_table_unique2.csv')
trial_data = np.genfromtxt('/home/james/final_project/src/experiment_data/policytest_221218_075425_zero', delimiter=',')

n1_list_fl = trial_data[:,16]
# print(n1_list_fl)
n2_list_fl = trial_data[:,17]
# print(len(n2_list_fl))

n1_list_fr = trial_data[:,18]
n2_list_fr = trial_data[:,19]

n1_list_rl = trial_data[:,20]
n2_list_rl = trial_data[:,21]

n1_list_rr = trial_data[:,22]
n2_list_rr = trial_data[:,23]

print(n1_list_rr)
print(n2_list_rr)

fl_x = []
fl_y = []

fr_x = []
fr_y = []

rl_x = []
rl_y = []

rr_x = []
rr_y = []


for i in range(len(n1_list_fl)):
    # print(i, n1_list[i], n2_list[i])
    # print(n1_list_fl[i], n2_list_fl[i])
    
    xa, ya = lut.interpolate_with_motors(n1_list_fl[i], n2_list_fl[i])
    fl_x.append(xa)
    fl_y.append(ya)
    
    xb, yb = lut.interpolate_with_motors(n1_list_fr[i], n2_list_fr[i])
    fr_x.append(xb)
    fr_y.append(yb)
    
    xc, yc = lut.interpolate_with_motors(n1_list_rl[i], n2_list_rl[i])
    rl_x.append(xc)
    rl_y.append(yc)
    
    xd, yd = lut.interpolate_with_motors(n1_list_rr[i], n2_list_rr[i])
    rr_x.append(xd)
    rr_y.append(yd)
    

# def animate(i):
#     fig.clear()
#     ax = fig.add_subplot(221, aspect='equal', autoscale_on=False, xlim=(-0.005, 0.005), ylim=(-0.005, 0.005))
#     ax.set_xlim(-0.02,0.02)
#     ax.set_ylim(-0.02,0.02)
#     ax.grid(b=None)
#     ax.set_xlabel('x [m]')
#     ax.set_ylabel('y [m]')
#     ax.set_title("HSA In-Plane Displacement")
#     ax.text(0.02, 0.95, 'Time Step = %d' % i, transform=ax.transAxes)
#     s = ax.scatter(x_list[i][:], y_list[i][:], cmap = "RdBu_r", marker = ".", edgecolor = None)




fig, ((ax_fl, ax_fr), (ax_rl, ax_rr)) = plt.subplots(nrows=2, ncols=2)

ax_fl.set_xlabel('X Axis', size = 6)
ax_fl.set_ylabel('Y Axis', size = 6)
ax_fr.set_xlabel('X Axis', size = 6)
ax_fr.set_ylabel('Y Axis', size = 6)
ax_rl.set_xlabel('X Axis', size = 6)
ax_rl.set_ylabel('Y Axis', size = 6)
ax_rr.set_xlabel('X Axis', size = 6)
ax_rr.set_ylabel('Y Axis', size = 6)

ax_fl.axis([-0.02,0.02,-0.006,0.002])
ax_fr.axis([-0.02,0.02,-0.006,0.002])
ax_rl.axis([-0.02,0.02,-0.006,0.002])
ax_rr.axis([-0.02,0.02,-0.006,0.002])

x_vals_fl = []
y_vals_fl = []

x_vals_fr = []
y_vals_fr = []

x_vals_rl = []
y_vals_rl = []

x_vals_rr = []
y_vals_rr = []

intensity_fl = []
intensity_fr = []
intensity_rl = []
intensity_rr = []

intensity = []

iterations = len(n1_list_fl)
t_vals = np.linspace(0,1, iterations)
n = 0


colors = [[0,0,1,0],[0,0,1,0.5],[0,0.2,0.4,1]]
cmap = LinearSegmentedColormap.from_list("", colors)
scatter_fl = ax_fl.scatter(x_vals_fl,y_vals_fl, c=[], cmap=cmap, vmin=0,vmax=1)

# colors = [[0,0,1,0],[0,0,1,0.5],[0,0.2,0.4,1]]
# cmap = LinearSegmentedColormap.from_list("", colors)
scatter_fr = ax_fr.scatter(x_vals_fr,y_vals_fr, c=[], cmap=cmap, vmin=0,vmax=1)

# colors = [[0,0,1,0],[0,0,1,0.5],[0,0.2,0.4,1]]
# cmap = LinearSegmentedColormap.from_list("", colors)
scatter_rl = ax_rl.scatter(x_vals_rl,y_vals_rl, c=[], cmap=cmap, vmin=0,vmax=1)

# colors = [[0,0,1,0],[0,0,1,0.5],[0,0.2,0.4,1]]
# cmap = LinearSegmentedColormap.from_list("", colors)
scatter_rr = ax_rr.scatter(x_vals_rr,y_vals_rr, c=[], cmap=cmap, vmin=0,vmax=1)

def get_new_vals():
    # update this to get all 8 new values
    global n
    
    x_fl = fl_x[n]
    y_fl = fl_y[n]
    
    x_fr = fr_x[n]
    y_fr = fr_y[n]
    
    x_rl = rl_x[n]
    y_rl = rl_y[n]
    
    x_rr = rr_x[n]
    y_rr = rr_y[n]
    
    # n1 = n1_list[n]
    # n2 = n2_list[n]
    
    n += 1
    # print(x, y)
    return x_fl, y_fl, x_fr, y_fr, x_rl, y_rl, x_rr, y_rr

def update(t):
    
    global x_vals_fl, y_vals_fl, x_vals_fr, y_vals_fr, x_vals_rl, y_vals_rl, x_vals_rr, y_vals_rr, intensity
    # Get intermediate points
    x_fl, y_fl, x_fr, y_fr, x_rl, y_rl, x_rr, y_rr = get_new_vals() # this will get all 8 new values
    
    x_vals_fl = np.append(x_vals_fl,[x_fl])
    y_vals_fl = np.append(y_vals_fl,[y_fl])
    
    x_vals_fr = np.append(x_vals_fr,[x_fr])
    y_vals_fr = np.append(y_vals_fr,[y_fr])
    
    x_vals_rl = np.append(x_vals_rl,[x_rl])
    y_vals_rl = np.append(y_vals_rl,[y_rl])
    
    x_vals_rr = np.append(x_vals_rr,[x_rr])
    y_vals_rr = np.append(y_vals_rr,[x_rr])

    # Put new values in your plot
    scatter_fl.set_offsets(np.c_[x_vals_fl,y_vals_fl])
    scatter_fr.set_offsets(np.c_[x_vals_fr,y_vals_fr])
    scatter_rl.set_offsets(np.c_[x_vals_rl,y_vals_rl])
    scatter_rr.set_offsets(np.c_[x_vals_rr,y_vals_rr])

    #calculate new color values
    intensity = np.concatenate((np.array(intensity)*0.96, np.ones(len([x_fl]))))
    
    scatter_fl.set_array(intensity)
    scatter_fr.set_array(intensity)
    scatter_rl.set_array(intensity)
    scatter_rr.set_array(intensity)



    # Set title
    # ax_fl.set_title('Time: %0.3f' %t)
    # while ax.artists != []:
    #     ax.artists[0].remove()
    # textname = "M: " + str(new_n1) + "\nN: " + str(new_n2)
    # at = AnchoredText(
    #     textname, prop=dict(size=15), frameon=False, loc='upper left')
    # at.patch.set_boxstyle("round,pad=0.,rounding_size=0.2")
    # ax.add_artist(at)
     
     
     
ani = animation.FuncAnimation(fig, update, frames=t_vals,interval=50)
# filename_out = filename + "points.gif"
# ani.save(filename_out, writer='imagemagick')

plt.show()