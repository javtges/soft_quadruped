import numpy as np
from gym_hsa_robot.resources.lookup_table_utils import LookupTable
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.offsetbox import AnchoredText
import csv
import pandas as pd



lut = LookupTable(lookup_table_filename='/home/james/final_project/src/lookup_table_unique2.csv')
n1_list = lut.num1
n2_list = lut.num2
x_list = []
y_list = []


for i in range(len(n1_list)):
    print(i, n1_list[i], n2_list[i])
    xa, ya = lut.interpolate_with_motors(n1_list[i], n2_list[i])
    x_list.append(xa)
    y_list.append(ya)
    
    
    

def animate(i):
    fig.clear()
    ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-0.005, 0.005), ylim=(-0.005, 0.005))
    ax.set_xlim(-0.02,0.02)
    ax.set_ylim(-0.02,0.02)
    ax.grid(b=None)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title("HSA In-Plane Displacement")
    ax.text(0.02, 0.95, 'Time Step = %d' % i, transform=ax.transAxes)
    s = ax.scatter(x_list[i][:], y_list[i][:], cmap = "RdBu_r", marker = ".", edgecolor = None)




fig, ax = plt.subplots()
ax.set_xlabel('X Axis', size = 12)
ax.set_ylabel('Y Axis', size = 12)
ax.axis([-0.02,0.02,-0.005,0.01])
x_vals = []
y_vals = []
intensity = []
iterations = len(n1_list)
n = 0

t_vals = np.linspace(0,1, iterations)

colors = [[0,0,1,0],[0,0,1,0.5],[0,0.2,0.4,1]]
cmap = LinearSegmentedColormap.from_list("", colors)
scatter = ax.scatter(x_vals,y_vals, c=[], cmap=cmap, vmin=0,vmax=1)

def get_new_vals():
    global n
    
    
    x = x_list[n]
    y = y_list[n]
    
    
    n1 = n1_list[n]
    n2 = n2_list[n]
    n += 1
    # print(x, y)
    return x, y, n1, n2

def update(t):
    global x_vals, y_vals, intensity
    # Get intermediate points
    new_xvals, new_yvals, new_n1, new_n2 = get_new_vals()
    # print(new_xvals, new_yvals)
    x_vals = np.append(x_vals,[new_xvals])
    y_vals = np.append(y_vals,[new_yvals])

    # Put new values in your plot
    scatter.set_offsets(np.c_[x_vals,y_vals])

    #calculate new color values
    intensity = np.concatenate((np.array(intensity)*0.96, np.ones(len([new_xvals]))))
    scatter.set_array(intensity)

    # Set title
    ax.set_title('Time: %0.3f' %t)
    while ax.artists != []:
        ax.artists[0].remove()
    textname = "M: " + str(new_n1) + "\nN: " + str(new_n2)
    at = AnchoredText(
        textname, prop=dict(size=15), frameon=False, loc='upper left')
    at.patch.set_boxstyle("round,pad=0.,rounding_size=0.2")
    ax.add_artist(at)
     
     
     
ani = animation.FuncAnimation(fig, update, frames=t_vals,interval=50)
# filename_out = filename + "points.gif"
# ani.save(filename_out, writer='imagemagick')

plt.show()