import matplotlib.animation as animation
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.offsetbox import AnchoredText
import csv
import pandas as pd

'''
Produces a fun point-by-point animation of a lookup table, from a CSV file or a set of CSV files.
'''


# filename_default = "table_221018_154600_"
# filename_default = "table_221107_212842_"
# filename_default = "/home/james/final_project/src/pmtg/gym-hsa_robot/gym_hsa_robot/resources/table_221109_223329_"
filename_default = "table_221121_142312__no_reset_"

num = 0
num_max = 13

x_list = []
z_list = []
y_list = []
n1_list = []
n2_list = []

for i in range(num_max):
    filename = filename_default + str(i)
    print(filename)
    # data = pd.read_csv(filename).values
    data = np.genfromtxt(filename, delimiter=',')
    x = data[:,4] # this should be X
    y = data[:,5] # this should be Y
    z = data[:,6] # this should be Z in the camera frame (Y in the planar frame)
    
    n1 = data[:,0]
    n2 = data[:,2]
    # data = data[1:][:]
    # print(data.shape)
    
    print(x.shape)
    # print(x)
    
    x = x.tolist()
    y = y.tolist()
    z = z.tolist()
    n1 = n1.tolist()
    n2 = n2.tolist()

    x_list += x
    y_list += y
    z_list += z
    n1_list += n1
    n2_list += n2
    # x_list.append(x)
    # y_list.append(y)
    # z_list.append(z)

# print(x_list)
print(len(x_list))

fig, ax = plt.subplots()
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
    s = ax.scatter(x_list[i][:], z_list[i][:], cmap = "RdBu_r", marker = ".", edgecolor = None)
    

# ani = animation.FuncAnimation(fig, animate, interval=1000, frames=range(num_max))
# ani.save('22_11_21_with_reset_xz_newservo.gif', writer='pillow')

###################################################################################

fig, ax = plt.subplots()
ax.set_xlabel('X Axis', size = 12)
ax.set_ylabel('Y Axis', size = 12)
ax.axis([-0.02,0.02,-0.005,0.01])
x_vals = []
y_vals = []
intensity = []
iterations = len(x_list)
n = 0

t_vals = np.linspace(0,1, iterations)

colors = [[0,0,1,0],[0,0,1,0.5],[0,0.2,0.4,1]]
cmap = LinearSegmentedColormap.from_list("", colors)
scatter = ax.scatter(x_vals,y_vals, c=[], cmap=cmap, vmin=0,vmax=1)

def get_new_vals():
    global n
    x = x_list[n]
    y = z_list[n] * -1
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
filename_out = filename + "points.gif"
# ani.save(filename_out, writer='imagemagick')

plt.show()
