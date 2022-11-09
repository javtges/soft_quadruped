import matplotlib.animation as animation
import numpy as np
import matplotlib.pyplot as plt
import csv
import pandas as pd


# filename_default = "table_221018_154600_"
filename_default = "table_221107_212842_"

num = 0
num_max = 3

x_list = []
z_list = []
y_list = []

for i in range(num_max):
    filename = filename_default + str(i)
    data = pd.read_csv(filename).values
    data = data[1:][:]
    print(data.shape)
    x = data[:, [4],].T
    print(x.shape)
    # print(x)
    y = data[:, [5],].T
    z = data[:, [6],].T
    
    
    x = x[0].tolist()
    y = y[0].tolist()
    z = z[0].tolist()
    x_list.append(x)
    y_list.append(y)
    z_list.append(z)



fig, ax = plt.subplots()
def animate(i):
    fig.clear()
    ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-0.005, 0.005), ylim=(-0.005, 0.005))
    ax.set_xlim(-0.005,0.005)
    ax.set_ylim(-0.005,0.005)
    ax.grid(b=None)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title("HSA In-Plane Displacement")
    ax.text(0.02, 0.95, 'Time Step = %d' % i, transform=ax.transAxes)
    s = ax.scatter(x_list[i][:], y_list[i][:], cmap = "RdBu_r", marker = ".", edgecolor = None)
    

ani = animation.FuncAnimation(fig, animate, interval=1000, frames=range(num_max))

# FFwriter = animation.FFMpegWriter(fps=3)
ani.save('animation_with_reset_xy.gif', writer='pillow')


    