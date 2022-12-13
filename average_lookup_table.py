import numpy as np
import matplotlib.pyplot as plt
import csv


def write_csv(filename, data, R):
    '''
    Writes to a csv with filename
    '''
    n = filename
    # Verify the ordering of the transform and everything is right
    with open(n, 'a') as f:
        writer = csv.writer(f)
        
        # Could be different ordering of motors
        writer.writerow([data[0], 180-data[0], data[1], 180-data[1], R[0], R[1], R[2]])


np.set_printoptions(suppress=True, formatter={'float_kind':'{:f}'.format})
data = np.genfromtxt('/home/james/final_project/src/logs/lookup_table_out.csv', delimiter=',')
print(data.shape)
num1 = data[:,0]
num2 = data[:,2]

# X_in is x, Yin in camera frame is z in legframe, Z in camera frame is Y in legframe
x_in = data[:,4]
y_in = data[:,5]
z_in = data[:,6]


print(data[1])

dict_x = {}
dict_y = {}
dict_z = {}


for idx, val in enumerate(data):
    nums = (num1[idx], num2[idx])
    # print(nums)
    
    if nums not in dict_x:
        # Add everything to dictionary
        dict_x[nums] = [x_in[idx]]
        dict_y[nums] = [y_in[idx]]
        dict_z[nums] = [z_in[idx]]
    
    else:
        dict_x[nums].append(x_in[idx])
        dict_y[nums].append(y_in[idx])
        dict_z[nums].append(z_in[idx])
        
# print(dict_x[(90,90)])

for key in dict_x:
    # print(key)
    # print(key[0], key[1])
    
    avg_x = np.average(dict_x[key])
    avg_y = np.average(dict_y[key])
    avg_z = np.average(dict_z[key])
    
    write_csv("lookup_table_unique2.csv", [key[0], key[1]], [avg_x, avg_y, avg_z])
    # print(dict_x[key])
    # print("average ", avg_x)   