import numpy as np
import csv
import time
import serial


def send_policy(policy):
    '''
    Sends the policy via serial to the robot.
    '''

    print("Sending Policy", policy)
    for i in range(len(policy)):
        if policy[i] < 0:
            policy[i] = 0
        if policy[i] > 180:
            policy[i] = 180
        policy[i] = int(policy[i])
    # policy = np.multiply(policy, 100) # Move decimal place 3 to the left
    # print("Intermediate Policy", policy)
    # policy = [int(x)%10000 for x in policy] # always only 4 digits long
    policy = [str(x).zfill(4) for x in policy]
    str_policy = str(policy)+'\n'
    # print("Updated Policy", str_policy)
    
    ser.write(str_policy.encode())


try:
    ser = serial.Serial('/dev/ttyACM1',1000000,timeout=0.1)
except:                      
    ser = serial.Serial('/dev/ttyACM0',1000000,timeout=0.1)
    
print('Opening port: ')
print(ser.name)


# lut = LookupTable(lookup_table_filename='/home/james/final_project/src/lookup_table_unique2.csv')
# trial_data = np.genfromtxt('/home/james/final_project/src/experiment_data/policytest_221218_075425_zero', delimiter=',')
trial_data = np.genfromtxt('/home/tommy/tommy-dev/soft_quadruped/policyreplay_circulargait interpolate python try', delimiter=',')


# n1_list_fl = trial_data[:,16]
# # print(n1_list_fl)
# n2_list_fl = trial_data[:,17]
# # print(len(n2_list_fl))

# n1_list_fr = trial_data[:,18]
# n2_list_fr = trial_data[:,19]

# n1_list_rl = trial_data[:,20]
# n2_list_rl = trial_data[:,21]

# n1_list_rr = trial_data[:,22]
# n2_list_rr = trial_data[:,23]

for line in trial_data:
    
    params = [line[16], line[17], line[18], line[19], line[20], line[21], line[22], line[23]]
    
    send_policy(params)
    
    time.sleep(0.1)