# Pseudocode:
# Go through step in the test data
# Convert to RPY and step trajectory generators
# Evaluate policy to get parameters, residuals
# Plot those over time


import numpy as np
from gym_hsa_robot.resources.lookup_table_utils import LookupTable
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.offsetbox import AnchoredText
import csv
import pandas as pd
import gym
import time
from gym_hsa_robot.train_ars import Policy, Ellipse_TG, Normalizer
from gym_hsa_robot.resources.lookup_table_utils import LookupTable
from gym_hsa_robot.resources.trajectory_generator import make_circle
from scipy.spatial.transform import Rotation as R


lut = LookupTable(lookup_table_filename='/home/james/final_project/src/lookup_table_unique2.csv')
# trial_data = np.genfromtxt('/home/james/final_project/src/experiment_data/policytest_221218_075425_zero', delimiter=',')
trial_data = np.genfromtxt('/home/james/final_project/src/experiment_data/policytest_221218_074207_trained_30s', delimiter=',')


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
    
roll = []
pitch = []
yaw = []
x = []
y = []
z = []
actions = []
prev_time = time.time()   


# Loop through rows
for f in trial_data:
    
    
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
    
    
    
TG_fl = Ellipse_TG()
    
TG_fr = Ellipse_TG()
TG_fr.phase = TG_fr.cycle_length/2

TG_rl = Ellipse_TG()
TG_rl.phase = TG_rl.cycle_length/2

TG_rr = Ellipse_TG()

tg_arr = [TG_fl, TG_fr, TG_rl, TG_rr]


env = gym.make("hsa_robot-v0")
n_inputs = env.observation_space.shape[0] + TG_fl.n_params + 1 - 2
n_outputs = 8 + TG_fl.n_params + 1

policy = Policy(input_size=n_inputs, output_size=n_outputs,
                    env_name="hsa_robot-v0", traj_generator=tg_arr)
    
policy.theta = np.load('/home/james/final_project/src/logs/beast_trial_6x11policy_epoch_15_0.331714094411355.npy')

normalizer = Normalizer(n_inputs)
lut = LookupTable(lookup_table_filename='/home/james/final_project/src/lookup_table_unique2.csv')


# print("seed = ", hp.seed)
np.random.seed(42)
    
for i in range(len(roll)):
    
    state = np.array([roll[i], pitch[i], yaw[i]]) # MAKE THIS THE POS, ORI, VEL -> changed to orientation only
                        
    # Evaluate the policy to get an action
    
    tg_params = np.array([tg_arr[0].width, tg_arr[0].height], dtype=float)
    phase = np.array([tg_arr[0].phase])

    state = np.concatenate((state, tg_params, phase), axis=0)
    # print("STATE BEFORE", state)
    # Augment this to include the variables we need from the TG
    
    
    # Uncomment and fix this later
    normalizer.observe(state)
    state = normalizer.normalize(state)
    action = policy.evaluate(input=state, delta=None, direction=None, hp=None)
    # print("action:", action)
    
    time_delta = time.time() - prev_time
    # print("time delta: ", time_delta-0.1)
    prev_time = time.time()
    
    actions.append(action)
    
    # eps_fl, theta_fl, x_fl, y_fl = tg_arr[0].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[2]), res_y=0.005*(action[3]), step_theta=24, step_time = time_delta + abs(action[10]))
    # eps_fr, theta_fr, x_fr, y_fr = tg_arr[1].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[4]), res_y=0.005*(action[5]), step_theta=24, step_time = time_delta + abs(action[10]))
    # eps_rl, theta_rl, x_rl, y_rl = tg_arr[2].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[6]), res_y=0.005*(action[7]), step_theta=24, step_time = time_delta + abs(action[10]))
    # eps_rr, theta_rr, x_rr, y_rr = tg_arr[3].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[8]), res_y=0.005*(action[9]), step_theta=24, step_time = time_delta + abs(action[10]))

    
    # Step trajectory
    # Use RPY to get parameters
    # pass
    
    
actions = np.asarray(actions)
print("##########################################################")


# print(actions[:,3])