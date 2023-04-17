import serial
import cv2
import pyrealsense2 as rs
import numpy as np
# import yaml
import time
import readchar
import gym
import struct
import csv
from datetime import datetime
# from pupil_apriltags import Detector
from gym_hsa_robot.train_ars import Policy, Ellipse_TG, Normalizer, Hp
from gym_hsa_robot.resources.lookup_table_utils import LookupTable
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

'''
Runs a policy on the robot, in pybullet. Takes in a .npy file for the policy.
'''


##########################################

def log_csv(data, now):
    filename = 'trial_' + now
    with open(filename, 'a') as f:
        writer = csv.writer(f)
        writer.writerow(data)

def eval_reward(tag_xyz_data, times, params):
    # print("Eval reward", tag_xyz_data)
    dist_array = []
    # print(imu_data)
    if len(tag_xyz_data) > 0:
        # print(tag_xyz_data[0].shape)
        # for tag in range(1, len(tag_xyz_data)):
        #     d = float((tag_xyz_data[tag][0][:] - tag_xyz_data[tag-1][0][:]))**2 + float((tag_xyz_data[tag][1][:] - tag_xyz_data[tag-1][1][:]))**2 + float((tag_xyz_data[tag][2][:] - tag_xyz_data[tag-1][2][:]))**2
        #     dist_array.append(np.sqrt(d))

        d = float((tag_xyz_data[-1][0][:] - tag_xyz_data[0][0][:]))**2 + float((tag_xyz_data[-1][1][:] - tag_xyz_data[0][1][:]))**2 + float((tag_xyz_data[-1][2][:] - tag_xyz_data[0][2][:]))**2
        # print(dist_array)
        # print(tag_xyz_data[len(tag_xyz_data)-1][0][:], tag_xyz_data[len(tag_xyz_data)-1][1][:] )
        distance = np.sqrt(d) # np.sum(dist_array)
        print("distance in m", distance)
        distance = distance/(times[-1] - times[0])
        print("distance in m/s", distance, times[-1] - times[0])
    else:
        distance = 0
        
    print(params)
    data = list(params)
    data.append(distance)
    data.append(time.time())
    
    log_csv(data, now)
    return distance


   
if __name__ == "__main__": 
    print("Evaluating Policy...")
    np.set_printoptions(suppress=True, formatter={'float_kind':'{:f}'.format})
    
    lut = LookupTable(lookup_table_filename='/home/james/final_project/src/lookup_table_unique2.csv')

    
    '''Pseudocode:
 
    Import a .npy file that contains our policy (or just copy-paste the values)
    
    Every step (100ms?):
        Sample the environment (observation: 8 params for TGs, observation = [pos_x, pos_y, ori, ori, vel_x, vel_y]) using AprilTag
        Use policy to determine action (from train_ars.py)
        Convert eps_theta to motor commands (TODO: need to interpolate better from 11/10 discussions)
        Send motor commands to serial

    '''
    TG_fl = Ellipse_TG()
    
    TG_fr = Ellipse_TG()
    TG_fr.phase = TG_fr.cycle_length/2

    TG_rl = Ellipse_TG()
    TG_rl.phase = TG_rl.cycle_length/2

    TG_rr = Ellipse_TG()

    traj_generators = [TG_fl, TG_fr, TG_rl, TG_rr]
    
    hp = Hp()

    # print("seed = ", hp.seed)
    np.random.seed(42)

    # make the environment
    env = gym.make("hsa_robot-v0")

    # number of inputs: number of columns
    # number of outputs: number of rows
    n_inputs = env.observation_space.shape[0] + TG_fl.n_params + 1 - 2
    
    # THIS DOESN'T EVEN NEED THE ACTION SPACE TO WORK! ONLY NEEDS TRAJ PARAMS
    # n_outputs = env.action_space.shape[0] + 8 + TG_fl.n_params*4
    n_outputs = 8 + TG_fl.n_params + 1

    print("Observation space =", n_inputs)
    print("Action space =", n_outputs)

    policy = Policy(input_size=n_inputs, output_size=n_outputs,
                    env_name="hsa_robot-v0", traj_generator=traj_generators)
    
    # policy.theta = np.load('/home/james/final_project/epoch_106_0.3988122556425218.npy')
    policy.theta = np.load('/home/james/final_project/src/logs/beast_trial_6x11policy_epoch_161_0.4218193610265332.npy')
    print(policy.theta)
    step = 24
    
    normalizer = Normalizer(n_inputs)
    
    starttime = time.time()
    
    state = env.reset()
    step_number = 0
    
    action_list = []
    state_list = []
    
    while True:

        tg_params = np.array([traj_generators[0].width, traj_generators[0].height], dtype=float)
        
        phase = np.array([traj_generators[0].phase])
        # state[4] *= 0.1
        print("X:", state[0], "Y:", state[1], "roll:", state[2], "pitch:", state[3], "yaw:", state[4])
        state_list.append(state)
        
        state = state[2:]
        
        # state vector, input to policy, is "roll, pitch, yaw, width, height, phase"
        state = np.concatenate((state, tg_params, phase), axis=0)

        # Augment this to include the variables we need from the TG
        normalizer.observe(state)
        
        # THIS DOESN'T PROPERLY APPLY NOISE!!!
        state = normalizer.normalize(state)
        
        action = policy.evaluate(state, None, None, hp)

        # print("old phase, step_time", traj_generators[0].phase, action[10])
        
        eps_fl, theta_fl, x_fl, y_fl = traj_generators[0].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[2]), res_y=0.005*(action[3]), step_theta=step, step_time=abs(action[10]))
        eps_fr, theta_fr, x_fr, y_fr = traj_generators[1].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[4]), res_y=0.005*(action[5]), step_theta=step, step_time=abs(action[10]))
        eps_rl, theta_rl, x_rl, y_rl = traj_generators[2].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[6]), res_y=0.005*(action[7]), step_theta=step, step_time=abs(action[10]))
        eps_rr, theta_rr, x_rr, y_rr = traj_generators[3].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[8]), res_y=0.005*(action[9]), step_theta=step, step_time=abs(action[10]))
        
        # print("new phase", traj_generators[0].phase)

        # Change the variable "action" so that it's 9-dimensional (the shape of the environment's input), using the TG

        # Make sure that the order of legs here is correct
        actions_tg = [0, theta_fl, eps_fl, theta_fr, eps_fr, theta_rl, eps_rl, theta_rr, eps_rr]
        # actions_tg = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        n1_fl, n2_fl = lut.interpolate_bilinear([x_fl], [y_fl+0.07])
        n1_fr, n2_fr = lut.interpolate_bilinear([x_fr], [y_fr+0.07])
        n1_rl, n2_rl = lut.interpolate_bilinear([x_rl], [y_rl+0.07])
        n1_rr, n2_rr = lut.interpolate_bilinear([x_rr], [y_rr+0.07])
        
        params = [n1_fr[0], n2_fr[0], n1_fl[0], n2_fl[0], n1_rl[0], n2_rl[0], n1_rr[0], n2_rr[0]]
        
        params = [round(90 + (n - 90)*0.8) for n in params]
        # print(params)
        
        action_list.append(action)
        
        # Remove this for the 'faster' policy
        for i in range(step):
            # noise = np.random.normal(scale=0.01, size=len(actions_tg))
            # actions_tg += noise

            # actions_tg = [0, eps_fl, theta_fl, eps_fr, theta_fr, eps_rl, theta_rl, eps_rr, theta_rr]
            # Here, generate the trajectory from the trajectory generator. Use the actions
            # env.render()
            # print(num_plays)
            state, reward, done, _ = env.step(actions_tg)
            # print("state", state)
            # print(reward)
            
            step_number += 1
        
        if step_number == 1200:
            print("Distance after 1200 simulation steps: ", state[0])
            
        if step_number > 8000:
            print("Distance after ", step_number, " simulation steps: ", state[0])

            break
        
    actions = np.asarray(action_list)
    states = np.asarray(state_list)
    print("##########################################################")

    time_list = np.linspace(0, step_number, len(actions))


    fig, axs = plt.subplots(6,2)

    axs[0,0].plot(time_list, actions[:,0])
    axs[0,0].set_title("width")

    axs[0,1].plot(time_list, actions[:,1])
    axs[0,1].set_title("height")

    axs[1,0].plot(time_list, actions[:,2])
    axs[1,0].set_title("FL_X")

    axs[1,1].plot(time_list, actions[:,3])
    axs[1,1].set_title("FL_Y")

    axs[2,0].plot(time_list, actions[:,4])
    axs[2,0].set_title("FR_X")

    axs[2,1].plot(time_list, actions[:,5])
    axs[2,1].set_title("FR_Y")

    axs[3,0].plot(time_list, actions[:,6])
    axs[3,0].set_title("RL_X")

    axs[3,1].plot(time_list, actions[:,7])
    axs[3,1].set_title("RL_Y")

    axs[4,0].plot(time_list, actions[:,8])
    axs[4,0].set_title("RR_X")

    axs[4,1].plot(time_list, actions[:,9])
    axs[4,1].set_title("RR_Y")

    axs[5,0].plot(time_list, abs(actions[:,10]))
    axs[5,0].set_title("time delta")

    plt.show()
    
    ax1 = plt.subplot(211)
    ax1.plot(states[:,2], label="roll")
    ax1.plot(states[:,3], label="pitch")
    ax1.plot(states[:,4], label="yaw")
    ax1.legend()

    ax2 = plt.subplot(212)
    ax2.plot(states[:,0], label="x")
    ax2.plot(states[:,1], label="y")
    ax2.legend()

    plt.show()

