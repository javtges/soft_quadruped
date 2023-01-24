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
    
    # policy.theta = np.load('epoch_66_1.9510125950834536.npy')
    policy.theta = np.load('/home/james/final_project/src/logs/beast_trial_6x11policy_epoch_161_0.4218193610265332.npy')
    print(policy.theta)
    
    normalizer = Normalizer(n_inputs)
    
    starttime = time.time()
    
    state = env.reset()
    step_number = 0
    
    while True:

        # print(traj_generators[0].width)
        tg_params = np.array([traj_generators[0].width, traj_generators[0].height], dtype=float)
        phase = np.array([traj_generators[0].phase])
        # print(state)
        state = state[2:]
        # print(state)
        state = np.concatenate((state, tg_params, phase), axis=0)
        # print(state)
        # Our state should be 15-dimensional: [x_pos, y_pos, roll, pitch, yaw, x_vel, y_vel, fl_w, fl_h, fr_w, fr_h, rl_w, rl_h, rr_w, rr_h]
        # print("the system's state", state, "length:", len(state))
        # Augment this to include the variables we need from the TG
        normalizer.observe(state)
        
        # THIS DOESN'T PROPERLY APPLY NOISE!!!
        state = normalizer.normalize(state)
        action = policy.evaluate(state, None, None, hp)

        # print("action", action)
        # Action is now 16-dimensional: [fl_w, fl_h, res_fl_x, res_fl_y, fr_w, fr_h, res_fr_x, res_fr_y, rl_w, rl_h, res_rl_x, res_rl_y, rr_w, rr_h, res_rr_x, res_rr_y]
        
        # print(action)
        eps_fl, theta_fl = traj_generators[0].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[2]), res_y=0.005*(action[3]), step_theta=24, step_time=abs(action[10]))
        eps_fr, theta_fr = traj_generators[1].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[4]), res_y=0.005*(action[5]), step_theta=24, step_time=abs(action[10]))
        eps_rl, theta_rl = traj_generators[2].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[6]), res_y=0.005*(action[7]), step_theta=24, step_time=abs(action[10]))
        eps_rr, theta_rr = traj_generators[3].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[8]), res_y=0.005*(action[9]), step_theta=24, step_time=abs(action[10]))
        
        # eps_fr, theta_fr = traj_generators[1].step_traj(width=0.02, height=0.01, res_x=action[6], res_y=action[7])
        # eps_fl, theta_fl = traj_generators[0].step_traj(width=0.02, height=0.01, res_x=action[2], res_y=action[3])
        # eps_rl, theta_rl = traj_generators[2].step_traj(width=0.02, height=0.01, res_x=action[10], res_y=action[11])
        # eps_rr, theta_rr = traj_generators[3].step_traj(width=0.02, height=0.01, res_x=action[14], res_y=action[15])


        # print(action[10])
        # Due to PMTG, our action now becomes... 9 + (x_val, y_val, width, height) * 4  = 25 dimensional

        # Change the variable "action" so that it's 9-dimensional (the shape of the environment's input), using the TG
        # Sample from all 4 trajectory generators, make an action from all of them
        # print(aaaaaaaaa)

        # Make sure that the order of legs here is correct
        actions_tg = [0, theta_fl, eps_fl, theta_fr, eps_fr, theta_rl, eps_rl, theta_rr, eps_rr]
        
        
        # Remove this for the 'faster' policy
        for i in range(24):
            # noise = np.random.normal(scale=0.01, size=len(actions_tg))
            # actions_tg += noise

            # actions_tg = [0, eps_fl, theta_fl, eps_fr, theta_fr, eps_rl, theta_rl, eps_rr, theta_rr]
            # Here, generate the trajectory from the trajectory generator. Use the actions
            # env.render()
            # print(num_plays)
            state, reward, done, _ = env.step(actions_tg)
            # print(reward)
            
            step_number += 1
        
        if step_number == 1200:
            print("Distance after 1200 simulation steps: ", state[0])