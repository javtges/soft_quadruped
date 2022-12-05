import serial
import cv2
import pyrealsense2 as rs
import numpy as np
import yaml
import time
import readchar
import gym
import struct
import csv
from datetime import datetime
from pupil_apriltags import Detector
from gym_hsa_robot.train_ars import Policy, Ellipse_TG, Normalizer, Hp
from gym_hsa_robot.resources.lookup_table_utils import LookupTable
from scipy.spatial.transform import Rotation as R

'''
Runs a policy on the robot, in pybullet
'''


# test = np.array([[0.05548582,0.02778726, -0.01589502, -0.06374404, -0.08504553, -0.0641136, 0.0776632, -0.01730579,0.10843946,0.03597338,0.16130424,0.00210581,-0.06405972,0.02267876, -0.13164678],
#  [0.02848594, -0.02683596, -0.19838869,0.29124055,0.10038628,0.01980106, 0.1319754,-0.22638672, -0.05542291, -0.03173225,0.15285969,0.16722815,-0.02629527,0.0644462,-0.00946523],
#  [-0.00060483, -0.02187964,0.09791141, -0.22468503, -0.06494231,0.09037158, 0.07670161, -0.06110922, -0.09655587,0.00454294, -0.18251117,0.14209855,-0.07728468, -0.07052926,0.0474957],
#  [0.13814465, -0.06073175,0.128225,0.11873651,0.11415548, -0.20250982,0.01757645, -0.36515054,0.01012128,0.06888453,0.0156512,-0.05329776,0.06211236,0.02860703,0.12729523],
#  [-0.04836605,0.00095809, -0.00202538, -0.13966033,0.09955836,0.01663016,-0.23158175,0.0156818,-0.09565041,0.03201035,0.09822361, -0.09454711,-0.1074514,-0.0638071, 0.1520668],
#  [-0.15017977,0.03273196, -0.07257175,0.24410137, -0.04956509,0.07540333, 0.00223938,0.14363916,0.15258964,0.01201355,0.00599402, -0.10085107,-0.07844161,0.03752311, -0.04527575],
#  [0.03470215, -0.04733576,0.16142548, -0.13937673, -0.09426414,0.06623054,-0.04128143,0.06739952, -0.00135451, -0.06765481, -0.12590869, -0.03249442, 0.0837373, 0.06530871, -0.0655105],
#  [0.06749868, -0.07697757,0.06174701,0.04862939, -0.02807671,0.09077639, 0.11329789,0.11512266,0.12177555, -0.05751877, -0.04180153,0.01602611,-0.30260731,0.16748067,0.13528352],
#  [-0.09993128, -0.0181117, 0.1213119, 0.00312246,0.16375998,0.14131907,-0.06779393, -0.14632752,0.02663868,0.1211977,-0.07874915, -0.06697787,-0.11163643,0.00169809,0.02749327],
#  [0.07910397,0.07217629, -0.06027431, -0.03026013,0.03685247,0.03135176,-0.07492236, -0.0092189,-0.11363768,0.04397194, -0.11787862,0.11638195,-0.07853223, -0.05285147,0.02659188],
#  [-0.00123026, -0.04679833,0.09654201,0.09560437, -0.1478075,-0.03989413,-0.07542643, -0.0354101, 0.05295304,0.17503728, -0.05489992,0.03140681, 0.15410353, -0.04881198, -0.23250205],
#  [-0.07351681,0.01750389,0.18641236, -0.03505071,0.0246363,-0.11077128, 0.12480808, -0.17637541,0.04914335,0.07420309,0.00052696,0.0665065,-0.05338109, -0.03074401, -0.24273489],
#  [-0.01447765,0.05137585,0.12045024, -0.05132037, -0.01794276,0.12601692, 0.18094388,0.05407477,0.00139147,0.16423118, -0.13512227,0.04130758,-0.0603018, 0.14080254, -0.076556,],
#  [0.08634523,0.12423814,0.04076784,0.20576327,0.02930062, -0.2201346,-0.05495059, -0.09960871,0.08881476,0.14366781,0.0936378, 0.15096616, 0.21388115, -0.09428805, -0.0205556],
#  [0.2089616,-0.09109709,0.03556977,0.06316592, -0.05565412,0.09959853,-0.0308624,-0.02279974, -0.06513741, -0.11034112,0.11761183, -0.05551424, 0.069417, -0.19547076, -0.03133406],
#  [0.03744193, -0.09422897,0.02929889,0.09522358,0.02906259,0.03210397,-0.03976357, -0.01716436, -0.12630022,0.05875975,0.0497502, 0.01313485, 0.01547067, -0.11107397,0.07053203]])
# print(test.shape)



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
    n_inputs = env.observation_space.shape[0] + TG_fl.n_params + 1
    
    # THIS DOESN'T EVEN NEED THE ACTION SPACE TO WORK! ONLY NEEDS TRAJ PARAMS
    # n_outputs = env.action_space.shape[0] + 8 + TG_fl.n_params*4
    n_outputs = 8 + TG_fl.n_params + 1

    print("Observation space =", n_inputs)
    print("Action space =", n_outputs)

    policy = Policy(input_size=n_inputs, output_size=n_outputs,
                    env_name="hsa_robot-v0", traj_generator=traj_generators)
    
    policy.theta = np.load('epoch_12_1.444586428300241.npy')
    
    normalizer = Normalizer(n_inputs)
    
    starttime = time.time()
    
    state = env.reset()
    step_number = 0
    
    while True:

        # print(traj_generators[0].width)
        tg_params = np.array([traj_generators[0].width, traj_generators[0].height], dtype=float)
        phase = np.array([traj_generators[0].phase])
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
        
        eps_fl, theta_fl = traj_generators[0].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[2]), res_y=0.005*(action[3]), step_time=abs(action[10]))
        eps_fr, theta_fr = traj_generators[1].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[4]), res_y=0.005*(action[5]), step_time=abs(action[10]))
        eps_rl, theta_rl = traj_generators[2].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[6]), res_y=0.005*(action[7]), step_time=abs(action[10]))
        eps_rr, theta_rr = traj_generators[3].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[8]), res_y=0.005*(action[9]), step_time=abs(action[10]))
        
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
        
        
        noise = np.random.normal(scale=0.01, size=len(actions_tg))
        actions_tg += noise

        # actions_tg = [0, eps_fl, theta_fl, eps_fr, theta_fr, eps_rl, theta_rl, eps_rr, theta_rr]
        # Here, generate the trajectory from the trajectory generator. Use the actions
        # env.render()
        # print(num_plays)
        state, reward, done, _ = env.step(actions_tg)
        # print(reward)
        
        step_number += 1
        
        if step_number == 1200:
            print("Distance after 1200 simulation steps: ", state[0])