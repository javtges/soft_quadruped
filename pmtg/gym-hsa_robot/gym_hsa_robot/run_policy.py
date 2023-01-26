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
import matplotlib.pyplot as plt
from datetime import datetime
from pupil_apriltags import Detector
from gym_hsa_robot.train_ars import Policy, Ellipse_TG, Normalizer
from gym_hsa_robot.resources.lookup_table_utils import LookupTable
from gym_hsa_robot.resources.trajectory_generator import make_circle
from scipy.spatial.transform import Rotation as R
import modern_robotics as mr

'''
Runs a policy on the robot, [closed loop]! Contains much of the same code from train_ars.py, takes in a .npy file for the policy.
'''

np.set_printoptions(suppress=True, formatter={'float_kind':'{:f}'.format})


at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

try:
    ser = serial.Serial('/dev/ttyACM1',115200,timeout=0.1)
except:                      
    ser = serial.Serial('/dev/ttyACM0',115200,timeout=0.1)

print('Opening port: ')
print(ser.name)

# Realsense stuff
################################################
pipeline = rs.pipeline()
config = rs.config()
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

#########################################

now = datetime.now().strftime("%y%m%d_%H%M%S")

##########################################
filename = 'policytest_' + now

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

def write_csv(filename, R, nums):
    
    '''
    Writes the observation, commands, and time to a CSV file.
    '''
    n = filename
    # Verify the ordering of the transform and everything is right
    with open(n, 'a') as f:
        writer = csv.writer(f)
        t = datetime.now().strftime("%y%m%d_%H%M%S")
        
        # Could be different ordering of motors
        writer.writerow([0, 0, 0, 0, R[0][3], R[1][3], R[2][3], R[0][0], R[0][1], R[0][2], R[1][0], R[1][1], R[1][2], R[2][0], R[2][1], R[2][2],
                         nums[0], nums[1], nums[2], nums[3], nums[4], nums[5], nums[6], nums[7], t])


# Start streaming
cfg = pipeline.start(config)
profile = cfg.get_stream(rs.stream.color)
intr = profile.as_video_stream_profile().get_intrinsics()
print(intr)
pause_flag = 0
tag_xyz = np.zeros((3,1))
   
def inverse(n):
    return 180-n
   
   
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
    
    # Make sure this is periodic in the correct way
    TG_fl = Ellipse_TG()
    
    TG_fr = Ellipse_TG()
    TG_fr.phase = TG_fr.cycle_length/2

    TG_rl = Ellipse_TG()
    TG_rl.phase = TG_rl.cycle_length/2

    TG_rr = Ellipse_TG()

    tg_arr = [TG_fl, TG_fr, TG_rl, TG_rr]

    # print("seed = ", hp.seed)
    np.random.seed(42)

    # make the environment
    env = gym.make("hsa_robot-v0")

    # number of inputs: number of columns
    # number of outputs: number of rows
    n_inputs = env.observation_space.shape[0] + TG_fl.n_params + 1 - 2
    # THIS DOESN'T EVEN NEED THE ACTION SPACE TO WORK! ONLY NEEDS TRAJ PARAMS
    # n_outputs = env.action_space =.shape[0] + 8 + TG_fl.n_params*4
    n_outputs = 8 + TG_fl.n_params + 1

    print("Observation space =", n_inputs)
    print("Action space =", n_outputs)

    policy = Policy(input_size=n_inputs, output_size=n_outputs,
                    env_name="hsa_robot-v0", traj_generator=tg_arr)
    
    policy.theta = np.load('/home/james/final_project/src/logs/beast_trial_6x11policy_epoch_15_0.331714094411355.npy')
    # policy.theta = np.random.random((11,6))
    
    
    normalizer = Normalizer(n_inputs)
    
    lut = LookupTable(lookup_table_filename='/home/james/final_project/src/lookup_table_unique2.csv')

    ##############################################
    print("x radius ", lut.width/2, "y radius ", lut.eps/2)
    x_cir, y_cir = make_circle(0, -0.074, 0.015, 0.003, 10)
    y_cir = [i+0.07 for i in y_cir]
    print("x circle, y circle", x_cir, y_cir)

    
    num1, num2 = lut.interpolate_bilinear(x_cir, y_cir)
    # num1, num2 = lut.interpolate_bilinear(x, y)

    x, y = lut.interpolate_bilinear_xy(x_cir, y_cir)

    lut_y = [-1 * a for a in lut.y]
    y_cir = [-1 * a for a in y_cir]
    y = [-1 * a for a in y]

    print(num1, num2)

    plt.scatter(lut.x, lut_y, s=2, label="Lookup Table Values")
    plt.scatter(x_cir,y_cir, label="Desired Circle Values")
    plt.scatter(x,y, label="Interpolated Circle Values")
    plt.title("Lookup Table with Desired Circle and Interpolation")
    plt.xlabel("X Coordinate (m)")
    plt.ylabel("Y Coordinate (m)")
    plt.legend()
    # plt.scatter(x,y)
    plt.show()
    ##################################################




    tag_buffer = []
    time_buffer = [] 
    prev_time = time.time()   
    first_frame = True
    euler_zero = np.zeros(3)
    q_first = np.zeros(3)
    
    identity = np.array([[1, 0, 0, 0],
                        [0, 1, 0 ,0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
    
    while True:

        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
            
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        tags = at_detector.detect(gray, estimate_tag_pose=True, camera_params=[intr.fx, intr.fy, intr.ppx, intr.ppy], tag_size=0.055)

        if len(tags) != 0:
            print("found tag")
            tag_xyz = tags[0].pose_t
            tag_R = tags[0].pose_R
            tag_xyz = np.array(tag_xyz)
            tag_R = np.array(tag_R)
            
            r = R.from_matrix(tag_R)
            q = r.as_quat()
            
            # print(q)
            # euler = r.as_euler('xyz', degrees=False)
            
            camera_to_tag = mr.RpToTrans(tag_R, tag_xyz)
            if first_frame:
                zero_to_cam = mr.TransInv(camera_to_tag)
                q_first = q
                transform = identity
                e = R.from_quat(q_first)
                eye = np.eye(3)
                eye_r = R.from_matrix(eye)
                euler = eye_r.as_euler("xyz", degrees=False)
                # print("AAAAAAA", euler)
                # print("first transform: ", zero_to_cam)
                first_frame = False
            else:
                transform = zero_to_cam @ camera_to_tag # Zero to tag transform?
                # print("zero to tag: ", transform)
                tag_R2 = transform[0:3, 0:3]
                # print(tag_R2)
                # r2 = R.from_matrix(tag_R2)
                # res = tag_R2 @ tag_R
                res = R.from_matrix(tag_R2)
                euler = res.as_euler("XYZ", degrees=False)
                print("Euler Angles, Relative:", euler)
                

            # Make a measurement in the format of the environment's observation space
            # observation = np.zeros((policy.output_size,))
            
            state = np.array([euler[0], euler[1], euler[2]]) # MAKE THIS THE POS, ORI, VEL -> changed to orientation only
                        
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
            
            eps_fl, theta_fl, x_fl, y_fl = tg_arr[0].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[2]), res_y=0.005*(action[3]), step_theta=24, step_time = time_delta + abs(action[10]))
            eps_fr, theta_fr, x_fr, y_fr = tg_arr[1].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[4]), res_y=0.005*(action[5]), step_theta=24, step_time = time_delta + abs(action[10]))
            eps_rl, theta_rl, x_rl, y_rl = tg_arr[2].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[6]), res_y=0.005*(action[7]), step_theta=24, step_time = time_delta + abs(action[10]))
            eps_rr, theta_rr, x_rr, y_rr = tg_arr[3].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[8]), res_y=0.005*(action[9]), step_theta=24, step_time = time_delta + abs(action[10]))
           
            # eps_fl, theta_fl, x_fl, y_fl = tg_arr[0].step_traj(width=0.015, height=0.003, step_theta=24)
            # eps_fr, theta_fr, x_fr, y_fr = tg_arr[1].step_traj(width=0.015, height=0.003, step_theta=24)
            # eps_rl, theta_rl, x_rl, y_rl = tg_arr[2].step_traj(width=0.015, height=0.003, step_theta=24)
            # eps_rr, theta_rr, x_rr, y_rr = tg_arr[3].step_traj(width=0.015, height=0.003, step_theta=24)


            # print("front left ", x_fl, y_fl, tg_arr[0].phase )
            # y_fl += 0.07
            
           
            # Due to PMTG, our action now becomes... 9 + (x_val, y_val, width, height) * 4  = 25 dimensional

            # Change the variable "action" so that it's 9-dimensional (the shape of the environment's input), using the TG
            # Sample from all 4 trajectory generators, make an action from all of them
            # print(aaaaaaaaa)

            # Make sure that the order of legs here is correct
            # actions_tg = [0, eps_fl, theta_fl, eps_fr, theta_fr, eps_rl, theta_rl, eps_rr, theta_rr]
            
            # # Convert eps and theta to XY, then to motor commands
            # x_fl, y_fl = tg_arr[0].joints_to_xy_legframe(theta_fl, eps_fl)
            # x_fr, y_fr = tg_arr[1].joints_to_xy_legframe(theta_fr, eps_fr)
            # x_rl, y_rl = tg_arr[2].joints_to_xy_legframe(theta_rl, eps_rl)
            # x_rr, y_rr = tg_arr[3].joints_to_xy_legframe(theta_rr, eps_rr)
            
            n1_fl, n2_fl = lut.interpolate_bilinear([x_fl], [y_fl+0.07])
            n1_fr, n2_fr = lut.interpolate_bilinear([x_fr], [y_fr+0.07])
            n1_rl, n2_rl = lut.interpolate_bilinear([x_rl], [y_rl+0.07])
            n1_rr, n2_rr = lut.interpolate_bilinear([x_rr], [y_rr+0.07])
            
            params = [n1_fr[0], n2_fr[0], n1_fl[0], n2_fl[0], n1_rl[0], n2_rl[0], n1_rr[0], n2_rr[0]]
           
            params = [round(90 + (n - 90)*0.8) for n in params]
            # params = np.clip(params, 70, 150)
            # print(params)
            print("transform:", transform[0][3], transform[1][3], transform[2][3])
            write_csv(filename, transform, params)
            
            # This is according to the notebook
            send_policy(params)
            
            # Change this so that it sleeps the right amount of time to send commands every 0.1s
            time.sleep(0.1)