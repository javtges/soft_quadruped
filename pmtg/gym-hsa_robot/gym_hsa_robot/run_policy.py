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
from gym_hsa_robot.train_ars import Policy, Ellipse_TG, Normalizer
from gym_hsa_robot.resources.lookup_table_utils import LookupTable
from scipy.spatial.transform import Rotation as R

'''
Runs a policy on the robot, [closed loop]
'''

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

def log_csv(data, now):
    filename = 'policytest_' + now
    with open(filename, 'a') as f:
        writer = csv.writer(f)
        writer.writerow(data)

def send_policy(policy):

    print("Sending Policy", policy)
    for i in range(len(policy)):
        if policy[i] < 0:
            policy[i] = 0
        if policy[i] > 180:
            policy[i] = 180
        policy[i] = int(policy[i])
    # policy = np.multiply(policy, 100) # Move decimal place 3 to the left
    print("Intermediate Policy", policy)
    # policy = [int(x)%10000 for x in policy] # always only 4 digits long
    policy = [str(x).zfill(4) for x in policy]
    str_policy = str(policy)+'\n'
    print("Updated Policy", str_policy)
    
    ser.write(str_policy.encode())

def make_policies(params, eps):
    R_list = np.zeros((t,p_count))
    eps_list = np.zeros((t,p_count))
    for row in range(t):
        e = np.random.choice([-eps,0,eps], 16)
        eps_list[row][:] = e
        R_list[row][:] = e + params

    return R_list, eps_list

# def eval_reward(tag_xyz_data, times, params):
#     # print("Eval reward", tag_xyz_data)
#     dist_array = []
#     # print(imu_data)
#     if len(tag_xyz_data) > 0:
#         # print(tag_xyz_data[0].shape)
#         # for tag in range(1, len(tag_xyz_data)):
#         #     d = float((tag_xyz_data[tag][0][:] - tag_xyz_data[tag-1][0][:]))**2 + float((tag_xyz_data[tag][1][:] - tag_xyz_data[tag-1][1][:]))**2 + float((tag_xyz_data[tag][2][:] - tag_xyz_data[tag-1][2][:]))**2
#         #     dist_array.append(np.sqrt(d))

#         d = float((tag_xyz_data[-1][0][:] - tag_xyz_data[0][0][:]))**2 + float((tag_xyz_data[-1][1][:] - tag_xyz_data[0][1][:]))**2 + float((tag_xyz_data[-1][2][:] - tag_xyz_data[0][2][:]))**2
#         # print(dist_array)
#         # print(tag_xyz_data[len(tag_xyz_data)-1][0][:], tag_xyz_data[len(tag_xyz_data)-1][1][:] )
#         distance = np.sqrt(d) # np.sum(dist_array)
#         print("distance in m", distance)
#         distance = distance/(times[-1] - times[0])
#         print("distance in m/s", distance, times[-1] - times[0])
#     else:
#         distance = 0
        
#     print(params)
#     data = list(params)
#     data.append(distance)
#     data.append(time.time())
    
#     log_csv(data, now)
#     return distance

# Start streaming
cfg = pipeline.start(config)
profile = cfg.get_stream(rs.stream.color)
intr = profile.as_video_stream_profile().get_intrinsics()
print(intr)
pause_flag = 0
tag_xyz = np.zeros((3,1))

# def eval_policy(duration, params):

#     starttime = time.time()
#     tag_xyz_data = []
#     x_data = []
#     y_data = []
#     z_data = []
#     imu_data = []
#     times = []
#     print("start eval wait")

#     while (time.time() - starttime) < duration:
    
#         frames = pipeline.wait_for_frames()
#         depth_frame = frames.get_depth_frame()
#         color_frame = frames.get_color_frame()
        
#         if not depth_frame or not color_frame:
#             continue

#         # Convert images to numpy arrays
#         depth_image = np.asanyarray(depth_frame.get_data())
#         color_image = np.asanyarray(color_frame.get_data())
            
#         gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

#         tags = at_detector.detect(gray, estimate_tag_pose=True, camera_params=[intr.fx, intr.fy, intr.ppx, intr.ppy], tag_size=0.055)

#         if len(tags) != 0:
#             tag_xyz = tags[0].pose_t
#             tag_R = tags[0].pose_R
#             tag_xyz = np.array(tag_xyz)
#             tag_R = np.array(tag_R)
            
#             r = R.from_matrix(tag_R)
#             euler = r.as_euler('zyx', degrees=False)
#             # print(tag_xyz)

#     print("end eval wait")
#     reward = eval_reward(tag_xyz_data, times, params)
#     return reward
   
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
    n_inputs = env.observation_space.shape[0] + TG_fl.n_params + 1
    
    # THIS DOESN'T EVEN NEED THE ACTION SPACE TO WORK! ONLY NEEDS TRAJ PARAMS
    # n_outputs = env.action_space.shape[0] + 8 + TG_fl.n_params*4
    n_outputs = 8 + TG_fl.n_params

    print("Observation space =", n_inputs)
    print("Action space =", n_outputs)

    policy = Policy(input_size=n_inputs, output_size=n_outputs,
                    env_name="hsa_robot-v0", traj_generator=tg_arr)
    
    policy.theta = np.load('test.npy')
    
    normalizer = Normalizer(n_inputs)
    
    lut = LookupTable(lookup_table_filename='/home/james/final_project/src/table_221107_212842__no_reset_0')


    tag_buffer = []
    time_buffer = [] 
    starttime = time.time()   
    first_frame = True
    
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
            tag_xyz = tags[0].pose_t
            tag_R = tags[0].pose_R
            tag_xyz = np.array(tag_xyz)
            tag_R = np.array(tag_R)
            
            r = R.from_matrix(tag_R)
            euler = r.as_euler('xyz', degrees=False)
            
            camera_to_tag = mr.RpToTrans(tag_R, tag_xyz)
            if first_frame:
                zero_to_cam = mr.TransInv(camera_to_tag)
                transform = identity
            else:
                transform = zero_to_cam @ camera_to_tag

            # Make a measurement in the format of the environment's observation space
            # observation = np.zeros((policy.output_size,))
            
            state = np.array([transform[0][3], transform[1][3], euler[0], euler[1], euler[2]]) # MAKE THIS THE POS, ORI, VEL
                        
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
            
            
            
            eps_fl, theta_fl = tg_arr[0].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[2]), res_y=0.005*(action[3]), step_time=abs(action[10]))
            eps_fr, theta_fr = tg_arr[1].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[4]), res_y=0.005*(action[5]), step_time=abs(action[10]))
            eps_rl, theta_rl = tg_arr[2].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[6]), res_y=0.005*(action[7]), step_time=abs(action[10]))
            eps_rr, theta_rr = tg_arr[3].step_traj(width=0.015*(1+action[0]), height=0.003*(1+action[1]), res_x=0.023*(action[8]), res_y=0.005*(action[9]), step_time=abs(action[10]))
            # Due to PMTG, our action now becomes... 9 + (x_val, y_val, width, height) * 4  = 25 dimensional

            # Change the variable "action" so that it's 9-dimensional (the shape of the environment's input), using the TG
            # Sample from all 4 trajectory generators, make an action from all of them
            # print(aaaaaaaaa)

            # Make sure that the order of legs here is correct
            actions_tg = [0, eps_fl, theta_fl, eps_fr, theta_fr, eps_rl, theta_rl, eps_rr, theta_rr]
            
            # Convert eps and theta to XY, then to motor commands
            x_fl, y_fl = tg_arr[0].joints_to_xy_legframe(theta_fl, eps_fl)
            x_fr, y_fr = tg_arr[1].joints_to_xy_legframe(theta_fr, eps_fr)
            x_rl, y_rl = tg_arr[2].joints_to_xy_legframe(theta_rl, eps_rl)
            x_rr, y_rr = tg_arr[3].joints_to_xy_legframe(theta_rr, eps_rr)
            
            n1_fl, n2_fl = lut.interpolate_bilinear(x_fl, y_fl)
            n1_fr, n2_fr = lut.interpolate_bilinear(x_fr, y_fr)
            n1_rl, n2_rl = lut.interpolate_bilinear(x_rl, y_rl)
            n1_rr, n2_rr = lut.interpolate_bilinear(x_rr, y_rr)
            
            
            # This is wrong, fix it
            send_policy([n1_fl, n2_fl, n1_fr, n2_fr, n1_rl, n2_rl, n2_rr, n2_rr])
            
            # Change this so that it sleeps the right amount of time to send commands every 0.1s
            # time.sleep(0.1)