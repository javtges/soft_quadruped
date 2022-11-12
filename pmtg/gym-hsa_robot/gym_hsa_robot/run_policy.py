import serial
import cv2
import pyrealsense2 as rs
import numpy as np
import yaml
import time
import readchar
import struct
import csv
from datetime import datetime
from pupil_apriltags import Detector
from gym_hsa_robot.train_ars import Policy, Ellipse_TG, Normalizer
from gym_hsa_robot.resources.lookup_table_utils import LookupTable
from scipy.spatial.transform import Rotation as R



at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

try:
    ser = serial.Serial('/dev/ttyACM1',115200,rtscts=1)
except:                      
    ser = serial.Serial('/dev/ttyACM0',115200,rtscts=1)

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

# imu_data = []
# tag_xyz_data= []
# p_count = 16
# # params = np.ones((p_count,))*90 # Our initial parameter vector pi
# params = np.random.randint(181, size=16)
# # params = np.array([0, 180, 180, 0, 0, 180, 0, 180, 0, 180, 180, 0, 0, 180, 0, 180]) # Our initial parameter vector pi
# t = 10
# eps = 15
# step_size = 5
now = datetime.now().strftime("%y%m%d_%H%M%S")

##########################################

def log_csv(data, now):
    filename = 'trial_' + now
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

# Start streaming
cfg = pipeline.start(config)
profile = cfg.get_stream(rs.stream.color)
intr = profile.as_video_stream_profile().get_intrinsics()
print(intr)
pause_flag = 0
tag_xyz = np.zeros((3,1))

def eval_policy(duration, params):

    starttime = time.time()
    tag_xyz_data = []
    x_data = []
    y_data = []
    z_data = []
    imu_data = []
    times = []
    print("start eval wait")

    while (time.time() - starttime) < duration:

        # data = ser.readline()
        # imu = []
        # data = data.split(b', ',-1)

        # for x in data:
        #     imu.append(float(x))
        # # print(imu)
        # # [M.x, M.y, M.z, G.x, G.y, G.z, A.x, A.y, A.z, Pitch (deg), Roll (deg), Heading (deg)]
        # imu_data.append(np.array(imu))
    
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        # print("test")
        
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        
        # Sometimes this seems to freeze everything
        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', color_image)
        # cv2.waitKey(1)
        
        # print(color_image)
            
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        tags = at_detector.detect(gray, estimate_tag_pose=True, camera_params=[intr.fx, intr.fy, intr.ppx, intr.ppy], tag_size=0.055)

        if len(tags) != 0:
            tag_xyz = tags[0].pose_t
            tag_R = tags[0].pose_R
            tag_xyz = np.array(tag_xyz)
            tag_R = np.array(tag_R)
            
            r = R.from_matrix(tag_R)
            euler = r.as_euler('zyx', degrees=False)
            # print(tag_xyz)
            
            # tag_xyz_data.append(tag_xyz)
            # times.append(time.time() - starttime)

    print("end eval wait")
    reward = eval_reward(tag_xyz_data, times, params)
    return reward
   
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
    TG_fl = Ellipse_TG()
    
    TG_fr = Ellipse_TG()
    TG_fr.phase = TG_fr.cycle_length/2

    TG_rl = Ellipse_TG()
    TG_rl.phase = TG_rl.cycle_length/2

    TG_rr = Ellipse_TG()

    tg_arr = [TG_fl, TG_fr, TG_rl, TG_rr]

    print("seed = ", hp.seed)
    np.random.seed(hp.seed)

    # make the environment
    env = gym.make("hsa_robot-v0")

    # number of inputs: number of columns
    # number of outputs: number of rows
    n_inputs = env.observation_space.shape[0] + TG_fl.n_params*4
    
    # THIS DOESN'T EVEN NEED THE ACTION SPACE TO WORK! ONLY NEEDS TRAJ PARAMS
    # n_outputs = env.action_space.shape[0] + 8 + TG_fl.n_params*4
    n_outputs = 8 + TG_fl.n_params*4

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
            euler = r.as_euler('zyx', degrees=False)
            tag_buffer.append(tag_xyz)
            time_buffer.append(time.time() - starttime)
            
        # Make a measurement in the format of the environment's observation space
        # observation = np.zeros((policy.output_size,))
        
        if len(tag_buffer) > 5:
            velocity = (tag_buffer[-1] - tag_buffer[-4]) / (time_buffer[-1], time_buffer[-4])
        else:
            velocity = np.array([0,0,0])
        
        
        state = np.array([tag_xyz[0], tag_xyz[1], np.cos(euler[0]), np.sin(euler[0]), velocity[0], velocity[1]]) # MAKE THIS THE POS, ORI, VEL
        
        # How to calculate velocity? Maybe: use the last N observations to find it rather than a single one
        
        # Evaluate the policy to get an action
        
        # print(traj_generators[0].width)
        tg_params = np.array([traj_generators[0].width, traj_generators[0].height,
                              traj_generators[1].width, traj_generators[1].height,
                              traj_generators[2].width, traj_generators[2].height,
                              traj_generators[3].width, traj_generators[3].height], dtype=float)
        state = np.concatenate((state, tg_params), axis=0)
        # print(state)
        # Augment this to include the variables we need from the TG
        
        normalizer.observe(state)
        state = normalizer.normalize(state)
        action = policy.evaluate(state, delta=None, direction=None, hp=None)
        
        time_delta = time_buffer[-1] - time_buffer[-2]
        
        # print("leg1")
        eps_fl, theta_fl = traj_generators[0].step_traj(width=action[0], height=action[1], res_x=action[2], res_y=action[3], step_time=time_delta)
        # print("leg2")
        eps_fr, theta_fr = traj_generators[1].step_traj(width=action[4], height=action[5], res_x=action[6], res_y=action[7], step_time=time_delta)
        # print("leg3")
        eps_rl, theta_rl = traj_generators[2].step_traj(width=action[8], height=action[9], res_x=action[10], res_y=action[11], step_time=time_delta)
        # print("leg4")
        eps_rr, theta_rr = traj_generators[3].step_traj(width=action[12], height=action[13], res_x=action[14], res_y=action[15], step_time=time_delta)

        # Due to PMTG, our action now becomes... 9 + (x_val, y_val, width, height) * 4  = 25 dimensional

        # Change the variable "action" so that it's 9-dimensional (the shape of the environment's input), using the TG
        # Sample from all 4 trajectory generators, make an action from all of them
        # print(aaaaaaaaa)

        # Make sure that the order of legs here is correct
        actions_tg = [0, eps_fl, theta_fl, eps_fr, theta_fr, eps_rl, theta_rl, eps_rr, theta_rr]
        
        # Convert eps and theta to XY, then to motor commands
        x_fl, y_fl = traj_generators[0].joints_to_xy_legframe(theta_fl, eps_fl)
        x_fr, y_fr = traj_generators[1].joints_to_xy_legframe(theta_fr, eps_fr)
        x_rl, y_rl = traj_generators[2].joints_to_xy_legframe(theta_rl, eps_rl)
        x_rr, y_rr = traj_generators[3].joints_to_xy_legframe(theta_rr, eps_rr)
        
        n1_fl, n2_fl = lut.interpolate_with_xy(x_fl, y_fl)
        n1_fr, n2_fr = lut.interpolate_with_xy(x_fr, y_fr)
        n1_rl, n2_rl = lut.interpolate_with_xy(x_rl, y_rl)
        n1_rr, n2_rr = lut.interpolate_with_xy(x_rr, y_rr)
        
        
        # This is wrong, fix it
        send_policy([n1_fl, n2_fl, n1_fr, n2_fr, n1_rl, n2_rl, n2_rr, n2_rr])