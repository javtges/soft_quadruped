import serial
import cv2
import pyrealsense2 as rs
import numpy as np
import yaml
import time
import readchar
import struct
from pupil_apriltags import Detector

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


pipeline = rs.pipeline()
config = rs.config()
# Get device product line for setting a supporting resolution
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

imu_data = []
tag_xyz_data= []
p_count = 16
params = np.ones((p_count,))*100 # Our initial parameter vector pi
t = 10
eps = 5
step_size = 5

##########################################

def send_policy(policy):

    print("Sending Policy", policy)
    for i in range(len(policy)):
        if policy[i] < 0:
            policy[i] = 0
    policy = np.multiply(policy, 100) # Move decimal place 3 to the left
    policy = [int(x)%10000 for x in policy] # always only 4 digits long
    policy = [str(x).zfill(4) for x in policy]
    
    ser.write((str(policy)+'\n').encode())


def make_policies(params, eps):
    R_list = np.zeros((t,p_count))
    eps_list = np.zeros((t,p_count))
    for row in range(t):
        e = np.random.choice([-eps,0,eps], 16)
        eps_list[row][:] = e
        R_list[row][:] = e + params

    return R_list, eps_list

def eval_reward(tag_xyz_data, times):
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
    return distance

# Start streaming
cfg = pipeline.start(config)
profile = cfg.get_stream(rs.stream.color)
intr = profile.as_video_stream_profile().get_intrinsics()
print(intr)
pause_flag = 0
tag_xyz = np.zeros((3,1))

def eval_policy(duration):

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
        
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
            
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        tags = at_detector.detect(gray, estimate_tag_pose=True, camera_params=[intr.fx, intr.fy, intr.ppx, intr.ppy], tag_size=0.040)

        if len(tags) != 0:
            tag_xyz = tags[0].pose_t
            tag_R = tags[0].pose_R
            tag_xyz = np.array(tag_xyz)
            tag_R = np.array(tag_R)
            # print(tag_xyz)
            
            tag_xyz_data.append(tag_xyz)
            times.append(time.time() - starttime)

    print("end eval wait")
    reward = eval_reward(tag_xyz_data, times)
    return reward
    
print("Starting Training")
for epoch in range(10):

    R_list, eps_list = make_policies(params, eps)
    print(R_list)
    rewards = []
    A = np.zeros_like(params)
    for policy in range(len(R_list)):

        # Send the policy data from R_list to the robot
        send_policy(R_list[policy])
        time.sleep(0.5)
        reward = eval_policy(5)
        print("Policy Reward", reward)
        rewards.append(reward)

    for col in range(p_count): # for each parameter
        lower = []
        upper = []
        zero = []
        for row in range(t): # for each policy
            if eps_list[row][col] > 0: # If the perturbation for this dimension is positive/negative/zero:
                upper.append(rewards[row])
            elif eps_list[row][col] == 0:
                zero.append(rewards[row])
            elif eps_list[row][col] < 0:
                lower.append(rewards[row])

        low_avg = 0
        zero_avg = 0
        up_avg = 0

        # Find the averages
        if len(lower) > 0:
            low_avg = np.average(lower)
        if len(zero) > 0:
            zero_avg = np.average(zero)
        if len(upper) > 0:
            up_avg = np.average(upper)

        # print(low_avg, zero_avg, up_avg)
        if zero_avg > low_avg and zero_avg > up_avg:
            A[col] = 90 # This is effectively "0"
        else:
            A[col] = up_avg - low_avg

    print(A)
    # r = max(np.sum(A), 0.001)
    # print(r)
    A = (A / max(np.sum(A), 0.001) ) * step_size

    params += A
    # Send the policy data from params to the robot
    send_policy(params)
    time.sleep(0.5)
    logged_reward = eval_policy(10)
    print(logged_reward)
    print(params)
    print(f"Epoch {epoch+1}, reward {logged_reward}, params {params}")
