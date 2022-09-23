# import serial
import cv2
import pyrealsense2 as rs
import numpy as np
import yaml
import time
import readchar
import threading
from pupil_apriltags import Detector

at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

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

def eval_reward(tag_xyz_data, times):
    # print("Eval reward", tag_xyz_data)
    dist_array = []
    # print(imu_data)
    if len(tag_xyz_data) > 0:
        # print(tag_xyz_data[0].shape)

        for tag in range(1, len(tag_xyz_data)):
            d = float((tag_xyz_data[tag][0][:] - tag_xyz_data[tag-1][0][:]))**2 + float((tag_xyz_data[tag][1][:] - tag_xyz_data[tag-1][1][:]))**2 + float((tag_xyz_data[tag][2][:] - tag_xyz_data[tag-1][2][:]))**2
            dist_array.append(np.sqrt(d))

        # print(dist_array)
        # print(tag_xyz_data[len(tag_xyz_data)-1][0][:], tag_xyz_data[len(tag_xyz_data)-1][1][:] )
        distance = np.sum(dist_array)
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


while True:

    if (pause_flag % 2 == 0): # If the loop isn't paused:

        starttime = time.time()
        tag_xyz_data = []
        imu_data = []
        times = []
        print("start 30s wait")

        while (time.time() - starttime) < 3:
        
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
            # print(tags)

            if len(tags) != 0:
                tag_xyz = tags[0].pose_t
                tag_R = tags[0].pose_R
                tag_xyz = np.array(tag_xyz)
                tag_R = np.array(tag_R)
                # print(tag_xyz)

                tag_xyz_data.append(tag_xyz)
                times.append(time.time() - starttime)
        
        # print("end 3s wait")
        reward = eval_reward(tag_xyz_data, times)

        

    