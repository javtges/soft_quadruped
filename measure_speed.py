"""Measures a speed of an HSA robot

This assumes that we have something already running and no serial communication is required.

All we do is write to a csv, and print out the speed of the robot

"""

import cv2
import pyrealsense2 as rs
import numpy as np
import yaml
import csv
import time
import readchar
import threading
from pupil_apriltags import Detector
import argparse
import modern_robotics as mr
from datetime import datetime

parser = argparse.ArgumentParser()
parser.add_argument('filename', help='output file name')
args = parser.parse_args()

filename = args.filename
now = datetime.now().strftime("%y%m%d_%H%M%S")
filename = filename + "_" + now + ".csv"

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

# Start streaming
cfg = pipeline.start(config)
profile = cfg.get_stream(rs.stream.color)
intr = profile.as_video_stream_profile().get_intrinsics()
print(intr)
pause_flag = 0
tag_xyz = np.zeros((3,1))

def start_csv(filename):
    n = filename   
    with open(n, 'a') as f:
        writer = csv.writer(f)
        t = datetime.now().strftime("%y%m%d_%H%M%S")
        # writer.writerow(['motor1', 'motor2', 'motor3', 'motor4', 'leg_trans_x', 'leg_trans_y', 'leg_trans_z', 'R_0_0', 'R_0_1', 'R_0_2', 'R_1_0', 'R_1_1', 'R_1_2','R_2_0', 'R_2_1', 'R_2_2', t])

def write_csv(filename, R):
    n = filename
    # Verify the ordering of the transform and everything is right
    with open(n, 'a') as f:
        writer = csv.writer(f)
        t = datetime.now().strftime("%y%m%d_%H%M%S")
        
        # Could be different ordering of motors
        writer.writerow([0, 0, 0, 0, R[0][3], R[1][3], R[2][3], R[0][0], R[0][1], R[0][2], R[1][0], R[1][1], R[1][2], R[2][0], R[2][1], R[2][2], t])


start_csv(filename)

while True:

        
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    
    if not depth_frame or not color_frame:
        continue

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', color_image)
    cv2.waitKey(delay=1)        
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    
    tags = at_detector.detect(gray, estimate_tag_pose=True, camera_params=[intr.fx, intr.fy, intr.ppx, intr.ppy], tag_size=0.055)
    # print(tags)

    if len(tags) != 0:
        tag_xyz = tags[0].pose_t
        tag_R = tags[0].pose_R
        tag_xyz = np.array(tag_xyz)
        tag_R = np.array(tag_R)
        print(tag_xyz)
        print(tag_R)
        camera_to_tag = mr.RpToTrans(tag_R, tag_xyz)

        write_csv(filename, camera_to_tag)

