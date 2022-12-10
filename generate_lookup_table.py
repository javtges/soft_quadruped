import csv
import numpy as np
from pupil_apriltags import Detector
import pyrealsense2 as rs
import cv2
import time
import serial
import modern_robotics as mr
from datetime import datetime


'''
Makes a lookup table, used in conjunction with planar_Sweep.ino. Writes the resulting table to a CSV file.
'''

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
    print("No camera found!!!")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
cfg = pipeline.start(config)
profile = cfg.get_stream(rs.stream.color)
intr = profile.as_video_stream_profile().get_intrinsics()
print(intr)
pause_flag = 0
tag_xyz = np.zeros((3,1))
zero_to_cam = np.array([[1, 0, 0, 0],
                        [0, 1, 0 ,0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

identity = np.array([[1, 0, 0, 0],
                        [0, 1, 0 ,0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])


range1 = np.linspace(0, 180, 19)
range2 = np.linspace(0, 180, 19)
# video = cv2.VideoCapture(0)


now = datetime.now().strftime("%y%m%d_%H%M%S")
lut_count = 0
filename = 'table_' + now + "_"
filename_zero = filename + "_no_reset_"
zero_transform_flag = True


def start_csv(lut_count, filename):
    
    '''
    Starts a csv file with filename
    '''
    n = filename + str(lut_count)   
    with open(n, 'a') as f:
        writer = csv.writer(f)
        t = datetime.now().strftime("%y%m%d_%H%M%S")
        writer.writerow(['motor1', 'motor2', 'motor3', 'motor4', 'leg_trans_x', 'leg_trans_y', 'leg_trans_z', 'R_0_0', 'R_0_1', 'R_0_2', 'R_1_0', 'R_1_1', 'R_1_2','R_2_0', 'R_2_1', 'R_2_2', t])


def write_csv(lut_count, filename, data, R):
    '''
    Writes to a csv with filename
    '''
    
    n = filename + str(lut_count)
    # Verify the ordering of the transform and everything is right
    with open(n, 'a') as f:
        writer = csv.writer(f)
        t = datetime.now().strftime("%y%m%d_%H%M%S")
        
        # Could be different ordering of motors
        writer.writerow([data[0], 180-data[0], data[1], 180-data[1], R[0][3], R[1][3], R[2][3], R[0][0], R[0][1], R[0][2], R[1][0], R[1][1], R[1][2], R[2][0], R[2][1], R[2][2], t])


start_csv(lut_count, filename)

while(True):
    
    # Bending Left and Right, and up/down?
    
    indata = ser.readline()
    data = [int(x) for x in indata.decode('utf-8').rstrip().split(sep=" ")]
    print(data)
    
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
        
    if not depth_frame or not color_frame:
        continue

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
        
    # Sometimes this seems to freeze everything. Unsure as to what's going on
    # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    # cv2.imshow('RealSense', color_image)
    # cv2.waitKey(delay=1)        
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    # print(gray)

    tags = at_detector.detect(gray, estimate_tag_pose=True, camera_params=[intr.fx, intr.fy, intr.ppx, intr.ppy], tag_size=0.055)
    if tags:
        tag_xyz = tags[0].pose_t
        tag_R = tags[0].pose_R
        tag_xyz = np.array(tag_xyz).T[0]
        tag_R = np.array(tag_R)
    
        print(tag_xyz)
        print(tag_R)
        
        
        camera_to_tag = mr.RpToTrans(tag_R, tag_xyz)
        
        
        print("cam to tag", camera_to_tag)
        if data == [90, 90]:
        # if data == [90, 90] and not zero_to_cam: # Thinking this may relieve the XZ plane issues? Could simply have a transform wrong
            zero_to_cam = mr.TransInv(camera_to_tag)
            transform = identity
            
            if zero_transform_flag:
                zero_transform_flag = False
                zero_to_cam_first = zero_to_cam
        else:
            transform = zero_to_cam @ camera_to_tag # this should give us zero->tag transform
    
    
        transform_zero = zero_to_cam_first @ camera_to_tag
        write_csv(lut_count, filename, data, transform)
        write_csv(lut_count, filename_zero, data, transform_zero)

    
    else:
        print("No tag!")
    
    # if data == [180, 40]:        
    #     lut_count += 1
    #     start_csv(lut_count, filename)
            
            
    
            
            