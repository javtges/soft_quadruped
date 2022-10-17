import csv
import numpy as np
from pupil_apriltags import Detector
import pyrealsense2 as rs
import cv2
import time
import serial
from datetime import datetime


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



range1 = np.linspace(0, 180, 19)
range2 = np.linspace(0, 180, 19)
# video = cv2.VideoCapture(0)


now = datetime.now().strftime("%y%m%d_%H%M%S")
lut_count = 0
filename = 'table_' + now + "_"


def start_csv(lut_count, filename):
    n = filename + lut_count
    with open(n, 'a') as f:
        writer = csv.writer(f)
        t = datetime.now().strftime("%y%m%d_%H%M%S")
        writer.writerow(['motor1', 'motor2', 'motor3', 'motor4', 'leg_trans_x', 'leg_trans_y', 'leg_trans_z', 'leg_rot_x', 'leg_rot_y', 'leg_rot_z', 'leg_rot_w', t])


def write_csv(lut_count, filename, xyz, rot):
    n = filename + lut_count
    # Verify the ordering of the quaternion and everything is right
    print(xyz)
    print(rot)
    with open(n, 'a') as f:
        writer = csv.writer(f)
        t = datetime.now().strftime("%y%m%d_%H%M%S")
        
        # Could be different ordering of motors
        writer.writerow([b1, b1, b2, b2, xyz[0][:], xyz[1][:], xyz[2][:], rot[0][:], rot[1][:], rot[2][:], rot[3][:], t])


while(True):
    
    # start_csv(lut_count, filename)
    
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
        
    # Sometimes this seems to freeze everything
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', color_image)
    cv2.waitKey(delay=1)        
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    
    # ret, frame = video.read()
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    print(gray)
    # cv2.imshow("q", np.array(gray, dtype = np.uint8 ))
    # cv2.waitKey(delay=1)
    # tags = at_detector.detect(gray, estimate_tag_pose=True, camera_params=[intr.fx, intr.fy, intr.ppx, intr.ppy], tag_size=0.055)
    # tag_xyz = tags[0].pose_t
    # tag_R = tags[0].pose_R
    # tag_xyz = np.array(tag_xyz)
    # tag_R = np.array(tag_R)
    
    # for b1 in range1:
    #     for b2 in range2:
    #         print(b1,b2)
            
    #         # Send to robot
    #         time.sleep(1)
            
    #         # Get a frame and find out the tag pose
    #         ret, frame = video.read()
    #         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #         tags = at_detector.detect(gray, estimate_tag_pose=True, camera_params=[intr.fx, intr.fy, intr.ppx, intr.ppy], tag_size=0.055)
    #         tag_xyz = tags[0].pose_t
    #         tag_R = tags[0].pose_R
    #         tag_xyz = np.array(tag_xyz)
    #         tag_R = np.array(tag_R)
            
            # write_csv(lut_count, filename, tag_xyz, tag_R)
            
    # for b1 in range1:
    #     pass
            
    lut_count += 1
    
    # if (lut_count == 9):
    #     break
            
            
    
            
            