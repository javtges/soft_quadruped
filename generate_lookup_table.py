import csv
import numpy as np
from pupil_apriltags import Detector
import pyrealsense2 as rs
import cv2
import time
from datetime import datetime


at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)


range1 = np.linspace(0, 180, 19)
range2 = np.linspace(0, 180, 19)
video = cv2.VideoCapture(0)

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
    
    start_csv(lut_count, filename)
    
    # Bending Left and Right, and up/down?
    for b1 in range1:
        for b2 in range2:
            print(b1,b2)
            
            # Send to robot
            time.sleep(1)
            
            # Get a frame and find out the tag pose
            ret, frame = video.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tags = at_detector.detect(gray, estimate_tag_pose=True, camera_params=[intr.fx, intr.fy, intr.ppx, intr.ppy], tag_size=0.055)
            tag_xyz = tags[0].pose_t
            tag_R = tags[0].pose_R
            tag_xyz = np.array(tag_xyz)
            tag_R = np.array(tag_R)
            
            write_csv(lut_count, filename, tag_xyz, tag_R)
            
    # for b1 in range1:
    #     pass
            
    lut_count += 1
    
    if (lut_count == 9):
        break
            
            
    
            
            