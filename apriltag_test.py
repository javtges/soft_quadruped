# import serial
import cv2
import pyrealsense2 as rs
import numpy as np
#import yaml
import time
import readchar
import threading
from pupil_apriltags import Detector
import modern_robotics as mr
from scipy.spatial.transform import Rotation as R



'''

TEST FILE

Tests apriltags, prints out the resulting transforms.
'''

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
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', color_image)
    cv2.waitKey(delay=1)        
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    
    tags = at_detector.detect(gray, estimate_tag_pose=True, camera_params=[intr.fx, intr.fy, intr.ppx, intr.ppy], tag_size=0.08687) #86.87mm
    
    # print("======================================")
    # print("Number of tags detected: ", len(tags))
    # print("tag 0: ", tags[0])
    # print("======================================")
    # print("tag 1: ", tags[1])


    
    
    if len(tags) != 0:

        # tag_xyz_0 = tags[0].pose_t
        # tag_R_0 = tags[0].pose_R
        # tag_xyz_0 = np.array(tag_xyz_0)
        # tag_R_0 = np.array(tag_R_0)

        # tag_xyz_1 = tags[1].pose_t
        # tag_R_1 = tags[1].pose_R
        # tag_xyz_1 = np.array(tag_xyz_1)
        # tag_R_1 = np.array(tag_R_1)

        # print("==================TAG 0====================")
        # print("tag_xyz_0: \n", tag_xyz_0)
        # print("tag_R_0: \n", tag_R_0)

        # print("==================TAG 1====================")
        # print("tag_xyz_1: \n", tag_xyz_1)
        # print("tag_R_1: \n", tag_R_1)
        # print(">>>>>")






        print("found tags")

        tag_xyz_0 = tags[0].pose_t
        tag_R_0 = tags[0].pose_R
        tag_xyz_0 = np.array(tag_xyz_0)
        tag_R_0 = np.array(tag_R_0)

        tag_xyz_1 = tags[1].pose_t
        tag_R_1 = tags[1].pose_R
        tag_xyz_1 = np.array(tag_xyz_1)
        tag_R_1 = np.array(tag_R_1)

        
        # print("==================TAG 0====================")
        # print("tag_xyz_0: \n", tag_xyz_0)
        # print("tag_R_0: \n", tag_R_0)

        # print("==================TAG 1====================")
        # print("tag_xyz_1: \n", tag_xyz_1)
        # print("tag_R_1: \n", tag_R_1)
        # print(">>>>>")
        
        # r = R.from_matrix(tag_R)
        # q = r.as_quat()
        
        # print(q)
        # euler = r.as_euler('xyz', degrees=False)
        
        #tag_0 -> robot tag
        print("tag_0 -> robot tag")
        camera_to_tag_0 = mr.RpToTrans(tag_R_0, tag_xyz_0)

        #tag_1 -> tabel tag
        print("tag_1 -> tabel tag")
        camera_to_tag_1 = mr.RpToTrans(tag_R_1, tag_xyz_1)

        r = R.from_matrix(tag_R_0)
        q = r.as_quat()
        euler = r.as_euler('xyz', degrees=False)

            # if first_frame:
            #     zero_to_cam = mr.TransInv(camera_to_tag)
            #     q_first = q
            #     transform = identity
            #     e = R.from_quat(q_first)
            #     eye = np.eye(3)
            #     eye_r = R.from_matrix(eye)
            #     euler = eye_r.as_euler("xyz", degrees=False)
            #     # print("AAAAAAA", euler)
            #     # print("first transform: ", zero_to_cam)
            #     first_frame = False
        
        transform = (mr.TransInv(camera_to_tag_1)) @ camera_to_tag_0 # Zero to tag transform?
        print("zero to tag: \n", transform)
        tag_R2 = transform[0:3, 0:3]
        # print(tag_R2)
        # r2 = R.from_matrix(tag_R2)
        # res = tag_R2 @ tag_R
        res = R.from_matrix(tag_R2)
        euler = res.as_euler("XYZ", degrees=False)
        print("Euler Angles, Relative:", euler)




        # print("found tag")
        # tag_xyz = tags[0].pose_t
        # tag_R = tags[0].pose_R
        # tag_xyz = np.array(tag_xyz)
        # tag_R = np.array(tag_R)
        
        # r = R.from_matrix(tag_R)
        # q = r.as_quat()
        
        # # print(q)
        # # euler = r.as_euler('xyz', degrees=False)
        
        # camera_to_tag = mr.RpToTrans(tag_R, tag_xyz)
        # if first_frame:
        #     zero_to_cam = mr.TransInv(camera_to_tag)
        #     q_first = q
        #     transform = identity
        #     e = R.from_quat(q_first)
        #     eye = np.eye(3)
        #     eye_r = R.from_matrix(eye)
        #     euler = eye_r.as_euler("xyz", degrees=False)
        #     print("AAAAAAA", euler)
        #     print("first transform: ", zero_to_cam)
        #     first_frame = False
        # else:
        #     transform = zero_to_cam @ camera_to_tag # Zero to tag transform?
        #     print("zero to tag: ", transform)
        #     tag_R2 = transform[0:3, 0:3]
        #     # print(tag_R2)
        #     # r2 = R.from_matrix(tag_R2)
        #     # res = tag_R2 @ tag_R
        #     res = R.from_matrix(tag_R2)
        #     euler = res.as_euler("XYZ", degrees=False)
        #     print("Euler Angles, Relative:", euler)

        

    