import pybullet as p
import pybullet_data
import os
import time

physicsClient = p.connect(p.GUI)
f_name = os.path.join(os.path.dirname(__file__), 'hsa_turtle_test.urdf')
plane_name = os.path.join(os.path.dirname(__file__), 'simpleplane.urdf')
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

planeID = p.loadURDF(plane_name)

robot_start_pos = [0,0,0.5]
robot = p.loadURDF(f_name, basePosition=robot_start_pos ,physicsClientId=physicsClient)

num_joints = p.getNumJoints(robot)
for i in range(num_joints):
    print("joint info", p.getJointInfo(robot, i))

for i in range(10000):
    maxForce = 100
    p.setJointMotorControl2(robot, jointIndex=2, controlMode=p.POSITION_CONTROL, targetPosition=-0.02, force=maxForce)
    p.setJointMotorControl2(robot, jointIndex=4, controlMode=p.POSITION_CONTROL, targetPosition=-0.02, force=maxForce)
    p.setJointMotorControl2(robot, jointIndex=6, controlMode=p.POSITION_CONTROL, targetPosition=-0.02, force=maxForce)
    p.setJointMotorControl2(robot, jointIndex=8, controlMode=p.POSITION_CONTROL, targetPosition=-0.2, force=maxForce)

    p.setJointMotorControl2(robot, jointIndex=1, controlMode=p.POSITION_CONTROL, targetPosition=0.4, force=maxForce)
    p.setJointMotorControl2(robot, jointIndex=0, controlMode=p.POSITION_CONTROL, targetPosition=0.4, force=maxForce)

    p.stepSimulation()
    time.sleep(1/240)
    # print("joint info", p.getBasePositionAndOrientation(robot))

    
p.disconnect()