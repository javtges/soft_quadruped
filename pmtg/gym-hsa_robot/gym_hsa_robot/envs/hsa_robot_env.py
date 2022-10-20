import gym
from gym import error, spaces, utils
from gym.utils import seeding
import pybullet as p
import time
import pybullet_data
import matplotlib.pyplot as plt
import keras
import numpy as np
import modern_robotics as mr
import math
from scipy.spatial.transform import Rotation as R
import sys
# sys.path.append("/home/peterjochem/Desktop/Deep_RL/PMTG/h3pper/wrench_generator")
# from trajectory_generator import trajectory_interpolator 

# absolute_path_urdf = "/home/peterjochem/Desktop/Deep_RL/PMTG/gym-hopping_robot_pmtg/gym_hopping_robot_pmtg/envs/urdf/hopping_robot.urdf"
# absolute_path_neural_net = "/home/peterjochem/Desktop/Deep_RL/DDPG/h3pper/gym-hopping_robot/gym_hopping_robot/envs/hopping_robot/neural_networks/model2.h5"  
# absolute_path_trajectories = "/home/peterjochem/Desktop/Deep_RL/PMTG/h3pper/wrench_generator/data/CSVs/zeta_1.0/"



class HSARobot_Env(gym.Env):       
    
    metadata = {'render.modes': ['human']}
    
    def __init__(self):
        
        self.physicsClient = p.connect(p.GUI)
          
        self.visualizeTrajectory = False 
        self.jointIds=[]
        self.paramIds=[]
    
        # self.neural_net = keras.models.load_model(absolute_path_neural_net)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        p.setGravity(0, 0, -10)

        self.plane = p.loadURDF("plane.urdf")
        self.hopper = p.loadURDF(absolute_path_urdf, [0.0, 0.0, 1.27], useFixedBase = False)
        
        self.gravId = p.addUserDebugParameter("gravity", -10, 10, -10)
        self.homePositionAngles = [0.0, 0.0, 0.0]
    
        # Desired Velocity from PMTG
        self.desiredVelocity = 0.35 
        
        # Trajectory generator -> NEED TO WRITE
        self.myTG = trajectory_interpolator(absolute_path_trajectories)
        self.minTime, self.maxTime, self.initial_conditions, self.wrenches = self.myTG.get_trajectories(0.35)
        
        self.u_y = self.wrenches[:,4]
        self.u_z = self.wrenches[:,5]
        self.u_theta = self.wrenches[:,0]
        
        self.periodTimes = np.linspace(self.minTime, self.maxTime, num = len(self.u_y))
        self.time = 0.0
        self.granularDepth = 0.3
        
        self.granular_points = []
        self.foot_points = []
        self.body_points = []

        # Setup the debugParam sliders
        self.gravId = p.addUserDebugParameter("gravity", -10, 10, -10) 
        self.homePositionAngles = [0.0, 0.0, 0.0]
        
        self.foot_points = []
        self.body_points = []
            
        activeJoint = 0
        for j in range (p.getNumJoints(self.hopper)):
            
            # Why set the damping factors to 0?
            p.changeDynamics(self.hopper, j, linearDamping = 0, angularDamping = 0)
            info = p.getJointInfo(self.hopper, j)
            jointName = info[1]
            jointType = info[2]

            if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
                
                self.jointIds.append(j)
                self.paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, self.homePositionAngles[activeJoint]))
                #p.resetJointState(self.hopper, j, self.homePositionAngles[activeJoint])
                activeJoint = activeJoint + 1

        self.setInitialConditions()
         
        p.setRealTimeSimulation(0) # Must do this to apply forces/torques with PyBullet method
        #self.plotGranular()
        self.stateId = p.saveState() # Stores state in memory rather than on disk
    

    """Reset the robot to the home position"""
    def defineHomePosition(self):

        self.gravId = p.addUserDebugParameter("gravity", -10, 10, -10)
        self.homePositionAngles = [0, 0, 0]

        activeJoint = 0
        for j in range (p.getNumJoints(self.hopper)):

            # Why set the damping factors to 0?
            p.changeDynamics(self.hopper, j, linearDamping = 0, angularDamping = 0)
            info = p.getJointInfo(self.hopper, j)
            jointName = info[1]
            jointType = info[2]

            if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):

                self.jointIds.append(j)
                self.paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, self.homePositionAngles[activeJoint]))
                p.resetJointState(self.hopper, j, self.homePositionAngles[activeJoint])
                activeJoint = activeJoint + 1
        
    """ Setup the robot in PyBullet to be at the indicated initial conditions """
    def setInitialConditions(self):
        x_b, dx_b, y_b, dy_b, theta_b, dtheta_b, x_f, dx_f, y_f, dy_f, theta_f, dtheta_f = self.initial_conditions
        
        # Set the bodys position and velocity
        orientation_body = p.getQuaternionFromEuler([theta_b, 0.0, 0.0]) 
        p.resetBasePositionAndOrientation(self.hopper, [0.0, 0.0, 1.22], orientation_body)
        p.resetBaseVelocity(self.hopper, [0.0, dx_b, 0.0], [-dtheta_b, 0.0, 0.0])
        
        # Set the foots position and velocity
        foot_index = 3
        p.resetJointState(self.hopper, foot_index, theta_f, dtheta_f)
        

    """Update robot's PID controller's control signals. controlSignal is a list of desired joint angles (rads).
    PyBullet does support direct torque control...iterate in this direction eventually?"""
    def controller(self, controlSignal):

        for i in range(len(self.paramIds)):
            nextJointId = self.paramIds[i]
            #targetPos = p.readUserDebugParameter(nextJointId) # This reads from the sliders. Useful for debugging        
            targetTorque = controlSignal[i] # This uses the control signal parameter
            
            p.setJointMotorControl2(bodyUniqueId=self.hopper, jointIndex=self.jointIds[i], controlMode=p.VELOCITY_CONTROL, force=0) # Must disable the vel control
            p.setJointMotorControl2(bodyUniqueId=self.hopper, jointIndex=self.jointIds[i], controlMode=p.TORQUE_CONTROL, force=targetTorque) 
 
    """Return the robot to its initial state"""
    def reset(self):

        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.hopper)
        self.jointIds=[]
        self.paramIds=[]
        p.removeAllUserParameters() # Must remove and replace
        p.restoreState(self.stateId) 
        self.defineHomePosition()
        
        for i in range(len(self.foot_points)):
            p.removeUserDebugItem(self.foot_points[i])
            p.removeUserDebugItem(self.body_points[i])
        
        self.time = 0.0
        return self.computeObservation()

    # def computeGRF(self, gamma, beta, depth, dx, dy, dz, ankle_angular_velocity):
        
    #     inVector = [gamma, beta, depth, dy, dz, ankle_angular_velocity]
    #     grf_y, grf_z, torque = (self.neural_net.predict([inVector]))[0]
        
    #     return grf_y, grf_z, torque
    
    # """ Convert a point from the hip frame to the world frame """
    # def hip_to_world(self, Point_in_hip_frame):
        
    #     hip_position, hip_orientation = p.getBasePositionAndOrientation(self.hopper)

    #     hip_x, hip_y, hip_z = Point_in_hip_frame
    #     P_Hip = np.zeros((4, 1), dtype="float32")
    #     P_Hip[0][0] = hip_x 
    #     P_Hip[0][0] = hip_y
    #     P_Hip[0][0] = hip_z
    #     P_Hip[0][0] = 1.0
        
    #     """Use scipy to convert the quaternion to a rotation matrix"""
    #     r = R.from_quat(hip_orientation) # scipy uses x, y, z, w
        
    #     T_world_hip = np.zeros((4, 4), dtype='float32')
    #     for i in range(3):
    #         for j in range(3):
    #             T_world_hip[i][j] = r[i][j]
        
    #     T_world_hip[0][3] = hip_x
    #     T_world_hip[1][3] = hip_y
    #     T_world_hip[2][3] = hip_z
    #     T_world_hip[3][3] = 1.0 

    #     P_world = np.matmul(T_world_hip, P_hip)  

    #     return P_world
    
    # """ Plot the current position of the robots foot in PyBullet """
    # def plotPosition(self):
    #     world_pos, orientation, localInertialFramePosition, localInertialFrameOrientation, worldLinkFramePosition, worldLinkFrameOrientation, worldLinkLinearVelocity, worldLinkAngularVelocity = p.getLinkState(self.hopper, 0, 1)
    #     body_x, body_y, body_z = world_pos
    #     self.foot_points.append(p.addUserDebugLine([foot_x - 0.025, foot_y - 0.025, foot_z - 0.025], [foot_x, foot_y, foot_z], [1, 0, 0]))
    #     self.body_points.append(p.addUserDebugLine([body_x - 0.025, body_y - 0.025, body_z + 0.2], [body_x, body_y, body_z + 0.2 + 0.025], [0, 0, 1]))
    
    # def createBodyScrews(self):

    #     foot_z_dim = 0.01 
    #     L1_length = 0.4
    #     L2_length = 0.4
    #     L3_length = 0.4
        
    #     s1 = np.array([1.0, 0.0, 0.0])  
    #     s2 = np.array([1.0, 0.0, 0.0])
    #     s3 = np.array([1.0, 0.0, 0.0])

    #     q1 = np.array([0.0, 0.0, foot_z_dim])
    #     q2 = np.array([0.0, 0.0, foot_z_dim + L1_length])
    #     q3 = np.array([0.0, 0.0, foot_z_dim + L1_length + L2_length])
            
    #     return np.array([mr.ScrewToAxis(q1, s1, 0.0), mr.ScrewToAxis(q2, s2, 0.0), mr.ScrewToAxis(q3, s3, 0.0)]) 
    
    # """ Construct the Jacobian 
    # The order must be [ankle angle, knee angle, hip angle] """
    # def Jacobian(self, thetaList):
         
    #     body_screws = self.createBodyScrews()
    #     return mr.JacobianBody(body_screws.transpose(), thetaList)
         
    # """ Map the wrench to a torque - details are in Modern robotics 
    # thetaList should be in the order [ankle angle, knee angle, hip angle] """
    # def wrenchToTorque(self, thetaList, wrench):
        
    #     # Torque = Jacobian(thetaList).T * Wrench
    #     return np.matmul(np.transpose(self.Jacobian(thetaList)), wrench)
         
    """ Move time forward a small increment """
    def step(self, action):
        
        
        # Find modulated u values from PMTG
        # Get the states of the legs / joint states
        # Use trajectory generator to get desired trajectory step
        # Use setJointMotorControl2 to set the corresponding joint states
        
        # Action: motor inputs (8 total)
        # Use action to 
        
        
           
        # Forward prop neural network to get GRF, use that to change the gravity
        # Shoud really compute after every p.stepSimulation
        ankle_position, ankle_angular_velocity, appliedTorque, foot_x, foot_y, foot_z, foot_dx, foot_dy, foot_dz, foot_roll, foot_pitch, foot_yaw = self.getFootState()
        #depth = plate_bottom_z - bed_z
        gamma = math.atan2(foot_dz, foot_dy) 
        beta = foot_roll
        
        # action = 30 delta terms + 30 residual terms 
        # 3_curves = TG(action[i]) # 30 delta terms 
        # interpolate between points
        # u_x, u_y, u_theta = TG[time] + residual_terms 
        
        periodTime = self.time % self.maxTime;

        modulated_u_y = self.u_y + action[0][3:13]
        chosen_u_y = np.interp(periodTime, self.periodTimes, modulated_u_y)
        
        modulated_u_z = self.u_z + action[0][13:23]
        chosen_u_z = np.interp(periodTime, self.periodTimes, modulated_u_y)

        modulated_u_theta = self.u_theta + action[0][23:33]
        chosen_u_theta = np.interp(periodTime, self.periodTimes, modulated_u_theta)
        
        # map the planar wrench to the robot's joint torques
        # j1 is top joint, and j3 is the ankle
        j1_pos, ankle_angular_velocity, ankle_joint_reaction_forces, appliedTorque = p.getJointStates(self.hopper, [1])[0]
        j2_pos, ankle_angular_velocity, ankle_joint_reaction_forces, appliedTorque = p.getJointStates(self.hopper, [2])[0]
        j3_pos, ankle_angular_velocity, ankle_joint_reaction_forces, appliedTorque = p.getJointStates(self.hopper, [3])[0]
        thetaList = np.array([j3_pos, j2_pos, j1_pos])
        
                        #-1
        joint_torques = -1 * self.wrenchToTorque(thetaList, [chosen_u_theta, 0.0, 0.0, 0.0, chosen_u_y, chosen_u_z])
        joint_torques = joint_torques # + action[0][:3] 

        if (self.visualizeTrajectory):
            self.plotPosition()

        customGRF = False
        if (foot_z < 0.3 and foot_z > 0.0001):
            # customGRF = True
            pass

        grf_y, grf_z, torque = self.computeGRF(gamma, beta, foot_z, foot_dx, foot_dy, foot_dz, ankle_angular_velocity)
          
        # Step forward some finite number of seconds or milliseconds
        # Send the joint torques to the robot  
        self.controller(joint_torques) 

        for i in range (3):
            foot_index = 3     
            # Must call this each time we stepSimulation 
            if (customGRF):
                p.applyExternalForce(self.hopper, foot_index, [0, grf_y, grf_z], [0.0, 0.0, 0.0], p.LINK_FRAME) 
                p.applyExternalTorque(self.hopper, foot_index, [torque, 0, 0], p.LINK_FRAME)
            
            p.stepSimulation()
            self.planarConstraint()
        
        self.time = self.time + (3) * (1.0/240.0)
        isOver = self.checkForEnd()
        return self.computeObservation(), self.computeReward(isOver), isOver, None
       
    # """ Plot visualization of the bed of granular material """
    # def plotGranular(self):
    #     x_value = -1.0
    #     y_min = -1.0
    #     y_max = 4.0
    #     z_min = 0.0
    #     z_max = self.granularDepth
    #     z_values = np.linspace(z_min, z_max, num = 100)
    #     delta = 0.1

    #     # Plot Plane 1
    #     for z_value in z_values:
    #         self.granular_points.append(p.addUserDebugLine([x_value - delta, y_min - delta, z_value], [x_value - delta, y_max + delta, z_value], [1.0, 0, 0]))

    #     # Plot Plane 2
    #     x_min = -1.0
    #     x_max = 1.0
    #     y_value = -1.0
    #     for z_value in z_values:
    #         self.granular_points.append(p.addUserDebugLine([x_min - delta, y_value - delta, z_value], [x_max + delta, y_value - delta, z_value], [1.0, 0, 0]))

    #     # Plot Plane 3
    #     x_value = 1.0
    #     for z_value in z_values:
    #         self.granular_points.append(p.addUserDebugLine([x_value + delta, y_min - delta, z_value], [x_value + delta, y_max + delta, z_value], [1.0, 0, 0]))

    #     # Plot Plane 4
    #     x_min = -1.0
    #     x_max = 1.0
    #     y_value = y_max
    #     for z_value in z_values:
    #         self.granular_points.append(p.addUserDebugLine([x_min - delta, y_value + delta, z_value], [x_max + delta, y_value + delta, z_value], [1.0, 0, 0]))

    #     # Add the sand
    #     sphere_radius = 0.03
    #     x_values = np.linspace(x_min, x_max, num = int((x_max - x_min)/(2.5 * sphere_radius)))
    #     y_values = np.linspace(y_min, y_max, num = int((y_max - y_min)/(2.5 * sphere_radius)))
    #     for x in x_values:
    #         for y in y_values:
    #             nextSphere = p.loadURDF("/home/peterjochem/Desktop/Deep_RL/DDPG/h3pper/gym-hopping_robot/gym_hopping_robot/envs/hopping_robot/urdf/sphere_1cm.urdf", [x, y, sphere_radius], useFixedBase = False)

    #             p.setCollisionFilterGroupMask(nextSphere, -1, 0, 0)
    #             enableCollision= 1
    #             p.setCollisionFilterPair(self.plane, nextSphere, -1, -1, enableCollision)


    """ Gather data about the foot from PyBullet """
    def getFootState(self):
        
        """Server keeps two lists. One of links and one of joints. These are the indexes into those lists"""
        ankle_joint_index = 3 # Known by printing world frame position of links with p.getLinkState(self.hopper, <index#>)
        foot_link_index = 3

        ankle_position, ankle_angular_velocity, ankle_joint_reaction_forces, appliedTorque = p.getJointStates(self.hopper, [ankle_joint_index])[0]
         
        world_pos, orientation, localInertialFramePosition, localInertialFrameOrientation, worldLinkFramePosition, worldLinkFrameOrientation, worldLinkLinearVelocity, worldLinkAngularVelocity = p.getLinkState(self.hopper, foot_link_index, 1)
        
        foot_roll, foot_pitch, foot_yaw = p.getEulerFromQuaternion(self.robot_orientation)

        foot_x, foot_y, foot_z = world_pos
        foot_dx, foot_dy, foot_dz = worldLinkLinearVelocity

        return ankle_position, ankle_angular_velocity, appliedTorque, foot_x, foot_y, foot_z, foot_dx, foot_dy, foot_dz, foot_roll, foot_pitch, foot_yaw

    def computeObservation(self, tg_state=np.zeros(30)):

        self.robot_position, self.robot_orientation = p.getBasePositionAndOrientation(self.hopper)
        base_roll, base_pitch, base_yaw = p.getEulerFromQuaternion(self.robot_orientation)
        base_x, base_y, base_z = self.robot_position
        
        ankle_position, ankle_angular_velocity, appliedTorque, foot_x, foot_y, foot_z, foot_dx, foot_dy, foot_dz, foot_roll, foot_pitch, foot_yaw = self.getFootState() 
        
        robot_state = [base_roll, base_pitch, base_yaw, base_x, base_y, base_z, ankle_position, ankle_angular_velocity, appliedTorque, foot_x, foot_y, foot_z, foot_dx, foot_dy, foot_dz, foot_roll, foot_pitch, foot_yaw]  

        return np.append(robot_state, tg_state)


    def checkForEnd(self):

        self.robot_position, self.robot_orientation = p.getBasePositionAndOrientation(self.hopper)
        roll, pitch, yaw = p.getEulerFromQuaternion(self.robot_orientation)

        x, y, z = self.robot_position

        # could also check the z coordinate of the robot?
        """
        if (abs(roll) > (1.0) or abs(pitch) > (1.0)):
            self.isOver = True
            return True
        """
        if (z < 0.5):
            return True

        return False

    """Required for the OpenAI Gym API""" 
    def render(self, mode='human', close = False):
        pass
    
    """Read the state of the simulation to compute and return the 
    reward scalar for the agent. A great video on reward shaping
    from the legendary control theory youtuber Brian Douglas -> 
    https://www.mathworks.com/videos/reinforcement-learning-part-4-the-walking-robot-problem-1557482052319.html
    Remember, you get what you incentivize, not what you want"""
    def computeReward(self, isOver):

        stillAliveBonus = 0.0625
        
        self.robot_position, self.robot_orientation = p.getBasePositionAndOrientation(self.hopper)
        x, y, z = self.robot_position 
        [dx, dy, dz], [wx, wy, wz] = p.getBaseVelocity(self.hopper)
        
        # Remember the actions are the joint angles, not the joint torques
        reward = dy + y  
        if (isOver == False):
            reward = reward + stillAliveBonus 

        return reward
