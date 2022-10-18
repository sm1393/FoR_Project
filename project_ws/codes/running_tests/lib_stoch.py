from operator import le
from random import randrange
import pybullet_data
import pybullet as pb
import time
import os
import math
import numpy as np
import sys
import common_paths

############################################################################################################################

# jointArray = [0,1,2,4,5,6,8,9,10,12,13,14] # for free stoch
jointArray = [1,2,3,5,6,7,9,10,11,13,14,15] # for fixed stoch
linkArray = np.arange(0, 18, 1)

# Leg link lengths
# L1 = 0.29691 #thigh
# L2 = 0.3 #shank
L1 = 0.29701 #thigh
L2 = 0.2999 #shank

# Offsets
X_SHIFT = 0.27318
X_SHIFT = 0.27138
Y_SHIFT = 0.22695
Z_SHIFT = 0.01422

# Shift of leg frames with respect to body frame
FL_SHIFT = np.array([X_SHIFT, Y_SHIFT, -Z_SHIFT])
FR_SHIFT = np.array([X_SHIFT, -Y_SHIFT, -Z_SHIFT])
BL_SHIFT = np.array([-X_SHIFT, Y_SHIFT, -Z_SHIFT])
BR_SHIFT = np.array([-X_SHIFT, -Y_SHIFT, -Z_SHIFT])

############################################################################################################################

def degree2Radians(degree):
    return math.pi*degree/180

def radian2Degree(radians):
    return 180*radians/(math.pi)

############################################################################################################################

# Print all joint info: joint ID, joint Name, joint Type, Lower limit, Upper limit
def printJointInfo(stochID):
    joints = pb.getNumJoints(stochID)
    print("ID   Name                Type        Lower limit     Upper limit")
    for i in range(joints):
        jointinfo = pb.getJointInfo(stochID,i)
        print(jointinfo[0]," ", jointinfo[1],"      ", jointinfo[2], "      ", jointinfo[8],"       ", jointinfo[9])
    print("\n")

# Returns complete joint information for given joint ID
def jointInfo(stochID, jointID):
    return pb.getJointInfo(stochID,jointID)

# Return base_link location in world frame
def baseTfPosition(stochID):
    position, orientation = pb.getBasePositionAndOrientation(stochID)
    orientation = pb.getEulerFromQuaternion(orientation)
    return [position, orientation]

# Returns base_link velocity in world frame
def baseTfVelocity(stochID):
    return pb.getBasePositionAndOrientation(stochID)

# Returns all joint angles in terms of radians
def jointStatesRadians(stochID):
    jointStates = pb.getJointStates(stochID, jointArray)
    jointArray = []
    for i in range(len(jointStates)):
        joint = jointStates[i]
        jointAngle = joint[0]
        jointArray.append(jointAngle)
    return jointArray

# Returns all joint angles in terms of degrees
def jointStatesDegree(stochID):
    jointStates = pb.getJointStates(stochID, jointArray)
    jointArray = []
    for i in range(len(jointStates)):
        joint = jointStates[i]
        jointAngle = radian2Degree(joint[0])
        jointArray.append(jointAngle)
    return jointArray

# Returns all joint angular velocities
def jointVelocities(stochID):
    jointStates = pb.getJointStates(stochID, jointArray)
    jointArray = []
    for i in range(len(jointStates)):
        jointState = jointStates[i]
        jointVelocities = jointState[1]
        jointArray.append(jointVelocities)
    return jointArray

# Return all joint torques
def jointTorques(stochID):
    jointStates = pb.getJointStates(stochID, jointArray)
    jointArray = []
    for i in range(len(jointStates)):
        jointState = jointStates[i]
        jointTorque = jointState[3]
        jointArray.append(jointTorque)
    return jointArray

############################################################################################################################

# Joint Position control for any leg
def leg_control(stochID, leg_joint_array, joint_angle_array, enablePrint):
    pb.setJointMotorControlArray(bodyUniqueId=stochID,
                                jointIndices = leg_joint_array,
                                controlMode= pb.POSITION_CONTROL,
                                targetPositions = [ degree2Radians(joint_angle_array[0]),
                                                    degree2Radians(joint_angle_array[1]),
                                                    degree2Radians(joint_angle_array[2]) ])
    if enablePrint:
        print("Angles written: "    "ID("+ str(leg_joint_array[0]) + ")=" + str(joint_angle_array[0]) + ", " 
                                    "ID("+ str(leg_joint_array[1]) + ")=" + str(joint_angle_array[1]) + ", " 
                                    "ID("+ str(leg_joint_array[2]) + ")=" + str(joint_angle_array[2]))

# joint Position control individual leg
def JointAngleControl_FL(stochID, angles, enablePrint):
    leg_control(stochID, jointArray[0:3], angles, enablePrint)
    
def JointAngleControl_FR(stochID, angles, enablePrint):
    leg_control(stochID, jointArray[3:6], angles, enablePrint)

def JointAngleControl_BL(stochID, angles, enablePrint):
    leg_control(stochID, jointArray[6:9], angles, enablePrint)

def JointAngleControl_BR(stochID, angles, enablePrint):
    leg_control(stochID, jointArray[9:12], angles, enablePrint)

# Joint control of all legs at same time
def JointAngleControl(stochID, angles, enablePrint):
    JointAngleControl_FL(stochID, angles[0:3], enablePrint)
    JointAngleControl_FR(stochID, angles[3:6], enablePrint)
    JointAngleControl_BL(stochID, angles[6:9], enablePrint)
    JointAngleControl_BR(stochID, angles[9:12], enablePrint)

############################################################################################################################

# Return foot coordinates with respect to body frame
def getObservedFootCoordinates(stochID):
    linkPositions, linkOrientations = [],[]
    links = pb.getLinkStates(stochID,linkArray)
    for i in range(len(links)):
        linkPositions.append(np.array(links[i][0]))
        linkOrientations.append(np.array(links[i][1]))
    foot_coordinates = [np.round(linkPositions[4] - linkPositions[0], 5), # Front Left
                        np.round(linkPositions[8] - linkPositions[0], 5), # Front Right
                        np.round(linkPositions[12] - linkPositions[0], 5), # Back left
                        np.round(linkPositions[16] - linkPositions[0], 5)] # Back Right
    return foot_coordinates

# Testing for front left leg
def getCalculatedFootCoordinates(stochID, angles):
    foot_in_leg_frame = forward_kinematics(angles)
    foot_in_body_frame = foot_in_leg_frame + FL_SHIFT
    return foot_in_body_frame

# Forward kinematics for 2 dof leg
def forward_kinematics(angles):
    theta1, theta2 = degree2Radians(angles[1]), degree2Radians(angles[2])
    Z = -(L1*math.cos(theta1) + L2*math.cos(theta1+theta2)) # X
    X = -(L1*math.sin(theta1) + L2*math.sin(theta1+theta2)) # Z
    print("X, Y, Z", X, "0", Z)
    return np.array([X,0,Z])
