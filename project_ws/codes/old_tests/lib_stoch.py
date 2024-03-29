from operator import le
from random import randrange
from re import L
from turtle import width
import pybullet_data
import pybullet as pb
import time
import os
import math
import numpy as np
import sys
import common_paths

############################################################################################################################
# Constants

jointArray = [0,1,2,4,5,6,8,9,10,12,13,14] # for free stoch
# jointArray = [1,2,3,5,6,7,9,10,11,13,14,15] # for fixed stoch
linkArray = np.arange(0, 18, 1)

# Leg link lengths
# L1 = 0.29691 #thigh
# L2 = 0.3 #shank
L1 = 0.29701 #thigh
L2 = 0.2999 #shank

# Offsets
#X_SHIFT = 0.27318
X_SHIFT = 0.27138
Y_SHIFT = 0.22695
Z_SHIFT = 0.01422

# Shift of leg frames with respect to body frame
FL_SHIFT = np.array([X_SHIFT, Y_SHIFT, -Z_SHIFT])
FR_SHIFT = np.array([X_SHIFT, -Y_SHIFT, -Z_SHIFT])
BL_SHIFT = np.array([-X_SHIFT, Y_SHIFT, -Z_SHIFT])
BR_SHIFT = np.array([-X_SHIFT, -Y_SHIFT, -Z_SHIFT])

# for cubic Bezier curve
cubicWeightMatrix = np.array([[-1,3,-3,1],
                        [3,-6,3,0],
                        [-3,3,0,0],
                        [1,0,0,0]])
# for quadratic Bezier curve
quadraticWeightMatrix = np.array([[1, -2, 1],
                                 [-2, 2, 0],
                                 [1, 0, 0]])

# for linear Bezier curve
linearWeightMatrix = np.array([[-1, 1],
                              [1, 0]])

############################################################################################################################
# Required functions

def degree2Radians(degree):
    return math.pi*degree/180

def radian2Degree(radians):
    return 180*radians/(math.pi)

def limitValue(value):
    if value < -1:
        return -1
    elif value > 1:
        return 1
    else:
        return value

############################################################################################################################
# Get information from pybullet

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
# Command to pybullet

# Joint Position control for any leg
def leg_control(stochID, leg_joint_array, joint_angle_array, enablePrint):
    joint_angle_array = -np.array(joint_angle_array)
    pb.setJointMotorControlArray(bodyUniqueId=stochID,
                                jointIndices = leg_joint_array,
                                controlMode= pb.POSITION_CONTROL,
                                targetPositions = [ degree2Radians(joint_angle_array[0]),
                                                    degree2Radians(joint_angle_array[1]),
                                                    degree2Radians(joint_angle_array[2]) ])
    if enablePrint:
        print("Angles written: "    "ID("+ str(leg_joint_array[0]) + ")=" + str(degree2Radians(joint_angle_array[0])) + ", " 
                                    "ID("+ str(leg_joint_array[1]) + ")=" + str(degree2Radians(joint_angle_array[1])) + ", " 
                                    "ID("+ str(leg_joint_array[2]) + ")=" + str(degree2Radians(joint_angle_array[2])))

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
def JointAngleControl(stochID, jointAngles, enablePrint):
    joint_angle_array = - np.array(jointAngles)
    pb.setJointMotorControlArray(bodyUniqueId=stochID,
                                jointIndices = jointArray,
                                controlMode= pb.POSITION_CONTROL,
                                targetPositions = joint_angle_array)
    if enablePrint:
        print("Angles written: "    "ID("+ str(jointArray[0]) + ")=" + str(joint_angle_array[0]) + ", " 
                                    "ID("+ str(jointArray[1]) + ")=" + str(joint_angle_array[1]) + ", " 
                                    "ID("+ str(jointArray[2]) + ")=" + str(joint_angle_array[2]) + ", " 
                                    "ID("+ str(jointArray[2]) + ")=" + str(joint_angle_array[3]) + ", " 
                                    "ID("+ str(jointArray[2]) + ")=" + str(joint_angle_array[4]) + ", " 
                                    "ID("+ str(jointArray[2]) + ")=" + str(joint_angle_array[5]) + ", " 
                                    "ID("+ str(jointArray[2]) + ")=" + str(joint_angle_array[6]) + ", " 
                                    "ID("+ str(jointArray[2]) + ")=" + str(joint_angle_array[7]) + ", " 
                                    "ID("+ str(jointArray[2]) + ")=" + str(joint_angle_array[8]) + ", " 
                                    "ID("+ str(jointArray[2]) + ")=" + str(joint_angle_array[9]) + ", " 
                                    "ID("+ str(jointArray[2]) + ")=" + str(joint_angle_array[10]) + ", " 
                                    "ID("+ str(jointArray[2]) + ")=" + str(joint_angle_array[11]))

############################################################################################################################
# Kinematics

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
    foot_in_leg_frame = forwardKinematics(angles)
    print("foot_in_leg_frame", foot_in_leg_frame)
    foot_in_body_frame = foot_in_leg_frame + FL_SHIFT
    print("foot_in_body_frame", foot_in_body_frame)
    return foot_in_leg_frame, foot_in_body_frame

# Forward kinematics for 2 dof leg
def forwardKinematics(angles):
    theta1, theta2 = degree2Radians(angles[1]), degree2Radians(angles[2])
    X = (L1*math.sin(theta1) + L2*math.sin(theta1+theta2))
    Z = -(L1*math.cos(theta1) + L2*math.cos(theta1+theta2))
    return np.array([X,0,Z])

# Inverse kinematics for 2 dof leg
def inverseKinmematics(endPositionInHipFrame):
    abdJoint = 0
    coskneeJoint = ((endPositionInHipFrame[0]**2 + endPositionInHipFrame[2]**2) - (L1**2 + L2**2))/ (2*L1*L2)
    kneeJoint = math.acos(limitValue(coskneeJoint))
    determinant = L1**2 + L2**2 + 2*L1*L2*math.cos(kneeJoint)
    sinHipJoint1 = (endPositionInHipFrame[0] * (L1 + L2*math.cos(kneeJoint)) + endPositionInHipFrame[2] * (L2*math.sin(kneeJoint)))/ determinant
    cosHipJoint1 = (endPositionInHipFrame[0] * (L2*math.sin(kneeJoint)) - endPositionInHipFrame[2] * (L1 + L2*math.cos(kneeJoint)))/ determinant
    sinHipJoint1 = limitValue(sinHipJoint1)
    cosHipJoint1 = limitValue(cosHipJoint1)
    hipJoint = math.asin(sinHipJoint1)
    return [abdJoint, hipJoint, kneeJoint]

############################################################################################################################
# Foot trajectory

def generateTransitionPointMatrices(xCentral, zCentral, upperWidth, lowerWidth, centralWidth, liftHeight, groundHeight, depth):
    pointInWorkspace = False
    if depth + groundHeight <= 0.6111:
        if centralWidth <=  (0.6111)**2 - depth**2:
            if depth > liftHeight:
                pointInWorkspace = True
    if pointInWorkspace == False:
        print("Trajectory points out of workspace!!!!")
        exit()

    transitionLiftPointMatrix = np.array([[xCentral, zCentral - depth],
                                         [xCentral + centralWidth/4, zCentral - depth + centralWidth/4],
                                         [xCentral + centralWidth/2, zCentral - depth]]).T

    transitionGroundPointMatrix = np.array([[xCentral , zCentral - depth],
                                           [xCentral - centralWidth/2, zCentral - depth]]).T

    return transitionLiftPointMatrix, transitionGroundPointMatrix

def generateWalkPointMatrices(xCentral, zCentral, upperWidth, lowerWidth, centralWidth, liftHeight, groundHeight, depth):
    pointInWorkspace = False
    if depth + groundHeight <= 0.6111:
        if centralWidth <=  (0.6111)**2 - depth**2:
            if depth > liftHeight:
                pointInWorkspace = True
    if pointInWorkspace == False:
        print("Trajectory points out of workspace!!!!")
        exit()
            
    liftPointMatrix = np.array([[xCentral - centralWidth/2 ,zCentral - depth],
                            [xCentral - upperWidth/2 ,zCentral - depth + liftHeight],
                            [xCentral + upperWidth/2, zCentral - depth + liftHeight],
                            [xCentral + centralWidth/2,zCentral - depth],]).T

    groundPointMatrix = np.array([[xCentral + centralWidth/2 ,zCentral - depth],
                                [xCentral + lowerWidth/2 ,zCentral - depth - groundHeight],
                                [xCentral - lowerWidth/2 , zCentral - depth - groundHeight],
                                [xCentral - centralWidth/2 ,zCentral - depth]]).T

    return liftPointMatrix, groundPointMatrix


def getPointForTrajectory(angle, liftPointMatrix, groundPointMatrix):
    if angle == 360: angle = 0
    if int(angle) in range(0,180):
        t = angle/180
        return liftPointMatrix @ cubicWeightMatrix @ np.array([t**3, t**2, t, 1])
    elif int(angle) in range(180,360):
        t = (angle - 180)/180
        return groundPointMatrix @ cubicWeightMatrix @ np.array([t**3, t**2, t, 1])
    
def getPointForTransition(t, transitionLiftPointMatrix, transitionGroundPointMatrix):
    quadraticTrajectoryPoint = transitionLiftPointMatrix @ quadraticWeightMatrix @ np.array([t**2, t, 1])
    linearTrajectoryPoint = transitionGroundPointMatrix @ linearWeightMatrix @ np.array([t, 1])
    return quadraticTrajectoryPoint, linearTrajectoryPoint

############################################################################################################################
# Crawl

def takePosition(stochID, transitionLiftPointMatrix, transitionGroundPointMatrix):
    quadraticTrajectoryPoint, linearTrajectoryPoint = getPointForTransition(0, transitionLiftPointMatrix, transitionGroundPointMatrix)
    jointAnglesPhase = inverseKinmematics([quadraticTrajectoryPoint[0], 0, quadraticTrajectoryPoint[1]])
    # Get 
    for i in range(1000):
        i = i/1000
        _jointAnglesPhase = [i*jointAnglesPhase[0], i*jointAnglesPhase[1], i*jointAnglesPhase[2]] 
        _jointAngles =  _jointAnglesPhase + _jointAnglesPhase + _jointAnglesPhase + _jointAnglesPhase
        JointAngleControl(stochID, _jointAngles, enablePrint=0)

    time.sleep(1)
    # Set
    for i in range(750):
        i = i/750
        quadraticTrajectoryPoint, linearTrajectoryPoint = getPointForTransition(i, transitionLiftPointMatrix, transitionGroundPointMatrix)
        jointAnglesPhase0 = inverseKinmematics([quadraticTrajectoryPoint[0], 0, quadraticTrajectoryPoint[1]])
        jointAnglesPhase180 = inverseKinmematics([linearTrajectoryPoint[0], 0, linearTrajectoryPoint[1]])
        _jointAngles =  jointAnglesPhase0 + jointAnglesPhase180 + jointAnglesPhase180 + jointAnglesPhase0
        JointAngleControl(stochID, _jointAngles, enablePrint=0)


def trot(stochID, i, liftPointMatrix, groundPointMatrix):
    if i in range(0,180): j = i + 180
    elif i in range(180, 360): j = i - 180
    phase0 = getPointForTrajectory(i, liftPointMatrix, groundPointMatrix)
    phase180 = getPointForTrajectory(j, liftPointMatrix, groundPointMatrix)
    jointAnglesPhase0 = inverseKinmematics([phase0[0], 0, phase0[1]])
    jointAnglesPhase180 = inverseKinmematics([phase180[0], 0, phase180[1]])
    _jointAngles =  jointAnglesPhase0 + jointAnglesPhase180 + jointAnglesPhase180 + jointAnglesPhase0
    JointAngleControl(stochID, _jointAngles, enablePrint=0)
    time.sleep(0.0001)