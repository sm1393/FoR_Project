import pybullet_data
import pybullet as pb
import time
import os
import math
import numpy as np
import sys
import common_paths

jointArray = np.arange(0, 12, 1)

def degree2Radians(degree):
    return math.pi*degree/180

def radian2Degree(radians):
    return 180*radians/(math.pi)

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

# Joint Position control for any leg
def leg_control(stockID, leg_joint_array, joint_angle_array, enablePrint):
    pb.setJointMotorControlArray(bodyUniqueId=stockID,
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
def JointAngleControl_FL(stockID, angles, enablePrint):
    leg_control(stockID, [0,1,2], angles, enablePrint)
    
def JointAngleControl_FR(stockID, angles, enablePrint):
    leg_control(stockID, [3,4,5], angles, enablePrint)

def JointAngleControl_BL(stockID, angles, enablePrint):
    leg_control(stockID, [6,7,8], angles, enablePrint)

def JointAngleControl_BR(stockID, angles, enablePrint):
    leg_control(stockID, [9,10,11], angles, enablePrint)

# Joint control of all legs at same time
def JointAngleControl(stockID, angles, enablePrint):
    JointAngleControl_FL(stockID, angles[0:3], enablePrint)
    JointAngleControl_FR(stockID, angles[3:6], enablePrint)
    JointAngleControl_BL(stockID, angles[6:9], enablePrint)
    JointAngleControl_BR(stockID, angles[9:12], enablePrint)
