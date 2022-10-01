import time
import os
import math
import pybullet as pb
import sys
import pybullet_data
import common_paths

def degree2Radians(degree):
    return math.pi*degree/180

def radian2Degree(radians):
    return 180*radians/(math.pi)

def printJointInfo(stochID):
    joints = pb.getNumJoints(stochID)
    print("ID   Name                Type        Lower limit     Upper limit")
    for i in range(joints):
        jointinfo = pb.getJointInfo(stochID,i)
        print(jointinfo[0]," ", jointinfo[1],"      ", jointinfo[2], "      ", jointinfo[8],"       ", jointinfo[9])
    print("\n")

def jointInfo(stochID, jointID):
    return pb.getJointInfo(stochID,jointID)

def baseTfInfo(stochID):
    position, orientation = pb.getBasePositionAndOrientation(stochID)
    orientation = pb.getEulerFromQuaternion(orientation)
    return [position, orientation]

def jointStatesRadians(stochID):
    jointStates = pb.getJointStates(stochID,[0,1,2,3,4,5,6,7,8,9,10,11])
    jointArray = []
    for i in range(len(jointStates)):
        joint = jointStates[i]
        jointAngle = joint[0]
        jointArray.append(jointAngle)
    return jointArray

def jointStatesDegree(stochID):
    jointStates = pb.getJointStates(stochID,[0,1,2,3,4,5,6,7,8,9,10,11])
    jointArray = []
    for i in range(len(jointStates)):
        joint = jointStates[i]
        jointAngle = radian2Degree(joint[0])
        jointArray.append(jointAngle)
    return jointArray

def jointVelocities(stochID):
    jointStates = pb.getJointStates(stochID,[0,1,2,3,4,5,6,7,8,9,10,11])
    jointArray = []
    for i in range(len(jointStates)):
        jointState = jointStates[i]
        jointVelocities = jointState[1]
        jointArray.append(jointVelocities)
    return jointArray

def jointTorques(stochID):
    jointStates = pb.getJointStates(stochID,[0,1,2,3,4,5,6,7,8,9,10,11])
    jointArray = []
    for i in range(len(jointStates)):
        jointState = jointStates[i]
        jointTorque = jointState[3]
        jointArray.append(jointTorque)
    return jointArray