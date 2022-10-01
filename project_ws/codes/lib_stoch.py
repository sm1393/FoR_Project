import pybullet_data
import pybullet as pb
import time
import os
import math
import numpy as np
import sys
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

def baseTfPosition(stochID):
    position, orientation = pb.getBasePositionAndOrientation(stochID)
    orientation = pb.getEulerFromQuaternion(orientation)
    return [position, orientation]

def baseTfVelocity(stochID):
    return pb.getBasePositionAndOrientation(stochID)

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

def JointAngleControl_FL(stockID, angles):
    pb.setJointMotorControlArray(bodyUniqueId=stockID,
                                jointIndices=[0,1,2],
                                controlMode= pb.POSITION_CONTROL,
                                targetPositions = [ degree2Radians(angles[0]),
                                                    degree2Radians(angles[1]),
                                                    degree2Radians(angles[2]) ])

def JointAngleControl_FR(stockID, angles):
    pb.setJointMotorControlArray(bodyUniqueId=stockID,
                                    jointIndices=[3,4,5],
                                    controlMode= pb.POSITION_CONTROL,
                                    targetPositions = [ degree2Radians(angles[0]),
                                                        degree2Radians(angles[1]),
                                                        degree2Radians(angles[2]) ])
def JointAngleControl_BL(stockID, angles):
    pb.setJointMotorControlArray(bodyUniqueId=stockID,
                                    jointIndices=[6,7,8],
                                    controlMode= pb.POSITION_CONTROL,
                                    targetPositions = [ degree2Radians(angles[0]),
                                                        degree2Radians(angles[1]),
                                                        degree2Radians(angles[2]) ])

def JointAngleControl_BR(stockID, angles):
    pb.setJointMotorControlArray(bodyUniqueId=stockID,
                                    jointIndices=[9,10,11],
                                    controlMode= pb.POSITION_CONTROL,
                                    targetPositions = [ degree2Radians(angles[0]),
                                                        degree2Radians(angles[1]),
                                                        degree2Radians(angles[2]) ])

def JointAngleControl(stockID, angles):
    JointAngleControl_FL(stockID, [angles[0], angles[1], angles[2]])
    JointAngleControl_FR(stockID, [angles[3], angles[4], angles[5]])
    JointAngleControl_BL(stockID, [angles[6], angles[7], angles[8]])
    JointAngleControl_BR(stockID, [angles[9], angles[10], angles[11]])


# def JointVelocityControl_FL(stockID, angles, velocities):
#     pb.setJointMotorControlArray(bodyUniqueId=stockID,
#                                 jointIndices=[0,1,2],
#                                 controlMode= pb.VELOCITY_CONTROL,
#                                 targetPositions = [ degree2Radians(angles[0]),
#                                                         degree2Radians(angles[1]),
#                                                         degree2Radians(angles[2]) ],
#                                 targetVelocities = [ degree2Radians(velocities[0]),
#                                                     degree2Radians(velocities[1]),
#                                                     degree2Radians(velocities[2]) ])