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
L_ABD = 0.1254 #
L1 = 0.29701 #thigh
L2 = 0.2999 #shank

# Offsets
X_SHIFT = 0.27138
Y_SHIFT = 0.22695 - L_ABD # Shift due to link between abd_joint and hip_joint
Z_SHIFT = 0.01422

# Shift of Hip frames with respect to body frame
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

def quaternionsToRotation(q):
    return 2*np.array([ [q[0]**2 + q[1]**2 -0.5,  q[1]*q[2] - q[0]*q[3],    q[0]*q[2] + q[1]*q[3]],
                        [q[1]*q[2] + q[0]*q[3],   q[0]**2 + q[2]**2 -0.5,   q[2]*q[3] - q[0]*q[1]],
                        [q[1]*q[3] - q[0]*q[2],   q[2]*q[3] + q[0]*q[1],    q[0]**2 + q[3]**2 -0.5]])

def eulerToRotation(e):
    phi = 0
    theta = 1
    psi = 2
    return np.array([   [math.cos(e[psi])*math.cos(e[theta]),
                                math.cos(e[psi])*math.sin(e[theta])*math.sin(e[phi]) - math.sin(e[psi])*math.cos(e[phi]),
                                        math.cos(e[psi])*math.sin(e[theta])*math.cos(e[phi]) + math.sin(e[psi])*math.sin(e[phi])],
                        [math.sin(e[psi])*math.cos(e[theta]),
                                math.sin(e[psi])*math.sin(e[theta])*math.sin(e[phi]) + math.cos(e[psi])*math.cos(e[phi]),
                                        math.sin(e[psi])*math.sin(e[theta])*math.cos(e[phi]) - math.cos(e[psi])*math.sin(e[phi])],
                        [-math.sin(e[theta]),
                                math.cos(e[theta])*math.sin(e[phi]),
                                        math.cos(e[theta])*math.cos(e[phi])]])

def bodyTwist(stochID):
    position, orientation = pb.getBasePositionAndOrientation(stochID)
    eulerAngles = pb.getEulerFromQuaternion(orientation)
    rotation = eulerToRotation(eulerAngles)
    velocity = pb.getBaseVelocity(stochID)
    pHat = np.array([   [0      , -position[2], position[1]],
                        [position[2],      0,  -position[0]],
                        [-position[1], position[0], 0]])
    bodyVelocity =  rotation.T @ np.array(velocity[0]) - rotation.T @ pHat @ np.array(velocity[1])
    return bodyVelocity

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
    joint_angle_array = np.array(joint_angle_array)
    pb.setJointMotorControlArray(bodyUniqueId=stochID,
                                jointIndices = leg_joint_array,
                                controlMode= pb.POSITION_CONTROL,
                                targetPositions = [ joint_angle_array[0],
                                                    joint_angle_array[1],
                                                    joint_angle_array[2] ])
    # pb.stepSimulation()
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
def JointAngleControl(stochID, jointAngles, enablePrint):
    joint_angle_array = np.array(jointAngles)
    pb.setJointMotorControlArray(bodyUniqueId=stochID,
                                jointIndices = jointArray,
                                controlMode= pb.POSITION_CONTROL,
                                targetPositions = joint_angle_array)
    pb.stepSimulation()
    if enablePrint:
        print(  
                "ID("+ str(jointArray[0]) + ")=" + str(np.round(joint_angle_array[0], 2)) + ", " 
                "ID("+ str(jointArray[1]) + ")=" + str(np.round(joint_angle_array[1], 2)) + ", " 
                "ID("+ str(jointArray[2]) + ")=" + str(np.round(joint_angle_array[2], 2)) + ", " 
                "ID("+ str(jointArray[3]) + ")=" + str(np.round(joint_angle_array[3], 2)) + ", " 
                "ID("+ str(jointArray[4]) + ")=" + str(np.round(joint_angle_array[4], 2)) + ", " 
                "ID("+ str(jointArray[5]) + ")=" + str(np.round(joint_angle_array[5], 2)) + ", " 
                "ID("+ str(jointArray[6]) + ")=" + str(np.round(joint_angle_array[6], 2)) + ", " 
                "ID("+ str(jointArray[7]) + ")=" + str(np.round(joint_angle_array[7], 2)) + ", " 
                "ID("+ str(jointArray[8]) + ")=" + str(np.round(joint_angle_array[8], 2)) + ", " 
                "ID("+ str(jointArray[9]) + ")=" + str(np.round(joint_angle_array[9], 2)) + ", " 
                "ID("+ str(jointArray[10]) + ")=" + str(np.round(joint_angle_array[10], 2)) + ", " 
                "ID("+ str(jointArray[11]) + ")=" + str(np.round(joint_angle_array[11], 2))
                )

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
    # return foot_coordinates
    return foot_coordinates

# Forward kinematics in XZ plane for theta 2 and theta 3
def forwardKinematicsin_XZ_plane(angles):
    theta1, theta2 = angles[1], angles[2]
    X = (L1*math.sin(theta1) + L2*math.sin(theta1+theta2))
    Z = -(L1*math.cos(theta1) + L2*math.cos(theta1+theta2))
    return np.array([X,0,Z])

# Inverse kinematics for 2 dof leg
def inverseKinmematics(endPositionInAbdFrame):
    delta = math.asin(L_ABD/np.sqrt(endPositionInAbdFrame[1]**2 + endPositionInAbdFrame[2]**2))
    abdJoint = math.atan(endPositionInAbdFrame[1]/ abs(endPositionInAbdFrame[2])) + delta
    xDash = endPositionInAbdFrame[0]
    yDash = endPositionInAbdFrame[1] * math.cos(abdJoint) + endPositionInAbdFrame[2] * math.sin(abdJoint)
    zDash = - endPositionInAbdFrame[1] * math.sin(abdJoint) + endPositionInAbdFrame[2] * math.cos(abdJoint)
    x = -zDash
    y = xDash
    coskneeJoint = ((x**2 + y**2) - (L1**2 + L2**2))/ (2*L1*L2)
    kneeJoint = math.acos(limitValue(coskneeJoint))
    determinant = L1**2 + L2**2 + 2*L1*L2*math.cos(kneeJoint)
    sinHipJoint1 = (x * (L1 + L2*math.cos(kneeJoint)) + y * (L2*math.sin(kneeJoint)))/ determinant
    sinHipJoint1 = limitValue(sinHipJoint1)
    hipJoint = math.asin(sinHipJoint1) - math.pi/2
    return [abdJoint, -hipJoint, -kneeJoint]

############################################################################################################################
# Foot trajectory

def generateTransitionPointMatrices(xCentral, zCentral, upperWidth, lowerWidth, centralWidth, liftHeight, groundHeight, depth):
    pointInWorkspace = False
    if depth + groundHeight <= 0.6111:
        if centralWidth <=  2*math.sqrt((0.6111)**2 - depth**2):
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
        if centralWidth <=  np.sqrt((0.6111)**2 - depth**2):
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
    
def getPointForTrajectory(angle, tilt, liftPointMatrix, groundPointMatrix):
    if angle == 360: angle = 0
    if int(angle) in range(0,180):
        t = angle/180
        curvePoint = liftPointMatrix @ cubicWeightMatrix @ np.array([t**3, t**2, t, 1])
    elif int(angle) in range(180,360):
        t = (angle - 180)/180
        curvePoint = groundPointMatrix @ cubicWeightMatrix @ np.array([t**3, t**2, t, 1])
    return [curvePoint[0]*math.cos(tilt), curvePoint[0]*math.sin(tilt), curvePoint[1]]

def getPointForTransition(t, transitionLiftPointMatrix, transitionGroundPointMatrix):
    quadraticTrajectoryPoint = transitionLiftPointMatrix @ quadraticWeightMatrix @ np.array([t**2, t, 1])
    linearTrajectoryPoint = transitionGroundPointMatrix @ linearWeightMatrix @ np.array([t, 1])
    return quadraticTrajectoryPoint, linearTrajectoryPoint

############################################################################################################################
# Crawl

def takePosition(stochID, transitionLiftPointMatrix, transitionGroundPointMatrix, transition2):
    quadraticTrajectoryPoint, linearTrajectoryPoint = getPointForTransition(0, transitionLiftPointMatrix, transitionGroundPointMatrix)
    jointAnglesPhaseL = inverseKinmematics([quadraticTrajectoryPoint[0], -L_ABD, quadraticTrajectoryPoint[1]])
    jointAnglesPhaseR = inverseKinmematics([quadraticTrajectoryPoint[0], -L_ABD, quadraticTrajectoryPoint[1]])
    # Get 
    for i in range(100):
        i = i/100
        _jointAnglesPhaseL = [i*jointAnglesPhaseL[0], i*jointAnglesPhaseL[1], i*jointAnglesPhaseL[2]]
        _jointAnglesPhaseR = [i*jointAnglesPhaseR[0], i*jointAnglesPhaseR[1], i*jointAnglesPhaseR[2]]
        _jointAngles =  _jointAnglesPhaseL + _jointAnglesPhaseR + _jointAnglesPhaseL + _jointAnglesPhaseR
        JointAngleControl(stochID, _jointAngles, enablePrint=0)

    # time.sleep(1)
    # Set
    if transition2:
        for i in range(100):
            i = i/100
            quadraticTrajectoryPoint, linearTrajectoryPoint = getPointForTransition(i, transitionLiftPointMatrix, transitionGroundPointMatrix)
            jointAnglesFL = inverseKinmematics([linearTrajectoryPoint[0], -L_ABD, linearTrajectoryPoint[1]])
            jointAnglesBR = inverseKinmematics([linearTrajectoryPoint[0], -L_ABD, linearTrajectoryPoint[1]])
            jointAnglesBL = inverseKinmematics([quadraticTrajectoryPoint[0], -L_ABD, quadraticTrajectoryPoint[1]])
            jointAnglesFR = inverseKinmematics([quadraticTrajectoryPoint[0], -L_ABD, quadraticTrajectoryPoint[1]])
            _jointAngles =  jointAnglesFL + jointAnglesFR + jointAnglesBL + jointAnglesBR
                #                   FL              FR                 BL             BR
            JointAngleControl(stochID, _jointAngles, enablePrint=0)


def trot(stochID, tilt, i, liftPointMatrix, groundPointMatrix):
    if i in range(0,180): j = i + 180
    elif i in range(180, 360): j = i - 180
    phase0_F = getPointForTrajectory(i, tilt, liftPointMatrix, groundPointMatrix)
    phase180_F = getPointForTrajectory(j, tilt, liftPointMatrix, groundPointMatrix)
    phase0_B = getPointForTrajectory(i, -tilt, liftPointMatrix, groundPointMatrix)
    phase180_B = getPointForTrajectory(j, -tilt, liftPointMatrix, groundPointMatrix)
    jointAnglesFL = inverseKinmematics([phase0_F[0], phase0_F[1]-L_ABD, phase0_F[2]])
    jointAnglesBR = inverseKinmematics([phase0_B[0], phase0_F[1]-L_ABD, phase0_B[2]])
    jointAnglesBL = inverseKinmematics([phase180_B[0], phase0_F[1]-L_ABD, phase180_B[2]])
    jointAnglesFR = inverseKinmematics([phase180_F[0], phase0_F[1]-L_ABD, phase180_F[2]])
    _jointAngles =  jointAnglesFL + jointAnglesFR + jointAnglesBL + jointAnglesBR
    #                   FL              FR                 BL             BR
    JointAngleControl(stochID, _jointAngles, enablePrint=0)
