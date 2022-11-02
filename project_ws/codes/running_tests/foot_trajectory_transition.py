import pybullet as pb
import time
import pybullet_data
import os
import math
import numpy as np
import lib_stoch as stoch
import common_paths

physicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
stochUrdf = common_paths.stochUrdfFile

print("#################################################################################################")

pb.setGravity(0,0,-10)
planeId = pb.loadURDF("plane.urdf")
stochID = pb.loadURDF(stochUrdf, [0,0,1])
maxForce = 500

stoch.printJointInfo(stochID)

pb.setRealTimeSimulation(enableRealTimeSimulation = 1)

########################################################################################################


xCentral = 0
zCentral = 0
upperWidth = 0.04
lowerWidth = 0.02
centralWidth = 0.05
liftHeight = 0.09
groundHeight = 0.025
depth = 0.4

liftPointMatrix, groundPointMatrix, transitionLiftPointMatrix, transitionGroundPointMatrix = stoch.generatePointMatrices(xCentral, zCentral, upperWidth, lowerWidth, centralWidth, liftHeight, groundHeight, depth)

time.sleep(2)

for i in range(1000):
    i = i/1000
    quadraticTrajectoryPoint, linearTrajectoryPoint = stoch.getPointForTransition(i, transitionLiftPointMatrix, transitionGroundPointMatrix)
    # print("trajectory points = ", quadraticTrajectoryPoint, linearTrajectoryPoint)
    endPositionPhase0 = [quadraticTrajectoryPoint[0], 0, quadraticTrajectoryPoint[1]]
    endPositionPhase180 = [linearTrajectoryPoint[0], 0, linearTrajectoryPoint[1]]
    jointAnglesPhase0 = stoch.inverseKinmematics(endPositionPhase0)
    jointAnglesPhase180 = stoch.inverseKinmematics(endPositionPhase180)
    # print("Joint angles = ", jointAnglesPhase0, jointAnglesPhase180)
    stoch.JointAngleControl_FL(stochID, jointAnglesPhase0, enablePrint=0)
    stoch.JointAngleControl_BR(stochID, jointAnglesPhase0, enablePrint=0)
    stoch.JointAngleControl_FR(stochID, jointAnglesPhase180, enablePrint=0)
    stoch.JointAngleControl_BL(stochID, jointAnglesPhase180, enablePrint=0)
    time.sleep(0.001)


time.sleep(1)

########################################################################################################

print("#################################################################################################")

pb.disconnect()