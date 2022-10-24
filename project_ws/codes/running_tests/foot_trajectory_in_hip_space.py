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

liftPointMatrix, groundPointMatrix = stoch.generatePointMatrices(xCentral, zCentral, upperWidth, lowerWidth, centralWidth, liftHeight, groundHeight, depth)

time.sleep(1)

steps = 1000
# while True:
pointArray = []
while True:
    for i in range(steps):
        if i in range(0,500): j = i + 500
        elif i in range(500, 1000): j = i - 500
        i = (i/1000)*360
        j = (j/1000)*360
        phase0 = stoch.getPointFromAngle(i, liftPointMatrix, groundPointMatrix)
        phase180 = stoch.getPointFromAngle(j, liftPointMatrix, groundPointMatrix)
        endPositionPhase0 = [phase0[0], 0, phase0[1]]
        endPositionPhase180 = [phase180[0], 0, phase180[1]]
        jointAnglesPhase0 = stoch.inverseKinmematics(endPositionPhase0)
        jointAnglesPhase180 = stoch.inverseKinmematics(endPositionPhase180)
        stoch.JointAngleControl_FL(stochID, jointAnglesPhase0, enablePrint=0)
        stoch.JointAngleControl_BR(stochID, jointAnglesPhase0, enablePrint=0)
        stoch.JointAngleControl_FR(stochID, jointAnglesPhase180, enablePrint=0)
        stoch.JointAngleControl_BL(stochID, jointAnglesPhase180, enablePrint=0)

time.sleep(1)

########################################################################################################

print("#################################################################################################")

pb.disconnect()