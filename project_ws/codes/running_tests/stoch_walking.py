import pybullet as pb
import time
import pybullet_data
import os
import math
import numpy as np
import lib_stoch_3D as stoch
import common_paths

physicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
stochUrdf = common_paths.stochUrdfFile

print("#################################################################################################")

pb.setGravity(0,0,-10)
planeId = pb.loadURDF("plane.urdf")
stochID = pb.loadURDF(stochUrdf, [0,0,0.6])

stoch.printJointInfo(stochID)

pb.setRealTimeSimulation(enableRealTimeSimulation = 0)

########################################################################################################

xCentral = 0
zCentral = 0
upperWidth = 0.04
lowerWidth = 0.02
centralWidth = 0.2
liftHeight = 0.09
groundHeight = 0 #0.025
depth = 0.4

transitionLiftPointMatrix, transitionGroundPointMatrix = stoch.generateTransitionPointMatrices(xCentral, zCentral, upperWidth, lowerWidth, centralWidth, liftHeight, groundHeight, depth)
liftPointMatrix, groundPointMatrix = stoch.generateWalkPointMatrices(xCentral, zCentral, upperWidth, lowerWidth, centralWidth, liftHeight, groundHeight, depth)

time.sleep(1)

stoch.takePosition(stochID, transitionLiftPointMatrix, transitionGroundPointMatrix, transition2 = True)
tilt = stoch.degree2Radians(20)

while True:
    for i in range(180):
        basePos, baseOrn = pb.getBasePositionAndOrientation(stochID)
        pb.resetDebugVisualizerCamera(cameraDistance = 3, cameraYaw = 90, cameraPitch = -85, cameraTargetPosition = basePos)
        # pb.resetDebugVisualizerCamera(cameraDistance = 3, cameraYaw = 30, cameraPitch = -30, cameraTargetPosition = basePos)
        stoch.trot(stochID, tilt, 2*i, liftPointMatrix, groundPointMatrix)

time.sleep(1)

########################################################################################################

print("#################################################################################################")

pb.disconnect()