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
groundHeight = 0
depth = 0.4

transitionLiftPointMatrix, transitionGroundPointMatrix = stoch.generateTransitionPointMatrices(xCentral, zCentral, upperWidth, lowerWidth, centralWidth, liftHeight, groundHeight, depth)

time.sleep(1)

stoch.takePosition(stochID, transitionLiftPointMatrix, transitionGroundPointMatrix, transition2 = True)

Kp = 0.0005
desiredVelocity = 1
tilt = 0
width = 0

while True:
    for i in range(180):
        basePos, baseOrn = pb.getBasePositionAndOrientation(stochID)
        pb.resetDebugVisualizerCamera(cameraDistance = 2, cameraYaw = 30, cameraPitch = -30, cameraTargetPosition = basePos)
        error = desiredVelocity - stoch.bodyTwist(stochID)[0]
        width += Kp * error
        width = np.clip(width, 0, np.sqrt((0.6111)**2 - depth**2))
        liftPointMatrix, groundPointMatrix = stoch.generateWalkPointMatrices(xCentral, zCentral, upperWidth, lowerWidth, width, liftHeight, groundHeight, depth)
        stoch.trot(stochID, tilt, i*2, liftPointMatrix, groundPointMatrix)

########################################################################################################

print("#################################################################################################")

pb.disconnect()