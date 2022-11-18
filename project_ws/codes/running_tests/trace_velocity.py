import pybullet as pb
import time
import pybullet_data
import os
import math
import numpy as np
import lib_stoch as stoch
import common_paths

physicsClient = pb.connect(pb.GUI)
# physicsClient = pb.connect(pb.DIRECT)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
stochUrdf = common_paths.stochUrdfFile

print("#################################################################################################")

pb.setGravity(0,0,-10)
planeId = pb.loadURDF("plane.urdf")
stochID = pb.loadURDF(stochUrdf, [0,0,0.6])

# stoch.printJointInfo(stochID)

pb.setRealTimeSimulation(enableRealTimeSimulation = 0)

########################################################################################################

xCentral = 0
zCentral = 0
upperWidth = 0.04
lowerWidth = 0.02
# centralWidth = 0.05
liftHeight = 0.09
groundHeight = 0.025
depth = 0.4

# transitionLiftPointMatrix, transitionGroundPointMatrix = stoch.generateTransitionPointMatrices(xCentral, zCentral, upperWidth, lowerWidth, centralWidth, liftHeight, groundHeight, depth)
# liftPointMatrix, groundPointMatrix = stoch.generateWalkPointMatrices(xCentral, zCentral, upperWidth, lowerWidth, centralWidth, liftHeight, groundHeight, depth)

time.sleep(1)

# stoch.takePosition(stochID, transitionLiftPointMatrix, transitionGroundPointMatrix)

############################################
# control params
desiredVelocity = 0.75
Kp = 0
Ki = 0.001
error = 0
cumError = 0
############################################

maxCentralWidth = math.sqrt((0.6111)**2 - depth**2) # maxWidth

while True:
    for i in range(180):
        bodyVelocity = stoch.bodyTwist(stochID)
        error = desiredVelocity - bodyVelocity[0]
        cumError += error
        width = Ki * cumError
        if width > maxCentralWidth:
            width = maxCentralWidth
            cumError -= error
        print("bodyVelocity = ", bodyVelocity, "width  = ", width, "error = ", error, "cumError = ", cumError,)
        liftPointMatrix, groundPointMatrix = stoch.generateWalkPointMatrices(xCentral, zCentral, upperWidth, lowerWidth, width, liftHeight, groundHeight, depth)
        stoch.trot(stochID, 2*i, liftPointMatrix, groundPointMatrix)
time.sleep(1)

########################################################################################################

print("#################################################################################################")

pb.disconnect()