import pybullet as pb
import time
import pybullet_data
import os
import math
import numpy as np
import lib_stoch as stoch
import common_paths

physicsClient = pb.connect(pb.GUI)
physicsClient = pb.connect(pb.DIRECT)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
stochUrdf = common_paths.stochUrdfFile

print("#################################################################################################")

pb.setGravity(0,0,-10)
planeId = pb.loadURDF("plane.urdf")
stochID = pb.loadURDF(stochUrdf, [0,0,0.6])
# stochID = pb.loadURDF(stochUrdf, [0,0,0.6], pb.getQuaternionFromEuler([0,0,math.pi/2]))

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

while True:
    for i in range(180):
        basePos, baseOrn = pb.getBasePositionAndOrientation(stochID) # Get model position
        pb.resetDebugVisualizerCamera( cameraDistance=2, cameraYaw=30, cameraPitch=-30, cameraTargetPosition=basePos) # fix camera onto model
        stoch.trot(stochID, 2*i, liftPointMatrix, groundPointMatrix)

time.sleep(1)

########################################################################################################

print("#################################################################################################")

pb.disconnect()