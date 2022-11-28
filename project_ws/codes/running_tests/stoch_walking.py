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
# stochID = pb.loadURDF(stochUrdf, [0,0,0.6], pb.getQuaternionFromEuler([0,0,math.pi/2]))

stoch.printJointInfo(stochID)

pb.setRealTimeSimulation(enableRealTimeSimulation = 0)

########################################################################################################
# q = 0.3335
q = 0.8
xCentral = 0
zCentral = 0
upperWidth = 0.04 * q
lowerWidth = 0 #0.02
centralWidth = 0.2 * q
liftHeight = 0.09*0.5* q
groundHeight = 0 #0.025
depth = 0.4 

# q = 1.5 
# xCentral = 0
# zCentral = 0
# upperWidth = 0.04 * q
# lowerWidth = 0 #0.02
# centralWidth = 0.2 * q
# liftHeight = 0.09 *0.5* q
# groundHeight = 0 #0.025
# depth = 0.4 

transitionLiftPointMatrix, transitionGroundPointMatrix = stoch.generateTransitionPointMatrices(xCentral, zCentral, upperWidth, lowerWidth, centralWidth, liftHeight, groundHeight, depth)
liftPointMatrix, groundPointMatrix = stoch.generateWalkPointMatrices(xCentral, zCentral, upperWidth, lowerWidth, centralWidth, liftHeight, groundHeight, depth)

time.sleep(1)

stoch.takePosition(stochID, transitionLiftPointMatrix, transitionGroundPointMatrix, transition2 = True)

while True:
    for i in range(360):
        basePos, baseOrn = pb.getBasePositionAndOrientation(stochID)
        pb.resetDebugVisualizerCamera( cameraDistance=1.5, cameraYaw=30, cameraPitch=-30, cameraTargetPosition=basePos)
        stoch.creep(stochID, i, liftPointMatrix, groundPointMatrix)
        # input()

time.sleep(1)

########################################################################################################

print("#################################################################################################")

pb.disconnect()