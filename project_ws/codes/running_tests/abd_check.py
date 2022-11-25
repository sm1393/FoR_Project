import pybullet as pb
import time
import pybullet_data
import os
import math
import numpy as np
import lib_stoch_3D as stoch
import common_paths

physicsClient = pb.connect(pb.GUI)
physicsClient = pb.connect(pb.DIRECT)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
stochUrdf = common_paths.stochUrdfFile


pb.setGravity(0,0,-10)
planeId = pb.loadURDF("plane.urdf")
stochID = pb.loadURDF(stochUrdf, [0,0,1])
# stochID = pb.loadURDF(stochUrdf, [0,0,0.6], pb.getQuaternionFromEuler([0,0,math.pi/2]))

# stoch.printJointInfo(stochID)

pb.setRealTimeSimulation(enableRealTimeSimulation = 1)
# pb.setRealTimeSimulation(enableRealTimeSimulation = 0)

print("#################################################################################################")

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

input()
angles = [-70,70,-70]   # knee joint only takes negative angles
stoch.JointAngleControl_FL(stochID, stoch.degree2Radians(np.array(angles)), enablePrint = 0) # requires degrees
stoch.JointAngleControl_FR(stochID, stoch.degree2Radians(np.array(angles)), enablePrint = 0) # requires degrees
stoch.JointAngleControl_BL(stochID, stoch.degree2Radians(np.array(angles)), enablePrint = 0) # requires degrees
stoch.JointAngleControl_BR(stochID, stoch.degree2Radians(np.array(angles)), enablePrint = 0) # requires degrees
time.sleep(1)
input()
endCoords = stoch.getObservedFootCoordinates(stochID)
jointAngles = stoch.inverseKinmematics(endCoords) # Return angles in radians
print(  "endCoords = ", endCoords,
        "\nangles given = ", angles,
        "\tjointAngles = ", stoch.radian2Degree(np.array(jointAngles)))
input()
stoch.JointAngleControl_FL(stochID, jointAngles, enablePrint = 0)
stoch.JointAngleControl_FR(stochID, jointAngles, enablePrint = 0)
stoch.JointAngleControl_BL(stochID, jointAngles, enablePrint = 0)
stoch.JointAngleControl_BR(stochID, jointAngles, enablePrint = 0)
input()

print("#################################################################################################")

pb.disconnect()