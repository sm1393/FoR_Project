import pybullet as pb
import time
import pybullet_data
import os
import math
import numpy as np
import lib_stoch_3D as stoch
import common_paths

import matplotlib.pyplot as plt

physicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
stochUrdf = common_paths.stochUrdfFile

print("#################################################################################################")

pb.setGravity(0,0,-10)
planeId = pb.loadURDF("plane.urdf")
# stochID = pb.loadURDF(stochUrdf, [0,0,0.6], pb.getQuaternionFromEuler([0,0,6*math.pi/18]))
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

# Straight Line
# x = np.array([i/100 for i in range(1000)])
# y = np.zeros((1000))

# Circle
r = 10
x = [r*math.cos(math.pi*(i/10 - 90)/180) for i in range(3600)]
y = [r*math.sin(math.pi*(i/10 - 90)/180)+10 for i in range(3600)]

# Sin Wave
# A = 5
# x = np.array([i/10 for i in range(500)])
# y = np.array([A*math.sin(0.05*math.pi*i) for i in x])

path = np.array([x,y]).T

pathtraced = stoch.tracePath(stochID, path)

plt.plot(pathtraced.T[0], pathtraced.T[1])
plt.plot(x, y)
plt.show()

input()
plt.close()

########################################################################################################

print("#################################################################################################")

pb.disconnect()