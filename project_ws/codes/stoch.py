import time
import os
import math
import pybullet as pb
import pybullet_data
import sys
import lib_stoch as stoch

stochUrdf = "/home/swapnil/FoR_Project/project_ws/stoch3/urdf/stoch3.urdf"

physicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.setGravity(0,0,-10)
planeId = pb.loadURDF("plane.urdf")
stochID = pb.loadURDF(stochUrdf, [0,0,1])
print("#################################################################################################")


print("#################################################################################################")

