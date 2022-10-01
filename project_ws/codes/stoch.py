import time
import os
import math
import pybullet as pb
import pybullet_data
import sys
import lib_stoch as stoch
import common_paths

stochUrdf = common_paths.stochUrdfFile

physicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.setGravity(0,0,-10)
planeId = pb.loadURDF("plane.urdf")
stochID = pb.loadURDF(stochUrdf, [0,0,1])
print("#################################################################################################")

time.sleep(5)

print("#################################################################################################")

