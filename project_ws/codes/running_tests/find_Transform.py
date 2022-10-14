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

# time.sleep(1)

while True:
    step_size = 1000
    for i in range(1,step_size):
        j = ( (i - 0) / (step_size - 0) ) * (1 - 0) + 0
        angles = [0, 60*j, -120*j, 0, 60*j, -120*j, 0, 60*j, -120*j, 0, 60*j, -120*j]
        stoch.JointAngleControl(stochID, angles, enablePrint=0)
        stoch.getLinkInfo(stochID)
        time.sleep(1/240)

    for i in range(1,step_size):
        j = ( ((step_size-i) - 0) / (step_size - 0) ) * (1 - 0) + 0
        angles = [0, 60*j, -120*j, 0, 60*j, -120*j, 0, 60*j, -120*j, 0, 60*j, -120*j]
        stoch.JointAngleControl(stochID, angles, enablePrint=0)
        stoch.getLinkInfo(stochID)
        time.sleep(1/240)

print("#################################################################################################")