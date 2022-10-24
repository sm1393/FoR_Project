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

def temp(angles):
    observed = stoch.getObservedFootCoordinates(stochID)
    print("observed = ", observed[0]) # Only FL foot
    # angles = [0, 0, -90, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # stoch.JointAngleControl(stochID, angles, enablePrint=0)
    angles = stoch.degree2Radians(angles)
    stoch.JointAngleControl_FL(stochID, angles, enablePrint=1)
    time.sleep(1)
    _, calculated = stoch.getCalculatedFootCoordinates(stochID, angles)
    observed = stoch.getObservedFootCoordinates(stochID)
    print("observed = ", observed[0], "calculated = ", calculated)
    print("#################################################################Error:", np.round(calculated - observed[0], 4))

print("#################################################################################################")

# angles = np.array([0,90,0])
# temp(angles)
# time.sleep(1)
# input()
angles = np.array([0,-90,90])
temp(angles)
time.sleep(1)
input()
# angles = np.array([0,-80,170])
# temp(angles)
# time.sleep(1)
# input()

print("#################################################################################################")

pb.disconnect()