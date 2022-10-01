import pybullet as pb
import time
import pybullet_data
import os
import math
import lib_stoch as stoch
import common_paths

def d2r(degree):
    return math.pi*degree/180

physicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
stochUrdf = common_paths.stochUrdfFile

print("#################################################################################################")

pb.setGravity(0,0,-10)
planeId = pb.loadURDF("plane.urdf")
stochID = pb.loadURDF(stochUrdf, [0,0,1])
maxForce = 500

joints = pb.getNumJoints(stochID)
for i in range(joints):
    joint = pb.getJointInfo(stochID,i)
    print(i, "=", joint[0]," ", joint[1]," ", joint[2])

pb.setRealTimeSimulation(enableRealTimeSimulation = 1)
pb.setJointMotorControlArray(bodyUniqueId=stochID,
                                jointIndices=[0,1,2,3,4,5,6,7,8,9,10,11],
                                controlMode= pb.POSITION_CONTROL,
                                targetPositions = [ d2r(0),   #0 
                                                    d2r(0),   #1 
                                                    d2r(0),   #2
                                                    d2r(0),   #3 
                                                    d2r(0),   #4 
                                                    d2r(0),   #5
                                                    d2r(0),   #6 
                                                    d2r(0),   #7 
                                                    d2r(0),   #8
                                                    d2r(0),   #9 
                                                    d2r(0),   #10 
                                                    d2r(0)])  #11
                                                    
time.sleep(1)

while True:
    step_size = 100
    for i in range(1,step_size):
        j = ( (i - 0) / (step_size - 0) ) * (1 - 0) + 0
        pb.setJointMotorControlArray(bodyUniqueId=stochID,
                                jointIndices=[0,1,2,3,4,5,6,7,8,9,10,11],
                                controlMode= pb.POSITION_CONTROL,
                                targetPositions = [d2r(0),   #0 
                                                    d2r(60*j),   #1 
                                                    d2r(-120*j),   #2
                                                    d2r(0),   #3 
                                                    d2r(60*j),   #4 
                                                    d2r(-120*j),   #5
                                                    d2r(0),   #6 
                                                    d2r(60*j),   #7 
                                                    d2r(-120*j),   #8
                                                    d2r(0),   #9 
                                                    d2r(60*j),   #10 
                                                    d2r(-120*j)])  #11
        time.sleep(5*1/240)

    for i in range(1,step_size):
        j = ( ((step_size-i) - 0) / (step_size - 0) ) * (1 - 0) + 0
        pb.setJointMotorControlArray(bodyUniqueId=stochID,
                                jointIndices=[0,1,2,3,4,5,6,7,8,9,10,11],
                                controlMode= pb.POSITION_CONTROL,
                                targetPositions = [d2r(0),   #0 
                                                    d2r(60*j),   #1 
                                                    d2r(-120*j),   #2
                                                    d2r(0),   #3 
                                                    d2r(60*j),   #4 
                                                    d2r(-120*j),   #5
                                                    d2r(0),   #6 
                                                    d2r(60*j),   #7 
                                                    d2r(-120*j),   #8
                                                    d2r(0),   #9 
                                                    d2r(60*j),   #10 
                                                    d2r(-120*j)])  #11
        time.sleep(5*1/240)

print("#################################################################################################")