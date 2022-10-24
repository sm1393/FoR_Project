from tokenize import Pointfloat
import numpy as np
import matplotlib.pyplot as plt

liftPointMatrix = np.array([[-1,0], [-0.5,1], [0.5,1], [1,0]]).T
groundPointMatrix = np.array([[1,0], [0.5,-0.25], [-0.5,-0.25], [-1,0]]).T

weightMatrix = np.array([[-1,3,-3,1],
                        [3,-6,3,0],
                        [-3,3,0,0],
                        [1,0,0,0]])

def trace(angle):
    if angle == 360: angle = 0
    if int(angle) in range(0,180):
        t = angle/180
        return liftPointMatrix @ weightMatrix @ np.array([t**3, t**2, t, 1])
    elif int(angle) in range(180,360):
        t = (angle - 180)/180
        return groundPointMatrix @ weightMatrix @ np.array([t**3, t**2, t, 1])

print("liftPointMatrix = ", liftPointMatrix)
print("len(liftPointMatrix) = ", liftPointMatrix.shape)

print("groundPointMatrix = ", groundPointMatrix)
print("len(groundPointMatrix) = ", groundPointMatrix.shape)

steps = 1000
pointArray = []
for i in range(steps):
    i = (i/1000)*360
    point = trace(i)
    pointArray.append(point)

pointArray = np.array(pointArray)
print("pointArray = ", pointArray.shape)

pointArrayX = pointArray[:,0]
pointArrayY = pointArray[:,1]

print("pointArrayX = ", pointArrayX.shape)
print("pointArrayY = ", pointArrayY.shape)

plt.plot(pointArrayX, pointArrayY, 'red')
plt.show()