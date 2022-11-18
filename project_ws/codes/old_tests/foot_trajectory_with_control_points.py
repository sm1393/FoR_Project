import numpy as np
import matplotlib.pyplot as plt

xCentral = 0
zCentral = 0
upperWidth = 0.04
lowerWidth = 0.02
centralWidth = 0.05
liftHeight = 0.15
groundHeight = 0.025
depth = 0.4

if depth + groundHeight <= 0.6111:
    if centralWidth <=  ((0.6111)**2 - depth**2):
        print("point in workspace")
    else:
        print("out of workspace")
else:
    print("out of workspace")


liftPointMatrix = np.array([[xCentral - centralWidth/2 ,zCentral - depth],
                            [xCentral - upperWidth/2 ,zCentral - depth + liftHeight],
                            [xCentral + upperWidth/2, zCentral - depth + liftHeight],
                            [xCentral + centralWidth/2,zCentral - depth],]).T

groundPointMatrix = np.array([[xCentral + centralWidth/2 ,zCentral - depth],
                              [xCentral + lowerWidth/2 ,zCentral - depth - groundHeight],
                              [xCentral - lowerWidth/2 , zCentral - depth - groundHeight],
                              [xCentral - centralWidth/2 ,zCentral - depth]]).T

def trace(angle):
    if angle == 360: angle = 0
    if int(angle) in range(0,180):
        t = angle/180
        return liftPointMatrix @ weightMatrix @ np.array([t**3, t**2, t, 1])
    elif int(angle) in range(180,360):
        t = (angle - 180)/180
        return groundPointMatrix @ weightMatrix @ np.array([t**3, t**2, t, 1])

weightMatrix = np.array([[-1,3,-3,1],
                        [3,-6,3,0],
                        [-3,3,0,0],
                        [1,0,0,0]])

print("liftPointMatrix = ", liftPointMatrix)
print("groundPointMatrix = ", groundPointMatrix)

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