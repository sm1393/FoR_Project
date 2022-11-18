import numpy as np
import matplotlib.pyplot as plt

P0 = [-1,0]
P1 = [-0.5,1]
P2 = [0.5,1]
P3 = [1,0]

pointMatrix = np.array([P0, P1, P2, P3])

weightMatrix = np.array([[-1,3,-3,1],
                        [3,-6,3,0],
                        [-3,3,0,0],
                        [1,0,0,0]])

def trace(t):
    point = pointMatrix.T @ weightMatrix  @ np.array([t**3, t**2, t, 1])
    return point


print("pointMatrix = ", pointMatrix)
print("len(pointMatrix) = ", pointMatrix.shape)
print("weightMatrix = ", weightMatrix)
print("len(weightMatrix) = ", weightMatrix.shape)

steps = 1000
pointArray = []

for i in range(steps):
    i = i/steps
    pointArray.append(trace(i))

pointArray = np.array(pointArray)
print("pointArray = ", pointArray.shape)

pointArrayX = pointArray[:,0]
pointArrayY = pointArray[:,1]

print("pointArrayX = ", pointArrayX.shape)
print("pointArrayY = ", pointArrayY.shape)

# plt.figure(figsize=(3,3))
plt.xlim([-1, 1])
plt.ylim([-1, 1])
plt.plot(pointArrayX, pointArrayY, 'red')
plt.show()
