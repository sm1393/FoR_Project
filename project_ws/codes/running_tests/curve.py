import math
import matplotlib.pyplot as plt

r = 10
angles = [i*(math.pi/360) for i in range(720)]
x = [(r*math.cos(i)) for i in angles]
y = [(r*math.sin(i) + 10) for i in angles]
print(len(x))
plt.plot(x,y)
plt.show()