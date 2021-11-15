from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import numpy as np


fig = plt.figure(figsize=(10,8))
ax = Axes3D(fig)

data = np.loadtxt("data3D.txt", delimiter=',', dtype=np.float64)

print(data.shape)

x = data[:,0]
y = data[:,1]
z = data[:,2]

ax.axes.set_xlim3d(-4.0, 4.0)
ax.axes.set_ylim3d(-4.0, 4.0)
ax.axes.set_zlim3d(-4.0, 4.0)

ax.scatter(x, y, z)
plt.show()