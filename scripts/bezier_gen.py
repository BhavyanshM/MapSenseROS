 
from mpl_toolkits import mplot3d

import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure(figsize=(10,10))
ax = plt.axes(projection="3d")

x_line = np.linspace(-1, 2, 4)
y_line = np.linspace(-1, 2, 4)
X, Y = np.meshgrid(x_line, y_line)

u_line = np.linspace(-1, 2, 50)
v_line = np.linspace(-1, 2, 50)
U, V = np.meshgrid(u_line, v_line)

W = 20 + U**2 + V**2 - 10*np.cos(U) - 10*np.cos(V)
Z = 20 + X**2 + Y**2 - 10*np.cos(X) - 10*np.cos(Y)


print("X:",X)
print("Y:",Y)
print("Z:",Z)

# Z = function(U,V)

# Z = X + Y

# ax.plot_wireframe(X, Y, Z, color='green')

ax.plot_surface(U, V, W, rstride=1, cstride=1,
                cmap='winter', edgecolor='none')

ax.scatter(X, Y, Z, color='r', marker='o', s=30)

plt.show()