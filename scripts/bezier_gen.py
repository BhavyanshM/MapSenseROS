 
from mpl_toolkits import mplot3d

import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure(figsize=(10,10))
ax = plt.axes(projection="3d")

x_line = np.linspace(-3, 3, 6)
y_line = np.linspace(-3, 3, 6)

U, V = np.meshgrid(x_line, y_line)


print(U,V)

# Z = function(U,V)

Z = 20 + U**2 + V**2 - 10*np.cos(U) - 10*np.cos(V)
# Z = X + Y

# ax.plot_wireframe(X, Y, Z, color='green')

ax.plot_surface(U, V, Z, rstride=1, cstride=1,
                cmap='winter', edgecolor='none')

ax.scatter(U, V, Z, color='r', marker='o')

plt.show()