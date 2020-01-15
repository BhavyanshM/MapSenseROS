 
from mpl_toolkits import mplot3d

import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure()
ax = plt.axes(projection="3d")

x_line = np.linspace(-2, 2, 5)
y_line = np.linspace(-2, 2, 5)

X, Y = np.meshgrid(x_line, y_line)


print(X,Y)

# Z = function(X,Y)

Z = 20 + X**2 + Y**2 - 10*np.cos(2*np.pi*X) - 10*np.cos(2*np.pi*Y)
# Z = X + Y

# ax.plot_wireframe(X, Y, Z, color='green')

ax.plot_surface(X, Y, Z, rstride=1, cstride=1,
                cmap='winter', edgecolor='none')

plt.show()