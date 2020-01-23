 
from mpl_toolkits import mplot3d

import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure(figsize=(20,20))
ax = plt.axes(projection="3d")

x_line = np.linspace(-1, 2, 4)
y_line = np.linspace(-1, 2, 4)
X, Y = np.meshgrid(x_line, y_line)

u_line = np.linspace(-1, 2, 30)
v_line = np.linspace(-1, 2, 30)
U, V = np.meshgrid(u_line, v_line)

W = 20 + U**2 + V**2 - 10*U**3 - 10*V**3
W += np.random.normal(0,5,W.shape)

Z = 20 + X**2 + Y**2 - 10*X**3 - 10*Y**3
# Z += np.random.normal(0,100,Z.shape)


print("X:",X.shape)
print("Y:",Y.shape)
print("Z:",Z.shape)

print("U:",U.shape)
print("V:",V.shape)
print("W:",W.shape)


# Z = function(U,V)

# Z = X + Y

# ax.plot_wireframe(X, Y, Z, color='green')

# ax.plot_surface(U, V, W, rstride=1, cstride=1,
#                 cmap='winter', edgecolor='none')

# ax.scatter(X, Y, Z, color='r', marker='o', s=50)

ax.scatter(U, V, W, color='r', marker='o', s=50)

plt.show()