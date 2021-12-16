from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
import random
import numpy as np
from NonLinearSolver import *
from Camera import *
np.set_printoptions(precision=2)
fig = plt.figure(figsize=(28,14))

SHOW_3D = True
GT = False

# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# ------------------------------Data Generated Here ---------------------------------
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

N = 30
C = 2

ax = fig.add_subplot(1,2,1,projection='3d')
ax.set_xlim(-8, 8)
ax.set_ylim(-8, 8)
ax.set_zlim(-8, 8)
line, = ax.plot([], [], [], 'ro')

mean = np.array([1,1,1])
zero_mean = np.array([0,0,0])
cov = np.eye(3)
data = np.random.multivariate_normal(zero_mean, cov * 0.5 , (N,))

# for i in range(data.shape[0]):
#     data[i,:] = data[i,:] / np.linalg.norm(data[i,:])

data += mean

noise = np.empty_like(data)
for i in range(noise.shape[0]):
    error = np.random.multivariate_normal(zero_mean, cov * 0.01, (1,))
    noise[i,:] = error

noisy_data = data + noise

# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# ------------------------------Bundle Adjustment Starts Here -----------------------
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

# Create first camera and generate measurements
cam = Camera()
cam.transform = get_translation_xyz(0,-1/2,-4) @ get_rotation_y(0.01)
print(cam.transform)
imgPoints = np.empty(shape=(noisy_data.shape[0],2))
for i in range(noisy_data.shape[0]):
    homo_point = np.array([noisy_data[i,0], noisy_data[i,1], noisy_data[i,2], 1])
    # print(cam.Project(homo_point))
    proj = cam.Project(homo_point)
    if cam.min_x < proj[0] < cam.min_x + cam.width and cam.min_y < proj[1] < cam.min_y + cam.height:
        imgPoints[i,:] = proj

# Create second camera and generate measurements
cam2 = Camera()
cam2.transform = get_translation_xyz(0,1/2,-4) @ get_rotation_y(0.01)
print(cam2.transform)
imgPoints2 = np.empty(shape=(noisy_data.shape[0],2))
for i in range(noisy_data.shape[0]):
    homo_point = np.array([noisy_data[i,0], noisy_data[i,1], noisy_data[i,2], 1])
    # print(cam.Project(homo_point))
    proj = cam2.Project(homo_point)
    if cam2.min_x < proj[0] < cam2.min_x + cam2.width and cam2.min_y < proj[1] < cam2.min_y + cam2.height:
        imgPoints2[i,:] = proj

# Instantiate and call solver methods
P = 6*C + 3*N
solver = NonLinearSolver( P, C, N )

initial = np.zeros(shape=(P,))
initial[:6] = SE3Log(cam.transform)
initial[6:12] = SE3Log(cam2.transform ) + np.random.multivariate_normal(np.zeros((6,)), np.eye(6) * 0.1)
initial[12:P] = np.reshape(noisy_data, (-1,))
measurements = np.vstack([imgPoints, imgPoints2])

solver.Compute(measurements, initial, np.reshape(data, (-1,)))

# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# ------------------------------Data Plotting Here ----------------------------------
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

print("Real Transforms: ", SE3Log(cam2.transform))

ax.plot([cam.transform[0, 3]], [cam.transform[1, 3]], [cam.transform[2, 3]], 'bo', markersize=10)
ax.plot([cam2.transform[0, 3]], [cam2.transform[1, 3]], [cam2.transform[2, 3]], 'go', markersize=10)

# Plot both measurements and 3D points
ax1 = fig.add_subplot(2, 2, 2)
ax1.set_xlim(-1,1)
ax1.set_ylim(-1,1)
# ax1.set((-1,1))
# ax1.ylim((-1,1))
ax1.plot(imgPoints[:,0], imgPoints[:,1], 'bo', markersize=4)

ax2 = fig.add_subplot(2, 2, 4)
ax2.set_xlim(-1,1)
ax2.set_ylim(-1,1)
# ax2.set((-1,1))
# ax2.ylim((-1,1))
ax2.plot(imgPoints2[:,0], imgPoints2[:,1], 'go', markersize=4)


# For Plotting Bundle Adjustment Uncomment This Section Only
x = noisy_data[:,0]
y = noisy_data[:,1]
z = noisy_data[:,2]

ax.plot(x, z, -y, 'ro', markersize=4)
plt.show()



# if False:
#     ax.plot(x, z, -y)
#     plt.show()
#
# else:
#
#
#     def init():
#         if SHOW_3D:
#             line.set_data([], [])
#             line.set_3d_properties([])
#         else:
#             line.set_data([], [])
#         return line,
#
#     def animate(i, line, data):
#         if SHOW_3D:
#             line.set_data(x, y)
#             line.set_3d_properties(z)
#         else:
#             line.set_data(x, y)
#         return line,
#
#     anim = animation.FuncAnimation(fig, animate, init_func=init, fargs=(line, data),
#                                    frames=data.shape[0], interval=10)
#     plt.show()