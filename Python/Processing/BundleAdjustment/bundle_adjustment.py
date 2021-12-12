from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
import random
import numpy as np
from NonLinearSolver import *
from Camera import *
np.set_printoptions(precision=2)
fig = plt.figure(figsize=(24,9))



SHOW_3D = True
GT = False

ax = fig.add_subplot(1,2,1,projection='3d')
ax.set_xlim(-8, 8)
ax.set_ylim(-8, 8)
ax.set_zlim(-8, 8)
line, = ax.plot([], [], [], 'ro')

mean = np.array([0,0,0])
zero_mean = np.array([0,0,0])
cov = np.eye(3)
data = np.random.multivariate_normal(mean, cov * 0.5 , (50,))

# for i in range(data.shape[0]):
#     data[i,:] = data[i,:] / np.linalg.norm(data[i,:])

noise = np.empty_like(data)
for i in range(noise.shape[0]):
    error = np.random.multivariate_normal(zero_mean, cov * 0.0000001, (1,))
    noise[i,:] = error

data = data + noise

cam = Camera()
imgPoints = np.empty(shape=(data.shape[0],2))
for i in range(data.shape[0]):
    homo_point = np.array([data[i,0], data[i,1], data[i,2], 1])
    # print(cam.Project(homo_point))
    proj = cam.Project(homo_point)
    if 0 < proj[0] < cam.width and 0 < proj[1] < cam.height:
        imgPoints[i,:] = proj

cam2 = Camera()
imgPoints2 = np.empty(shape=(data.shape[0],2))
for i in range(data.shape[0]):
    homo_point = np.array([data[i,0], data[i,1], data[i,2], 1])
    # print(cam.Project(homo_point))
    proj = cam.Project(homo_point)
    if 0 < proj[0] < cam.width and 0 < proj[1] < cam.height:
        imgPoints2[i,:] = proj

solver = NonLinearSolver(162)
# plt.spy(A)
# plt.show()

initial = np.zeros(shape=(162,))
initial[:6] = SE3Log(cam.transform)
initial[6:12] = SE3Log(cam2.transform)
initial[12:162] = np.reshape(data, (-1,))

measurements = np.vstack([imgPoints, imgPoints2])
solver.Compute(measurements, initial)


ax1 = fig.add_subplot(1, 2, 2)
ax1.set_xlim(0,800)
ax1.set_ylim(0,600)
# ax1.set((0,800))
# ax1.ylim((0,600))
ax1.plot(imgPoints[:,0], imgPoints[:,1], 'bo', markersize=4)


# For Plotting Bundle Adjustment Uncomment This Section Only
# x = data[:,0]
# y = data[:,1]
# z = data[:,2]
#
# ax.plot(x, z, -y, 'ro', markersize=4)
# plt.show()



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