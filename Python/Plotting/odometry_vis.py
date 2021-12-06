from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
import random
import numpy as np


fig = plt.figure(figsize=(10,8))

SHOW_3D = True
GT = False

if SHOW_3D:
    ax = Axes3D(fig)
    line, = ax.plot([], [], [], lw=2)
else:
    ax = plt.axes(xlim=(-800, 800), ylim=(-800, 400))
    line, = ax.plot([], [], lw=5)



if GT:
    data = np.loadtxt("/home/quantum/Workspace/Storage/Other/Temp/dataset/data_odometry_poses/poses/00.txt", delimiter=' ', dtype=np.float64)
else:
    data = np.loadtxt("odometry.txt", delimiter=' ', dtype=np.float64)

# data = data[:120, :]

print(data.shape)

trajectory = np.empty(shape=(data.shape[0], 3))

prev_vel_mag = 1
prev_vel = np.array([0,0,1])
final_pose = np.eye(4,4)
last_good_pose = np.eye(4)
last_good_pose[:3,:] = np.reshape(data[0,:], (3,4))

old_pose_count = 0
pose_count = 0

for i in range(data.shape[0]):

    pose = np.eye(4,4)
    pose[:3,:] = np.reshape(data[i,:], (3,4))

    if GT:
        trajectory[i, :3] = pose[:3,3].T

    else:
        final_pose = final_pose @ pose
        trajectory[i, :3] = final_pose[:3,3].T

# trajectory[:,1] = 0

x = trajectory[:,0]
y = trajectory[:,1]
z = trajectory[:,2]

# ax.axes.set_xlim3d(-4.0, 4.0)
# ax.axes.set_ylim3d(-4.0, 4.0)
# ax.axes.set_zlim3d(-4.0, 4.0)

if False:
    ax.plot(x, z, -y, lw=2)
    plt.show()

else:


    def init():
        if SHOW_3D:
            line.set_data([], [])
            line.set_3d_properties([])
        else:
            line.set_data([], [])
        return line,

    def animate(i, line, data):
        if SHOW_3D:
            line.set_data(trajectory[:i,0], -trajectory[:i,2])
            line.set_3d_properties(-trajectory[:i,1])
        else:
            line.set_data(trajectory[:i,0], -trajectory[:i,2])
        return line,

    anim = animation.FuncAnimation(fig, animate, init_func=init, fargs=(line, trajectory),
                                   frames=trajectory.shape[0], interval=10)
    plt.show()