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
    line, = ax.plot([], [], [], 'b', lw=2)
    gt_line, = ax.plot([], [], [], 'r', lw=2)
else:
    ax = plt.axes(xlim=(-800, 800), ylim=(-800, 400))
    line, = ax.plot([], [], lw=5)



if GT:
    gt_data = np.loadtxt("/home/quantum/Workspace/Storage/Other/Temp/dataset/data_odometry_poses/poses/00.txt", delimiter=' ', dtype=np.float64)
data = np.loadtxt("odometry_zed.txt", delimiter=' ', dtype=np.float64)

# data = data[:120, :]

print(data.shape)

if GT:
    gt_trajectory = np.empty(shape=(gt_data.shape[0], 3))
    final_gt_pose = np.eye(4,4)

trajectory = np.empty(shape=(data.shape[0], 3))
final_pose = np.eye(4,4)

old_pose_count = 0
pose_count = 0

for i in range(data.shape[0]):

    pose = np.eye(4,4)
    pose[:3,:] = np.reshape(data[i,:], (3,4))

    final_pose = final_pose @ pose
    trajectory[i, :3] = final_pose[:3,3].T

    if GT:
        gt_pose = np.eye(4,4)
        gt_pose[:3,:] = np.reshape(gt_data[i,:], (3,4))

        final_gt_pose = final_gt_pose @ gt_pose
        gt_trajectory[i, :3] = gt_pose[:3,3].T

# trajectory[:,1] = 0

x = trajectory[:,0]
y = trajectory[:,1]
z = trajectory[:,2]

ax.axes.set_xlim3d(-400, 400)
ax.axes.set_ylim3d(-400, 400)
ax.axes.set_zlim3d(-400, 400)

if False:
    ax.plot(x, z, -y, lw=2)
    plt.show()

else:


    def init():
        if SHOW_3D:
            line.set_data([], [])
            line.set_3d_properties([])
            if GT:
                gt_line.set_data([], [])
                gt_line.set_3d_properties([])
        else:
            line.set_data([], [])
        return line,

    def animate(i, line, gt_line, data, gt_data):
        if SHOW_3D:
            line.set_data(trajectory[:i,0], -trajectory[:i,2])
            line.set_3d_properties(-trajectory[:i,1])
            if GT:
                gt_line.set_data(gt_trajectory[:i,0], -gt_trajectory[:i,2])
                gt_line.set_3d_properties(-gt_trajectory[:i,1])
        else:
            line.set_data(trajectory[:i,0], -trajectory[:i,2])

        if GT:
            return line, gt_line
        else:
            return line,

    if GT:
        anim = animation.FuncAnimation(fig, animate, init_func=init, fargs=(line, gt_line, trajectory, gt_trajectory),
                                   frames=trajectory.shape[0], interval=1)

    else:
        anim = animation.FuncAnimation(fig, animate, init_func=init, fargs=(line, None, trajectory, None),
                                   frames=trajectory.shape[0], interval=1)

plt.show()