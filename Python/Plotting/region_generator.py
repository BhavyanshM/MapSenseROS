import numpy as np
import matplotlib.pyplot as plt


def generate_circular_region(offset, size, noise_scale):
    t = np.linspace(0, 2 * np.pi, 50)
    x = size[0] * np.sin(t) + offset[0]
    y = size[1] * np.cos(t) + offset[1]

    x_d = np.random.random(size=(x.shape[0],))
    y_d = np.random.random(size=(x.shape[0],))

    x += x_d * noise_scale
    y += y_d * noise_scale

    return x,y

def plot_region(region):
    plt.plot(region[0], region[1], 'o')
    plt.plot(region[0], region[1], '-')

def save_region(region, file):
    file.write("RegionID:-1\n")
    file.write("Center:0.0,0.0,0.0\n")
    file.write("Normal:0.0,0.0,1.0\n")
    file.write("NumPatches:{}\n".format(region[0].shape[0]))
    for i in range(region[0].shape[0]):
        file.write("{},{},{}\n".format(region[0][i], region[1][i], 0))

plt.figure(figsize=(12,12))
plt.xlim(-8,8)
plt.ylim(-8,8)

regions = []
regions.append(generate_circular_region(offset=(-2,1), size=(1,3), noise_scale=0.5))
regions.append(generate_circular_region(offset=(1,-1), size=(4,2), noise_scale=0.3))
regions.append(generate_circular_region(offset=(3,-3), size=(2.4,3), noise_scale=0.2))

for region in regions:
    plot_region(region)

f = open("../../../Extras/Regions/Tests/Test_Set_01/0011.txt", 'w+')
f.write("NumRegions:{}\n".format(len(regions)))
for region in regions:
    save_region(region, f)

plt.show()

