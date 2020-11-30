import matplotlib.pyplot as plt
import numpy as np

plt.figure(figsize=(10,8))

data = np.loadtxt("plotter_data.csv", delimiter=',', dtype=np.float64)

plt.plot(data[:,0], data[:,1], 'go')

for i in range(data.shape[0]):
    plt.text(data[i][0] + 0.1, data[i][1] + 0.1, str(i))

plt.show()
