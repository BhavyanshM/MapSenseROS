import matplotlib.pyplot as plt
import numpy as np

plt.figure(figsize=(20,20))

data = np.loadtxt("data2D.txt", delimiter=',', dtype=np.float64)

x,y = data[:,1], -data[:,0]

plt.plot(x, y, 'go')

for i in range(data.shape[0]):
    plt.text(x[i]+0.1 , y[i]+0.1 , str(i))

plt.show()
