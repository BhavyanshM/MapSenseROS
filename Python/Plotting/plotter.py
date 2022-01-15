import numpy as np
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 10})

plt.figure(figsize=(10,8))

data = np.loadtxt("data2D.txt", delimiter=' ', dtype=np.float64)


x,y = data[:,0], data[:,1]


# print("Min", np.min(x), "Max", np.max(x), "Mean", np.mean(x))
# print("Min", np.min(y), "Max", np.max(y), "Mean", np.mean(y))



plt.plot(x, y, 'go', markersize=4)

for i in range(data.shape[0]):
    plt.text(x[i]+ 0.01 , y[i]+0.01 , str(i))

plt.show()
