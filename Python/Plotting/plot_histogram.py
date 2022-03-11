import matplotlib.pyplot as plt

plt.rcParams.update({'font.size': 16})

# Build PDF and turn into pandas Series
indices = [0, 1, 2, 3, 4, 5, 6, 7]
values = [18287, 41835, 5200, 207, 6, 1, 0, 0]

# Plot previously histogrammed data
fig = plt.figure()
plt.bar(indices, values)
plt.show()
# ax.legend(['PDF', 'Point Hash'])
