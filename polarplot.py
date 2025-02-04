import numpy as np
import matplotlib.pyplot as plt

laser_range = np.loadtxt('lidar.txt')
plt.figure()
angles = np.arange(0, 360, 360/len(laser_range))
plt.polar(angles/180*np.pi, laser_range)
plt.show()
