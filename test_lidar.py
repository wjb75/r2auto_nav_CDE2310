import numpy as np

# Load the lidar range data from the file
laser_range = np.loadtxt('lidar.txt')

# Replace zeros with np.nan
laser_range[laser_range == 0] = np.nan

# Find the index of the maximum value ignoring nan
lr2i = np.nanargmax(laser_range)
print("Index with max value:", lr2i)

# Print the maximum value
print("Maximum value:", laser_range[lr2i])
