import numpy as np
quat = np.loadtxt('odom.txt', skiprows=13, delimiter=':',usecols=1, max_rows=4)
print(quat)