import numpy as np
import matplotlib.pyplot as plt

# Define Indeces
TIME = 0
P1 = 1
P2 = 2
P3 = 3
P4 = 4
V1 = 5
V2 = 6
V3 = 7
V4 = 8

# Load Data
data = np.loadtxt("joint_stiffness_config.txt", skiprows=2)


# Convert Timestamps to seconds and shift to zeros
data[:,TIME] = (data[:,TIME] - data[0, TIME]) * 1e-9

fig, ax  = plt.subplots()

ax.plot(data[:,TIME], data[:,P1:P4+1]*180.0/np.pi)
ax.legend(["Joint 1", "Joint 2", "Joint 3", "Joint 4"])
ax.grid()
ax.set_xlabel("Time[s]")
ax.set_ylabel("Angle [degree]")
plt.show()