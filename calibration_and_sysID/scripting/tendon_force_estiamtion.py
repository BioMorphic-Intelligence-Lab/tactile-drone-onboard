import numpy as np
import matplotlib.pyplot as plt
import scipy as sp

from am import AerialManipulator
from am import rot_x,rot_y, rot_z

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
data = np.loadtxt("tendon_force_config.txt", skiprows=2)

# Load Stiffness Matrix
K = np.diag(np.loadtxt("stiffness.txt"))
A = np.array([0, 0.02, 0.02, 0.015])

# Convert Timestamps to seconds and shift to zeros
data[:,TIME] = (data[:,TIME] - data[0, TIME]) * 1e-9

# Trim the date to the steady state
index = np.abs(data[:, TIME] - 32.5).argmin()

base_rot = rot_z(-np.pi/2) @ rot_x(np.pi)
l = np.array([0.025, 0.025, 0.0578, 0.058, 0.045])
m = np.array([0.03, 0.005, 0.06, 0.03, 0.01])
alignments = (np.eye(3), rot_z(np.pi/2), rot_z(np.pi), rot_z(np.pi), rot_z(-np.pi/2))

am = AerialManipulator(m, l, base_rot, alignments)

q = np.average(data[index:,P1:P4+1], axis=0)
G = am.get_gravity_contribution(q)

def error(tau):
   return np.dot(G + np.matmul(K,q) + A * tau, G + np.matmul(K,q) + A * tau)

# Finding the force that minimizes the square of the 2-norm of the error function
res = sp.optimize.minimize(error, 2)

print(res) # This is the force
