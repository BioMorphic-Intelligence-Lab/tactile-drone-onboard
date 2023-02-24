import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from am import AerialManipulator, rot_x, rot_y, rot_z

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

# Define the regions of static equilibrium
equil1 = [(30, 36), (42, 50)]
equilRest = [(82,83), (92, 98)]

base_rot = np.eye(3)
l = np.array([0.025, 0.025, 0.0578, 0.058, 0.045])
m = np.array([0.03, 0.005, 0.06, 0.03, 0.01])
alignments = (np.eye(3), rot_z(np.pi/2), rot_z(np.pi), rot_z(np.pi), rot_z(-np.pi/2))

am = AerialManipulator(m, l, base_rot, alignments)

print("Estimate the stiffness for joint 0")
# This is for the sequence where the first joint was in
# in static equilibrium
q_test1 = data[
     np.abs(data[:, TIME] - equil1[0][0]).argmin():
     np.abs(data[:, TIME] - equil1[0][1]).argmin(), P1:P4+1]

q_test2 = data[
     np.abs(data[:, TIME] - equil1[1][0]).argmin():
     np.abs(data[:, TIME] - equil1[1][1]).argmin(), P1:P4+1]

g_test1 = np.array([am.get_gravity_contribution(q_test1[i, :]) for i in range(len(q_test1))])
g_test2 = np.array([am.get_gravity_contribution(q_test2[i, :]) for i in range(len(q_test2))])


def error_q1(k1):
    return np.dot(np.concatenate((g_test1[:,0], g_test2[:,0])) * np.concatenate((1.0 / q_test1[:,0], 1.0 / q_test2[:,0])) - k1, 
                  np.concatenate((g_test1[:,0], g_test2[:,0])) * np.concatenate((1.0 / q_test1[:,0], 1.0 / q_test2[:,0])) - k1)

def const_q1(k1):
    return k1

res_q1 = minimize(error_q1, 0.07, constraints={'type':'ineq', 'fun': const_q1})
print(res_q1)

print("Estimate the stiffness for joints 1 -> 3")

# This is for the sequence where all the other joints where
# in a static equilibrium
q_test1 = data[
     np.abs(data[:, TIME] - equilRest[0][0]).argmin():
     np.abs(data[:, TIME] - equilRest[0][1]).argmin(), P1:P4+1]

q_test2 = data[
     np.abs(data[:, TIME] - equilRest[1][0]).argmin():
     np.abs(data[:, TIME] - equilRest[1][1]).argmin(), P1:P4+1]

g_test1 = np.array([am.get_gravity_contribution(q_test1[i, :]) for i in range(len(q_test1))])
g_test2 = np.array([am.get_gravity_contribution(q_test2[i, :]) for i in range(len(q_test2))])

plt.plot(data[:, TIME], data[:,1:5])
plt.legend([f"Joint {i+1}" for i in range(5)])
plt.grid()
plt.show()


def error_qrest(k):
    return np.linalg.norm(np.transpose(np.concatenate((g_test1[:,1:], g_test2[:,1:])) * np.concatenate((1.0 / q_test1[:,1:], 1.0 / q_test2[:,1:])) - k)\
                       @ (np.concatenate((g_test1[:,1:], g_test2[:,1:])) * np.concatenate((1.0 / q_test1[:,1:], 1.0 / q_test2[:,1:])) - k))

def const_qrest(k):
    return np.ones_like(k)

res_qrest = minimize(error_qrest, np.array([0.073, 1, 0.026]),constraints={'type':'ineq', 'fun': const_qrest})
print(res_qrest)

print("Saving to file... ")

np.savetxt("stiffness.txt", np.concatenate((res_q1.x, res_qrest.x)))