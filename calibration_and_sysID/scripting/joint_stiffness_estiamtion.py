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

# Define the regions of static equilibrium
equil1 = [(37.4, 39.0), (69.8, 70.6)]
equilRest = [(32.5,34.3), (56.3,56.70)]

g = np.array([0, 0, -9.81])
l = np.array([0.025, 0.0578, 0.058, 0.045])
m = np.array([0.005, 0.06, 0.03, 0.01])

def gravity_contribution(q):
    rs = get_rs(q)
    joint_locs = get_joint_locs(q)
    coms = get_coms(q)

    gravity_contribution = np.zeros_like(q) 

    for i in range(len(q)):
        
        proj = np.array([0, 1, 0])
        if i == 0: proj = np.array([1, 0, 0])

        proj = np.matmul(rs[i], proj)

        for j in range(i,len(q)):
            torque = np.cross((coms[:,j] - joint_locs[:,i]), m[j] * g)
            gravity_contribution[i] = gravity_contribution[i] + np.linalg.norm(np.dot(torque, proj))

    return gravity_contribution

def get_joint_locs(q):
    rs = get_rs(q)
    loc = np.zeros([3, len(q)])

    for i in range(len(q)):
        for j in range(i):
            bar = np.array([0,0, l[j]])
            loc[:, i] = loc[:, i] + np.matmul(rs[j], bar)

        
    return loc

def get_coms(q):
    rs = get_rs(q)
    com = np.zeros([3, len(q)])

    for i in range(len(q)):
        for j in range(i):
            bar = np.array([0,0, l[j]])
            com[:, i] = com[:, i] + np.matmul(rs[j], bar)

        half_bar = np.array([0, 0, 0.5*l[i]])
        com[:, i] = com[:, i] + np.matmul(rs[i], half_bar)
    
    return com

def get_rs(q):
    
    rs = np.array([np.eye(3) for i in range(len(q))])
    rs[0] = rot_x(q[0])

    for i in range(1, len(q)):
        rs[i] = np.matmul(rs[i-1], rot_y(q[i]))

    return rs
    
def rot_x(theta):
    sT = np.sin(theta)
    cT = np.cos(theta)

    return np.array([[1, 0, 0], 
                     [0, cT, -sT],
                     [0, sT, cT]])

def rot_y(theta):
    sT = np.sin(theta)
    cT = np.cos(theta) 

    return np.array([[cT, 0, sT],
                     [0, 1, 0],
                     [-sT, 0, cT]])


# This is for the sequence where the first joint was in
# in static equilibrium
q_test1 = np.average(data[
     np.abs(data[:, TIME] - equil1[0][0]).argmin():
     np.abs(data[:, TIME] - equil1[0][1]).argmin(), P1:P4+1], axis=0)

q_test2 = np.average(data[
     np.abs(data[:, TIME] - equil1[1][0]).argmin():
     np.abs(data[:, TIME] - equil1[1][1]).argmin(), P1:P4+1], axis=0)

g_test1 = gravity_contribution(q_test1)
g_test2 = gravity_contribution(q_test2)

k1_test1 = g_test1[0] / q_test1[0]
k1_test2 = g_test2[0] / q_test2[0]

print(k1_test1)
print(k1_test2)

# This is for the sequence where all the other joints where
# in a static equilibrium
q_test1 = np.average(data[
     np.abs(data[:, TIME] - equilRest[0][0]).argmin():
     np.abs(data[:, TIME] - equilRest[0][1]).argmin(), P1:P4+1], axis=0)

q_test2 = np.average(data[
     np.abs(data[:, TIME] - equilRest[1][0]).argmin():
     np.abs(data[:, TIME] - equilRest[1][1]).argmin(), P1:P4+1], axis=0)

g_test1 = gravity_contribution(q_test1)
g_test2 = gravity_contribution(q_test2)

k1_test1 = g_test1[1:] / q_test1[1:]
k1_test2 = g_test2[1:] / q_test2[1:]

print(k1_test1)
print(k1_test2)

# TODO Proper statistics and export to file