import numpy as np
from scipy.optimize import minimize
from am import rot_x, rot_y, rot_z
from am import AerialManipulator

base_rot = rot_x(np.pi)
m = np.array([0.03, 0.005, 0.06, 0.03, 0.01])
l = np.array([0.025, 0.025, 0.0578, 0.058, 0.045])
alignments = (np.eye(3), rot_z(np.pi/2), rot_z(np.pi), rot_z(np.pi), rot_z(-np.pi/2))
K = [1.148684645825963357e-01, 1.733173644303289129e-01, 6.413511406290881012e-02, 3.144847601149513422e-02]
A = [0, 0.02, 0.02, 0.015]
tau = 0.75569389

dp = 0.15

mani = AerialManipulator(m, l, base_rot, alignments, K, A)

np.set_printoptions(suppress=True)

f_ext = np.array([0.0, 0.0, 0.0])
xi_nom = mani.find_steady_state(f_ext, tau).x
print(180.0/np.pi * xi_nom)

err = lambda xi: (dp - np.dot(mani.f_pos(xi) - mani.f_pos(xi_nom), [1, 0, 0]))**2

xi_prime = minimize(err, xi_nom).x

print(180.0/np.pi * xi_prime)

f_prime  = mani.find_force(xi_prime, tau)

print(f_prime, np.linalg.norm(f_prime))