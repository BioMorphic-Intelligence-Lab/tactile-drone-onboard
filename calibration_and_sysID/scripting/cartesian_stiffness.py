import numpy as np

from am import AerialManipulator
from am import rot_x, rot_z

base_rot = rot_x(np.pi)
m = np.array([0.03, 0.005, 0.06, 0.03, 0.01])
l = np.array([0.025, 0.025, 0.0578, 0.058, 0.045])
alignments = (np.eye(3), rot_z(np.pi/2), rot_z(np.pi), rot_z(np.pi), rot_z(-np.pi/2))
K = [1.148684645825963357e-01, 1.733173644303289129e-01, 6.413511406290881012e-02, 3.144847601149513422e-02]
A = [0, 0.02, 0.02, 0.015]
tau = 0.75569389

mani = AerialManipulator(m, l, base_rot, alignments, K, A)

np.set_printoptions(suppress=True, precision=3)

increment = np.array([0.0, -0.1, 0.1, -0.1]) * np.pi/4
xi = 0.0001 * np.ones(4)
K_joints = np.diag(K)

for i in range(10):

    J = mani.get_jacobian(xi)
    Kc = np.linalg.inv(J @ np.linalg.inv(K_joints) @ J.T)

    print("######################")
    print(f"xi = {180.0 / np.pi * xi}")
    print("K_c = ")
    print(f"{Kc}")
    print("######################")

    xi += increment