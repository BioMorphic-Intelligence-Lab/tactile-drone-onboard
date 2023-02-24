import numpy as np

def rot_x(theta):
    cT = np.cos(theta)
    sT = np.sin(theta)
    return np.array([[1,  0,   0],
                     [0, cT, -sT],
                     [0, sT, cT]])

def rot_y(theta):
    cT = np.cos(theta)
    sT = np.sin(theta)
    return np.array([[cT,  0,  sT],
                     [  0, 1,   0],
                     [-sT, 0,  cT]])

def rot_z(theta):
    cT = np.cos(theta)
    sT = np.sin(theta)
    return np.array([[cT, -sT,  0],
                     [sT,  cT,  0],
                     [ 0,   0,  1]]) 

class AerialManipulator(object):
    def __init__(self, m, l, base_rot, alignments) -> None:
        self.m = np.array(m)
        self.l = np.array(l)
        self.base_rot = base_rot
        self.alignments = alignments
    

    def f(self, q) -> tuple[np.array, np.array]:
        bar = np.array([0,0,1])
        rot = (self.alignments[0] 
            @ rot_x(q[0]) @ self.alignments[1] 
            @ rot_x(q[1]) @ self.alignments[2]
            @ rot_x(q[2]) @ self.alignments[3]
            @ rot_x(q[3]) @ self.alignments[4]
            )

        t = (self.l[0] * bar 
        + self.l[1] * self.alignments[0] @ rot_x(q[0]) @ bar
        + self.l[2] * self.alignments[0] @ rot_x(q[0]) @ self.alignments[1] @ rot_x(q[1]) @ bar
        + self.l[3] * self.alignments[0] @ rot_x(q[0]) @ self.alignments[1] @ rot_x(q[1]) @ self.alignments[2] @  rot_x(q[2]) @ bar
        + self.l[4] * self.alignments[0] @ rot_x(q[0]) @ self.alignments[1] @ rot_x(q[1]) @ self.alignments[2] @  rot_x(q[2]) @ self.alignments[3] @ rot_x(q[3]) @ bar
        )
        return rot, t

    def get_joints(self, q) -> tuple[np.array, np.array]:
        bar = np.array([0,0,1])
        joints = np.zeros([3, 4])
        rot = self.base_rot @ self.alignments[0]
        rots = (rot, )
        joints[:, 0] = self.l[0] * rot @ bar
        for i in range(1, 4):
            rot = rots[-1] @ rot_x(q[i-1])
            joints[:, i] = joints[:, i-1] + self.l[i] * rot @ bar
            rots = rots + (rot @ self.alignments[i],)
        return joints, rots

    def get_points(self, q):
        bar = np.array([0,0,1])
        rot = self.base_rot
        points = np.zeros([3, 6])

        for i in range(1, 5):        
            points[:, i] = points[:, i-1] + self.l[i-1] * rot @ bar
            rot = rot @ self.alignments[i-1] @ rot_x(q[i-1])
        
        points[:, -1] = points[:, 4] + self.l[-1] * rot @ bar
        
        return points

    def get_coms(self, q):
        bar = np.array([0,0,1])
        rot = self.base_rot
        coms = np.zeros([3, 4])
        points = self.get_points(q)

        for i in range(1, 5):        
            rot = rot @ self.alignments[i-1] @ rot_x(q[i-1])
            coms[:, i-1] = points[:, i] + 0.5 * self.l[i] * rot @ bar
        
        return coms

    def get_jacobian(self, q):
        c0 = np.cos(q[0])
        c1 = np.cos(q[1])
        s0 = np.sin(q[0])
        s1 = np.sin(q[1])

        c12 = np.cos(q[1] - q[2])
        s12 = np.sin(q[1] - q[2])
        c123 = np.cos(q[1] - q[2] + q[3])
        s123 = np.sin(q[1] - q[2] + q[3])


        return np.array([[0, self.l[2]*c1+self.l[3]*c12+self.l[4]*c123, -self.l[3]*c12-self.l[4]*c123, self.l[4]*c123],
                        [-c0*(self.l[1]+self.l[2]*c1+self.l[3]*c12+self.l[4]*c123), s0*(self.l[2]*s1+self.l[3]*s12+self.l[4]*s123), -s0*(self.l[3]*s12+self.l[4]*s123), self.l[4]*s0*s123],
                        [-((self.l[1]+self.l[2]*c1+self.l[3]*c12+self.l[4]*c123)*s0), -c0*(self.l[2]*s1+self.l[3]*s12+self.l[4]*s123), c0*(self.l[3]*s12+self.l[4]*s123), -self.l[4]*c0*s123]
                    ])
    
    def get_gravity_contribution(self, q):
        g = np.array([0, 0, -9.81])
        joints, rots = self.get_joints(q)
        coms = self.get_coms(q) 

        G = np.zeros_like(q)
        for i in range(len(q)):
            proj = rots[i] @ np.array([1, 0, 0])
            for j in range(i,len(q)):
                torque = np.cross((coms[:,j] -joints[:,i]), self.m[j] * g)
                G[i] = G[i] + np.dot(torque, proj)
        
        return G