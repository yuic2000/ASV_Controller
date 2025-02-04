import numpy as np
from mpc_python.src.utils.LieGroup import skew
from scipy.linalg import expm
from scipy.spatial.transform import Rotation


class Param:
    def __init__(self):
        self.dt = 0.1                               # control frequency, 10 Hz, original 0.05
        self.Nsim = int(np.ceil(65 / self.dt))      # Total simulation time + Planning time
        self.mode = 1                               # mode 1: zigzag, mode 2: circling
        self.V_c = 0.0                              # Ocean current velocity
        self.beta_c = 0.0                           # Ocean current direction

        self.Nx = 12                                # dimension x   
        self.Nu = 2                                 # dimension of u
        self.Nt = 100                               # horizon length, original 100

        # state constraints
        self.xmax = np.inf * np.ones(self.Nx)
        self.xmin = -self.xmax
        # control constraints
        # self.umax = np.ones(self.Nu) * 115
        # self.umin = -np.ones(self.Nu) * 65
        self.umax = np.ones(self.Nu) * 150
        self.umin = -np.ones(self.Nu) * 100

        # Mass matrix, from Lie Group MPC paper, original
        self.I = np.array([
            [17.1572, 0, 4.4, 0, 11, 0],
            [0, 37.07, 0, -11, 0, -11],
            [4.4, 0, 45.265, 0, 11, 0],
            [0, -11, 0, 60.2815, 0, 0],
            [11, 0, 11, 0, 137.5, 0],
            [0, -11, 0, 0, 0, 110]
        ])
        # self.I = np.array([                     # from vrx simulation, added by yui Feb 1 2025
        #     [250.18,   0,       0,      0,      -50.0,         0],
        #     [0,     250.18,     0,     50.0,      0,        50.0],
        #     [0,        0,     250.18,   0,      -50.0,         0],
        #     [0,       50.0,     0,   225.1712,  0.12162, 0.99187],
        #     [-50.0,    0,     -50.0,  0.12162,  454.141, 0.29216],
        #     [0,       50.0,     0,    0.99187,  0.29216,499.7524]
        # ])
        # self.I = np.array([                   # from Otter.m, added by yui Feb 1 2025
        #     [55.0, 0, 0, 0, -11.0, 0],
        #     [0, 55.0, 0, 11.0, 0, 11.0],
        #     [0, 0, 55.0, 0, -11.0, 0],
        #     [0, 11.0, 0, 14.6643, 0, 4.4],
        #     [-11.0, 0, -11.0, 0, 22.55, 0],
        #     [0, 11.0, 0, 4.4, 0, 18.15]
        # ])


        # MPC cost matrices
        # self.P = np.diag([1e-5, 1e-5, 0.5, 10, 10, 1e-5, 1e-5, 1e-5, 1e-5, 1, 1e-5, 1e-5]) * 10
        # self.Q = np.diag([1e-5, 1e-5, 0.5, 10, 10, 1e-5, 1e-5, 1e-5, 1e-5, 1, 1e-5, 1e-5]) * 1
        self.P = np.diag([1e-5, 1e-5, 1, 10, 10, 1e-5, 1e-5, 1e-5, 1e-2, 1, 1e-5, 1e-5]) * 10
        self.Q = np.diag([1e-5, 1e-5, 1, 10, 10, 1e-5, 1e-5, 1e-5, 1e-2, 1, 1e-5, 1e-5]) * 1
        self.R = np.eye(2) * 1e-5

        # initial states
        self.q0_ref = np.array([1, 0, 0, 0])            # desired quaternion
        self.p0_ref = np.array([0, 0, 0])               # desired position
        self.w0_ref = np.array([0, 0, 0.1])             # desired yaw velocity 0.1 rad/s
        self.v0_ref = np.array([0.5, 0, 0])             # desired surge velocity 0.5 m/s
        self.x0_ref = np.concatenate([self.q0_ref, self.p0_ref])
        self.xid_ref = np.concatenate([self.w0_ref, self.v0_ref])
        # reference trajectory
        self.X_ref = self.x0_ref.reshape(1, -1)
        self.xi_ref = self.xid_ref.reshape(1, -1)

    def get_ref_trajectory(self):
        X = np.eye(4)
        for i in range(self.Nsim):
            xid_ref_rt = self.xid_ref.copy()
            
            if self.mode == 1:  # zigzag maneuver
                xid_ref_rt[2] = -0.1 * np.cos(i/50)      # 100: 20 Hz, 50: 10 Hz
            
            Xi = np.zeros((4, 4))
            Xi[:3, :3] = skew(xid_ref_rt[:3])
            Xi[:3, 3] = xid_ref_rt[3:]
            
            X = X @ expm(Xi * self.dt)
            new_quat = Rotation.from_matrix(X[:3, :3]).as_quat()

            self.X_ref = np.vstack([self.X_ref, np.concatenate([new_quat, X[:3, 3]])])
            self.xi_ref = np.vstack([self.xi_ref, xid_ref_rt])


class Logger:
    def __init__(self, Nsim):
        self.X = np.zeros((13, Nsim - 11))
        self.Err = np.zeros((6, Nsim - 12))
        self.U = np.zeros((6, Nsim - 12))
        self.Err2 = np.zeros((4, Nsim - 12))
        self.taken_time = 0