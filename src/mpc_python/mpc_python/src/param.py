import numpy as np
from mpc_python.src.utils.LieGroup import skew
from scipy.linalg import expm
from scipy.spatial.transform import Rotation


class Param:
    def __init__(self):
        self.dt = 0.04                              # control frequency, 25 Hz, original 20Hz
        self.Nsim = int(np.ceil(64 / self.dt))      # Total simulation time + Planning time
        self.mode = 2                               # mode 1: zigzag, mode 2: circling
        self.V_c = 0.5                              # Ocean current velocity, 0.5
        self.beta_c = 0.0                           # Ocean current direction, 1

        self.Nx = 12                                # dimension x   
        self.Nu = 2                                 # dimension of u
        self.Nt = 100                               # horizon length

        # state constraints
        self.xmax = np.inf * np.ones(self.Nx)
        self.xmin = -self.xmax
        
        # control constraints, for Otter
        # self.umax = np.ones(self.Nu) * 115
        # self.umin = -np.ones(self.Nu) * 65

        # control constraints, for blueboat
        self.umax = np.ones(self.Nu) * 40           # 3.5 kgF
        self.umin = -np.ones(self.Nu) * 30          # -3.0 kgF

        # Mass matrix, from Lie Group MPC paper, original
        # self.I = np.array([
        #     [17.1572,   0,     4.4,     0,     11,     0],
        #     [0,       37.07,    0,     -11,     0,   -11],
        #     [4.4,       0,    45.265,   0,     11,     0],
        #     [0,        -11,     0,    60.2815,  0,     0],
        #     [11,        0,     11,      0,     137.5,  0],
        #     [0,        -11,     0,      0,      0,   110]
        # ])
        # Mass matrix, from Blueboat CAD model
        self.I = np.array([
            [2.2759,   -0.6860,  -0.1751,   0,      3.53,     0],
            [-0.6860,   6.5509,  -0.0234, -3.53,     0,   -3.53],
            [-0.1751,  -0.0234,  10.5172,   0,      3.53,     0],
            [0,         -3.53,      0,    18.4444,   0,       0],
            [3.53,        0,      3.53,     0,     44.125,    0],
            [0,         -3.53,      0,      0,       0,    35.3]
        ])

        # MPC cost matrices
        self.P = np.diag([1e-5, 1e-5, 0.5, 10, 10, 1e-5, 1e-5, 1e-5, 1e-5, 1, 1e-5, 1e-5]) * 10
        self.Q = np.diag([1e-5, 1e-5, 0.5, 10, 10, 1e-5, 1e-5, 1e-5, 1e-5, 1, 1e-5, 1e-5]) * 1
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
                xid_ref_rt[2] = 0.1 * np.cos(i/100)      # 100: 20 Hz, 50: 10 Hz
            
            Xi = np.zeros((4, 4))
            Xi[:3, :3] = skew(xid_ref_rt[:3])
            Xi[:3, 3] = xid_ref_rt[3:]
            
            X = X @ expm(Xi * self.dt)
            new_quat = Rotation.from_matrix(X[:3, :3]).as_quat()

            self.X_ref = np.vstack([self.X_ref, np.concatenate([new_quat, X[:3, 3]])])

            self.xi_ref = np.vstack([self.xi_ref, xid_ref_rt])
        
        # self.X_ref[:, 5] += 2   # offset in y direction, use when using ground truth odometry
        



class Logger:
    def __init__(self, Nsim, Nt):
        self.X = np.zeros((13, Nsim - Nt))
        self.Err = np.zeros((6, Nsim - Nt))
        self.U = np.zeros((2, Nsim - Nt))
        self.Err2 = np.zeros((4, Nsim - Nt))
        self.taken_time = 0