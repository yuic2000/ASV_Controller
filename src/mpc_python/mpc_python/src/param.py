import numpy as np


class Param:
    def __init__(self):
        self.dt = 0.1                               # control frequency, 10 Hz
        self.Nsim = int(np.ceil(65 / self.dt))      # Total simulation time + Planning time
        self.mode = 1                               # mode 1: zigzag, mode 2: circling
        self.V_c = 0.0                              # Ocean current velocity
        self.beta_c = 1.0                           # Ocean current direction

        self.Nx = 12                                # dimension x   
        self.Nu = 2                                 # dimension of u
        self.Nt = 100                               # horizon length 

        # state constraints
        self.xmax = np.inf * np.ones(self.Nx)
        self.xmin = -self.xmax
        # control constraints
        self.umax = np.ones(self.Nu) * 115
        self.umin = -np.ones(self.Nu) * 65

        # Mass matrix
        self.I = np.array([
            [17.1572, 0, 4.4, 0, 11, 0],
            [0, 37.07, 0, -11, 0, -11],
            [4.4, 0, 45.265, 0, 11, 0],
            [0, -11, 0, 60.2815, 0, 0],
            [11, 0, 11, 0, 137.5, 0],
            [0, -11, 0, 0, 0, 110]
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


class Logger:
    def __init__(self, Nsim):
        self.X = np.zeros((13, Nsim - 11))
        self.Err = np.zeros((6, Nsim - 12))
        self.U = np.zeros((6, Nsim - 12))
        self.Err2 = np.zeros((4, Nsim - 12))
        self.taken_time = 0