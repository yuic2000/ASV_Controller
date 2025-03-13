import numpy as np
from mpc_python.src.utils.LieGroup import skew
from scipy.linalg import expm
from scipy.spatial.transform import Rotation

import rclpy
from mpc_python.communication.gtodom_sub import GtOdomSubscriberNode


class Param:
    def __init__(self):
        self.dt = 0.04                              # control frequency, 25 Hz, original 20Hz
        self.Nsim = int(np.ceil(64 / self.dt))      # Total simulation time + Planning time

        self.mode = 1                               # mode 1: zigzag, mode 2: circling
        self.V_c = 0.0                              # Ocean current velocity, 0.5
        self.beta_c = 0.0                           # Ocean current direction, 1

        self.Nx = 12                                # dimension x   
        self.Nu = 2                                 # dimension of u
        self.Nt = 100                               # horizon length

        # state constraints
        self.xmax = np.inf * np.ones(self.Nx)
        self.xmin = -self.xmax

        # control constraints, for blueboat
        self.umax = np.ones(self.Nu) * 50           # 3.5 kgF
        self.umin = -np.ones(self.Nu) * 26          # -3.0 kgF

        # Mass matrix, from Blueboat CAD model
        self.I = np.array([
            [14.47,  -0.35,  6.55,    0,   17.66,   0.25],
            [-0.35,  21.55,  0.13, -17.66,   0,   -10.79],
            [6.55,   0.13, 12.33,  -0.25, 10.79,      0],
            [0,    -17.66, -0.25,  34.31,   0,        0],
            [17.66,   0,   10.79,    0,   71.58,      0],
            [0.25, -10.79,    0,     0,     0,    57.26]
        ])

        # MPC cost matrices
        # [0:3] is angle: roll, pitch, yaw ,
        # [3:6] is position: x, y, z
        # [6:9] is angular velocity: roll, pitch, yaw,
        # [9:12] is linear velocity: surge (Vx), sway (Vy), heave (Vz),
        self.P = np.diag([1e-5, 1e-5, 0.5, 10, 10, 1e-5, 1e-5, 1e-5, 1e-5, 1, 1e-5, 1e-5]) * 10
        self.Q = np.diag([1e-5, 1e-5, 0.5, 10, 10, 1e-5, 1e-5, 1e-5, 1e-5, 1, 1e-5, 1e-5]) * 1
        self.R = np.eye(2) * 1e-5


    def set_desired_posistion(self, x_init):
        # self.q0_ref = x_init[0:4]
        self.q0_ref = np.array([1, 0, 0, 0])            # desired quaternion
        self.p0_ref = x_init[4:7]
        self.w0_ref = np.array([0, 0, 0.1])             # desired yaw velocity 0.1 rad/s
        self.v0_ref = np.array([0.5, 0, 0])             # desired surge velocity 0.5 m/s

        self.x0_ref = np.concatenate([self.q0_ref, self.p0_ref])
        self.xid_ref = np.concatenate([self.w0_ref, self.v0_ref])
        # reference trajectory
        self.X_ref = self.x0_ref.reshape(1, -1)
        self.xi_ref = self.xid_ref.reshape(1, -1)

        self.init_pose = np.eye(4)
        # self.init_pose[:3, :3] = Rotation.from_quat(self.q0_ref).as_matrix()
        self.init_pose[:3, 3] = self.p0_ref


    def get_ref_trajectory(self):   
        # X = np.eye(4)
        X = self.init_pose.copy()

        for i in range(self.Nsim):
            xid_ref_rt = self.xid_ref.copy()
            
            if self.mode == 1:  # zigzag maneuver
                xid_ref_rt[2] = 0.1 * np.cos(i/100)
            
            Xi = np.zeros((4, 4))
            Xi[:3, :3] = skew(xid_ref_rt[:3])
            Xi[:3, 3] = xid_ref_rt[3:]
            
            X = X @ expm(Xi * self.dt)
            new_quat = Rotation.from_matrix(X[:3, :3]).as_quat()

            self.X_ref = np.vstack([self.X_ref, np.concatenate([new_quat, X[:3, 3]])])

            self.xi_ref = np.vstack([self.xi_ref, xid_ref_rt])
        
        self.X_ref[:, 5] += 2   # offset in y direction, use when using vrx ground truth odometry


class Logger:
    def __init__(self, Nsim, Nt):
        self.X = np.zeros((13, Nsim - Nt))
        self.Err = np.zeros((6, Nsim - Nt))
        self.U = np.zeros((2, Nsim - Nt))
        self.Err2 = np.zeros((4, Nsim - Nt))
        self.taken_time = 0