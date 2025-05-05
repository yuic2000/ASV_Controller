import numpy as np
from mpc_python.src.utils.LieGroup import skew, pose_to_T_matrix
from scipy.linalg import expm, logm
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

        # control constraints, for vrx wamv
        # self.umax = np.ones(self.Nu) * 200
        # self.umin = -np.ones(self.Nu) * 125
        # control constraints, for blueboat
        self.umax = np.ones(self.Nu) * 50           # 3.5 kgF
        self.umin = -np.ones(self.Nu) * 26          # -3.0 kgF

        # Mass matrix, vrx wamv
        # self.I = np.array([      # blueboat rg
        #     [365.47,  -0.70,  59.17,      0,    154.36,   2.18],
        #     [-0.70,  948.23,  1.64,    -154.36,   0,    -94.29],
        #     [59.17,    1.64,  1384.89,  -2.18,   94.29,      0],
        #     [  0,    -154.36, -2.18,    260.73,   0,         0],
        #     [154.36,    0,    94.29,      0,    625.45,      0],
        #     [ 2.18,  -94.29,    0,        0,      0,    500.36]
        # ])
        # self.I = np.array([      # otter rg
        #     [280.21,  0.12,   11.00,    0,    50.04,       0],
        #     [ 0.12,  837.47,   0.29, -50.04,    0,    -50.04],
        #     [11.00,   0.29,  1359.34,   0,    50.04,       0],
        #     [  0,    -50.04,    0,    260.73,   0,         0],
        #     [50.04,    0,     50.04,    0,    625.45,      0],
        #     [  0,    -50.04,    0,      0,      0,    500.36]
        # ])
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
        self.q0_ref = np.array([0, 0, 0, 1])            # desired quaternion: x, y, z, w
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
            new_quat = Rotation.from_matrix(X[:3, :3]).as_quat()        # x, y, z, w

            self.X_ref = np.vstack([self.X_ref, np.concatenate([new_quat, X[:3, 3]])])

            self.xi_ref = np.vstack([self.xi_ref, xid_ref_rt])
        
        # self.X_ref[:, 5] += 2   # offset in y direction, use when using vrx ground truth odometry


    def generate_zigzag_waypoints(self):
        amplitude=1.0
        wavelength=10.0

        segment_steps = self.Nsim       # Number of waypoints

        x_start, y_start, z_start = self.x0_ref[4:7]  # Starting position
        
        x_vals = np.linspace(x_start, x_start + 3 * wavelength, segment_steps)  # X-axis progression
        y_vals = y_start + amplitude * np.sin((2 * np.pi / wavelength) * (x_vals - x_start))


        for i in range(segment_steps):
            x = x_vals[i]
            y = y_vals[i]

            # Compute yaw angle as tangent to the sine curve
            theta = np.arctan2(
                (2 * np.pi / wavelength) * amplitude * np.cos((2 * np.pi / wavelength) * x),
                1
            )

            # Convert yaw to quaternion (x, y, z, w)
            quaternion = Rotation.from_euler('xyz', [0, 0, theta]).as_quat()

            # Store waypoint
            self.X_ref = np.vstack([self.X_ref, np.concatenate([quaternion, [x, y, 0]])])
    
    def generate_circle_waypoints(self):
        radius = 5.0  

        x_start, y_start, z_start = self.x0_ref[4:7]  # Starting position

        center = np.array([x_start, y_start + radius])  # Center at (0,5)
        num_waypoints = self.Nsim  # Number of waypoints
        
        angles = np.linspace(-np.pi/2, 3*np.pi/2, num_waypoints)  # Counterclockwise from (0,0)
        
        waypoints = []
        for theta in angles:
            x = center[0] + radius * np.cos(theta)
            y = center[1] + radius * np.sin(theta)

            yaw_angle = theta + np.pi/2  # Align tangent direction
            quaternion = Rotation.from_euler('xyz', [0, 0, yaw_angle]).as_quat()  # Convert to quaternion (x, y, z, w)

            self.X_ref = np.vstack([self.X_ref, np.concatenate([quaternion, [x, y, 0]])])

    def waypoints2twists(self):
        waypoints = self.X_ref
        # Generate twist based on waypoints
        for k in range(waypoints.shape[0] - 1):
            # Get the current and next waypoint
            X1 = pose_to_T_matrix(waypoints[k, :])  # Transformation of waypoint k
            X2 = pose_to_T_matrix(waypoints[k + 1, :])  # Transformation of waypoint k+1
    
            # Compute the relative transformation
            delta_X = np.linalg.inv(X1) @ X2
    
            # Compute the twist using logm (logarithm map of the relative transformation)
            twist_matrix = logm(delta_X) / self.dt  # Divide by dt to get the twist
    
            # Angular velocity (from skew-symmetric part of matrix)
            # omega = np.array((twist_matrix[2, 1], twist_matrix[0, 2], twist_matrix[1, 0]))
            omega = np.array((0, 0, twist_matrix[1, 0]))  

            # Linear velocity (from translational part)
            # v = twist_matrix[:3, 3]  
            v = np.array((twist_matrix[0, 3], 0, 0))  

            self.xi_ref = np.vstack([self.xi_ref, np.concatenate([omega, v])])


class Logger:
    def __init__(self, Nsim, Nt):
        self.X = np.zeros((13, Nsim - Nt + 1))    # +1 for initial state
        self.Err = np.zeros((6, Nsim - Nt))
        self.U = np.zeros((2, Nsim - Nt))
        self.Err2 = np.zeros((4, Nsim - Nt))
        self.taken_time = 0