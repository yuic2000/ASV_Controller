import numpy as np
from mpc_python.src.utils.LieGroup import skew, pose_to_T_matrix
from scipy.linalg import expm, logm
from scipy.spatial.transform import Rotation

import rclpy
import yaml
import os
from mpc_python.communication.gtodom_sub import GtOdomSubscriberNode

class Param:
    def __init__(self):

        yaml_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'config', 'mpc.yaml'))
        if not os.path.isfile(yaml_path):
            import ament_index_python.packages
            pkg_share = ament_index_python.packages.get_package_share_directory('mpc_python')
            yaml_path = os.path.join(pkg_share, 'config', 'mpc.yaml')
        if not os.path.isfile(yaml_path):
            raise FileNotFoundError(f"YAML file not found at {yaml_path}")
        self.load_from_yaml(yaml_path)

        # ───────── state ─────────
        # self.dt = 0.04                            # control frequency, 25 Hz, original 20Hz
        # self.Nt = 100                             # horizon length
        self.Nsim = int(np.ceil(64 / self.dt))      # Temp, Total simulation time + Planning time

        self.mode = 1                               # mode 1: zigzag, mode 2: circling

        self.Nx = 12                                # dimension x   
        self.Nu = 2                                 # dimension of u

        # state constraints
        self.xmax = np.inf * np.ones(self.Nx)
        self.xmin = -self.xmax

        # control constraints, for vrx wamv
        self.umax = np.ones(self.Nu) * self.max_thrust   # 200 N
        self.umin = np.ones(self.Nu) * self.min_thrust  # -100 N
        # control constraints, for blueboat
        # self.umax = np.ones(self.Nu) * 50           # 3.5 kgF
        # self.umin = -np.ones(self.Nu) * 26          # -3.0 kgF

        # Mass matrix, vrx wamv
        self.I = np.array([
            [225.17,  0.12,  0.99,    0,   0,   0],
            [0.12,  454.1,  0.29, 0,   0,   0],
            [0.99,   0.29, 499.75,  0, 0,      0],
            [0,    0, 0,  250.18,   0,        0],
            [0,   0,   0,    0,   250.18,      0],
            [0, 0,    0,     0,     0,    250.18]
        ])
        # # Mass matrix, from Blueboat CAD model
        # self.I = np.array([
        #     [14.47,  -0.35,  6.55,    0,   17.66,   0.25],
        #     [-0.35,  21.55,  0.13, -17.66,   0,   -10.79],
        #     [6.55,   0.13, 12.33,  -0.25, 10.79,      0],
        #     [0,    -17.66, -0.25,  34.31,   0,        0],
        #     [17.66,   0,   10.79,    0,   71.58,      0],
        #     [0.25, -10.79,    0,     0,     0,    57.26]
        # ])


    def load_from_yaml(self, yaml_path):
        # Load parameters from YAML file
        with open(yaml_path, 'r') as file:
            config = yaml.safe_load(file)

        if 'ros_topic' in config:
            self.path_topic = config['ros_topic']['path_topic']
            self.pose_topic = config['ros_topic']['pose_topic']
        
        if 'parameters' in config:
            self.goal_tol = config['parameters']['goal_tol']
            self.vx_init = config['parameters']['v_surge']
            self.yaw_rate_init = config['parameters']['yaw_rate']
            self.max_thrust = config['parameters']['max_thrust']
            self.min_thrust = config['parameters']['min_thrust']

        if 'cost' in config:        
            # Load P and Q matrices
            # [0:3] is angle: roll, pitch, yaw ,
            # [3:6] is position: x, y, z
            # [6:9] is angular velocity: roll, pitch, yaw,
            # [9:12] is linear velocity: surge (Vx), sway (Vy), heave (Vz),
            diag_values_cost_P = np.array(config['cost']['P'], dtype=float)
            diag_values_cost_Q = np.array(config['cost']['Q'], dtype=float)
            P_scale = float(config['cost']['P_scale'])
            Q_scale = float(config['cost']['Q_scale'])
            
            self.P = np.diag(diag_values_cost_P) * P_scale
            self.Q = np.diag(diag_values_cost_Q) * Q_scale
            
            # Load R matrix
            self.R = np.zeros((2, 2))
            for i in range(2):
                for j in range(2):
                    self.R[i, j] = float(config['cost']['R'][i][j])
    
        if 'Horizon' in config:
            self.dt = float(config['Horizon']['dt'])
            self.Nt = int(config['Horizon']['Nt'])
            
        print('Parameters loaded from YAML file')

    def set_desired_posistion(self, x_init):
        # self.q0_ref = x_init[0:4]
        self.q0_ref = np.array([0, 0, 0, 1])                   # desired quaternion: x, y, z, w
        self.p0_ref = x_init[4:7]
        self.w0_ref = np.array([0, 0, self.yaw_rate_init])     # desired yaw velocity 0.1 rad/s
        self.v0_ref = np.array([self.vx_init, 0, 0])           # desired surge velocity 0.5 m/s
        # self.w0_ref = np.array([0, 0, 0.1])                  # desired yaw velocity 0.1 rad/s
        # self.v0_ref = np.array([0.5, 0, 0])                  # desired surge velocity 0.5 m/s

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

    def generate_zigzag_waypoints(self):
        amplitude=0.8
        wavelength=12.0

        segment_steps = self.Nsim       # Number of waypoints

        x_start, y_start, z_start = self.x0_ref[4:7]  # Starting position

        x_vals = np.linspace(x_start, x_start + 2.5 * wavelength, segment_steps)  # X-axis progression
        y_vals = (y_start + amplitude) - amplitude * np.cos((2 * np.pi / wavelength) * (x_vals - x_start))


        for i in range(segment_steps):
            x = x_vals[i]
            y = y_vals[i]

            # Compute yaw angle as tangent to the sine curve
            dy_dx = (2 * np.pi / wavelength) * amplitude * np.sin((2 * np.pi / wavelength) * (x - x_start))
            theta = np.arctan2(dy_dx, 1.0)

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