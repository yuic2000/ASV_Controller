import pickle
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.linalg import expm

from mpc_python.src.plot import plot_mpc
from mpc_python.src.utils.LieGroup import skew

# Initialize parameters
np.random.seed(2)
Ns = 10
dt = 0.05                       # MPC simulation time step
Nsim = int(np.ceil(65 / dt))    # Total simulation time + Planning time
mode = 3                        # mode 1: zigzag, mode 2: circling, else: debug
V_c = 0.0                       # Ocean current velocity

# Parameter structure
param = {
    'Nsim': Nsim,
    'Nx': 12,           # dimension x
    'Nu': 2,            # dimension of u
    'Nt': 100,          # horizon length
    'dt': dt,           # step time
    'V_c': V_c
    # 'mpc_cal_t': 100    # computation limit of NMPC
}

# State constraints
param['xmax'] = np.inf * np.ones(param['Nx'])
param['xmin'] = -param['xmax']

# Mass Matrix
I = np.array([
    [17.1572, 0, 4.4, 0, 11, 0],
    [0, 37.07, 0, -11, 0, -11],
    [4.4, 0, 45.265, 0, 11, 0],
    [0, -11, 0, 60.2815, 0, 0],
    [11, 0, 11, 0, 137.5, 0],
    [0, -11, 0, 0, 0, 110]
])

# Input constraints
param['umax'] = np.ones(param['Nu']) * 115
param['umin'] = -np.ones(param['Nu']) * 65

param['I'] = I          # body inertial tensor

# Generate reference trajectory
q0_ref = np.array([1, 0, 0, 0])     # desired quaternion
p0_ref = np.array([0, 0, 0])        # desired position
w0_ref = np.array([0, 0, 0.1])      # desired yaw velocity 0.1 rad/s
v0_ref = np.array([0.5, 0, 0])      # desired surge velocity 0.5 m/s
x0_ref = np.concatenate([q0_ref, p0_ref])
xid_ref = np.concatenate([w0_ref, v0_ref])

X = np.eye(4)
X_ref = x0_ref.reshape(1, -1)
xi_ref = xid_ref.reshape(1, -1)

for i in range(Nsim):
    xid_ref_rt = xid_ref.copy()
    
    if mode == 1:  # zigzag maneuver
        xid_ref_rt[2] = 0.1 * np.cos(i/100)
    
    Xi = np.zeros((4, 4))
    Xi[:3, :3] = skew(xid_ref_rt[:3])
    Xi[:3, 3] = xid_ref_rt[3:]
    
    X = X @ expm(Xi * dt)
    new_quat = Rotation.from_matrix(X[:3, :3]).as_quat()
    X_ref = np.vstack([X_ref, np.concatenate([new_quat, X[:3, 3]])])
    xi_ref = np.vstack([xi_ref, xid_ref_rt])

param['X_ref'] = X_ref
param['xi_ref'] = xi_ref

# Load Logger_eMPC_list from a file
if mode == 1:                 # mode 1: zigzag, 
    with open('result/zigzag/zigzag_data_currentFree.pkl', 'rb') as f:
        Logger_eMPC_list = pickle.load(f)

elif mode == 2:               # mode 2: circling
    with open('result/circling/circling_data_currentFree.pkl', 'rb') as f:
        Logger_eMPC_list = pickle.load(f)

else:       # debug
    with open('circling_data.pkl', 'rb') as f:
        Logger_eMPC_list = pickle.load(f)



plot_mpc(Logger_eMPC_list, param['X_ref'], dt)