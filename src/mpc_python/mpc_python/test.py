import pickle
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.linalg import expm

from mpc_python.src.plot import plot_mpc
from mpc_python.src.utils.LieGroup import skew

from mpc_python.src.param import Param


# Initialize parameters
np.random.seed(2)

param = Param()             # Initialize parameters in param.py

# Ns = 10
dt = param.dt                   # MPC simulation time step
Nsim = param.Nsim               # Total simulation time + Planning time
mode = 3                        # mode 1: zigzag, mode 2: circling, 3: zigzag debug, 4: circling debug
V_c = param.V_c                 # Ocean current velocity
beta_c = param.beta_c           # Ocean current direction


x0_ref = param.x0_ref
xid_ref = param.xid_ref
X_ref = param.X_ref
xi_ref = param.xi_ref

X = np.eye(4)

for i in range(Nsim):
    xid_ref_rt = xid_ref.copy()
    
    if mode in {1, 3}:  # zigzag maneuver
        xid_ref_rt[2] = 0.1 * np.cos(i / 50)    # 100: 20 Hz, 50: 10 Hz
    
    Xi = np.zeros((4, 4))
    Xi[:3, :3] = skew(xid_ref_rt[:3])
    Xi[:3, 3] = xid_ref_rt[3:]
    
    X = X @ expm(Xi * dt)
    new_quat = Rotation.from_matrix(X[:3, :3]).as_quat()
    X_ref = np.vstack([X_ref, np.concatenate([new_quat, X[:3, 3]])])
    xi_ref = np.vstack([xi_ref, xid_ref_rt])

param.X_ref = X_ref
param.xi_ref = xi_ref

# Load Logger_eMPC_list from a file
if mode == 1:                 # mode 1: zigzag, 
    with open('result/zigzag/zigzag_data_currentFree.pkl', 'rb') as f:
        Logger_eMPC_list = pickle.load(f)

elif mode == 2:               # mode 2: circling
    with open('result/circling/circling_data_currentFree.pkl', 'rb') as f:
        Logger_eMPC_list = pickle.load(f)

elif mode == 3:               # zigzag debug
    with open('zigzag_data.pkl', 'rb') as f:
        Logger_eMPC_list = pickle.load(f)

elif mode == 4:               # circling debug
    with open('circling_data.pkl', 'rb') as f:
        Logger_eMPC_list = pickle.load(f)
else:
    raise ValueError('Invalid mode')


plot_mpc(Logger_eMPC_list, param.X_ref, dt)