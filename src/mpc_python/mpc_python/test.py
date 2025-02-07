import pickle
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.linalg import expm

from mpc_python.src.plot import plot_mpc
from mpc_python.src.utils.LieGroup import skew

from mpc_python.src.param import Param


# Initialize parameters
np.random.seed(2)

param = Param()                 # Initialize parameters in param.py

# Ns = 10
dt = param.dt                   # MPC simulation time step
Nsim = param.Nsim               # Total simulation time + Planning time
mode = 4                        # mode 1: zigzag, mode 2: circling, 3: zigzag debug, 4: circling debug

param.get_ref_trajectory()      # Get reference trajectory

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