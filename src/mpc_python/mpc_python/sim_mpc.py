import numpy as np
import time
from scipy.spatial.transform import Rotation as R
from scipy.linalg import logm

from mpc_python.src.controller.eMPC import eMPC
from mpc_python.src.otter_true import otter_true
from mpc_python.src.param import *

def sim_mpc(q0, p0, w0, v0, param, mpc_node):
    X_ref = param.X_ref     # [q_ref, p_ref]
    xi_ref = param.xi_ref   # [w_ref, v_ref]
    Nsim = param.Nsim
    Nt = param.Nt
    dt = param.dt

    # log the states
    x0 = np.concatenate((q0, p0, w0, v0))
    X = [x0]
    U = []
    Err = []
    Err2 = []

    logger = Logger(Nsim)
    solver = {'init': 0}
    logger.taken_time = 0

    for i in range(Nsim - Nt):
        X_ref_rt = X_ref[i]
        xi_ref_rt = xi_ref[i:i + Nt]

        start_time = time.time()
        
        u, solver = eMPC(x0, X_ref_rt, xi_ref_rt, param, solver)        # u[0]: left, u[1]: right

        mpc_node.left_thrust_command_pub(u[0])
        mpc_node.right_thrust_command_pub(u[1])

        logger.taken_time += (time.time() - start_time)

        # Update x_otter four times for simulation frequency, 4 * dt
        x_otter = np.concatenate((x0[10:13], x0[7:10], x0[4:7], R.from_quat(x0[0:4]).as_euler('xyz')))
        for j in range(5):  # 4 -> 5
            x_otter += otter_true(x_otter, u, param.V_c, param.beta_c) * dt / 5

        x0 = np.concatenate((R.from_euler('xyz', x_otter[9:12]).as_quat(), x_otter[6:9], x_otter[3:6], x_otter[0:3]))

        Xd = np.eye(4)
        Xd[0:3, 0:3] = R.from_quat(X_ref_rt[0:4]).as_matrix()
        Xd[0:3, 3] = X_ref_rt[4:7]

        X0 = np.eye(4)
        X0[0:3, 0:3] = R.from_quat(x0[0:4]).as_matrix()
        X0[0:3, 3] = x0[4:7]

        Xerr = logm(np.linalg.inv(X0).dot(Xd))
        XXerr = np.linalg.inv(X0).dot(Xd)

        x00 = np.zeros(6)
        x00[0:3] = [Xerr[2, 1], Xerr[0, 2], Xerr[1, 0]]
        x00[3:6] = Xerr[0:3, 3]

        X.append(x0)
        U.append(u)
        Err.append(x00)
        Err2.append([np.sqrt(np.sum(logm(XXerr[:3, :3])**2) / 2), *XXerr[0:3, 3]])

    logger.X = np.array(X).T
    logger.U = np.array(U).T
    logger.Err = np.array(Err).T
    logger.Err2 = np.array(Err2).T

    return logger
