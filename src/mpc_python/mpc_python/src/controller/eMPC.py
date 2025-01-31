import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.linalg import logm
import osqp
from scipy import sparse

from mpc_python.src.utils.eMPCConstraints import eMPCConstraints
from mpc_python.src.utils.eMPCCost import eMPCCost

def quat2rotm(quat):
    """ Convert quaternion to rotation matrix. """
    return R.from_quat(quat).as_matrix()

def eMPC(x0, xd, xid, param, solver):
    Xd = np.block([
        [quat2rotm(xd[:4]), xd[4:7].reshape(-1, 1)],
        [np.zeros((1, 3)), np.ones(1)]
    ])
    
    X00 = np.block([
        [quat2rotm(x0[:4]), x0[4:7].reshape(-1, 1)],
        [np.zeros((1, 3)), np.ones(1)]
    ])

    X00 = np.linalg.inv(X00) @ Xd
    X0 = np.real(logm(X00))

    # print(X00)
    # print('\n')
    # print(X0)

    p0 = np.concatenate([
        np.array([X0[2, 1], X0[0, 2], X0[1, 0]]),  # first 3 elements
        X0[:3, 3].flatten(),                        # next 3 elements from the last column
        x0[7:]                                      # 6 elements
    ])

    A, bmin, bmax = eMPCConstraints(xid, p0, x0, param)
    M, q, solver = eMPCCost(xid, param, solver)
    
    # Setup and solve the optimization problem using OSQP, yui
    solver['solver'] = osqp.OSQP()
    solver['solver'].setup(P=sparse.csc_matrix(M), q=q, A=sparse.csc_matrix(A), l=bmin, u=bmax, verbose=False)

    # Solve the optimization problem
    sol = solver['solver'].solve()
    u = sol.x[(param.Nt + 1) * param.Nx : (param.Nt + 1) * param.Nx + param.Nu]
    
    return u, solver
