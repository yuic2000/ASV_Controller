import numpy as np

from mpc_python.src.utils.LieGroup import *


def eMPCCost(xid, param, solver):
    """
    Compute the cost function matrices for an extended MPC setup.

    Parameters:
    Q, R, P : ndarray
        Cost matrices.
    xid: Reference trajectory.
    param: Parameters with attributes Nt, Nx, Nu.
    solver: Solver information or configuration (not used directly here).

    Returns:
    M: The cost matrix.
    q: The linear cost vector.
    solver: Possibly returned unchanged.
    """
    # eMPC cost matrices
    Q = param.Q
    R = param.R
    P = param.P

    Nx = param.Nx
    Nu = param.Nu
    Nt = param.Nt
    
    n_total = (Nt + 1) * Nx + Nt * Nu
    q = np.zeros((n_total, 1))
    M = np.zeros((n_total, n_total))

    for k in range(Nt - 1):
        C = np.eye(12)
        C[6:12, 0:6] = -adjoint_(xid[k, :])

        M[k*Nx:(k+1)*Nx, k*Nx:(k+1)*Nx] = C.T @ Q @ C

        b = np.zeros(12)
        b[6:12] = xid[k, :]
        q[k*Nx:(k+1)*Nx, 0] = -C.T @ Q @ b

    # Handle the final stage k = Nt-1
    k = Nt - 1      # the 100th element: 99, start from 0
    C = np.eye(12)
    C[6:12, 0:6] = -adjoint_(xid[k, :])

    b = np.zeros(12)
    b[6:12] = xid[k, :]
    b = C.T @ P @ b

    M[k*Nx:(k+1)*Nx, k*Nx:(k+1)*Nx] = C.T @ P @ C
    q[k*Nx:(k+1)*Nx, 0] = -b

    for j in range(1, Nt + 1):
        start_index = (k+1)*Nx + (j-1)*Nu
        M[start_index:start_index+Nu, start_index:start_index+Nu] = R

    return M, q, solver