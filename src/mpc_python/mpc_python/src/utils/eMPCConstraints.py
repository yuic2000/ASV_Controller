import numpy as np

from mpc_python.src.utils.LieGroup import *


def eMPCConstraints(xid, p0, x0, param):

    # min x'Qx + u'Ru
    # xkp1 = Ak xk + Bk uk + bk, k = 0,1, ..., N-1
    # x0 = x_init,
    # umin < uk < umax, k = 0, 1, ..., N-1

    I = param.I
    dt = param.dt
    
    Nx = param.Nx
    Nu = param.Nu
    Nt = param.Nt
    
    A = np.zeros((Nx * (Nt + 1) + Nu * Nt, Nt * (Nx + Nu) + Nx))
    Noff = Nx * (Nt + 1)
    bmin = np.zeros(A.shape[0])
    

    D = np.diag([-300, -900, -800, -100, -100, -500])  # from vrx wamv (vrx), Kp Mq, Nr, Xu, Yv, Zw
    # D = np.diag([-12.2948, -45.5947, -12.3312 * (1 + 10 * abs(x0[9])), -77.5544, 0, -250.3037])     # for blueboat
    I_inv = np.linalg.inv(I)
    
    for k in range(Nt):
        # Hydrodynamic Damping Matrix
        D_t = D.copy()
        # D_t[2, 2] -= 12.3312 * (10 * abs(xid[k, 2]))        # for blueboat

        # Rigid Body Mass and Inertia Matrix, VRX WAMV (no added mass and rg)
        Irb = np.array([        # otter rg
            [250.18,    0,      0,      0,      0,        0],
            [  0,    250.18,    0,      0,      0,        0],
            [  0,       0,    250.18,   0,      0,        0],
            [  0,       0,      0,    225.17,  0.12,   0.99],
            [  0,       0,      0,    0.12,   454.1,   0.29],
            [  0,       0,      0,    0.99,   0.29,  499.75]
        ])

        # # Rigid Body matrix from blueboat CAD model
        # Irb = np.array([
        #     [28.63, 0, 0, 0, -17.66, -0.25],
        #     [0, 28.63, 0, 17.66, 0, 10.79],
        #     [0, 0, 28.63, 0.25, -10.79, 0],
        #     [0, 17.66, 0.25, 13.87, -0.35, 6.55],
        #     [-17.66, 0, -10.79, -0.35, 18.62, 0.13],
        #     [-0.25, 10.79, 0, 6.55, 0.13, 7.13]
        # ])

        M11 = Irb[0:3, 0:3]
        M12 = Irb[0:3, 3:6]
        M21 = Irb[3:6, 0:3]
        M22 = Irb[3:6, 3:6]

        xi_w = xid[k, 0:3]
        xi_v = xid[k, 3:6]

        G = np.zeros((6, 6))
        G[0:3, 0:3] = skew(xi_w) @ M22 + skew(xi_v) @ M12
        G[0:3, 3:6] = skew(xi_v) @ M11 + skew(xi_w) @ M21
        G[3:6, 0:3] = skew(xi_w) @ M12
        G[3:6, 3:6] = skew(xi_w) @ M11

        # Rigid Body Coriolis Matrix
        Crb = np.zeros((6, 6))
        Crb[0:3, 0:3] = skew(M21 @ xi_v) + skew(M22 @ xi_w)
        Crb[0:3, 3:6] = skew(M11 @ xi_v) + skew(M12 @ xi_w)
        Crb[3:6, 0:3] = skew(M11 @ xi_v) + skew(M12 @ xi_w)

        # Additional Mass Matrix, mass: 250.18kg, from vrx wamv
        Iam = np.zeros((6, 6))
        # Additional Mass Matrix, mass: 28.63kg, from Blueboat CAD model
        # Iam = np.diag([5.678, 42.945, 28.63, 0.594, 2.928, 5.202])

        Mam11 = Iam[0:3, 0:3]
        Mam22 = Iam[3:6, 3:6]

        # Additional Mass Coriolis Matrix
        Cam = np.zeros((6, 6))
        Cam[0:3, 0:3] = skew(Mam22 @ xi_w)
        Cam[0:3, 3:6] = skew(Mam11 @ xi_v)
        Cam[2, 3] = 0
        Cam[2, 4] = 0
        Cam[3:6, 0:3] = skew(Mam11 @ xi_v)
        Cam[3, 2] = 0
        Cam[4, 2] = 0

        G2 = np.zeros((6, 6))
        G2[0:3, 0:3] = skew(xi_w) @ Mam22
        G2[0:3, 3:6] = skew(xi_v) @ Mam11
        G2[3:6, 3:6] = skew(xi_w) @ Mam11
        G2[2, 3] = 0
        G2[2, 4] = 0
        G2[3, 5] = 0
        G2[4, 5] = 0

        Cam = -Cam
        Crb = -Crb
        
        H = I_inv @ (Crb + Cam + G + G2 + D_t)
        
        Ac = np.vstack([
            np.hstack([-adjoint_(xid[k, :]), -np.eye(6)]),
            np.hstack([np.zeros((6, 6)), H])
        ])
        
        Bc = np.zeros((12, 2))
        Bc[6:12, :] = I_inv @ np.array([
            [0, 0],
            [0, 0],
            [1.0, -1.0],      # vrx wamv
            # [0.285, -0.285],    # blueboat
            [1, 1],
            [0, 0],
            [0, 0]
        ])
        
        b = (G + G2) @ x0[7:13]

        # b[2] += 12.3312 * (10 * abs(xid[k, 2])) * xid[k, 2]      # for blueboat
        b = -I_inv @ b
        hc = np.hstack([xid[k, :], b])
        
        Ad = np.eye(param.Nx) + Ac * dt
        Bd = dt * Bc
        hd = dt * hc
        
        A[(k * Nx):(k + 1) * Nx, (k * Nx):(k + 1) * Nx] = -Ad
        A[(k * Nx):(k + 1) * Nx, (k + 1) * Nx:(k + 2) * Nx] = np.eye(Nx)
        A[(k * Nx):(k + 1) * Nx, Noff + (k * Nu):(Noff + (k + 1) * Nu)] = -Bd
        bmin[(k * Nx):(k + 1) * Nx] = hd
    
    A[Nt * Nx:(Nt + 1) * Nx, 0:Nx] = np.eye(Nx)
    for k in range(Nt):
        A[Noff + (k * Nu):Noff + (k + 1) * Nu, Noff + (k * Nu):Noff + (k + 1) * Nu] = np.eye(Nu)


    bmin[Nt * Nx:(Nt + 1) * Nx] = p0

    bmax = np.copy(bmin)
    bmax[(Nt + 1) * Nx:] = np.tile(param.umax, Nt)
    bmin[(Nt + 1) * Nx:] = np.tile(param.umin, Nt)

    return A, bmin, bmax