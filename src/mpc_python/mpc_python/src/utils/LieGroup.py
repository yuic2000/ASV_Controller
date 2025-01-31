import numpy as np

def skew(w):

    return np.array([[0, -w[2], w[1]],
                     [w[2], 0, -w[0]],
                     [-w[1], w[0], 0]])

####################################################################

def coadjoint(x):
    """
    Calculate the coadjoint representation of a twist vector in SE(3).
    """

    adTx = adjoint_(x).T

    return adTx

####################################################################

def adjoint_(x):
    """
    Calculate the adjoint representation of a twist vector in SE(3).
    """
    
    w = np.array(x[0:3])
    v = np.array(x[3:6])
    
    adx = np.block([
        [skew(w), np.zeros((3, 3))],
        [skew(v), skew(w)]
    ])
    
    return adx

####################################################################

def AdjointMat(x):

    R = x[0:3, 0:3]
    p = x[0:3, 3]

    Adx = np.block([
        [R, np.zeros((3, 3))],
        [skew(p) @ R, R]
    ])

    return Adx