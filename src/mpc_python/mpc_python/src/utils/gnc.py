#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GNC functions. 

Reference: T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and
Motion Control. 2nd. Edition, Wiley. 
URL: www.fossen.biz/wiley

Author:     Thor I. Fossen
"""

import numpy as np

def sat(x, x_min, x_max):
    """
    x = sat(x,x_min,x_max) saturates a signal x such that x_min <= x <= x_max
    """
    if x > x_max:
        x = x_max 
    elif x < x_min:
        x = x_min
        
    return x 

#################################################################################

def Smtrx(a):
    """
    S = Smtrx(a) computes the 3x3 vector skew-symmetric matrix S(a) = -S(a)'.
    The cross product satisfies: a x b = S(a)b. 
    """
 
    S = np.array([ 
        [ 0, -a[2], a[1] ],
        [ a[2],   0,     -a[0] ],
        [-a[1],   a[0],   0 ]  ])

    return S

#################################################################################

def Hmtrx(r):
    """
    H = Hmtrx(r) computes the 6x6 system transformation matrix
    H = [eye(3)     S'
         zeros(3,3) eye(3) ]       Property: inv(H(r)) = H(-r)

    If r = r_bg is the vector from the CO to the CG, the model matrices in CO and
    CG are related by: M_CO = H(r_bg)' * M_CG * H(r_bg). Generalized position and
    force satisfy: eta_CO = H(r_bg)' * eta_CG and tau_CO = H(r_bg)' * tau_CG 
    """

    H = np.identity(6,float)
    H[0:3, 3:6] = Smtrx(r).T

    return H

#################################################################################

def Rzyx(phi,theta,psi):
    """
    R = Rzyx(phi,theta,psi) computes the Euler angle rotation matrix R in SO(3)
    using the zyx convention
    """
    
    cphi = np.cos(phi)
    sphi = np.sin(phi)
    cth  = np.cos(theta)
    sth  = np.sin(theta)
    cpsi = np.cos(psi)
    spsi = np.sin(psi)
    
    R = np.array([
        [ cpsi*cth, -spsi*cphi+cpsi*sth*sphi, spsi*sphi+cpsi*cphi*sth ],
        [ spsi*cth,  cpsi*cphi+sphi*sth*spsi, -cpsi*sphi+sth*spsi*cphi ],
        [ -sth,      cth*sphi,                 cth*cphi ] ])

    return R

#################################################################################

def Tzyx(phi,theta):
    """
    T = Tzyx(phi,theta) computes the Euler angle attitude
    transformation matrix T using the zyx convention
    """
    
    cphi = np.cos(phi)
    sphi = np.sin(phi)
    cth  = np.cos(theta)
    sth  = np.sin(theta)    

    try: 
        T = np.array([
            [ 1,  sphi*sth/cth,  cphi*sth/cth ],
            [ 0,  cphi,          -sphi],
            [ 0,  sphi/cth,      cphi/cth] ])
        
    except ZeroDivisionError:  
        print ("Tzyx is singular for theta = +-90 degrees." )
        
    return T

#################################################################################

def eulerang(phi,theta,psi):
    '''
    eulerang(phi,theta,psi) computes the Euler angle transformation matrix

    J = [ Rzyx     0
           0  Tzyx ]
    '''
    J1 = Rzyx(phi,theta,psi)
    J2 = Tzyx(phi,theta)
    
    J = np.block([[J1, np.zeros((3,3))], [np.zeros((3,3)), J2]])
    
    return J

#################################################################################

def m2c(M, nu):
    """
    C = m2c(M,nu) computes the Coriolis and centripetal matrix C(nu) from the
    mass matrix M and generalized velocity vector nu (Fossen 2021, Ch. 3)
    
    If M is a 6x6 matrix and nu = [u, v, w, p, q, r]', the output is a 6x6 C matrix
    If M is a 3x3 matrix and nu = [u, v, r]', the output is a 3x3 C matrix.
    """

    M = 0.5 * (M + M.T)     # systematization of the inertia matrix

    if (len(nu) == 6):      #  6-DOF model
    
        M11 = M[0:3,0:3]
        M12 = M[0:3,3:6] 
        M21 = M12.T
        M22 = M[3:6,3:6] 
    
        nu1 = nu[0:3]
        nu2 = nu[3:6]
        dt_dnu1 = np.matmul(M11,nu1) + np.matmul(M12,nu2)
        dt_dnu2 = np.matmul(M21,nu1) + np.matmul(M22,nu2)

        # C  = [  zeros(3,3)      -Smtrx(dt_dnu1)
        #       -Smtrx(dt_dnu1)  -Smtrx(dt_dnu2) ]
        C = np.zeros( (6,6) )    
        C[0:3,3:6] = -Smtrx(dt_dnu1)
        C[3:6,0:3] = -Smtrx(dt_dnu1)
        C[3:6,3:6] = -Smtrx(dt_dnu2)
            
    else:   # 3-DOF model (surge, sway and yaw)
        
        # C = [ 0             0            -M(2,2)*nu(2)-M(2,3)*nu(3)
        #       0             0             M(1,1)*nu(1)
        #       M(2,2)*nu(2)+M(2,3)*nu(3)  -M(1,1)*nu(1)          0  ]    
        C = np.zeros( (3,3) ) 
        C[0,2] = -M[1,1] * nu[1] - M[1,2] * nu[2]
        C[1,2] =  M[0,0] * nu[0] 
        C[2,0] = -C[0,2]       
        C[2,1] = -C[1,2]
        
    return C

#################################################################################

def Hoerner(B, T):
    """
    CY_2D = Hoerner(B,T)
    Hoerner computes the 2D Hoerner cross-flow form coeff. as a function of beam 
    B and draft T.The data is digitized and interpolation is used to compute 
    other data point than those in the table
    """
    
    # DATA = [B/2T  C_D]
    DATA1 = np.array([
        0.0109,0.1766,0.3530,0.4519,0.4728,0.4929,0.4933,0.5585,0.6464,0.8336,
        0.9880,1.3081,1.6392,1.8600,2.3129,2.6000,3.0088,3.4508, 3.7379,4.0031 
        ])
    DATA2 = np.array([
        1.9661,1.9657,1.8976,1.7872,1.5837,1.2786,1.2108,1.0836,0.9986,0.8796,
        0.8284,0.7599,0.6914,0.6571,0.6307,0.5962,0.5868,0.5859,0.5599,0.5593 
        ])

    CY_2D = np.interp( B / (2 * T), DATA1, DATA2 )
        
    return CY_2D

#################################################################################

def crossFlowDrag(L, B, T, nu_r):
    """
    tau_crossflow = crossFlowDrag(L,B,T,nu_r) computes the cross-flow drag 
    integrals for a marine craft using strip theory. 

    M d/dt nu_r + C(nu_r)*nu_r + D*nu_r + g(eta) = tau + tau_crossflow
    Inputs: L:  length
            B:  beam
            T:  draft 
            nu_r = [u-u_c, v-v_c, w-w_c, p, q, r].T: relative velocity vector

    Output: tau_crossflow = [0 Yh 0 0 0 Nh]:  cross-flow drag in sway and yaw
    """

    rho = 1026               # density of water
    n = 20                   # number of strips

    dx = L / 20            
    Cd_2D = Hoerner(B, T)    # 2D drag coefficient based on Hoerner's curve

    Yh = 0
    Zh = 0
    Mh = 0
    Nh = 0
    # xL = -L/2
    
    for i in range(0, n+1):
        xL = -L/2 + i * dx
        v_r = nu_r[1]             # relative sway velocity
        w_r = nu_r[2]             # relative heave velocity
        r = nu_r[5]               # yaw rate

        U_h = abs(v_r + xL * r) * (v_r + xL * r)
        U_v = abs(w_r + xL * r) * (v_r + xL * r)
        
        Yh = Yh - 0.5 * rho * T * Cd_2D * U_h * dx         # sway force
        Zh = Zh - 0.5 * rho * T * Cd_2D * U_v * dx         # heave force
        Mh = Mh - 0.5 * rho * T * Cd_2D * xL * U_v * dx    # pitch moment
        Nh = Nh - 0.5 * rho * T * Cd_2D * xL * U_h * dx    # yaw moment
        
    tau_crossflow = np.array([0, Yh, Zh, 0, Mh, Nh], float)

    return tau_crossflow

#################################################################################

def addedMassSurge(m, L, rho=1025):
    """
    Approximates the added mass in surge using Söding's (1982) formula:
        A11 = -Xudot = 2.7 * rho * nabla^(5/3) / L^2

    Inputs:
        m (float): ship/vehicle mass (kg)
        L (float): ship/vehicle length (m)
        rho (float, optional): density of water (kg/m3). Defaults to 1025.

    Outputs:
        A11: added mass in surge due to linear acceleration in surge (kg)
        ratio: mass ratio A11/m (typical values < 0.20)

    Reference: 
        H. Söding (1982). Prediction of Ship Steering Capabilities.
        Schiffstechnik, 3-29.
    """
    nabla = m / rho                         # volume displacement
    A11 = 2.7 * rho * nabla**(5/3) / L**2  # added mass A11
    ratio = A11 / m

    return A11, ratio

#################################################################################

