import numpy as np

from mpc_python.src.utils.gnc import *

def otter_true(x, n, V_c, beta_c):

# [xdot,U] = otter(x,n,mp,rp,V_c,beta_c) returns the speed U in m/s (optionally) 
# and the time derivative of the state vector: 
#    x = [ u v w p q r x y z phi theta psi ]' 
# for the Maritime Robotics Otter USV, see www.maritimerobotics.com. 
# The length of the USV is L = 2.0 m, while the state vector is defined as:
#
#  u:     surge velocity          (m/s)
#  v:     sway velocity           (m/s)
#  w:     heave velocity          (m/s)
#  p:     roll velocity           (rad/s)
#  q:     pitch velocity          (rad/s)
#  r:     yaw velocity            (rad/s)
#  x:     position in x direction (m)
#  y:     position in y direction (m)
#  z:     position in z direction (m)
#  phi:   roll angle              (rad)
#  theta: pitch angle             (rad)
#  psi:   yaw angle               (rad)
#
# The other inputs are:
#
# n = [ n(1) n(2) ]' where
#    n(1): propeller shaft speed, left (rad/s)
#    n(2): propeller shaft speed, right (rad/s)
#
#  mp = payload mass (kg), maximum 45 kg
#  rp = [xp, yp, zp]' (m) is the location of the payload w.r.t. the CO
#  V_c:     current speed (m/s)
#  beta_c:  current direction (rad)
#
# See, ExOtter.m and demoOtterUSVHeadingControl.slx
#
# Author:    Thor I. Fossen
# Date:      2019-07-17
# Revisions: 2021-04-25 added call to new function crossFlowDrag.m
#            2021-06-21 Munk moment in yaw is neglected
#            2021-07-22 Added a new state for the trim moment
#            2021-12-17 New method Xudot = -addedMassSurge(m,L,rho) 
#            2023-03-28 Trim state is replaced by payload mp and rp



    if len(x) != 12:
        raise ValueError('x vector must have dimension 12!')
    if len(n) != 2:
        raise ValueError('n vector must have dimension 2!')

    mp = 0                          # payload mass (kg)
    rp = np.array([0, 0, 0])        # payload location (m) wrt CO
    # V_c = 0                       # current speed (m/s)
    # beta_c = 1                    # current direction (rad)

    # main parameters
    g = 9.81                        # acceleration of gravity (m/s^2)
    rho = 1025                      # density of water (kg/m^3)
    L = 2.0                         # length (m)
    B = 1.08                        # beam (m)
    m = 55.0                        # mass (kg)
    # m = 250.18                        # mass (kg), from vrx simulation, added by yui Feb 1 2025
    rg = np.array([0.2, 0, -0.2])   # CG for hull only (m)
    R44 = 0.4 * B                   # radii of gyrations (m)  
    R55 = 0.25 * L   
    R66 = 0.25 * L 
    T_yaw = 1                       # time constan in yaw (s)
    Umax = 6 * 0.5144               # maximum forwardspeed (m/s)  

    # Data for one pontoon
    B_pont = 0.25                   # beam of one pontoon (m)
    y_pont = 0.395                  # distance from centerline to waterarea center (m)
    Cw_pont = 0.75                  # waveline area coefficient (-)
    Cb_pont = 0.4                   # block coefficient, computed from m = 55kg

    #############################################################
    
    # State and current variables   
    nu = x[:6]                          # velocity (m/s)    
    nu1 = x[:3]                         # [u, v, w] (m/s)   
    nu2 = x[3:6]                        # [p, q, r] (rad/s)
    eta = x[6:]                         # [x, y, z, phi, theta, psi], position and orientation
    U = np.sqrt(np.sum(nu[:3]**2))      # speed (m/s)
    u_c = V_c * np.cos(beta_c - eta[5])             # current speed in surge direction (m/s)
    v_c = V_c * np.sin(beta_c - eta[5])             # current speed in sway direction (m/s)   
    nu_r = nu - np.array([u_c, v_c, 0, 0, 0, 0])    # relative velocity vector    

    # Inertia dyadic, volume displacement, draft
    nabla = (m + mp) / rho                              # volume displacement (m^3)
    T = nabla / (2 * Cb_pont * B_pont * L)              # draft (m)
    Ig_CG = m * np.diag([R44**2, R55**2, R66**2])       # only hull in CG (kg m^2)
    rg = (m * rg + mp * rp) / (m + mp)                  # CG location corrected for payload (m)
    smtrx_rg_sq = np.linalg.matrix_power(Smtrx(rg), 2)
    smtrx_rp_sq = np.linalg.matrix_power(Smtrx(rp), 2)
    Ig = Ig_CG - m * smtrx_rg_sq - mp * smtrx_rp_sq     # hull + payload in CO (kg m^2)

    # Experitmental propeller data including lever arms
    l1 = -y_pont                                        # lever arm, left propeller (m)     
    l2 = y_pont                                         # lever arm right propeller (m)        
    k_pos = 0.02216 / 2                                 # Positive Bollard, one propeller
    k_neg = 0.01289 / 2                                 # Negative Bollard, one propeller
    n_max = np.sqrt((0.5 * 24.4 * g) / k_pos)           # maximum propeller rev. (rad/s)        
    n_min = -np.sqrt((0.5 * 13.6 * g) / k_neg)          # minimum propeller rev. (rad/s)

    # MRB and CRB matrices (Fossen 2021)
    I3 = np.eye(3)
    O3 = np.zeros((3, 3))

    MRB_CG = np.block([[ (m + mp) * I3, O3], 
                       [O3, Ig]])
    CRB_CG = np.block([[(m + mp) * Smtrx(nu2), O3], 
                       [O3, -Smtrx(Ig @ nu2)]])

    H = Hmtrx(rg)           # Transform MRB and CRB from CG to CO
    MRB = H.T @ MRB_CG @ H
    CRB = H.T @ CRB_CG @ H
    # CRB = m2c(MRB, nu_r)

    # Hydrodynamic added mass (best practice)
    Xudot = -addedMassSurge(m, L, rho)[0]
    # Xudot = -0.1 * m
    Yvdot = -1.5 * m
    Zwdot = -1.0 * m
    Kpdot = -0.2 * Ig[0, 0]
    Mqdot = -0.8 * Ig[1, 1]
    Nrdot = -1.7 * Ig[2, 2]

    MA = -np.diag([Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot])
    CA  = m2c(MA, nu_r)
    CA[5, 0] = 0    # Assume that the Munk moment in yaw can be neglected
    CA[5, 1] = 0    # These terms, if nonzero, must be balanced by adding nonlinear damping
    CA[0, 5] = 0
    CA[1, 5] = 0

    # System mass and Coriolis-centripetal matrices
    M = MRB + MA
    C = CRB + CA
    # C = m2c(M, nu_r)

    # Hydrostatic quantities (Fossen 2021)
    Aw_pont = Cw_pont * L * B_pont       # waterline area, one pontoon
    I_T = 2 * (1 / 12) * L * B_pont**3 * (6 * Cw_pont**3 / ((1 + Cw_pont) * (1 + 2 * Cw_pont))) + 2 * Aw_pont * y_pont**2
    I_L = 0.8 * 2 * (1 / 12) * B_pont * L**3
    KB = (1 / 3) * (5 * T / 2 - 0.5 * nabla / (L * B_pont))
    BM_T = I_T / nabla      # BM values
    BM_L = I_L / nabla
    KM_T = KB + BM_T        # KM values
    KM_L = KB + BM_L
    KG = T - rg[2]
    GM_T = KM_T - KG        # GM values
    GM_L = KM_L - KG

    G33 = rho * g * (2 * Aw_pont)   # spring stiffness
    G44 = rho * g * nabla * GM_T
    G55 = rho * g * nabla * GM_L

    G_CF = np.diag([0, 0, G33, G44, G55, 0])    # CF spring stiffness
    LCF = -0.2
    H = Hmtrx([LCF, 0, 0])                      # transform G_CF from CF to CO
    G = H.T @ G_CF @ H

    # Natural frequencies
    w3 = np.sqrt(G33 / M[2, 2])
    w4 = np.sqrt(G44 / M[3, 3])
    w5 = np.sqrt(G55 / M[4, 4])

    # Linear hydrodynamic damping (hydrodynamic derivatives)
    Xu = -24.4 * g / Umax           # specified using the maximum speed  
    Yv = 0
    Zw = -2 * 0.3 * w3 * M[2, 2]    # specified using relative damping factors
    Kp = -2 * 0.2 * w4 * M[3, 3]
    Mq = -2 * 0.4 * w5 * M[4, 4]
    Nr = -M[5, 5] / T_yaw           # specified using the time constant in T_yaw


    # Control forces and moments - with propeller revolution saturation
    # thrust = np.zeros(2)
    thrust = n.copy()
    # for i in range(0, 2):

    #     n[i] = sat(n[i], n_min, n_max)  # saturation, physical limits

    #     if n[i] > 0:    # positive thrust (N)
    #         thrust[i] = k_pos * n[i] * abs(n[i])
    #     else:           # negative thrust (N)
    #         thrust[i] = k_neg * n[i] * abs(n[i])
    
    # Control forces and moments
    tau = np.array([thrust[0] + thrust[1], 0, 0, 0, 0, -l1 * thrust[0] - l2 * thrust[1]])

    Xh = Xu * nu_r[0]
    Yh = Yv * nu_r[1]
    Zh = Zw * nu_r[2]
    Kh = Kp * nu_r[3]
    Mh = Mq * nu_r[4]
    Nh = Nr * (1 + 10 * abs(nu_r[5])) * nu_r[5]

    tau_damp = np.array([Xh, Yh, Zh, Kh, Mh, Nh])

    # Strip theory: cross-flow drag integrals for Yh and Nh
    tau_crossflow = crossFlowDrag(L, B_pont, T, nu_r)

    # Payload force and moment expressed in BODY
    # R = Rzyx(eta[3], eta[4], eta[5])
    # f_payload = np.matmul(R.T, np.array([0, 0, mp * g], float))              
    # m_payload = np.matmul(Smtrx(rp), f_payload)
    # g_0 = np.array([ f_payload[0],f_payload[1],f_payload[2], 
    #                  m_payload[0],m_payload[1],m_payload[2] ])

    # Kinematics
    J = eulerang(eta[3], eta[4], eta[5])
    # [tau]
    # [tau_damp]
    # [tau_crossflow]
    # Time derivative of the state vector - numerical integration; see ExOtter.m  
    xdot = np.concatenate([np.linalg.inv(M) @ (tau + tau_damp + tau_crossflow - C @ nu_r - G @ eta), J @ nu])
    
    return xdot