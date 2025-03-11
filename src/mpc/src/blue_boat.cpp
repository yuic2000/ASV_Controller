#include "../include/mpc/blue_boat.h"


VectorXd blue_boat(const VectorXd& x, const VectorXd& n, double V_c, double beta_c) {

    // [xdot,U] = otter(x,n,mp,rp,V_c,beta_c) returns the speed U in m/s (optionally) 
    // and the time derivative of the state vector: 
    //    x = [ u v w p q r x y z phi theta psi ]' 
    // for the Maritime Robotics Otter USV, see www.maritimerobotics.com. 
    // The length of the USV is L = 2.0 m, while the state vector is defined as:
    // 
    //  u:     surge velocity          (m/s)
    //  v:     sway velocity           (m/s)
    //  w:     heave velocity          (m/s)
    //  p:     roll velocity           (rad/s)
    //  q:     pitch velocity          (rad/s)
    //  r:     yaw velocity            (rad/s)
    //  x:     position in x direction (m)
    //  y:     position in y direction (m)
    //  z:     position in z direction (m)
    //  phi:   roll angle              (rad)
    //  theta: pitch angle             (rad)
    //  psi:   yaw angle               (rad)
    //
    // The other inputs are:
    //
    // n = [ n(1) n(2) ]' where
    //     n(1): propeller shaft speed, left (rad/s)
    //     n(2): propeller shaft speed, right (rad/s)
    // 
    // mp = payload mass (kg), maximum 45 kg
    // rp = [xp, yp, zp]' (m) is the location of the payload w.r.t. the CO
    // V_c:     current speed (m/s)
    // beta_c:  current direction (rad)

    // Input validation
    if (x.size() != 12) {
        throw std::invalid_argument("x vector must have dimension 12!");
    }
    if (n.size() != 2) {
        throw std::invalid_argument("n vector must have dimension 2!");
    }

    // Initialize parameters
    double mp = 0.0;                           // payload mass (kg)
    Vector3d rp(0.0, 0.0, 0.0);                // payload location (m) wrt CO
    // double V_c = 0.0;                       // current speed (m/s)
    // beta_c = 1.0;                           // current direction (rad)

    // Main parameters
    const double g = 9.81;                      // acceleration of gravity (m/s^2)
    const double rho = 1025.0;                  // density of water (kg/m^3)
    // for otter simulation model
    // const double L = 2.0;                    // length (m)
    // const double B = 1.08;                   // beam (m)
    // const double m = 55.0;                   // mass (kg), for otter simulation model

    // for blueboat CAD model
    const double L = 1.1195;                    // length (m)
    const double B = 0.93;                      // beam (m)
    const double m = 28.63;                     // mass (kg), for blueboat CAD model
    // const double m = 17.65;                     // mass (kg), for blueboat CAD model

    Vector3d rg_vec(0.2, 0.0, -0.2);            // CG for hull only (m), for otter simulation model
    // Vector3d rg_vec(0.3769, 0.00872, -0.617);      // CG for hull only (m), for blueboat CAD model
    double R44 = 0.4 * B;                       // radii of gyrations (m)
    double R55 = 0.25 * L;
    double R66 = 0.25 * L;
    double T_yaw = 1.0;                         // time constant in yaw (s)
    double Umax = 6.0 * 0.5144;                 // maximum forwardspeed (m/s)

    // // Data for one pontoon, for otter simulation model
    // double B_pont = 0.25;                       // beam of one pontoon (m)
    // double y_pont = 0.395;                      // distance from centerline to waterarea center (m), for otter simulation model
    // double Cw_pont = 0.75;                      // waveline area coefficient (-)
    // double Cb_pont = 0.4;                       // block coefficient

    // Data for one pontoon, for blueboat model
    double B_pont = 0.18;                       // beam of one pontoon (m)
    double y_pont = 0.285;                      // distance from centerline to waterarea center (m), for blueboat model
    double Cw_pont = 0.75;                      // waveline area coefficient (-)
    double Cb_pont = 0.4;                       // block coefficient

    /////////////////////////////////////////////////////////////////

    // State and current variables
    VectorXd nu = x.segment(0, 6);       // velocity (m/s)
    Vector3d nu1 = nu.segment(0, 3);     // [u, v, w] (m/s)
    Vector3d nu2 = nu.segment(3, 3);     // [p, q, r] (rad/s)
    VectorXd eta = x.segment(6, 6);      // [x, y, z, phi, theta, psi]
    double U = nu1.norm();                       // speed (m/s)

    double u_c = V_c * std::cos(beta_c - eta(5)); // current speed in surge direction (m/s)
    double v_c = V_c * std::sin(beta_c - eta(5)); // current speed in sway direction (m/s)
    VectorXd nu_r = nu;
    nu_r(0) = nu(0) - u_c;
    nu_r(1) = nu(1) - v_c;
    // nu_r(2),  nu_r(3), nu_r(4), nu_r(5) remains the same with nu

    // Inertia dyadic, volume displacement, draft
    double nabla = (m + mp) / rho;                        // volume displacement (m^3)
    double T = nabla / (2.0 * Cb_pont * B_pont * L);      // draft (m)
    Matrix3d Ig_CG = Matrix3d::Identity();
    Ig_CG(0, 0) = m * R44 * R44;
    Ig_CG(1, 1) = m * R55 * R55;
    Ig_CG(2, 2) = m * R66 * R66;

    rg_vec = (m * rg_vec + mp * rp) / (m + mp);          // CG location corrected for payload (m)
    Matrix3d smtrx_rg_sq = Smtrx(rg_vec) * Smtrx(rg_vec);
    Matrix3d smtrx_rp_sq = Smtrx(rp) * Smtrx(rp);
    Matrix3d Ig = Ig_CG - m * smtrx_rg_sq - mp * smtrx_rp_sq; // hull + payload in CO (kg m^2)

    // Experimental propeller data including lever arms
    double l1 = -y_pont;                                // lever arm, left propeller (m)
    double l2 = y_pont;                                 // lever arm, right propeller (m)
    double k_pos = 0.02216 / 2.0;                       // Positive Bollard, one propeller
    double k_neg = 0.01289 / 2.0;                       // Negative Bollard, one propeller
    double n_max = std::sqrt((0.5 * 24.4 * g) / k_pos); // maximum propeller rev. (rad/s)
    double n_min = -std::sqrt((0.5 * 13.6 * g) / k_neg); // minimum propeller rev. (rad/s)

    // MRB and CRB matrices
    Matrix3d I3 = Matrix3d::Identity();
    Matrix3d O3 = Matrix3d::Zero();

    MatrixXd MRB_CG(6, 6);
    MRB_CG << (m + mp) * I3, O3,
              O3, Ig;

    MatrixXd CRB_CG(6, 6);
    CRB_CG << (m + mp) * Smtrx(nu2), O3,
              O3, -Smtrx(Ig * nu2);
    
    MatrixXd H = Hmtrx(rg_vec);                  // Transform MRB and CRB from CG to CO
    MatrixXd MRB = H.transpose() * MRB_CG * H;
    MatrixXd CRB = H.transpose() * CRB_CG * H;
    // CRB = m2c(MRB, nu_r); 

    // Hydrodynamic added mass (best practice)
    double Xudot = -addedMassSurge(m, L, rho).first;  
    // double Xudot = -0.1 * m;
    double Yvdot = -1.5 * m;
    double Zwdot = -1.0 * m;
    double Kpdot = -0.2 * Ig(0, 0);
    double Mqdot = -0.8 * Ig(1, 1);
    double Nrdot = -1.7 * Ig(2, 2);

    MatrixXd MA = MatrixXd::Zero(6, 6);
    MA.diagonal() << -Xudot, -Yvdot, -Zwdot, -Kpdot, -Mqdot, -Nrdot;
    MatrixXd CA = m2c(MA, nu_r);
    CA(5, 0) = 0.0;    // Assume that the Munk moment in yaw can be neglected
    CA(5, 1) = 0.0;    // These terms, if nonzero, must be balanced by adding nonlinear damping
    CA(0, 5) = 0.0;
    CA(1, 5) = 0.0;

    // System mass and Coriolis-centripetal matrices
    MatrixXd M = MRB + MA;
    MatrixXd C = CRB + CA;
    // C = m2c(M, nu_r);

    // Hydrostatic quantities (Fossen 2021)
    double Aw_pont = Cw_pont * L * B_pont;        // waterline area, one pontoon
    double I_T = 2.0 * (1.0 / 12.0) * L * std::pow(B_pont, 3) * (6.0 * std::pow(Cw_pont, 3) / ((1.0 + Cw_pont) * (1.0 + 2.0 * Cw_pont))) 
                 + 2.0 * Aw_pont * std::pow(y_pont, 2);
    double I_L = 0.8 * 2.0 * (1.0 / 12.0) * B_pont * std::pow(L, 3);
    double KB = (1.0 / 3.0) * (5.0 * T / 2.0 - 0.5 * nabla / (L * B_pont));
    double BM_T = I_T / nabla;      // BM values
    double BM_L = I_L / nabla;
    double KM_T = KB + BM_T;        // KM values
    double KM_L = KB + BM_L;
    double KG = T - rg_vec(2);
    double GM_T = KM_T - KG;        // GM values
    double GM_L = KM_L - KG;

    double G33 = rho * g * (2.0 * Aw_pont);   // spring stiffness
    double G44 = rho * g * nabla * GM_T;
    double G55 = rho * g * nabla * GM_L;

    MatrixXd G_CF = MatrixXd::Zero(6,6);
    G_CF(2,2) = G33;
    G_CF(3,3) = G44;
    G_CF(4,4) = G55;                          // CF spring stiffness
    double LCF_val = -0.2;
    Vector3d LCF_vec(LCF_val, 0.0, 0.0);
    H = Hmtrx(LCF_vec);                        // transform G_CF from CF to CO
    MatrixXd G = H.transpose() * G_CF * H;

    // Natural frequencies
    double w3 = std::sqrt(G33 / M(2, 2));
    double w4 = std::sqrt(G44 / M(3, 3));
    double w5 = std::sqrt(G55 / M(4, 4));

    // Linear hydrodynamic damping (hydrodynamic derivatives)
    double Xu = -24.4 * g / Umax;                  // specified using the maximum speed  
    double Yv = 0.0;
    double Zw = -2.0 * 0.3 * w3 * M(2, 2);         // specified using relative damping factors
    double Kp = -2.0 * 0.2 * w4 * M(3, 3);
    double Mq = -2.0 * 0.4 * w5 * M(4, 4);
    double Nr = -M(5, 5) / T_yaw;                  // specified using the time constant in T_yaw

    // Control forces and moments - with propeller revolution saturation
    Vector2d thrust = n;                        // thrust = n.copy()
    
    // for(int i = 0; i < 2; ++i) {
    //     double n_sat = sat(n(i), n_min, n_max);  // saturation, physical limits
    //     if(n_sat > 0) {
    //         thrust(i) = k_pos * n(i) * std::abs(n(i)); // positive thrust (N)
    //     } else {
    //         thrust(i) = k_neg * n(i) * std::abs(n(i)); // negative thrust (N)
    //     }
    // }
    

    // Control forces and moments
    VectorXd tau(6);
    tau << thrust(0) + thrust(1), 0.0, 0.0, 0.0, 0.0, -l1 * thrust(0) - l2 * thrust(1);

    double Xh = Xu * nu_r(0);
    double Yh = Yv * nu_r(1);
    double Zh = Zw * nu_r(2);
    double Kh = Kp * nu_r(3);
    double Mh = Mq * nu_r(4);
    double Nh = Nr * (1.0 + 10.0 * std::abs(nu_r(5))) * nu_r(5);

    VectorXd tau_damp(6);
    tau_damp << Xh, Yh, Zh, Kh, Mh, Nh;

    // Strip theory: cross-flow drag integrals for Yh and Nh
    VectorXd tau_crossflow = crossFlowDrag(L, B_pont, T, nu_r);

    // Payload force and moment expressed in BODY
    // Matrix3d R = Rzyx(eta(3), eta(4), eta(5));
    // Vector3d f_payload = R.transpose() * Vector3d(0, 0, mp * g);
    // Vector3d m_payload = Smtrx(rp) * f_payload;
    // VectorXd g_0(6);
    // g_0 << f_payload, m_payload;

    // Kinematics
    MatrixXd J = eulerang(eta(3), eta(4), eta(5));
    // [tau]
    // [tau_damp]
    // [tau_crossflow]
    // Time derivative of the state vector - numerical integration; see ExOtter.m  
    VectorXd xdot_part = M.inverse() * (tau + tau_damp + tau_crossflow - C * nu_r - G * eta);
    VectorXd xdot = VectorXd::Zero(12);
    xdot << xdot_part, J * nu;

    // Compute speed U
    // double U = nu1.norm(); // Already computed earlier

    // return BlueBoatOutput{ xdot, U };

    // std::cout << "xdot: " << xdot.transpose() << std::endl;
    return xdot;

}