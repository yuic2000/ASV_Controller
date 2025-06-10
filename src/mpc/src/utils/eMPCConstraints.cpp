#include "../../include/mpc/utils/eMPCConstraints.h"

void eMPCConstraints(MatrixXd &A, VectorXd &bmin, VectorXd &bmax, const MatrixXd& xid, const VectorXd& p0, const VectorXd& x0, const Param& param) {
// std::tuple<MatrixXd, VectorXd, VectorXd> eMPCConstraints(const MatrixXd& xid, const VectorXd& p0, const VectorXd& x0, const Param& param) {
    
    /*
    min x'Qx + u'Ru
    xkp1 = Ak xk + Bk uk + bk, k = 0,1, ..., N-1
    x0 = x_init,
    umin < uk < umax, k = 0, 1, ..., N-1
    */
    
    const MatrixXd I = param.I;
    const double dt = param.dt;
    
    const int Nx = param.Nx;
    const int Nu = param.Nu;
    const int Nt = param.Nt;
    const int Noff = Nx * (Nt + 1);
    
    MatrixXd D = MatrixXd::Zero(6, 6);  // Kp, Mq, Nr, Xu, Yv, Zw
    D.diagonal() << -300, -900, -800, -100, -100, -500;                                               // for vrx wamv
    // D.diagonal() << -12.2948, -45.5947, -12.3312 * (1 + 10 * abs(x0(9))), -77.5544, 0, -250.3037;     // for blueboat model, 28.63 kg
    
    MatrixXd I_inv = I.inverse();
    
    for (int k = 0; k < Nt; ++k) {
        MatrixXd D_t = D;
        // D_t(2, 2) -= 12.3312 * (10 * abs(xid(k, 2)));           // for blueboat model

        // for blueboat CAD model, 28.63 kg
        MatrixXd Irb(6, 6);
        Irb << 250.18,    0,     0,      0,      0,        0,
                 0,    250.18,   0,      0,      0,        0,
                 0,       0,   250.18,   0,      0,        0,
                 0,       0,     0,    225.17,  0.12,   0.99,
                 0,       0,     0,     0.12,  454.1,   0.29,
                 0,       0,     0,     0.99,   0.29, 499.75;
        // Irb << 28.63,      0,      0,     0,   -17.66, -0.25,
        //        0,       28.63,     0,   17.66,    0,   10.79,
        //        0,          0,    28.63,  0.25, -10.79,     0,
        //        0,        17.66,   0.25, 13.87,  -0.35,  6.55,
        //       -17.66,      0,   -10.79, -0.35,  18.62,  0.13,
        //       -0.25,    10.79,     0,    6.55,   0.13,  7.13;

        MatrixXd M11 = Irb.block<3, 3>(0, 0);
        MatrixXd M12 = Irb.block<3, 3>(0, 3);
        MatrixXd M21 = Irb.block<3, 3>(3, 0);
        MatrixXd M22 = Irb.block<3, 3>(3, 3);

        Vector3d xi_w = xid.row(k).segment<3>(0);
        Vector3d xi_v = xid.row(k).segment<3>(3);

        MatrixXd G = MatrixXd::Zero(6, 6);
        G.block<3, 3>(0, 0) = skew(xi_w) * M22 + skew(xi_v) * M12;
        G.block<3, 3>(0, 3) = skew(xi_v) * M11 + skew(xi_w) * M21;
        G.block<3, 3>(3, 0) = skew(xi_w) * M12;
        G.block<3, 3>(3, 3) = skew(xi_w) * M11;

        MatrixXd Crb = MatrixXd::Zero(6, 6);
        Crb.block<3, 3>(0, 0) = skew(M21 * xi_v) + skew(M22 * xi_w);
        Crb.block<3, 3>(0, 3) = skew(M11 * xi_v) + skew(M12 * xi_w);
        Crb.block<3, 3>(3, 0) = skew(M11 * xi_v) + skew(M12 * xi_w);

        MatrixXd Iam = MatrixXd::Zero(6, 6);
        // Iam.diagonal() << 5.678, 42.945, 28.63, 0.594, 2.928, 5.202;      // from blueboat CAD model, 28.63 kg

        MatrixXd Mam11 = Iam.block<3, 3>(0, 0);
        MatrixXd Mam22 = Iam.block<3, 3>(3, 3);

        MatrixXd Cam = MatrixXd::Zero(6, 6);
        Cam.block<3, 3>(0, 0) = skew(Mam22 * xi_w);
        Cam.block<3, 3>(0, 3) = skew(Mam11 * xi_v);
        Cam(2, 3) = 0;
        Cam(2, 4) = 0;
        Cam.block<3, 3>(3, 0) = skew(Mam11 * xi_v);
        Cam(3, 2) = 0;
        Cam(4, 2) = 0;

        MatrixXd G2 = MatrixXd::Zero(6, 6);
        G2.block<3, 3>(0, 0) = skew(xi_w) * Mam22;
        G2.block<3, 3>(0, 3) = skew(xi_v) * Mam11;
        G2.block<3, 3>(3, 3) = skew(xi_w) * Mam11;
        G2(2, 3) = 0;
        G2(2, 4) = 0;
        G2(3, 5) = 0;
        G2(4, 5) = 0;

        Cam = -Cam;
        Crb = -Crb;
        
        MatrixXd H = I_inv * (Crb + Cam + G + G2 + D_t);
        
        MatrixXd Ac = MatrixXd::Zero(12, 12);
        Ac.block<6, 6>(0, 0) = -adjoint_(xid.row(k).transpose()); 
        Ac.block<6, 6>(0, 6) = -MatrixXd::Identity(6, 6);
        Ac.block<6, 6>(6, 6) = H;
        
        MatrixXd Bc = MatrixXd::Zero(12, 2);
        Bc.block<6, 2>(6, 0) = I_inv * (MatrixXd(6, 2) << 0, 0, 0, 0, 1.0, -1.0, 1, 1, 0, 0, 0, 0).finished();      // for vrx wamv
        // Bc.block<6, 2>(6, 0) = I_inv * (MatrixXd(6, 2) << 0, 0, 0, 0, 0.285, -0.285, 1, 1, 0, 0, 0, 0).finished();  // for blueboat model
        
        VectorXd b = (G + G2) * x0.segment<6>(7);
        // b(2) += 12.3312 * (10 * abs(xid(k, 2))) * xid(k, 2);     // for blueboat model
        b = -I_inv * b;
        VectorXd hc(12);
        hc << xid.row(k).transpose(), b;

        
        MatrixXd Ad = MatrixXd::Identity(Nx, Nx) + Ac * dt;
        MatrixXd Bd = dt * Bc;
        VectorXd hd = dt * hc;
        
        A.block(Nx * k, Nx * k, Nx, Nx) = -Ad;
        A.block(Nx * k, Nx * (k + 1), Nx, Nx) = MatrixXd::Identity(Nx, Nx);
        A.block(Nx * k, Noff + Nu * k, Nx, Nu) = -Bd;
        bmin.segment(Nx * k, Nx) = hd;

    }
    A.block(Nt * Nx, 0, Nx, Nx) = MatrixXd::Identity(Nx, Nx);
    
    
    for (int k = 0; k < Nt; ++k) {
        A.block(Noff + Nu * k, Noff + Nu * k, Nu, Nu) = MatrixXd::Identity(Nu, Nu);
    }

    bmin.segment(Nt * Nx, Nx) = p0;


    bmax = bmin;

    const VectorXd umax = param.umax;
    const VectorXd umin = param.umin;

    bmax.segment((Nt + 1) * Nx, Nu * Nt) = umax.replicate(Nt, 1);
    bmin.segment((Nt + 1) * Nx, Nu * Nt) = umin.replicate(Nt, 1);

}
