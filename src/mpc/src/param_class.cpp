#include "../include/mpc/param_class.h"

using namespace Eigen;
using namespace std;

// Param class constructor
Param::Param() {
    mode = 2;               // 1: zigzag, 2: circling
    dt = 0.05;              // MPC simulation time step, model: 20Hz, Gt: 25 Hz
    Nx = 12; 
    Nu = 2; 
    Nt = 100; 
    Nsim = (int)(ceil(65.0 / dt));
    V_c = 0.5;
    beta_c = 1.0;

    I = MatrixXd(6, 6); 
    I << 17.1572, 0, 4.4, 0, 11, 0,
         0, 37.07, 0, -11, 0, -11,
         4.4, 0, 45.265, 0, 11, 0,
         0, -11, 0, 60.2815, 0, 0,
         11, 0, 11, 0, 137.5, 0,
         0, -11, 0, 0, 0, 110;

    xmax = VectorXd::Constant(Nx, numeric_limits<double>::infinity()); 
    xmin = -xmax; 
    umax = VectorXd::Constant(Nu, 115); 
    umin = -VectorXd::Constant(Nu, 65);

    // cost matrices
    diag_values_cost = VectorXd(12);
    diag_values_cost << 1e-5, 1e-5, 0.5, 10, 10, 1e-5, 1e-5, 1e-5, 1e-5, 1, 1e-5, 1e-5;
    // diag_values_cost << 1e-5, 1e-5, 5, 100, 100, 1e-5, 1e-5, 1e-5, 1e-5, 10, 1e-5, 1e-5;
    P = diag_values_cost.asDiagonal() * 10;
    Q = diag_values_cost.asDiagonal() * 1;
    R = MatrixXd::Identity(2, 2) * 1e-5;
}

// Param class member function
void Param::set_reference_trajectory() {
    Vector4d q0_ref(1, 0, 0, 0);    // desired quaternion
    Vector3d p0_ref(0, 0, 0);       // desired position
    Vector3d w0_ref(0, 0, 0.1);     // desired yaw velocity, 0.1 rad/s
    Vector3d v0_ref(0.5, 0, 0);     // desired surge velocity, 0.5 m/s
    VectorXd x0_ref(7);
    x0_ref << q0_ref, p0_ref;
    VectorXd xid_ref(6);
    xid_ref << w0_ref, v0_ref;

    Matrix4d X = Matrix4d::Identity();
    MatrixXd X_ref(1, 7);
    X_ref.row(0) = x0_ref.transpose();
    MatrixXd xi_ref(1, 6);
    xi_ref.row(0) = xid_ref.transpose();

    for (int i = 0; i < Nsim; ++i) {
        VectorXd xid_ref_rt = xid_ref;

        if (mode == 1) {
            xid_ref_rt(2) = 0.1 * cos(i / 100.0);
        }

        Matrix4d Xi = Matrix4d::Zero();
        Xi.block<3, 3>(0, 0) = skew(xid_ref_rt.head<3>());
        Xi.block<3, 1>(0, 3) = xid_ref_rt.tail<3>();

        X = X * (Xi * dt).exp();
        Quaterniond new_quat(X.block<3, 3>(0, 0));
        VectorXd new_state(7);
        new_state << new_quat.coeffs(), X.block<3, 1>(0, 3);
        X_ref.conservativeResize(X_ref.rows() + 1, NoChange);
        X_ref.row(X_ref.rows() - 1) = new_state.transpose();
        xi_ref.conservativeResize(xi_ref.rows() + 1, NoChange);
        xi_ref.row(xi_ref.rows() - 1) = xid_ref_rt.transpose();
    }

    this->X_ref = X_ref;
    this->xi_ref = xi_ref;
}

// Logger class constructor
Logger::Logger(int Nsim, int Nt) {
    X = MatrixXd::Zero(13, Nsim - Nt + 1);  // +1 for the initial state
    Err = MatrixXd::Zero(6, Nsim - Nt);
    U = MatrixXd::Zero(2, Nsim - Nt);
    Err2 = MatrixXd::Zero(4, Nsim - Nt);
    taken_time = 0.0;
}
