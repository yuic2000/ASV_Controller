#include "../../include/mpc/utils/eMPCCost.h"

void eMPCCost(MatrixXd& M, VectorXd& q, const MatrixXd& xid, const Param& param){
    /*
    Compute the cost function matrices for an extended MPC setup.

    Parameters:
    Q, R, P : Cost matrices.
    xid: Reference trajectory.
    param: Parameters with attributes Nt, Nx, Nu.
    solver: Solver information or configuration (not used directly here).

    Returns:
    M: The cost matrix.
    q: The linear cost vector.
    solver: Possibly returned unchanged.
    */
    
    const int Nx = param.Nx;
    const int Nu = param.Nu;
    const int Nt = param.Nt;

    const MatrixXd Q = param.Q;
    const MatrixXd R = param.R;
    const MatrixXd P = param.P;


    for (int k = 0; k < Nt - 1; ++k) {
        MatrixXd C = MatrixXd::Identity(12, 12);
        C.block<6, 6>(6, 0) = -adjoint_(xid.row(k).transpose());

        M.block(k * Nx, k * Nx, Nx, Nx) = C.transpose() * Q * C;    // start at (k*Nx, k*Nx) with size Nx x Nx

        VectorXd b = VectorXd::Zero(12);
        b.segment<6>(6) = xid.row(k).transpose();
        q.segment(k * Nx, Nx) = -C.transpose() * Q * b;
    }

    // Handle the final stage k = Nt-1
    int k = Nt - 1;
    MatrixXd C = MatrixXd::Identity(12, 12);
    C.block<6, 6>(6, 0) = -adjoint_(xid.row(k).transpose());

    VectorXd b = VectorXd::Zero(12);
    b.segment<6>(6) = xid.row(k).transpose();

    b = C.transpose() * P * b;

    M.block(k * Nx, k * Nx, Nx, Nx) = C.transpose() * P * C;
    q.segment(k * Nx, Nx) = -b;

    for (int j = 1; j <= Nt; ++j) {
        int start_index = (k + 1) * Nx + (j - 1) * Nu;
        M.block(start_index, start_index, Nu, Nu) = R;
    }

}