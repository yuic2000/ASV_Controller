#include "../../include/mpc/controller/eMPC.h"
 

VectorXd eMPC(const VectorXd& x0, const VectorXd& xd, const MatrixXd& xid,
    const Param& param){

    Eigen::setNbThreads(4); // Use 4 threads
    
    const int Nx = param.Nx;
    const int Nu = param.Nu;
    const int Nt = param.Nt;
    const int n_total = (Nt + 1) * Nx + Nt * Nu;    // 1412

// start
    // auto timer0 = std::chrono::high_resolution_clock::now();

    MatrixXd Xd = MatrixXd::Zero(4, 4);
    Xd.block<3, 3>(0, 0) = quat2rotm(xd.head<4>());
    Xd.block<3, 1>(0, 3) = xd.segment<3>(4);
    Xd(3, 3) = 1.0;

    MatrixXd X00 = MatrixXd::Zero(4, 4);
    X00.block<3, 3>(0, 0) = quat2rotm(x0.head<4>());
    X00.block<3, 1>(0, 3) = x0.segment<3>(4);
    X00(3, 3) = 1.0;

    X00 = X00.inverse() * Xd;
    MatrixXd X0 = X00.log();

    VectorXd p0(12);
    p0 << X0(2, 1), X0(0, 2), X0(1, 0),
           X0.block<3,1>(0, 3),
           x0.segment(7, 6);


    // eMPCConstraints
    MatrixXd A = MatrixXd::Zero(n_total, n_total);          // 1412 x 1412
    VectorXd bmin = VectorXd::Zero(n_total);                // 1412
    VectorXd bmax = VectorXd::Zero(n_total);                // 1412
    // MatrixXd A = MatrixXd::Zero(n_total + 1, n_total);     // 1413 x 1413
    // VectorXd bmin = VectorXd::Zero(n_total + 1);           // 1413
    // VectorXd bmax = VectorXd::Zero(n_total + 1);           // 1413
    // MatrixXd A = MatrixXd::Zero(n_total + Nt, n_total);     // 1413 x 1413
    // VectorXd bmin = VectorXd::Zero(n_total + Nt);           // 1413
    // VectorXd bmax = VectorXd::Zero(n_total + Nt);           // 1413
    eMPCConstraints(A, bmin, bmax, xid, p0, x0, param);     // find A, bmin, bmax via eMPCConstraints


    // eMPCCost
    MatrixXd M = MatrixXd::Zero(n_total, n_total);          // 1412 x 1412
    VectorXd q = VectorXd::Zero(n_total);                   // 1412
    eMPCCost(M, q, xid, param);                             // find M, q via eMPCCost


    // Setup OSQP problem
    OSQPSolver*   solver   = nullptr;
    OSQPSettings *settings = (OSQPSettings *)malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(settings);
    if (settings) {
        osqp_set_default_settings(settings);
        settings->verbose = false;
    }

    OSQPInt exitflag = 0;

    // Convert matrices to sparse matrices
    M = M.triangularView<Eigen::Upper>();

    SparseMatrix<double> M_sparse = M.sparseView();         // 1412 x 1412
    SparseMatrix<double> A_sparse = A.sparseView();         // 1412 x 1412

    // Populate OSQPData with M_sparse, q, A_sparse, l, u
    OSQPCscMatrix P_csc;
    OSQPCscMatrix A_csc;
    eigen2CscMatrix(P_csc, M_sparse);       
    eigen2CscMatrix(A_csc, A_sparse);

    OSQPInt m = A_sparse.rows();
    OSQPInt n = M_sparse.cols();

    // Setup workspace with existing solver
    exitflag = osqp_setup(&solver, &P_csc, q.data(), &A_csc, bmin.data(), bmax.data(), m, n, settings);

    // Solve the optimization problem
    if(!exitflag) exitflag = osqp_solve(solver);
    // std::cout << "exitflag: " << exitflag << std::endl;


    // Calculate the starting index for u
    int start_idx = (Nt + 1) * Nx;

    // Extract solution
    VectorXd u(Nu);
    for(int i = 0; i < Nu; ++i) {
        u(i) = solver->solution->x[start_idx + i];
    }

    // Cleanup
    osqp_cleanup(solver);
    if(P_csc.p) free(P_csc.p);
    if(P_csc.i) free(P_csc.i);
    if(P_csc.x) free(P_csc.x);
    if(A_csc.p) free(A_csc.p);
    if(A_csc.i) free(A_csc.i);
    if(A_csc.x) free(A_csc.x);
    if (settings) free(settings);

    return u;
}
