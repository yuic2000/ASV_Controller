#include "../../include/mpc/controller/eMPC.h"
 

VectorXd eMPC(const VectorXd& x0, const VectorXd& xd, const MatrixXd& xid,
    const Param& param){
    
    const int Nx = param.Nx;
    const int Nu = param.Nu;
    const int Nt = param.Nt;
    const int n_total = (Nt + 1) * Nx + Nt * Nu;    // 1412

// start
    // auto start_time = std::chrono::high_resolution_clock::now();

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


    // ------ eMPCConstraints (A is ~60% sparse) ------ 
    MatrixXd A = MatrixXd::Zero(n_total, n_total);          // 1412 x 1412
    VectorXd bmin = VectorXd::Zero(n_total);                // 1412
    VectorXd bmax = VectorXd::Zero(n_total);                // 1412
    eMPCConstraints(A, bmin, bmax, xid, p0, x0, param);     // find A, bmin, bmax via eMPCConstraints


    // ------ eMPCCost (M is ~90% sparse, directly construct matrix in triplets rather than dense matrix M) ------
    // MatrixXd M = MatrixXd::Zero(n_total, n_total);          // 1412 x 1412
    std::vector<Triplet<double>> M_triplets;
    M_triplets.reserve(n_total * 5);                        // Reserve space for non-zero elements
    VectorXd q = VectorXd::Zero(n_total);                   // 1412
    // eMPCCost(M, q, xid, param);                          // find M, q via eMPCCost, M is dense matrix
    eMPCCostSparse(M_triplets, q, xid, param);              // find M, q via eMPCCostSparse, M is sparse matrix in triplets

    // ------ Setup OSQP problem ------
    OSQPSolver*   solver   = nullptr;
    OSQPSettings *settings = (OSQPSettings *)malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(settings);
    settings->verbose = false;
    // settings->max_iter = 1500;
    settings->eps_abs = 1e-3;
    settings->eps_rel = 1e-3;
    settings->polishing = true;
    settings->time_limit = 3.5e-2;        // 0.05s, 50 ms, should test in Jetson

    OSQPInt exitflag = 0;


    // ------ Convert matrices to Sparse Matrices, A and M ------ 
    // (M is ~90% sparse, directly use triplets in eMPCCostSparse rather than dense matrix M)
    SparseMatrix<double> M_sparse(n_total, n_total);
    M_sparse.setFromTriplets(M_triplets.begin(), M_triplets.end());
    
    SparseMatrix<double> A_sparse = A.sparseView(1e-9);         // 1412 x 1412

    // ------ Populate OSQPData with M_sparse, q, A_sparse, l, u ------
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

    // auto end_time = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> solving_time = end_time - start_time;    
    // std::cout << "exitflag: " << exitflag << std::endl;
    // OSQP status
    // if (!exitflag && solver->info) {
    //     std::cout << "Solver iterations: " << solver->info->iter << std::endl;
    //     std::cout << "Solving time: " << solving_time.count() << " seconds" << std::endl;
    // }
    
    // if (!exitflag) {
    //     // Solver ran — now inspect status
    //     if (solver->info) {
    //         std::cout << "Solver status string: " << solver->info->status << std::endl;
    //         std::cout << "Solver status val: " << solver->info->status_val << std::endl;
    
    //         if (solver->info->status_val == OSQP_MAX_ITER_REACHED) {
    //             std::cout << "[⚠️] Solver reached maximum iteration limit!" << std::endl;
    //         } else if (solver->info->status_val == OSQP_SOLVED) {
    //             std::cout << "[✅] Solver converged successfully." << std::endl;
    //         } else {
    //             std::cout << "[❗] Solver terminated with status: " << solver->info->status << std::endl;
    //         }
    
    //         std::cout << "Iterations used: " << solver->info->iter << std::endl;
    //     } else {
    //         std::cerr << "[❌] Solver info is null!" << std::endl;
    //     }
    // } else {
    //     std::cerr << "[❌] osqp_solve failed with exitflag: " << exitflag << std::endl;
    // }
    


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
