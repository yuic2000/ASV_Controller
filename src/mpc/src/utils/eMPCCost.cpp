#include "../../include/mpc/utils/eMPCCost.h"

// Helper function to add upper triangular block to triplets
void addUpperTriangularBlockToTriplets(std::vector<Triplet<double>>& triplets,
    const MatrixXd& block, int start_row, int start_col, double threshold) {
    for (int i = 0; i < block.rows(); ++i) {
        for (int j = 0; j < block.cols(); ++j) {
            int global_row = start_row + i;
            int global_col = start_col + j;

            // Only add upper triangular elements
            if (global_col >= global_row && std::abs(block(i, j)) > threshold) {
                triplets.emplace_back(global_row, global_col, block(i, j));
            }
        }
    }
}

// Helper function to add diagonal block to triplets
void addDiagonalBlockToTriplets(std::vector<Triplet<double>>& triplets,
    const MatrixXd& block, int start_index, double threshold) {
    for (int i = 0; i < block.rows(); ++i) {
        for (int j = 0; j < block.cols(); ++j) {  
            if (std::abs(block(i, j)) > threshold) {
                triplets.emplace_back(start_index + i, start_index + j, block(i, j));
            }
        }
    }
}

void eMPCCostSparse(std::vector<Triplet<double>>& M_triplets, 
                    VectorXd& q, const MatrixXd& xid, const Param& param) {

    const int Nx = param.Nx;
    const int Nu = param.Nu;
    const int Nt = param.Nt;
    const int n_total = (Nt + 1) * Nx + Nt * Nu;

    const MatrixXd Q = param.Q;
    const MatrixXd R = param.R;
    const MatrixXd P = param.P;

    // Estimate sparsity: Each block contributes Nx*Nx elements + Nu*Nu for controls
    M_triplets.reserve(2 * n_total);

    q = VectorXd::Zero(n_total);

    // Pre-compute C^T * Q and C^T * P to avoid repeated computation
    std::vector<MatrixXd> CtQ_precomputed(Nt);
    std::vector<MatrixXd> CtQC_precomputed(Nt);
    std::vector<VectorXd> q_contributions(Nt);

    // Process trajectory points k = 0 to Nt-2 (using Q matrix)
    #pragma omp parallel for
    for (int k = 0; k < Nt - 1; ++k) {
        MatrixXd C = MatrixXd::Identity(12, 12);
        C.block<6, 6>(6, 0) = -adjoint_(xid.row(k).transpose());

        // Pre-compute for this iteration
        MatrixXd CtQ = C.transpose() * Q;
        CtQC_precomputed[k] = CtQ * C;

        VectorXd b = VectorXd::Zero(12);
        b.segment<6>(6) = xid.row(k).transpose();
        q_contributions[k] = -CtQ * b;
    }

    // Handle the final stage k = Nt-1 (using P matrix)
    int k = Nt - 1;
    MatrixXd C = MatrixXd::Identity(12, 12);
    C.block<6, 6>(6, 0) = -adjoint_(xid.row(k).transpose());

    MatrixXd CtP = C.transpose() * P;
    CtQC_precomputed[k] = CtP * C;

    VectorXd b = VectorXd::Zero(12);
    b.segment<6>(6) = xid.row(k).transpose();
    q_contributions[k] = -CtP * b;

    // Add all state cost blocks to triplets (sequential to avoid race conditions)
    for (int k = 0; k < Nt; ++k) {
        addUpperTriangularBlockToTriplets(M_triplets, CtQC_precomputed[k], 
        k * Nx, k * Nx);
        q.segment(k * Nx, Nx) = q_contributions[k];
    }

    // Add control cost blocks (R matrices) - these are diagonal blocks
    const int control_start = (Nt) * Nx;  // Controls start after all states
    for (int j = 0; j < Nt; ++j) {
        int start_index = control_start + j * Nu;
        addDiagonalBlockToTriplets(M_triplets, R, start_index);
    }

    // std::cout << "M_triplets final size: " << M_triplets.size() << std::endl;
}

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

    #pragma omp parallel for
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

    #pragma omp parallel for
    for (int j = 1; j <= Nt; ++j) {
        int start_index = (k + 1) * Nx + (j - 1) * Nu;
        M.block(start_index, start_index, Nu, Nu) = R;
    }

}