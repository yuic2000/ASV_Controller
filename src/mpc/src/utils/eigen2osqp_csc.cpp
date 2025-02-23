#include "../../include/mpc/utils/eigen2osqp_csc.h"

// Utility function to convert Eigen::SparseMatrix to OSQPCscMatrix
void eigen2CscMatrix(OSQPCscMatrix& csc, const SparseMatrix<double>& mat) {

    csc.m = mat.rows();
    csc.n = mat.cols();
    csc.nzmax = mat.nonZeros();
    csc.nz = -1; // Indicate CSC format

    // Allocate memory for p, i, x
    csc.p = (OSQPInt*)malloc((csc.n + 1) * sizeof(OSQPInt));
    csc.i = (OSQPInt*)malloc(csc.nzmax * sizeof(OSQPInt));
    csc.x = (OSQPFloat*)malloc(csc.nzmax * sizeof(OSQPFloat));

    // Populate p, i, x from Eigen SparseMatrix
    for(int col = 0; col < mat.outerSize(); ++col) {
        csc.p[col] = mat.outerIndexPtr()[col];
    }
    csc.p[csc.n] = mat.outerIndexPtr()[mat.outerSize()];

    for(int idx = 0; idx < mat.nonZeros(); ++idx) {
        csc.i[idx] = mat.innerIndexPtr()[idx];
        csc.x[idx] = mat.valuePtr()[idx];
    }

}

// int main() {
//     MatrixXd Irb(6, 6);
//     Irb << 55.0, 0, 0, 0, -11.0, 0,
//            0, 55.0, 0, 11.0, 0, 11.0,
//            0, 0, 55.0, 0, -11.0, 0,
//            0, 11.0, 0, 14.6643, 0, 4.4,
//            -11.0, 0, -11.0, 0, 22.55, 0,
//             0, 11.0, 0, 4.4, 0, 18.15;
//     SparseMatrix<double> Irb_sparse = Irb.sparseView();

//     OSQPCscMatrix csc = eigenToCscMatrix(Irb_sparse);

//     std::cout << "m: " << csc.m << ", n: " << csc.n << ", nzmax: " << csc.nzmax << ", nz: " << csc.nz << std::endl;
//     std::cout << "p: ";
//     for (int i = 0; i <= csc.n; ++i) {
//         std::cout << csc.p[i] << " ";
//     }
//     std::cout << std::endl;

//     std::cout << "i: ";
//     for (int i = 0; i < csc.nzmax; ++i) {
//         std::cout << csc.i[i] << " ";
//     }
//     std::cout << std::endl;

//     std::cout << "x: ";
//     for (int i = 0; i < csc.nzmax; ++i) {
//         std::cout << csc.x[i] << " ";
//     }
//     std::cout << std::endl;

//     // Free allocated memory
//     free(csc.p);
//     free(csc.i);
//     free(csc.x);

//     return 0;
// }