#pragma once

#include <eigen3/Eigen/Sparse>
#include <osqp/osqp.h>
#include <stdlib.h>

using namespace Eigen;

// Utility function to convert Eigen::SparseMatrix to OSQPCscMatrix
void eigen2CscMatrix(OSQPCscMatrix& csc, const SparseMatrix<double>& mat);
// OSQPCscMatrix eigen2CscMatrix(const SparseMatrix<double>& mat);