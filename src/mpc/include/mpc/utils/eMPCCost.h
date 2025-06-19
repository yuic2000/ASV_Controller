#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <map>
#include <variant>

#include "LieGroup.h"
#include "../param_class.h"


using namespace Eigen;

void eMPCCost(MatrixXd& M, VectorXd& q, const MatrixXd& xid, const Param& param);
void eMPCCostSparse(std::vector<Triplet<double>>& M_triplets, 
    VectorXd& q, 
    const MatrixXd& xid, 
    const Param& param);

void addDiagonalBlockToTriplets(std::vector<Triplet<double>>& triplets,
                                const MatrixXd& block,
                                int start_index,
                                double threshold = 1e-12);

void addUpperTriangularBlockToTriplets(std::vector<Triplet<double>>& triplets,
                                        const MatrixXd& block,
                                        int start_row,
                                        int start_col,
                                        double threshold = 1e-12);
