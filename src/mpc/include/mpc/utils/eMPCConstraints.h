#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>
#include <map>
#include <variant>

#include "LieGroup.h"
#include "../param_class.h"

using namespace Eigen;

void eMPCConstraints(MatrixXd &A, VectorXd &bmin, VectorXd &bmax, const MatrixXd& xid, const VectorXd& p0, const VectorXd& x0, const Param& param);

// std::tuple<MatrixXd, VectorXd, VectorXd> eMPCConstraints(
//     const MatrixXd& xid, const VectorXd& p0, const VectorXd& x0,
//     const Param& param);