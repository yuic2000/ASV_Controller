#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>
#include <map>
#include <variant>

#include "LieGroup.h"
#include "../param_class.h"


using namespace Eigen;

void eMPCCost(MatrixXd& M, VectorXd& q, const MatrixXd& xid, const Param& param);
// std::tuple<MatrixXd, VectorXd> eMPCCost(const MatrixXd& xid, const Param& param);

