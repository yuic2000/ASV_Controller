#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <map>
#include <variant>
#include <osqp/osqp.h>
#include <tuple>
#include <stdlib.h>
#include <chrono>
#include <iostream>

#include "../utils/eMPCConstraints.h"
#include "../utils/eMPCCost.h"
#include "../utils/LieGroup.h"
#include "../utils/eigen2osqp_csc.h"
#include "../param_class.h"

using namespace Eigen;

VectorXd eMPC(
    const VectorXd& x0, const VectorXd& xd, const MatrixXd& xid,
    const Param& param);