#pragma once

#include <eigen3/Eigen/Dense>
#include <tuple>
#include <vector>
#include <cmath>
#include "utils/gnc.h"

using namespace Eigen;

// Struct to hold the output
// struct BlueBoatOutput {
//     VectorXd xdot;
//     double U;
// };

VectorXd blue_boat(const VectorXd& x, const VectorXd& n, double V_c, double beta_c);