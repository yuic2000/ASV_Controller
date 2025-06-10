#pragma once

#include <iostream>
#include <vector>
#include <cmath>

#include "utils/matplotlibcpp.h"
#include "param_class.h"

namespace plt = matplotlibcpp;
using namespace Eigen;

void plot_mpc(const Logger& logger, const Param& param);
