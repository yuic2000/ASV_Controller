#pragma once

#include <iostream>
#include <vector>
#include <cmath>

#include "utils/matplotlibcpp.h"
#include "param_class.h"

namespace plt = matplotlibcpp;
using namespace Eigen;

void plot_mpc(const std::vector<Logger>& Logger_list, const Param& param);