#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/EulerAngles>
#include <vector>
#include <map>
#include <variant>
#include <chrono>
#include <osqp/osqp.h>
#include <iostream>


#include "../communication/mpc_cmd_pub.hpp"
#include "../communication/gtodom_sub.hpp"
#include "../param_class.h"
#include "../blue_boat.h"
#include "eMPC.h"

using namespace Eigen;

Logger sim_mpc(const VectorXd& q0, const VectorXd& p0, const VectorXd& w0, const VectorXd& v0, const Param& param, 
    const std::shared_ptr<MPCCmdPublishNode>& cmd_pub, 
    const std::shared_ptr<GtOdomSubscriberNode>& gtodom_sub);