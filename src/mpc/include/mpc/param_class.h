#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "utils/LieGroup.h"

using namespace Eigen;
using namespace std;

class Param {
public:
    int mode;                   // 1: zigzag, 2: circling
    double dt;               
    int Nx;                    
    int Nu;                     
    int Nt;                    
    int Nsim; 
    double V_c;
    double beta_c;             
    MatrixXd I;
    VectorXd xmax;
    VectorXd xmin;
    VectorXd umax;
    VectorXd umin;

    // Initial states
    Vector4d q0_ref;    // desired quaternion
    Vector3d p0_ref;    // desired position
    Vector3d w0_ref;    // desired yaw velocity
    Vector3d v0_ref;    // desired surge velocity
    // Reference trajectory setting
    VectorXd x0_ref;
    VectorXd xid_ref;
    MatrixXd X_ref;
    MatrixXd xi_ref;
    Matrix4d init_pose;

    // Design cost function parameters
    VectorXd diag_values_cost;      // Diagonal values for the cost matrices for P, Q
    MatrixXd P;
    MatrixXd Q;
    MatrixXd R;

    // Constructor
    Param();

    void set_desired_init_position(VectorXd& x_init);
    void set_reference_trajectory();

};
class Logger {
    public:
        // Member variables
        MatrixXd X;
        MatrixXd Err;
        MatrixXd U;
        MatrixXd Err2;
        double taken_time;
    
        // Constructor
        Logger(int Nsim, int Nt);
    };
