#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

using namespace Eigen;

Matrix3d quat2rotm(const Vector4d& quat);
Matrix4d pose_to_T_matrix(const VectorXd& waypoint);
Matrix3d skew(const Vector3d& w);
MatrixXd adjoint_(const VectorXd& x);
MatrixXd coadjoint(const VectorXd& x);
MatrixXd AdjointMat(const Matrix4d& x);