#pragma once

#include <eigen3/Eigen/Dense>

using namespace Eigen;

Matrix3d skew(const Vector3d& w);
MatrixXd adjoint_(const VectorXd& x);
MatrixXd coadjoint(const VectorXd& x);
MatrixXd AdjointMat(const Matrix4d& x);
