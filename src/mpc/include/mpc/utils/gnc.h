#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>
#include <utility>
#include <algorithm>
#include <cmath>

using namespace Eigen;

double sat(double x, double x_min, double x_max);
Matrix3d Smtrx(const Vector3d& a);
MatrixXd Hmtrx(const Vector3d& r);
Matrix3d Rzyx(double phi, double theta, double psi);
Matrix3d Tzyx(double phi, double theta);
MatrixXd eulerang(double phi, double theta, double psi);
MatrixXd m2c(const MatrixXd& M, const VectorXd& nu);
double Hoerner(double B, double T);
VectorXd crossFlowDrag(double L, double B, double T, const VectorXd& nu_r);
std::pair<double, double> addedMassSurge(double m, double L, double rho = 1025);
