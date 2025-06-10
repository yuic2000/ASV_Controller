#include "../../include/mpc/utils/LieGroup.h"

// Convert quaternion to rotation matrix
Matrix3d quat2rotm(const Vector4d& quat) {      // quat: x, y, z, w
    Quaterniond q(quat);                        // w, x, y, z
    q.normalize();
    return q.toRotationMatrix();
}

Matrix4d pose_to_T_matrix(const VectorXd& waypoint) {
    // Converts a waypoint [qx, qy, qz, qw, x, y, z] to a 4x4 transformation matrix
    Vector4d quaternion = waypoint.segment<4>(0);  // x, y, z, w
    Vector3d position = waypoint.segment<3>(4);

    // Create rotation matrix from quaternion
    Matrix3d R = quat2rotm(quaternion);

    // Create transformation matrix
    Matrix4d T = Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = position;

    return T;
}

Matrix3d skew(const Vector3d& w) {
    Matrix3d m;
    m << 0, -w.z(), w.y(),
         w.z(), 0, -w.x(),
         -w.y(), w.x(), 0;
    return m;
}

MatrixXd adjoint_(const VectorXd& x) {
    Vector3d w = x.segment<3>(0);
    Vector3d v = x.segment<3>(3);

    MatrixXd adx(6, 6);
    adx << skew(w), Matrix3d::Zero(),
           skew(v), skew(w);
    return adx;
}

MatrixXd coadjoint(const VectorXd& x) {
    return adjoint_(x).transpose();
}

MatrixXd AdjointMat(const Matrix4d& x) {
    Matrix3d R = x.block<3, 3>(0, 0);
    Vector3d p = x.block<3, 1>(0, 3);

    MatrixXd Adx(6, 6);
    Adx << R, Matrix3d::Zero(),
           skew(p) * R, R;
    return Adx;
}

// int main() {
//     // Example usage
//     VectorXd x(6);
//     x << 1, 2, 3, 4, 5, 6;

//     Matrix4d mat;
//     mat << 1, 0, 0, 1,
//            0, 1, 0, 2,
//            0, 0, 1, 3,
//            0, 0, 0, 1;

//     std::cout << "Adjoint Matrix:\n" << AdjointMat(mat) << std::endl;
//     std::cout << "Adjoint Representation:\n" << adjoint_(x) << std::endl;
//     std::cout << "Coadjoint Representation:\n" << coadjoint(x) << std::endl;

//     return 0;
// }